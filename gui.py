from lib.game_engine import *
from controller.server import *
import lib.logging as logging

import tkinter as tk
import math

# --- Game Specific Settings ---
PLAYER_THRUST = 0.15
PLAYER_TURN_SPEED = 5  # degrees
PLAYER_DRAG = 0.985  # Multiplier for velocity, a bit of slowdown
BULLET_SPEED = 7
BULLET_LIFESPAN = 70  # Frames
BULLET_DELAY = 0.1  # Min wait between each shoot in ms
MAX_BULLETS = 7  # Max bullets on screen from player
CONTROL_SERVER_HOST = 'localhost'
CONTROL_SERVER_PORT = 12345
SERVER_ADDRESS = (CONTROL_SERVER_HOST, CONTROL_SERVER_PORT)

# --- Helper Functions ---


def deg_to_rad(degrees):
    return degrees * (math.pi / 180)


# --- Game Entity Classes ---
class Player(PlayerObject):
    def __init__(self, canvas, x, y):
        self.base_shape_coords = [15, 0, -10, -10, -10, 10]
        self.angle = -90
        self.rotation_speed = PLAYER_TURN_SPEED
        self.thrust_power = PLAYER_THRUST
        self.drag = PLAYER_DRAG

        initial_rotated_relative_coords = self._calculate_current_relative_shape()
        super().__init__(canvas, x, y, 0, 0,
                         initial_rotated_relative_coords, "cyan", tags="player")

    def _calculate_current_relative_shape(self):
        rad = deg_to_rad(self.angle)
        cos_a = math.cos(rad)
        sin_a = math.sin(rad)
        rotated_coords = []
        for i in range(0, len(self.base_shape_coords), 2):
            x0, y0 = self.base_shape_coords[i], self.base_shape_coords[i+1]
            new_x = x0 * cos_a - y0 * sin_a
            new_y = x0 * sin_a + y0 * cos_a
            rotated_coords.extend([new_x, new_y])
        return rotated_coords

    def _update_shape(self):
        new_relative_rotated_coords = self._calculate_current_relative_shape()
        self.shape_coords = new_relative_rotated_coords

        abs_coords = [c_val + self.x if i % 2 == 0 else c_val +
                      self.y for i, c_val in enumerate(self.shape_coords)]
        if self.shape_id:
            try:
                self.canvas.coords(self.shape_id, *abs_coords)
            except tk.TclError:
                self.shape_id = None

    def rotate(self, direction):
        self.angle += self.rotation_speed * direction
        self.angle %= 360
        self._update_shape()

    def thrust(self):
        rad = deg_to_rad(self.angle)
        self.dx += math.cos(rad) * self.thrust_power
        self.dy += math.sin(rad) * self.thrust_power

    def apply_drag(self):
        self.dx *= self.drag
        self.dy *= self.drag

    def update(self, canvas_width, canvas_height):
        if not self.is_active:
            return
        self.apply_drag()
        # Player uses its own move that integrates _update_shape for correct visual
        super().move(canvas_width, canvas_height, wrap=True)  # Use GameObject's move
        self._update_shape()  # Ensure rotation is applied after logical position update

    def shoot(self, bullets_list_ref, last_shoot=0):  # Pass by reference to check length
        if len(bullets_list_ref) < MAX_BULLETS and time.time() - last_shoot > BULLET_DELAY:
            rad = deg_to_rad(self.angle)
            tip_local_x = 15
            tip_local_y = 0
            tip_offset_x = tip_local_x * \
                math.cos(rad) - tip_local_y * math.sin(rad)
            tip_offset_y = tip_local_x * \
                math.sin(rad) + tip_local_y * math.cos(rad)
            bullet_x = self.x + tip_offset_x
            bullet_y = self.y + tip_offset_y
            bullet_dx = math.cos(rad) * BULLET_SPEED + \
                self.dx * 0.5  # Add some of ship's velocity
            bullet_dy = math.sin(rad) * BULLET_SPEED + self.dy * 0.5
            return Bullet(self.canvas, bullet_x, bullet_y, bullet_dx, bullet_dy)
        return None


class Bullet(GameObject):
    def __init__(self, canvas, x, y, dx, dy):
        shape = [-2, -2, 2, -2, 2, 2, -2, 2]
        super().__init__(canvas, x, y, dx, dy, shape, "yellow", tags="bullet")
        self.lifespan = BULLET_LIFESPAN

    def update(self):  # Bullets have their own update logic for lifespan
        if not self.is_active:
            return
        self.lifespan -= 1
        if self.lifespan <= 0:
            self.is_active = False
        # Movement is handled by the main game loop's processing of game_objects


# --- Main Application Class ---
class ExternallyControlledShipGame(Game):
    def __init__(self):
        super().__init__(title="Externally Controlled Ship")
        self.log = logging.getLogger('Game')
        self.player = None
        self.bullets = []
        self.last_shoot = 0

        self.external_command_state = {
            'throttle': False,
            'pitch': 0,
            'shoot_request': False
        }

        self.initialize_game_elements()

    def initialize_game_elements(self):
        self.log.debug('Initializing game elements')
        # Clear previous game objects if any (for a potential restart)

        for obj in list(self.game_objects):
            self.remove_object(obj)
        self.bullets.clear()

        self.player = Player(self.canvas, self.width / 2, self.height / 2)
        self.add_object(self.player)

    def handle_external_commands(self):
        if not self.player or not self.player.is_active:
            return

        if self.external_command_state['throttle']:
            self.player.thrust()

        pitch_val = self.external_command_state['pitch']
        if pitch_val < 0:
            self.player.rotate(-1)
        elif pitch_val > 0:
            self.player.rotate(1)

        if self.external_command_state['shoot_request']:
            # Pass self.bullets to check MAX_BULLETS
            bullet = self.player.shoot(self.bullets, self.last_shoot)
            if bullet:
                self.bullets.append(bullet)
                self.add_object(bullet)
                self.last_shoot = time.time()
            # Consume the request
            self.external_command_state['shoot_request'] = False

    def update_game_logic(self):
        self.handle_external_commands()

        if self.player and self.player.is_active:
            self.player.update(self.width, self.height)

        # Update and manage bullets
        active_bullets = []
        for bullet in self.bullets:
            if bullet.is_active:
                bullet.update()  # For lifespan
                # Bullet movement is handled by the main game loop's processing of game_objects
                if bullet.is_active:  # Check again after its own update
                    active_bullets.append(bullet)
            else:  # Bullet became inactive (e.g. lifespan expired)
                # Remove from engine's list and canvas
                self.remove_object(bullet)
        self.bullets = active_bullets

    def stop(self):
        self.log.info("Initiating shutdown...")
        super().stop()  # Calls parent Game.stop() which handles Tkinter window


# --- Main Execution Block ---
if __name__ == '__main__':
    log = logging.getLogger('main')
    game_instance = ExternallyControlledShipGame()
    control_server = ControlServer(
        SERVER_ADDRESS,
        game_instance.external_command_state
    )
    try:
        control_server.start()
        game_instance.start()
    except KeyboardInterrupt:
        log.debug("Keyboard interrupt received, stopping game.")
        control_server.stop()
        game_instance.stop()
    except Exception as e:
        log.error(f"An unexpected error occurred: {e}")
        game_instance.stop()
