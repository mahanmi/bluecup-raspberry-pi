import tkinter as tk

# --- Game Engine Constants (Optional, can be defined in the main game class) ---
UPDATE_DELAY = 16  # approx 62.5 FPS
DEFAULT_WIDTH = 800
DEFAULT_HEIGHT = 600


class GameObject:
    """
    Represents a generic object in the game.
    """

    def __init__(self, canvas, x, y, dx, dy, shape_coords, color="white", tags="game_object"):
        self.canvas = canvas
        self.x = x
        self.y = y
        self.dx = dx  # Velocity in x
        self.dy = dy  # Velocity in y
        self.shape_coords = shape_coords  # Relative coordinates for the polygon
        self.color = color
        self.tags = tags

        abs_coords = [c + self.x if i % 2 == 0 else c +
                      self.y for i, c in enumerate(self.shape_coords)]
        self.shape_id = self.canvas.create_polygon(
            abs_coords, outline=self.color, fill="", tags=self.tags)
        self.is_active = True

    def move(self, canvas_width, canvas_height, wrap=True):
        if not self.is_active:
            return

        self.x += self.dx
        self.y += self.dy

        if wrap:
            coords = self.canvas.coords(self.shape_id)
            if not coords:
                return

            min_x_rel = min(self.shape_coords[::2])
            max_x_rel = max(self.shape_coords[::2])
            min_y_rel = min(self.shape_coords[1::2])
            max_y_rel = max(self.shape_coords[1::2])

            obj_pixel_width = max_x_rel - min_x_rel
            obj_pixel_height = max_y_rel - min_y_rel

            # More accurate wrapping based on logical center and relative shape extends
            if self.x + max_x_rel < 0:  # Object's rightmost point is off left screen
                self.x = canvas_width - min_x_rel
            elif self.x + min_x_rel > canvas_width:  # Object's leftmost point is off right screen
                self.x = -max_x_rel
            if self.y + max_y_rel < 0:  # Object's bottommost point is off top screen
                self.y = canvas_height - min_y_rel
            elif self.y + min_y_rel > canvas_height:  # Object's topmost point is off bottom screen
                self.y = -max_y_rel

        # Update canvas position
        # For rotated objects, this move is just a delta, _update_shape will set final coords
        # For non-rotated, this is fine, but we will re-coord for wrapping anyway.
        # self.canvas.move(self.shape_id, self.dx, self.dy) # This can be problematic with rotation + wrapping

        # Always use absolute coordinates after logical position update for robustness
        new_abs_coords = []
        for i, c_val in enumerate(self.shape_coords):
            if i % 2 == 0:
                new_abs_coords.append(c_val + self.x)
            else:
                new_abs_coords.append(c_val + self.y)

        if self.shape_id:
            try:
                self.canvas.coords(self.shape_id, *new_abs_coords)
            except tk.TclError:
                self.shape_id = None

    def delete(self):
        self.is_active = False
        if self.shape_id:
            try:
                self.canvas.delete(self.shape_id)
            except tk.TclError:
                pass  # Widget might already be destroyed
            self.shape_id = None

    def get_bounding_box(self):
        if self.is_active and self.shape_id:
            try:
                return self.canvas.bbox(self.shape_id)
            except tk.TclError:
                return None
        return None

    def collision_check(self, other_object):
        if not self.is_active or not other_object.is_active or not self.shape_id or not other_object.shape_id:
            return False
        bb1 = self.get_bounding_box()
        bb2 = other_object.get_bounding_box()
        if bb1 and bb2:
            overlapping_items = self.canvas.find_overlapping(
                bb1[0], bb1[1], bb1[2], bb1[3])
            return other_object.shape_id in overlapping_items
        return False


class PlayerObject(GameObject):
    pass


class Game():
    def __init__(self, width=DEFAULT_WIDTH, height=DEFAULT_HEIGHT, title="Game", update_delay=UPDATE_DELAY):
        self.width = width
        self.height = height
        self.title = title
        self.update_delay = update_delay

        self.root = tk.Tk()
        self.root.title(self.title)
        self.root.resizable(False, False)
        # Ensure window closes cleanly
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        self.canvas = tk.Canvas(
            self.root, width=self.width, height=self.height, bg="black")
        self.canvas.pack()

        self.game_objects = []
        self.key_pressed = {}

        # Keyboard bindings remain in engine, but game logic won't use them for player
        self.root.bind("<KeyPress>", self._on_key_press)
        self.root.bind("<KeyRelease>", self._on_key_release)

        self.is_running = False

    def on_closing(self):
        """Handles window close button."""
        self.stop()

    def _on_key_press(self, event):
        self.key_pressed[event.keysym.lower()] = True

    def _on_key_release(self, event):
        self.key_pressed[event.keysym.lower()] = False

    def is_key_down(self, key_symbol):  # Will not be used by game logic for player control
        return self.key_pressed.get(key_symbol.lower(), False)

    def add_object(self, game_object):
        self.game_objects.append(game_object)

    def remove_object(self, game_object):
        if game_object in self.game_objects:
            game_object.delete()
            self.game_objects.remove(game_object)

    def update_game_logic(self):
        """This method should be overridden by the specific game."""
        pass

    def game_loop(self):
        if not self.is_running:
            return

        self.update_game_logic()

        active_game_objects = []
        for obj in self.game_objects:
            if obj.is_active:
                # Player object handles its own move/update_shape for rotation
                # Player move is handled in its update
                if not isinstance(obj, PlayerObject):
                    obj.move(self.width, self.height)
                active_game_objects.append(obj)
            else:
                obj.delete()  # Ensure canvas item is gone if not already
        self.game_objects = active_game_objects

        self.canvas.update_idletasks()
        self.canvas.update()

        self.root.after(self.update_delay, self.game_loop)

    def start(self):
        self.is_running = True
        self.game_loop()
        self.root.mainloop()

    def stop(self):
        self.is_running = False
        # Safely destroy Tkinter window if it exists
        try:
            if self.root:
                self.root.destroy()
        except tk.TclError:
            pass  # Window might already be gone
