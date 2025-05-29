import gui
import controller.server
import controller.client

SERVER_HOST = 'localhost'
SERVER_PORT = 12345
SERVER_ADDRESS = (SERVER_HOST, SERVER_PORT)


class CommandState(controller.client.State):
    def __init__(self, is_key_down):
        super().__init__()
        self.is_key_down = is_key_down
        self.reset()

    def serialize(self):
        return f"{int(self.throttle)}:{int(self.pitch)}:{int(self.shoot)}"

    def deserialize(self, raw_state):
        throttle_state, pitch_state, shoot_request = raw_state.split(':')
        self.throttle = (throttle_state == '1')
        self.pitch = int(pitch_state)
        if shoot_request == '1':
            self.shoot_request = True

    def update(self):
        # Update control_state based on the currently pressed key
        # Throttle is ON if 'w' is the key currently detected as pressed
        self.throttle = self.is_key_down('w')

        # Pitch:
        if self.is_key_down('a'):
            self.pitch = -1
        elif self.is_key_down('d'):
            self.pitch = 1
        else:  # If 'a' or 'd' are not the currently pressed key, pitch is neutral
            self.pitch = 0

        # Shoot:
        self.shoot = self.is_key_down('space')

        if self.is_key_down('q'):
            return False

    def after_update(self):
        self.shoot = False

    def reset(self):
        self.throttle = False
        self.pitch = 0
        self.shoot = False


if __name__ == '__main__':
    print("\nExternal UDP Spaceship Controller (Stateful Protocol - Smoother Attempt)")
    print("  W: Throttle ON (Hold)")
    print("  A: Pitch LEFT (Hold)")
    print("  D: Pitch RIGHT (Hold)")
    print("  Space: Shoot (Press)")
    print("  Q: Quit Controller")

    game_instance = gui.ExternallyControlledShipGame()
    client = controller.client.ControllerClient(
        SERVER_ADDRESS, CommandState(game_instance.is_key_down))
    server = controller.server.ControlServer(
        SERVER_ADDRESS, game_instance.external_command_state)

    server.start()
    client.start()
    game_instance.start()
    game_instance.stop()
    client.stop()
    server.stop()
