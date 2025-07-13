from typing import Callable, Any
import logging
# Assuming Robot class is defined in robot.py within the same package
from .robot import Robot  # Use relative import


class CommandProcessor:
    def __init__(self, robot_instance: Robot):
        self.logger = logging.getLogger(self.__class__.__name__)
        self.robot = robot_instance
        self.logger.info("CommandProcessor initialized.")

        # Define available commands and their handlers
        # Handlers are methods of this class or the Robot class
        self.commands: dict[str, Callable[[], dict | Any]] = {
            "ARM": self.handle_arm,
            "DISARM": self.handle_disarm,
            "SET_MOVEMENT": self.handle_set_movement,  # Expects x, y, z, yaw
            "STOP_MOVEMENT": self.handle_stop_movement,
            "SET_CAMERA_PAN_TILT": self.handle_set_camera_pan_tilt,  # Expects pan, tilt
            "UPDATE_TELEMETRY": self.handle_update_telemetry,  # Force update
            "GET_STATE": self.handle_get_state,
            # Add more commands as needed: e.g., "SET_LIGHTS", "GRIPPER_CLOSE"
        }

    def process_command(self, command_name: str, **kwargs) -> dict:
        """
        Processes a given command.
        Args:
            command_name (str): The name of the command (e.g., "ARM", "SET_MOVEMENT").
                                Case-insensitive for robustness.
            **kwargs: Arguments specific to the command.
        Returns:
            dict: A result dictionary, typically {"status": "success/error", "message": "...", "data": ...}.
        """
        command_name_upper = command_name.upper()
        handler = self.commands.get(command_name_upper)

        if not handler:
            self.logger.warning(f"Unknown command: {command_name}")
            return {"status": "error", "message": f"Unknown command: {command_name}"}

        try:
            self.logger.info(
                f"Processing command: {command_name_upper} with args: {kwargs}")
            result = handler(**kwargs)  # Call the appropriate handler method
            # Handlers should return a dict or relevant data that can be wrapped
            if isinstance(result, dict) and "status" in result:
                return result  # Handler provided full response
            else:  # Wrap simple boolean or data results
                return {"status": "success", "message": f"Command '{command_name_upper}' executed.", "data": result}

        except TypeError as e:  # Mismatch in arguments
            self.logger.error(
                f"Argument error for command {command_name_upper} with {kwargs}: {e}")
            return {"status": "error", "message": f"Invalid arguments for {command_name_upper}: {e}"}
        except Exception as e:
            self.logger.error(
                f"Error processing command {command_name_upper}: {e}", exc_info=True)
            return {"status": "error", "message": f"Error executing {command_name_upper}: {str(e)}"}

    # --- Command Handler Methods ---

    def handle_arm(self, **kwargs) -> bool:
        return self.robot.arm()

    def handle_disarm(self, **kwargs) -> bool:
        return self.robot.disarm()

    def handle_set_movement(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, yaw: float = 0.0, **kwargs) -> None:
        # Values should be normalized (-1.0 to 1.0)
        self.robot.set_movement_targets(
            float(x), float(y), float(z), float(yaw))
        return None  # Or return current thruster outputs if desired

    def handle_stop_movement(self, **kwargs) -> None:
        # This will also stop motors if armed
        self.robot.set_movement_targets(0.0, 0.0, 0.0, 0.0)
        return None

    def handle_set_camera_pan_tilt(self, pan: int | None = None, tilt: int | None = None, **kwargs) -> bool:
        # Ensure types if coming from API (e.g., strings to int)
        pan_angle = int(pan) if pan is not None else None
        tilt_angle = int(tilt) if tilt is not None else None
        return self.robot.set_camera_pan_tilt(pan_angle=pan_angle, tilt_angle=tilt_angle)

    def handle_update_telemetry(self, **kwargs) -> dict:
        self.robot.update_telemetry(force_update=True)
        return self.robot.telemetry_data  # Return the updated telemetry

    def handle_get_state(self, **kwargs) -> dict:
        return self.robot.get_current_state()


# Example Usage (requires a Robot instance, which in turn requires hardware interfaces)
if __name__ == "__main__":
    print("Setting up CommandProcessor example...")
    try:
        # This will try to initialize the full Robot stack, including hardware.
        # For isolated testing, you might mock the Robot class.
        test_robot = Robot()
        processor = CommandProcessor(test_robot)

        if not test_robot.comm.is_connected():
            print(
                "WARNING: Robot is not connected to serial. Commands might not have full effect.")
            # For some tests to pass without serial, we might need a more sophisticated mock.

        print("\n--- Testing Command Processor ---")

        # Test ARM
        print("Sending ARM command...")
        result = processor.process_command("ARM")
        print(f"ARM Result: {result}")
        assert result["status"] == "success" or not test_robot.comm.is_connected()

        # Test SET_MOVEMENT
        print("\nSending SET_MOVEMENT command (x=0.5)...")
        result = processor.process_command(
            "SET_MOVEMENT", x=0.5, y=0.0, z=0.0, yaw=0.0)
        print(f"SET_MOVEMENT Result: {result}")
        assert result["status"] == "success"

        # Test GET_STATE
        print("\nSending GET_STATE command...")
        result = processor.process_command("GET_STATE")
        print(f"GET_STATE Result: {result['data']}")
        assert result["status"] == "success"
        if test_robot.is_armed:  # Only if arming was successful
            assert result['data']['target_movement']['x'] == 0.5

        # Test SET_CAMERA_PAN_TILT
        print("\nSending SET_CAMERA_PAN_TILT (pan=45, tilt=15)...")
        result = processor.process_command(
            "SET_CAMERA_PAN_TILT", pan="45", tilt="-15")  # API might send strings
        print(f"SET_CAMERA_PAN_TILT Result: {result}")
        assert result["status"] == "success" or not test_robot.comm.is_connected()
        if result["status"] == "success":
            current_state = processor.process_command("GET_STATE")['data']
            assert current_state['telemetry']['camera_pan'] == 45
            assert current_state['telemetry']['camera_tilt'] == -15

        # Test STOP_MOVEMENT
        print("\nSending STOP_MOVEMENT command...")
        result = processor.process_command("STOP_MOVEMENT")
        print(f"STOP_MOVEMENT Result: {result}")
        current_state = processor.process_command("GET_STATE")['data']
        assert current_state['target_movement']['x'] == 0.0

        # Test DISARM
        print("\nSending DISARM command...")
        result = processor.process_command("DISARM")
        print(f"DISARM Result: {result}")
        assert result["status"] == "success"

        # Test Unknown command
        print("\nSending UNKNOWN_COMMAND...")
        result = processor.process_command("DO_A_BARREL_ROLL")
        print(f"UNKNOWN_COMMAND Result: {result}")
        assert result["status"] == "error"

        # Test command with missing arguments (if handler expects them and doesn't have defaults)
        # print("\nSending SET_MOVEMENT with missing args...")
        # result = processor.process_command("SET_MOVEMENT", x=0.1) # y, z, yaw missing
        # print(f"SET_MOVEMENT (missing args) Result: {result}")
        # assert result["status"] == "error" # Our current handler has defaults, so this won't error

        test_robot.shutdown()
        print("\nCommandProcessor example finished.")

    except ConnectionError as e:
        print(
            f"Connection Error during example: {e}. Ensure serial port or mock is correctly set up.")
    except Exception as e:
        print(f"An unexpected error occurred in CommandProcessor example: {e}")
