import sys
import os
# Add project root to Python path for direct execution
if __name__ == "__main__":
    project_root = os.path.abspath(os.path.join(
        os.path.dirname(__file__), '..', '..'))
    if project_root not in sys.path:
        sys.path.insert(0, project_root)

# Use absolute imports when running directly, relative when imported
try:
    from . import communication
except ImportError:
    # Running directly, use absolute import
    try:
        from src.hardware_interface import communication
    except ImportError:
        # Running from src directory
        from hardware_interface import communication

# Smart import strategy for config and log
try:
    from src.config import ROV_SERIAL_PORT
    import src.log as log
except ImportError:
    # Running from src directory
    from config import ROV_SERIAL_PORT
    import log

logger = log.getLogger(__name__)

# Global array to store motor values for all 6 motors
# Index corresponds to motor_id (0-5)
motor_values = [0, 0, 0, 0, 0, 0]


def get_command():
    global motor_values
    return "SP " + " ".join(str(val) for val in motor_values) + "\n"


def set_motor_speed(motor_id: int, speed: int) -> bool:
    """
    Sets the speed of a specific motor and sends updated command to STM32.
    Args:
        motor_id (int): The ID of the motor (0-5 for 6 motors).
        speed (int): The speed, e.g., -255 to 255 (0 is stop).
                        The range and meaning depend on your hardware.
    Returns:
        bool: True if the command was likely sent successfully, False otherwise.
    """
    global motor_values

    motor_id = motor_id-1  # Convert to 0-based index

    # Validate motor_id range
    if motor_id < 0 or motor_id >= len(motor_values):
        logger.error(
            f"Invalid motor_id {motor_id}. Must be 0-{len(motor_values)-1}")
        return False

    # Update the global motor values array
    motor_values[motor_id] = speed

    # Send all motor values to STM32 in the required format
    # "SP 50 -30 0 20 -15 75\n"
    command = get_command()

    if communication.send_command(command):
        logger.info(
            f"Motor {motor_id} speed set to {speed}. All motors: {motor_values}")
        return True
    else:
        logger.error(f"Failed to send command to set motor {motor_id} speed.")
        return False


def get_motor_values() -> list[int]:
    """
    Returns the current motor values array.
    Returns:
        list[int]: Current motor values for all 6 motors.
    """
    return motor_values.copy()


def set_all_motor_speeds(speeds: list[int]) -> bool:
    """
    Sets all motor speeds at once.
    Args:
        speeds (list[int]): List of 6 motor speeds.
    Returns:
        bool: True if command sent successfully, False otherwise.
    """
    global motor_values

    if len(speeds) != 6:
        logger.error(f"Expected 6 motor speeds, got {len(speeds)}")
        return False

    # Update all motor values
    motor_values = speeds.copy()

    # Send command to STM32
    command = get_command()

    if communication.send_command(command):
        logger.info(f"All motor speeds set: {motor_values}")
        return True
    else:
        logger.error("Failed to send command to set all motor speeds.")
        return False


def send_current_motor_command() -> bool:
    """
    Sends the current motor values to STM32 without changing them.
    Useful for re-sending the current state.
    Returns:
        bool: True if command sent successfully, False otherwise.
    """
    command = get_command()

    if communication.send_command(command):
        logger.info(f"Current motor command sent: {motor_values}")
        return True
    else:
        logger.error("Failed to send current motor command.")
        return False


def stop_all_motors() -> bool:
    """
    Stops all motors by setting all values to 0.
    """
    global motor_values

    # Set all motor values to 0
    motor_values = [0, 0, 0, 0, 0, 0]

    # Send stop command to STM32
    command = get_command()

    if communication.send_command(command):
        logger.info("All motors commanded to stop.")
        return True
    else:
        logger.error("Failed to send stop all motors command.")
        return False


def set_thruster_speeds(speeds: list[int]) -> bool:
    """
    Sets speeds for multiple thrusters at once.
    Args:
        speeds (list[int]): A list of speeds, one for each thruster.
                            Should contain exactly 6 values for the 6 motors.
    Returns:
        bool: True if command sent successfully, False otherwise.
    """
    if len(speeds) != 6:
        logger.error(f"Expected 6 thruster speeds, got {len(speeds)}")
        return False

    # Use the new set_all_motor_speeds function
    return set_all_motor_speeds(speeds)


# Example Usage (if you were to test motors.py directly)
if __name__ == "__main__":
    # This assumes communication.py is in the same directory or Python path
    # For actual use, it would be `from .communication import SerialCommunicator`
    # from communication import SerialCommunicator # For standalone test

    communication.connect()

    if communication.is_connected():
        import time

        print("Setting motor 0 to speed 50")
        set_motor_speed(5, 50)
        print(get_motor_values())
        time.sleep(1)  # Keep motor running for a bit

        print("Setting motor 0 to speed -50 (reverse)")
        set_motor_speed(5, -50)
        time.sleep(1)

        # print("Setting thruster speeds [50, -50, 75, -75]")
        # set_thruster_speeds([50, -50, 75, -75])
        # time.sleep(2)

        print("Stopping all motors")
        stop_all_motors()

        communication.disconnect()
    else:
        print(f"Failed to connect to {ROV_SERIAL_PORT} for motor control.")
