import sys
import os
# Add project root to Python path for direct execution
if __name__ == "__main__":
    project_root = os.path.abspath(os.path.join(
        os.path.dirname(__file__), '..', '..'))
    if project_root not in sys.path:
        sys.path.insert(0, project_root)

# Smart import strategy for log
try:
    import src.log as log
except ImportError:
    # Running from src directory
    import log

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

logger = log.getLogger(__name__)


def set_servo_angle(servo_id, angle):
    if not (0 <= angle <= 180):
        logger.error("Angle must be between 0 and 180 degrees.")
        return

    # **IMPORTANT**: Replace "M" and the command format with your hardware's protocol.
    # Example command: "R:<servo_id>:<angle>"
    # e.g., "S:0:150" sets servo 0 to angle
    # e.g., "S:1:-100" sets servo 1 to angle -100 (if servo support it)
    command = f"R:{servo_id}:{angle}"

    if communication.send_command(command):
        logger.info(f"Servo {servo_id} set to {angle} degrees.")
    else:
        logger.error(f"Failed to send command to servo {servo_id}")


def camera_tilt(angle):
    set_servo_angle(2, angle)  # Assuming servo 2 is for camera tilt


def camera_pan(angle):
    set_servo_angle(1, angle)  # Assuming servo 1 is for camera pan


if __name__ == '__main__':
    # Example usage:

    if communication.connect():
        try:
            set_servo_angle(0, 90)  # Set servo 1 to 90 degrees
            camera_pan(45)  # Set pan to 45 degrees
            camera_tilt(0)   # Set tilt 1 to 0 degrees
        finally:
            communication.disconnect()
    else:
        logger.error("Failed to connect to the serial port.")
