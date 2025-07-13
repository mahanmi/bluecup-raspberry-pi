# robot_core/robot.py
import time
import logging
from hardware_interface.communication import SerialCommunicator
from hardware_interface.motors import MotorController
from hardware_interface.sensors import SensorInterface
from hardware_interface.camera import CameraControllerTTY
# from common.config import ROV_SERIAL_PORT, ROV_BAUD_RATE # Ideal
from .control import ROVControlSystem  # We'll define this next
from .telemetry import TelemetryCollector  # And this one

# Placeholder for config values if not using common/config.py yet
ROV_SERIAL_PORT = "/dev/ttyS10"  # Change to your actual port or config
ROV_BAUD_RATE = 9600      # Change to your actual baud or config

logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s - %(levelname)s - %(name)s - %(message)s')


class Robot:
    """
    The main class representing the ROV.
    Manages state, hardware interfaces, and high-level operations.
    """

    def __init__(self):
        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.info("Initializing ROV...")

        # --- State Variables ---
        self.is_armed = False
        # Target normalized (-1 to 1)
        self.current_target_movement = {
            "x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}
        self.current_thruster_outputs = []  # Actual values sent to thrusters
        self.telemetry_data = {
            "depth": 0.0,
            "temperature": 0.0,
            "imu": None,  # e.g., {'ax': 0, 'ay': 0, 'az': 0, ...}
            "battery_voltage": None,
            "camera_pan": 0,
            "camera_tilt": 0,
        }
        self.last_telemetry_update = time.time()

        # --- Hardware Interfaces ---
        try:
            # It's often better to pass the port and baud from a central config
            self.comm = SerialCommunicator(
                port=ROV_SERIAL_PORT, baudrate=ROV_BAUD_RATE)
            if not self.comm.is_connected():
                raise ConnectionError(
                    f"Failed to connect to ROV on {ROV_SERIAL_PORT}")

            self.motors = MotorController(self.comm)
            self.sensors = SensorInterface(self.comm)
            self.camera_tty_control = CameraControllerTTY(self.comm)
            # self.camera_video = cv2.VideoCapture(0) # For actual video, if applicable

        except ConnectionError as e:
            self.logger.error(f"HAL Initialization Error: {e}")
            # Depending on desired behavior, you might re-raise, or allow offline mode
            raise  # Re-raise to indicate critical failure
        except ImportError as e:
            self.logger.error(
                f"Import error, ensure pyserial is installed or hardware_interface modules are correct: {e}")
            raise
        except Exception as e:
            self.logger.error(
                f"Unexpected error during hardware initialization: {e}")
            raise

        # --- Core Logic Components ---
        self.control_system = ROVControlSystem()  # Add thruster config if needed
        self.telemetry_collector = TelemetryCollector(self.sensors)

        self.logger.info("ROV Initialized Successfully.")

    def arm(self) -> bool:
        """Arms the ROV, enabling motor control."""
        if not self.comm.is_connected():
            self.logger.warning("Cannot arm: ROV not connected.")
            return False
        self.is_armed = True
        self.logger.info("ROV Armed.")
        return True

    def disarm(self) -> bool:
        """Disarms the ROV, disabling motor control and stopping motors."""
        self.is_armed = False
        if hasattr(self, 'motors'):  # Check if motors object exists
            self.motors.stop_all_motors()
        self.logger.info("ROV Disarmed. Motors stopped.")
        return True

    def set_movement_targets(self, x: float, y: float, z: float, yaw: float):
        """
        Sets the desired movement targets for the ROV.
        Args:
            x (float): Forward/backward thrust (-1.0 to 1.0).
            y (float): Strafe left/right thrust (-1.0 to 1.0).
            z (float): Up/down thrust (-1.0 to 1.0).
            yaw (float): Rotational thrust (-1.0 to 1.0 for turn rate or torque).
        """
        self.current_target_movement = {"x": x, "y": y, "z": z, "yaw": yaw}
        self.logger.debug(
            f"Movement targets set: {self.current_target_movement}")

        if self.is_armed and self.comm.is_connected():
            thruster_outputs = self.control_system.calculate_thruster_outputs(
                x, y, z, yaw
            )
            self.current_thruster_outputs = thruster_outputs
            # Assumes MotorController has this
            self.motors.set_thruster_speeds(thruster_outputs)
        elif not self.is_armed:
            self.logger.warning("Cannot apply movement: ROV not armed.")
            self.motors.stop_all_motors()  # Ensure motors are stopped if disarmed
        else:
            self.logger.warning("Cannot apply movement: ROV not connected.")

    def update_telemetry(self, force_update=False):
        """
        Updates telemetry data from sensors.
        Args:
            force_update (bool): If True, updates regardless of time since last update.
        """
        if not self.comm.is_connected():
            self.logger.warning("Cannot update telemetry: ROV not connected.")
            # Potentially clear or mark telemetry as stale
            self.telemetry_data = {k: None for k in self.telemetry_data}
            self.telemetry_data["camera_pan"] = self.telemetry_data.get(
                "camera_pan", 0)  # Keep last known cam angles
            self.telemetry_data["camera_tilt"] = self.telemetry_data.get(
                "camera_tilt", 0)
            return

        # Limit update frequency unless forced
        now = time.time()
        # Update at most ~2Hz
        if not force_update and (now - self.last_telemetry_update < 0.5):
            return

        self.logger.debug("Updating telemetry...")
        updated_data = self.telemetry_collector.collect_all_data()
        self.telemetry_data.update(updated_data)  # Merge new data
        self.last_telemetry_update = now
        self.logger.debug(f"Telemetry updated: {self.telemetry_data}")

    def get_current_state(self) -> dict:
        """
        Returns a comprehensive dictionary of the robot's current state.
        """
        return {
            "is_armed": self.is_armed,
            "is_connected": self.comm.is_connected() if hasattr(self, 'comm') else False,
            "target_movement": self.current_target_movement,
            "thruster_outputs": self.current_thruster_outputs,
            "telemetry": self.telemetry_data,
            "timestamp": time.time()
        }

    def set_camera_pan_tilt(self, pan_angle: int | None = None, tilt_angle: int | None = None) -> bool:
        """Controls the camera pan and/or tilt via TTY commands."""
        success = True
        if not self.comm.is_connected():
            self.logger.warning("Cannot control camera: ROV not connected.")
            return False

        if pan_angle is not None:
            if self.camera_tty_control.pan(pan_angle):
                self.telemetry_data["camera_pan"] = pan_angle
            else:
                self.logger.warning(f"Failed to set camera pan to {pan_angle}")
                success = False

        if tilt_angle is not None:
            if self.camera_tty_control.tilt(tilt_angle):
                self.telemetry_data["camera_tilt"] = tilt_angle
            else:
                self.logger.warning(
                    f"Failed to set camera tilt to {tilt_angle}")
                success = False
        return success

    def get_camera_feed_url(self) -> str | None:
        """
        Placeholder for getting a camera feed URL.
        Actual video streaming typically doesn't go through the main TTY.
        This might return an RTSP URL or similar if the camera has an IP interface.
        """
        # If your camera is an IP camera, you might store its URL here or in config.
        # For a USB camera with OpenCV, the GUI would handle the frame capture directly.
        self.logger.info(
            "get_camera_feed_url() called. Note: TTY interface is for control, not primary video.")
        return "http://<ROV_CAMERA_IP_ADDRESS_OR_STREAM_ENDPOINT>"  # Example

    def shutdown(self):
        """Safely shuts down the ROV."""
        self.logger.info("ROV Shutting down...")
        self.disarm()  # This also stops motors
        if hasattr(self, 'comm') and self.comm.is_connected():
            self.comm.disconnect()
        # if hasattr(self, 'camera_video') and self.camera_video.isOpened():
        #     self.camera_video.release()
        self.logger.info("ROV Shutdown complete.")


# Example usage (for testing this module in isolation)
if __name__ == "__main__":
    try:
        my_rov = Robot()

        if my_rov.comm.is_connected():
            print("ROV Connected. Arming...")
            my_rov.arm()
            print(f"ROV Armed: {my_rov.is_armed}")

            print("\nUpdating telemetry...")
            my_rov.update_telemetry(force_update=True)
            print(f"Initial State: {my_rov.get_current_state()}")

            print("\nSetting movement: Forward 0.5")
            my_rov.set_movement_targets(x=0.5, y=0.0, z=0.0, yaw=0.0)
            # Allow time for command to be processed notionally
            time.sleep(0.1)
            print(f"State after move command: {my_rov.get_current_state()}")

            print("\nSetting camera pan to 30, tilt to -10")
            my_rov.set_camera_pan_tilt(pan_angle=30, tilt_angle=-10)
            print(f"State after camera command: {my_rov.get_current_state()}")

            time.sleep(1)
            print("\nDisarming ROV...")
            my_rov.disarm()
            print(f"ROV Armed: {my_rov.is_armed}")

            my_rov.shutdown()
        else:
            print("Could not run example: ROV not connected (this is expected if no virtual serial port is set up).")

    except ConnectionError as e:
        print(
            f"Connection Error: {e}. Ensure a serial port is available or mock hardware_interface.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

