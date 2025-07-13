# hardware_interface/motors.py
from .communication import SerialCommunicator
import logging

# This would ideally come from a config file (e.g., common/config.py)
# SERIAL_PORT_MOTORS = "/dev/ttyUSB0" # Or your specific port
# BAUD_RATE_MOTORS = 115200


class MotorController:
    def __init__(self, communicator: SerialCommunicator):
        """
        Initializes the MotorController.
        Args:
            communicator (SerialCommunicator): An instance of SerialCommunicator.
        """
        self.comm = communicator
        # self.num_motors = 4 # Example, configure as needed

    def set_motor_speed(self, motor_id: int, speed: int) -> bool:
        """
        Sets the speed of a specific motor.
        Args:
            motor_id (int): The ID of the motor (e.g., 0 for front-left, 1 for front-right).
            speed (int): The speed, e.g., -255 to 255 (0 is stop).
                         The range and meaning depend on your hardware.
        Returns:
            bool: True if the command was likely sent successfully, False otherwise.
        """
        # **IMPORTANT**: Replace "M" and the command format with your hardware's protocol.
        # Example command: "M:<motor_id>:<speed>\n"
        # e.g., "M:0:150" sets motor 0 to speed 150
        # e.g., "M:1:-100" sets motor 1 to speed -100 (reverse)
        command = f"M:{motor_id}:{speed}"

        if self.comm.send_command(command):
            logging.info(f"Motor {motor_id} speed set to {speed}")
            # Optional: Wait for an acknowledgment if your hardware sends one
            # ack = self.comm.read_line(timeout_override=0.5)
            # if ack and "OK" in ack: # Or whatever your hardware returns
            #     return True
            # else:
            #     logging.warning(f"No/invalid ACK for motor command: {command} -> {ack}")
            #     return False
            return True  # Assuming command sent is enough for now
        else:
            logging.error(
                f"Failed to send command to set motor {motor_id} speed.")
            return False

    def stop_all_motors(self) -> bool:
        """
        Stops all motors.
        This might involve sending individual stop commands or a global stop command.
        """
        # **IMPORTANT**: Replace with your hardware's global stop command or loop
        # Example: Assuming a global stop command "M:ALL:0"
        command = "M:ALL:0"
        # Or, if you need to stop them individually:
        # success = True
        # for i in range(self.num_motors):
        #     if not self.set_motor_speed(i, 0):
        #         success = False
        # return success

        if self.comm.send_command(command):
            logging.info("All motors commanded to stop.")
            return True
        else:
            logging.error("Failed to send stop all motors command.")
            return False

    def set_thruster_speeds(self, speeds: list[int]) -> bool:
        """
        Sets speeds for multiple thrusters at once.
        Args:
            speeds (list[int]): A list of speeds, one for each thruster.
                                The order should match your ROV's thruster configuration.
        Returns:
            bool: True if all commands were likely sent successfully, False otherwise.
        """
        # Example: Sending commands for thrusters 0, 1, 2, 3
        # This assumes your `set_motor_speed` handles individual thrusters
        # and your `motor_id` maps to thruster indices.
        all_successful = True
        for i, speed_value in enumerate(speeds):
            if not self.set_motor_speed(motor_id=i, speed=speed_value):
                all_successful = False
                logging.warning(f"Failed to set speed for thruster {i}")

        if all_successful:
            logging.info(f"Thruster speeds set: {speeds}")
        else:
            logging.error(
                f"One or more thruster speed commands failed for speeds: {speeds}")
        return all_successful


# Example Usage (if you were to test motors.py directly)
if __name__ == "__main__":
    # This assumes communication.py is in the same directory or Python path
    # For actual use, it would be `from .communication import SerialCommunicator`
    # from communication import SerialCommunicator # For standalone test

    # --- Configuration ---
    # Replace with your ROV's actual serial port and baud rate
    ROV_SERIAL_PORT = "/dev/ttyS10"  # Use a virtual serial port pair for testing
    ROV_BAUD_RATE = 9600
    # --- End Configuration ---

    rov_comm = SerialCommunicator(ROV_SERIAL_PORT, ROV_BAUD_RATE)

    if rov_comm.is_connected():
        import time

        motor_ctrl = MotorController(rov_comm)

        print("Setting motor 0 to speed 100")
        motor_ctrl.set_motor_speed(0, 100)
        time.sleep(1)  # Keep motor running for a bit

        print("Setting motor 0 to speed -100 (reverse)")
        motor_ctrl.set_motor_speed(0, -100)
        time.sleep(1)

        print("Setting thruster speeds [50, -50, 75, -75]")
        motor_ctrl.set_thruster_speeds([50, -50, 75, -75])
        time.sleep(2)

        print("Stopping all motors")
        motor_ctrl.stop_all_motors()

        rov_comm.disconnect()
    else:
        print(f"Failed to connect to {ROV_SERIAL_PORT} for motor control.")
