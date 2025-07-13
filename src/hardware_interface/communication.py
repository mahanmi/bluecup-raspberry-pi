# hardware_interface/communication.py
import serial
import time
import logging

# Configure logging
logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s - %(levelname)s - %(message)s')


class SerialCommunicator:
    """
    Handles serial communication with a TTY device.
    """

    def __init__(self, port, baudrate=115200, timeout=1, eol='\n'):
        """
        Initializes the serial connection.
        Args:
            port (str): The serial port (e.g., '/dev/ttyUSB0' or 'COM3').
            baudrate (int): The baud rate for the connection.
            timeout (float): Read timeout in seconds.
            eol (str): End-of-line character for commands and responses.
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.eol = eol.encode('utf-8')  # Encode EOL to bytes
        self.serial_connection = None
        self.connect()

    def connect(self):
        """
        Establishes the serial connection.
        """
        try:
            self.serial_connection = serial.Serial(
                self.port, self.baudrate, timeout=self.timeout
            )
            # Allow some time for the connection to establish
            time.sleep(2)
            if self.serial_connection.is_open:
                logging.info(
                    f"Successfully connected to {self.port} at {self.baudrate} baud.")
            else:
                logging.error(f"Failed to open serial port {self.port}.")
        except serial.SerialException as e:
            logging.error(f"Error connecting to serial port {self.port}: {e}")
            self.serial_connection = None  # Ensure it's None if connection failed

    def disconnect(self):
        """
        Closes the serial connection.
        """
        if self.serial_connection and self.serial_connection.is_open:
            try:
                self.serial_connection.close()
                logging.info(f"Disconnected from {self.port}.")
            except Exception as e:
                logging.error(f"Error disconnecting from {self.port}: {e}")
        self.serial_connection = None

    def send_command(self, command_str):
        """
        Sends a command string over the serial port.
        Args:
            command_str (str): The command to send.
        Returns:
            bool: True if the command was sent successfully, False otherwise.
        """
        if not self.is_connected():
            logging.warning(
                f"Not connected to {self.port}. Cannot send command: {command_str}")
            # Optionally, try to reconnect:
            # logging.info("Attempting to reconnect...")
            # self.connect()
            # if not self.is_connected():
            #     return False
            return False

        try:
            full_command = command_str.encode('utf-8') + self.eol
            self.serial_connection.write(full_command)
            logging.debug(
                f"Sent to {self.port}: {full_command.decode('utf-8').strip()}")
            return True
        except serial.SerialTimeoutException:
            logging.error(
                f"Timeout writing to {self.port} for command: {command_str}")
        except serial.SerialException as e:
            logging.error(
                f"Serial error writing command '{command_str}' to {self.port}: {e}")
        except Exception as e:
            logging.error(
                f"Unexpected error writing command '{command_str}' to {self.port}: {e}")
        return False

    def read_line(self, timeout_override=None):
        """
        Reads a line of text from the serial port until EOL or timeout.
        Args:
            timeout_override (float, optional): Specific timeout for this read operation.
        Returns:
            str or None: The line read (without EOL), or None if timeout or error.
        """
        if not self.is_connected():
            logging.warning(f"Not connected to {self.port}. Cannot read line.")
            return None

        original_timeout = self.serial_connection.timeout
        if timeout_override is not None:
            self.serial_connection.timeout = timeout_override

        try:
            line_bytes = self.serial_connection.readline()
            if line_bytes:
                decoded_line = line_bytes.decode(
                    'utf-8', errors='ignore').strip()
                logging.debug(f"Read from {self.port}: {decoded_line}")
                return decoded_line
            else:
                # This can happen on timeout if no data is received
                logging.debug(f"Read timeout or no data from {self.port}.")
                return None
        except serial.SerialException as e:
            logging.error(f"Serial error reading line from {self.port}: {e}")
        except Exception as e:
            logging.error(
                f"Unexpected error reading line from {self.port}: {e}")
        finally:
            if timeout_override is not None:
                self.serial_connection.timeout = original_timeout  # Restore original
        return None

    def read_bytes(self, num_bytes):
        """
        Reads a specific number of bytes from the serial port.
        Args:
            num_bytes (int): The number of bytes to read.
        Returns:
            bytes or None: The bytes read, or None if timeout or error.
        """
        if not self.is_connected():
            logging.warning(
                f"Not connected to {self.port}. Cannot read bytes.")
            return None
        try:
            data_bytes = self.serial_connection.read(num_bytes)
            if data_bytes:
                logging.debug(
                    f"Read {len(data_bytes)} bytes from {self.port}.")
                return data_bytes
            else:
                logging.debug(
                    f"Read timeout or no data (expected {num_bytes} bytes) from {self.port}.")
                return None
        except serial.SerialException as e:
            logging.error(
                f"Serial error reading {num_bytes} bytes from {self.port}: {e}")
        except Exception as e:
            logging.error(
                f"Unexpected error reading {num_bytes} bytes from {self.port}: {e}")
        return None

    def flush_input(self):
        """ Clears input buffer. """
        if self.is_connected():
            self.serial_connection.reset_input_buffer()
            logging.debug(f"Input buffer flushed for {self.port}")

    def flush_output(self):
        """ Clears output buffer. """
        if self.is_connected():
            self.serial_connection.reset_output_buffer()
            logging.debug(f"Output buffer flushed for {self.port}")

    def is_connected(self):
        """
        Checks if the serial port is connected.
        """
        return self.serial_connection is not None and self.serial_connection.is_open


# Example usage (for testing this module directly)
if __name__ == "__main__":
    # Replace with your ROV's actual serial port and baud rate
    # On Linux, it might be /dev/ttyUSB0, /dev/ttyACM0, etc.
    # On Windows, it might be COM3, COM4, etc.
    # Example, use a virtual serial port for testing if no hardware
    ROV_PORT = "/dev/ttyS10"
    # To create virtual serial ports for testing on Linux:
    # sudo apt-get install socat
    # socat -d -d pty,raw,echo=0 pty,raw,echo=0
    # This will output two port names like /dev/pts/X and /dev/pts/Y

    communicator = SerialCommunicator(ROV_PORT, baudrate=9600)

    if communicator.is_connected():
        # Example: Send a command and try to read a response
        # THIS IS A PLACEHOLDER - use actual commands for your hardware
        test_command = "STATUS?"
        if communicator.send_command(test_command):
            print(f"Sent command: {test_command}")
            response = communicator.read_line()
            if response:
                print(f"Received response: {response}")
            else:
                print("No response or timeout.")
        else:
            print(f"Failed to send command: {test_command}")

        communicator.disconnect()
    else:
        print(f"Could not connect to {ROV_PORT}")
