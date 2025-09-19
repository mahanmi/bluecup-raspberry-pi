import logging

IP_ADDR = "255.255.255.255:14551"
CAMERA_ID_OR_PIPELINE = "0"  # Default camera ID for OpenCV
LOG_REPEAT_INTERVAL = 2  # Interval for repeating log messages

# Logging Configuration
LOG_LEVEL = logging.INFO  # Set to logging.DEBUG for more verbose output

# Serial Communication Configuration
ROV_SERIAL_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
ROV_BAUD_RATE = 115200
ROV_TIMEOUT = 1.0
EOL = "\r\n"  # End of Line character for serial communication
