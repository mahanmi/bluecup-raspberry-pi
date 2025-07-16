from typing import Any
import logging
# Assuming this is where get_temp etc. are
from hardware_interface import sensors


logger = logging.getLogger(__name__)
logger.info("TelemetryCollector initialized.")


def collect_all_data() -> dict:
    """
    Collects data from all available sensors.
    Returns:
        dict: A dictionary containing all collected telemetry data.
                Keys are sensor names (e.g., "temperature", "depth"),
                values are the readings or None if unavailable.
    """
    if not sensors or not sensors.communication.is_connected():
        logger.warning(
            "Cannot collect telemetry: Sensor interface not available or not connected.")
        return {
            "temperature": None,
            "depth": None,
            "imu": None,
            "battery_voltage": None,
            # Add other expected telemetry fields with None
        }

    data: dict[str, Any] = {}
    try:
        data["temperature"] = sensors.get_temperature()
    except Exception as e:
        logger.error(f"Error getting temperature: {e}")
        data["temperature"] = None

    try:
        data["depth"] = sensors.get_depth()
    except Exception as e:
        logger.error(f"Error getting depth: {e}")
        data["depth"] = None

    try:
        data["imu"] = sensors.get_imu_data()  # Returns a dict or None
    except Exception as e:
        logger.error(f"Error getting IMU data: {e}")
        data["imu"] = None

    try:
        data["battery_voltage"] = sensors.get_battery_voltage()
    except Exception as e:
        logger.error(f"Error getting battery voltage: {e}")
        data["battery_voltage"] = None

    # You can add more specific sensor readings here
    # e.g., data["leak_detected"] = sensors.get_leak_status()

    logger.debug(f"Telemetry data collected: {data}")
    return data


# Example Usage (requires a mock or real SensorInterface and SerialCommunicator)
if __name__ == "__main__":
    from hardware_interface import communication
    from config import ROV_SERIAL_PORT

    if communication.is_connected():
        print(f"Mock communicator connected to {ROV_SERIAL_PORT}")

        print("\nCollecting telemetry data (attempt 1):")
        telemetry = collect_all_data()
        for key, value in telemetry.items():
            print(f"  {key}: {value}")

        print("\nCollecting telemetry data (attempt 2):")
        telemetry = collect_all_data()
        for key, value in telemetry.items():
            print(f"  {key}: {value}")

        communication.disconnect()
    else:
        print(
            f"Failed to connect communicator to {ROV_SERIAL_PORT}. Ensure serial port is running.")
