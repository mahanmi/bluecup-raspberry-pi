from typing import Any
import logging
# Assuming this is where get_temp etc. are
from hardware_interface.sensors import SensorInterface


class TelemetryCollector:
    def __init__(self, sensor_interface: SensorInterface):
        self.logger = logging.getLogger(self.__class__.__name__)
        self.sensors = sensor_interface
        self.logger.info("TelemetryCollector initialized.")

    def collect_all_data(self) -> dict:
        """
        Collects data from all available sensors.
        Returns:
            dict: A dictionary containing all collected telemetry data.
                  Keys are sensor names (e.g., "temperature", "depth"),
                  values are the readings or None if unavailable.
        """
        if not self.sensors or not self.sensors.comm.is_connected():
            self.logger.warning(
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
            data["temperature"] = self.sensors.get_temperature()
        except Exception as e:
            self.logger.error(f"Error getting temperature: {e}")
            data["temperature"] = None

        try:
            data["depth"] = self.sensors.get_depth()
        except Exception as e:
            self.logger.error(f"Error getting depth: {e}")
            data["depth"] = None

        try:
            data["imu"] = self.sensors.get_imu_data()  # Returns a dict or None
        except Exception as e:
            self.logger.error(f"Error getting IMU data: {e}")
            data["imu"] = None

        try:
            data["battery_voltage"] = self.sensors.get_battery_voltage()
        except Exception as e:
            self.logger.error(f"Error getting battery voltage: {e}")
            data["battery_voltage"] = None

        # You can add more specific sensor readings here
        # e.g., data["leak_detected"] = self.sensors.get_leak_status()

        self.logger.debug(f"Telemetry data collected: {data}")
        return data


# Example Usage (requires a mock or real SensorInterface and SerialCommunicator)
if __name__ == "__main__":
    from hardware_interface.communication import SerialCommunicator

    # --- Mock SensorInterface for testing ---
    class MockSensorInterface:
        def __init__(self, comm):
            self.comm = comm  # Needs comm to check is_connected
            self.temp_val = 25.5
            self.depth_val = 10.1
            self.imu_data = {"ax": 0.1, "ay": -0.1, "az": 9.8}
            self.batt_val = 12.3

        def get_temperature(self):
            if not self.comm.is_connected():
                return None
            self.temp_val += 0.1  # Simulate changing value
            return self.temp_val if self.temp_val < 30 else 25.5

        def get_depth(self):
            if not self.comm.is_connected():
                return None
            self.depth_val -= 0.05
            return self.depth_val if self.depth_val > 5 else 10.1

        def get_imu_data(self):
            if not self.comm.is_connected():
                return None
            self.imu_data["ax"] += 0.01
            return self.imu_data

        def get_battery_voltage(self):
            if not self.comm.is_connected():
                return None
            self.batt_val -= 0.01
            return self.batt_val if self.batt_val > 11.0 else 12.3

    # --- Test Setup ---
    # Replace with your ROV's actual serial port and baud rate for a real test
    ROV_SERIAL_PORT = "/dev/ttyS10"  # Example for virtual serial port
    ROV_BAUD_RATE = 9600

    print("Setting up mock telemetry test...")
    mock_comm = SerialCommunicator(ROV_SERIAL_PORT, ROV_BAUD_RATE)

    if mock_comm.is_connected():
        print(f"Mock communicator connected to {ROV_SERIAL_PORT}")
        mock_sensors = MockSensorInterface(mock_comm)
        collector = TelemetryCollector(mock_sensors)  # type: ignore[arg-type]

        print("\nCollecting telemetry data (attempt 1):")
        telemetry = collector.collect_all_data()
        for key, value in telemetry.items():
            print(f"  {key}: {value}")

        print("\nCollecting telemetry data (attempt 2):")
        telemetry = collector.collect_all_data()
        for key, value in telemetry.items():
            print(f"  {key}: {value}")

        mock_comm.disconnect()
    else:
        print(
            f"Failed to connect mock communicator to {ROV_SERIAL_PORT}. Ensure a virtual serial port is running.")

    print("\nTesting with disconnected communicator:")
    # Simulate disconnected communicator for the disconnected case

    class DisconnectedComm:
        def is_connected(self): return False

    disconnected_sensors = MockSensorInterface(DisconnectedComm())
    collector_disconnected = TelemetryCollector(
        disconnected_sensors)  # type: ignore[arg-type]
    telemetry_offline = collector_disconnected.collect_all_data()
    print("Telemetry when disconnected:")
    for key, value in telemetry_offline.items():
        print(f"  {key}: {value}")
