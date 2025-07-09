"""
ROV Controller AI Interface Library
=================================

This library provides a clean interface for AI systems to control and monitor the ROV.

Example usage:
-------------
```python
from rov_controller import ROVController

# Initialize the controller
rov = ROVController()

# Control the ROV
rov.move(x=0.5, y=0.0, z=0.0, yaw=0.0)
rov.set_camera_angle(pan=30, tilt=45)

# Get state information
depth = rov.state.depth
orientation = rov.state.orientation

# Register event handlers
@rov.on_sensor_update
def handle_sensor_update(data):
    print(f"New depth: {data.depth}m")
"""

from .controller import ROVController
from .types import (
    SensorData,
    CameraFrame,
    RobotState,
    Orientation,
    Position,
    MotorState,
    ThrusterConfiguration
)
from .events import EventType

__version__ = "0.1.0"
__all__ = [
    "ROVController",
    "SensorData",
    "CameraFrame",
    "RobotState",
    "Orientation",
    "Position",
    "MotorState",
    "ThrusterConfiguration",
    "EventType"
] 