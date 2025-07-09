"""Data types and structures for the ROV Controller AI Interface."""

from dataclasses import dataclass
from typing import List, Dict, Optional
from enum import Enum
import numpy as np


@dataclass
class Position:
    """3D position in meters relative to starting position."""
    x: float  # Forward/backward (positive = forward)
    y: float  # Left/right (positive = right)
    z: float  # Up/down (positive = up)


@dataclass
class Orientation:
    """Orientation in degrees."""
    roll: float   # Roll angle in degrees
    pitch: float  # Pitch angle in degrees
    yaw: float    # Yaw angle in degrees


@dataclass
class MotorState:
    """Current state of a motor/thruster."""
    speed: float  # Current speed (-1.0 to 1.0)
    current: float  # Current draw in amps
    temperature: float  # Temperature in Celsius


@dataclass
class ThrusterConfiguration:
    """Configuration of all thrusters."""
    front_left: MotorState
    front_right: MotorState
    back_left: MotorState
    back_right: MotorState
    vertical_front: MotorState
    vertical_back: MotorState


@dataclass
class SensorData:
    """Data from all sensors."""
    timestamp: float  # Unix timestamp
    depth: float  # Depth in meters
    temperature: float  # Water temperature in Celsius
    pressure: float  # Pressure in bar
    humidity: float  # Internal humidity percentage
    voltage: float  # Main battery voltage
    current: float  # Total current draw in amps


@dataclass
class CameraFrame:
    """Camera frame data."""
    timestamp: float  # Unix timestamp
    frame: np.ndarray  # OpenCV/numpy image array
    width: int
    height: int
    camera_id: str  # Identifier for multi-camera setups


@dataclass
class RobotState:
    """Complete state of the ROV."""
    timestamp: float
    position: Position
    orientation: Orientation
    thrusters: ThrusterConfiguration
    sensors: SensorData
    is_armed: bool
    is_emergency: bool
    current_mode: str
    battery_percentage: float
    
    def to_dict(self) -> Dict:
        """Convert state to dictionary format."""
        return {
            "timestamp": self.timestamp,
            "position": vars(self.position),
            "orientation": vars(self.orientation),
            "thrusters": {k: vars(v) for k, v in vars(self.thrusters).items()},
            "sensors": vars(self.sensors),
            "is_armed": self.is_armed,
            "is_emergency": self.is_emergency,
            "current_mode": self.current_mode,
            "battery_percentage": self.battery_percentage
        }

    @classmethod
    def from_dict(cls, data: Dict) -> 'RobotState':
        """Create state from dictionary format."""
        return cls(
            timestamp=data["timestamp"],
            position=Position(**data["position"]),
            orientation=Orientation(**data["orientation"]),
            thrusters=ThrusterConfiguration(**{
                k: MotorState(**v) for k, v in data["thrusters"].items()
            }),
            sensors=SensorData(**data["sensors"]),
            is_armed=data["is_armed"],
            is_emergency=data["is_emergency"],
            current_mode=data["current_mode"],
            battery_percentage=data["battery_percentage"]
        ) 