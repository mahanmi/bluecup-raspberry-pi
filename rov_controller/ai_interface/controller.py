"""Main controller class for the ROV Controller AI Interface."""

import asyncio
from typing import Optional, Callable, Dict, Any
import logging
from functools import wraps

from .types import (
    Position, 
    Orientation, 
    RobotState, 
    SensorData,
    CameraFrame,
    ThrusterConfiguration
)
from .events import EventEmitter, EventType


logger = logging.getLogger(__name__)


def require_armed(func):
    """Decorator to ensure ROV is armed before executing command."""
    @wraps(func)
    def wrapper(self, *args, **kwargs):
        if not self.is_armed:
            raise RuntimeError("ROV must be armed before executing commands")
        return func(self, *args, **kwargs)
    return wrapper


def require_not_emergency(func):
    """Decorator to prevent commands during emergency state."""
    @wraps(func)
    def wrapper(self, *args, **kwargs):
        if self.is_emergency:
            raise RuntimeError("Cannot execute commands while in emergency state")
        return func(self, *args, **kwargs)
    return wrapper


class ROVController:
    """Main interface for controlling the ROV.
    
    This class provides high-level control methods and state access for the ROV.
    It handles command validation, state management, and event distribution.
    """

    def __init__(self):
        """Initialize the ROV controller."""
        self._events = EventEmitter()
        self._state = RobotState(
            timestamp=0.0,
            position=Position(0.0, 0.0, 0.0),
            orientation=Orientation(0.0, 0.0, 0.0),
            thrusters=ThrusterConfiguration(None, None, None, None, None, None),
            sensors=None,
            is_armed=False,
            is_emergency=False,
            current_mode="STANDBY",
            battery_percentage=100.0
        )
        
        # Initialize connection to robot core
        self._initialize_core_connection()

    def _initialize_core_connection(self):
        """Initialize connection to the robot core.
        
        This method should be implemented to establish communication
        with the robot_core module.
        """
        # TODO: Implement connection to robot core
        pass

    @property
    def state(self) -> RobotState:
        """Get the current state of the ROV."""
        return self._state

    @property
    def is_armed(self) -> bool:
        """Check if the ROV is armed."""
        return self._state.is_armed

    @property
    def is_emergency(self) -> bool:
        """Check if the ROV is in emergency state."""
        return self._state.is_emergency

    def arm(self) -> None:
        """Arm the ROV, enabling motor control."""
        if self.is_armed:
            logger.warning("ROV is already armed")
            return
        
        # TODO: Implement arming sequence
        self._state.is_armed = True
        self._events.emit_sync(EventType.STATE_CHANGE, self._state)

    def disarm(self) -> None:
        """Disarm the ROV, disabling motor control."""
        if not self.is_armed:
            logger.warning("ROV is already disarmed")
            return
        
        # TODO: Implement disarming sequence
        self._state.is_armed = False
        self._events.emit_sync(EventType.STATE_CHANGE, self._state)

    def emergency_stop(self) -> None:
        """Immediately stop all motors and enter emergency state."""
        self._state.is_emergency = True
        # TODO: Implement emergency stop sequence
        self._events.emit_sync(EventType.EMERGENCY, self._state)

    def clear_emergency(self) -> None:
        """Clear emergency state if safe to do so."""
        if not self.is_emergency:
            logger.warning("Not in emergency state")
            return
        
        # TODO: Implement emergency clear sequence
        self._state.is_emergency = False
        self._events.emit_sync(EventType.STATE_CHANGE, self._state)

    @require_armed
    @require_not_emergency
    def move(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, yaw: float = 0.0) -> None:
        """Move the ROV with the specified velocities.
        
        Args:
            x: Forward/backward velocity (-1.0 to 1.0)
            y: Left/right velocity (-1.0 to 1.0)
            z: Up/down velocity (-1.0 to 1.0)
            yaw: Rotation velocity (-1.0 to 1.0)
        """
        # Validate inputs
        for val, name in [(x, 'x'), (y, 'y'), (z, 'z'), (yaw, 'yaw')]:
            if not -1.0 <= val <= 1.0:
                raise ValueError(f"{name} velocity must be between -1.0 and 1.0")

        # TODO: Implement movement command
        pass

    @require_armed
    @require_not_emergency
    def set_camera_angle(self, pan: float = 0.0, tilt: float = 0.0) -> None:
        """Set the camera angles.
        
        Args:
            pan: Pan angle in degrees (-180 to 180)
            tilt: Tilt angle in degrees (-90 to 90)
        """
        if not -180.0 <= pan <= 180.0:
            raise ValueError("Pan angle must be between -180 and 180 degrees")
        if not -90.0 <= tilt <= 90.0:
            raise ValueError("Tilt angle must be between -90 and 90 degrees")

        # TODO: Implement camera control
        pass

    def on_sensor_update(self, handler: Callable[[SensorData], None], async_handler: bool = False) -> None:
        """Register a handler for sensor updates.
        
        Args:
            handler: Callback function that takes a SensorData argument
            async_handler: Whether the handler is an async function
        """
        self._events.on(EventType.SENSOR_UPDATE, handler, async_handler)

    def on_camera_frame(self, handler: Callable[[CameraFrame], None], async_handler: bool = False) -> None:
        """Register a handler for camera frames.
        
        Args:
            handler: Callback function that takes a CameraFrame argument
            async_handler: Whether the handler is an async function
        """
        self._events.on(EventType.CAMERA_FRAME, handler, async_handler)

    def on_state_change(self, handler: Callable[[RobotState], None], async_handler: bool = False) -> None:
        """Register a handler for state changes.
        
        Args:
            handler: Callback function that takes a RobotState argument
            async_handler: Whether the handler is an async function
        """
        self._events.on(EventType.STATE_CHANGE, handler, async_handler)

    def on_emergency(self, handler: Callable[[RobotState], None], async_handler: bool = False) -> None:
        """Register a handler for emergency events.
        
        Args:
            handler: Callback function that takes a RobotState argument
            async_handler: Whether the handler is an async function
        """
        self._events.on(EventType.EMERGENCY, handler, async_handler) 