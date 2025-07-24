from pymavlink.dialects.v20 import ardupilotmega as mavlink
from pymavlink import mavutil
from typing import cast, Any
from enum import Enum
from config import *
import os

__all__ = [
    'mavlink',
    'client',
]


class VehicleModes(Enum):
    # Mode not set by vehicle yet
    PRE_FLIGHT = -1
    # Manual angle with manual depth/throttle
    STABILIZE = 0
    # Manual body-frame angular rate with manual depth/throttle
    ACRO = 1
    # Manual angle with automatic depth/throttle
    ALT_HOLD = 2
    # Fully automatic waypoint control using mission commands
    AUTO = 3
    # Fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
    GUIDED = 4
    # Automatic circular flight with automatic throttle
    CIRCLE = 7
    # Automatically return to surface, pilot maintains horizontal control
    SURFACE = 9
    # Automatic position hold with manual override, with automatic throttle
    POSHOLD = 16
    # Pass-through input with no stabilization
    MANUAL = 19
    # Automatically detect motors orientation
    MOTOR_DETECT = 20
    # Manual angle with automatic depth/throttle (from rangefinder altitude)
    SURFTRAK = 21


class TypedMav(mavutil.mavudp):
    """
    A custom mavudp class that correctly types the .mav attribute.
    """

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        # Inform the type checker that .mav is an instance of our specific dialect.
        self.mav: mavlink.MAVLink = cast(mavlink.MAVLink, self.mav)


os.environ['MAVLINK20'] = '1'  # Ensure MAVLink 2.0 is used
mavutil.set_dialect(mavlink.DIALECT)

client = TypedMav(
    device=f'{IP_ADDR}',
    source_system=mavlink.MAV_TYPE_SUBMARINE,
    source_component=mavlink.MAV_COMP_ID_MISSIONPLANNER,
)
