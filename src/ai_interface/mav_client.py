from pymavlink.dialects.v20 import ardupilotmega as mavlink
from pymavlink import mavutil
from typing import cast, Any
import os

__all__ = [
    'mavlink',
    'client',
]


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
    device='192.168.55.255:14550',
    source_system=mavlink.MAV_TYPE_SUBMARINE,
    source_component=mavlink.MAV_COMP_ID_AUTOPILOT1,
    broadcast=True,
    input=False,
)
