import sys
import os

# Smart import strategy - try src. prefix first, then without
try:
    from src.config import *
except ImportError:
    # Running from src directory, use relative imports
    from config import *

from . import ardupilotmega as mavlink
from .connection import *
from .utils import *


__all__ = [
    'VehicleModes',
    'ButtonFunctions',
    'mavlink',
    'client',
]


client = AsyncUdp(
    device=f'{IP_ADDR}',
    source_system=mavlink.MAV_TYPE_SUBMARINE,
    source_component=mavlink.MAV_COMP_ID_AUTOPILOT1,
    broadcast=True,
    input=False,
)
