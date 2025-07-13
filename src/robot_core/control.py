# robot_core/control.py
import logging
import math

# --- Thruster Configuration ---
# This is a conceptual example. You'll need to define this based on your ROV.
# For a typical 6-DOF ROV, you might have 6 or 8 thrusters.
# This example assumes a simple setup:
# - 2 thrusters for forward/backward (surge)
# - 2 thrusters for up/down (heave)
# - 2 thrusters for yaw (rotation)
# - Strafing (sway) might be achieved by differential thrust or dedicated thrusters.

# For simplicity, let's assume thruster outputs are scaled from -255 to 255.
# This would be defined by your MotorController's expected input.
THRUSTER_MAX_OUTPUT = 255


class TestClass(object):
    def __init__(self,lon:float,lat:float,alt:float):
        self.lon=lon
        self.lat=lat
        self.alt=alt
    def move(self,x,y,z):
        self.lon+=z
        self.lat+=y
        self.alt+=x


test_class = TestClass(54.358386,31.839036,5)
