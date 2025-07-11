from pymavlink.dialects.v20 import ardupilotmega as mavlink


def set_camera_zoom(client: mavlink.MAVLink, msg: mavlink.MAVLink_command_int_message):
    pass


def set_camera_focus(client: mavlink.MAVLink, msg: mavlink.MAVLink_command_int_message):
    pass


def component_arm_disarm(client: mavlink.MAVLink, msg: mavlink.MAVLink_command_int_message):
    pass


def nav_takeoff(client: mavlink.MAVLink, msg: mavlink.MAVLink_command_int_message):
    pass


def do_set_home(client: mavlink.MAVLink, msg: mavlink.MAVLink_command_int_message):
    pass


def do_reposition(client: mavlink.MAVLink, msg: mavlink.MAVLink_command_int_message):
    pass


def request_message(client: mavlink.MAVLink, msg: mavlink.MAVLink_command_int_message):
    pass


def get_home_position(client: mavlink.MAVLink, msg: mavlink.MAVLink_command_int_message):
    pass


def mission_start(client: mavlink.MAVLink, msg: mavlink.MAVLink_command_int_message):
    pass


def nav_waypoint(client: mavlink.MAVLink, msg: mavlink.MAVLink_command_int_message):
    pass
