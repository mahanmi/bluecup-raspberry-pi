from pymavlink.dialects.v20 import ardupilotmega as mavlink


def command_int_handler(msg: mavlink.MAVLink_command_int_message):
    pass


def command_long_handler(msg: mavlink.MAVLink_command_long_message):
    pass


def command_ack_handler(msg: mavlink.MAVLink_command_ack_message):
    pass
