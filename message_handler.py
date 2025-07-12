from pymavlink.dialects.v20 import ardupilotmega as mavlink


def command_int_handler(client: mavlink.MAVLink, msg: mavlink.MAVLink_command_int_message):
    pass




def command_long_handler(client: mavlink.MAVLink, msg: mavlink.MAVLink_command_long_message):
    pass


def command_ack_handler(client: mavlink.MAVLink, msg: mavlink.MAVLink_command_ack_message):
    pass


def manual_control_handler(client: mavlink.MAVLink, msg: mavlink.MAVLink_manual_control_message):
    pass


def heartbeat_handler(client: mavlink.MAVLink, msg: mavlink.MAVLink_heartbeat_message):
    pass


def set_position_target_local_ned_handler(client: mavlink.MAVLink, msg: mavlink.MAVLink_set_position_target_local_ned_message):
    pass


def param_request_list_handler(client: mavlink.MAVLink, msg: mavlink.MAVLink_param_request_list_message):
    pass


def param_set_handler(client: mavlink.MAVLink, msg: mavlink.MAVLink_param_set_message):
    pass


def mission_count_handler(client: mavlink.MAVLink, msg: mavlink.MAVLink_mission_count_message):
    pass


def mission_request_handler(client: mavlink.MAVLink, msg: mavlink.MAVLink_mission_request_message):
    pass


def mission_request_int_handler(client: mavlink.MAVLink, msg: mavlink.MAVLink_mission_request_int_message):
    pass


def mission_ack_handler(client: mavlink.MAVLink, msg: mavlink.MAVLink_mission_ack_message):
    pass


def mission_item_handler(client: mavlink.MAVLink, msg: mavlink.MAVLink_mission_item_message):
    pass


def mission_item_int_handler(client: mavlink.MAVLink, msg: mavlink.MAVLink_mission_item_int_message):
    pass
