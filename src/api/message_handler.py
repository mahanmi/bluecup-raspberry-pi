from .mav_client import mavlink, client
from robot_core.control import test_class


def command_int_handler(msg: mavlink.MAVLink_command_int_message):
    pass


def command_long_handler(msg: mavlink.MAVLink_command_long_message):
    pass


def command_ack_handler(msg: mavlink.MAVLink_command_ack_message):
    pass


def manual_control_handler(msg: mavlink.MAVLink_manual_control_message):
    test_class.move(msg.x/10000,msg.y/10000,msg.z/10000)


def heartbeat_handler(msg: mavlink.MAVLink_heartbeat_message):
    pass


def set_position_target_local_ned_handler(msg: mavlink.MAVLink_set_position_target_local_ned_message):
    pass


# THis is just for example you should implement your own!
params: dict[str, float] = {
    'PARAM_1': 1.0,
}

params_index: dict[str, int] = {
    'PARAM_1': 1,
}


def param_request_list_handler(msg: mavlink.MAVLink_param_request_list_message):
    if msg.target_system != client.source_system or msg.target_component != client.source_component:
        return
    for param_id, value in params.items():
        client.mav.param_value_send(
            param_id, value, mavlink.MAV_PARAM_TYPE_FLOAT, len(params), params_index[param_id])


def param_set_handler(msg: mavlink.MAVLink_param_set_message):
    pass


def mission_count_handler(msg: mavlink.MAVLink_mission_count_message):
    pass


def mission_request_handler(msg: mavlink.MAVLink_mission_request_message):
    pass


def mission_request_int_handler(msg: mavlink.MAVLink_mission_request_int_message):
    pass


def mission_ack_handler(msg: mavlink.MAVLink_mission_ack_message):
    pass


def mission_item_handler(msg: mavlink.MAVLink_mission_item_message):
    pass


def mission_item_int_handler(msg: mavlink.MAVLink_mission_item_int_message):
    pass
