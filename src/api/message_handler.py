from .mav_client import mavlink, client

x = 0.0
y = 0.0
z = 0.0


def command_int_handler(msg: mavlink.MAVLink_command_int_message):
    pass


def command_long_handler(msg: mavlink.MAVLink_command_long_message):
    pass


def command_ack_handler(msg: mavlink.MAVLink_command_ack_message):
    pass


def manual_control_handler(msg: mavlink.MAVLink_manual_control_message):
    global x, y, z
    x = msg.x/10000
    y = msg.y/10000
    z = msg.z/10000


def heartbeat_handler(msg: mavlink.MAVLink_heartbeat_message):
    pass


def set_position_target_local_ned_handler(msg: mavlink.MAVLink_set_position_target_local_ned_message):
    pass


# THis is just for example you should implement your own!
params: dict[bytes, float] = {
    b'PARAM_1': 1.0,


}

params_index: dict[bytes, int] = {
    b'PARAM_1': 0,
}


def param_request_list_handler(msg: mavlink.MAVLink_param_request_list_message):
    print(f"{msg.target_system} {msg.target_component} requested parameters")
    if msg.target_system != client.source_system or (msg.target_component != client.source_component and msg.target_component != mavlink.MAV_COMP_ID_ALL):
        return
    for param_id, value in params.items():
        client.mav.param_value_send(
            param_id, value, mavlink.MAV_PARAM_TYPE_REAL32, len(params), params_index[param_id])


def param_request_read_handler(msg: mavlink.MAVLink_param_request_read_message):
    print(f"{msg.target_system} {msg.target_component} requested parameter {msg.param_id}")
    if msg.target_system != client.source_system or (msg.target_component != client.source_component and msg.target_component != mavlink.MAV_COMP_ID_ALL):
        return
    if msg.param_id in params:
        value = params[msg.param_id]
        client.mav.param_value_send(
            msg.param_id, value, mavlink.MAV_PARAM_TYPE_REAL32, len(params), params_index[msg.param_id])


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
