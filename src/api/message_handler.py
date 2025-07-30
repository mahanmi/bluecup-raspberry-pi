
from .mavlink import mavlink, client
from . import command_handler
from robot_core import robot


async def command_recv_handler(msg: mavlink.MAVLink_command_long_message | mavlink.MAVLink_command_int_message):
    if msg.command in command_handler.handlers:
        await command_handler.handlers[msg.command](msg)


async def command_ack_handler(msg: mavlink.MAVLink_command_ack_message):
    pass


async def manual_control_handler(msg: mavlink.MAVLink_manual_control_message):
    # robot.lat += msg.x/10000
    # robot.lon += msg.y/10000
    # robot.alt += msg.z/10000
    # robot.yaw += msg.r/10000
    print(msg.x , msg.y , msg.z)



async def heartbeat_handler(msg: mavlink.MAVLink_heartbeat_message):
    pass


# THis is just for example you should implement your own!
params: dict[str, float] = {
    'PARAM_1': 1.0,


}

params_index: dict[str, int] = {
    'PARAM_1': 0,
}


async def param_request_list_handler(msg: mavlink.MAVLink_param_request_list_message):
    print(f"{msg.target_system} {msg.target_component} requested parameters")
    if msg.target_system != client.source_system or (msg.target_component != client.source_component and msg.target_component != mavlink.MAV_COMP_ID_ALL):
        return
    for param_id, value in params.items():
        client.mav.param_value_send(
            param_id.encode(), value, mavlink.MAV_PARAM_TYPE_REAL32, len(params), params_index[param_id])


async def param_request_read_handler(msg: mavlink.MAVLink_param_request_read_message):
    print(f"{msg.target_system} {msg.target_component} requested parameter {msg.param_id}")
    if msg.target_system != client.source_system or (msg.target_component != client.source_component and msg.target_component != mavlink.MAV_COMP_ID_ALL):
        return
    if msg.param_id in params:
        value = params[msg.param_id]
        await client.mav.param_value_send(
            msg.param_id.encode(), value, mavlink.MAV_PARAM_TYPE_REAL32, len(params), params_index[msg.param_id])


async def param_set_handler(msg: mavlink.MAVLink_param_set_message):
    pass


handlers = {
    mavlink.MAVLINK_MSG_ID_COMMAND_LONG: command_recv_handler,
    mavlink.MAVLINK_MSG_ID_COMMAND_INT: command_recv_handler,
    mavlink.MAVLINK_MSG_ID_COMMAND_ACK: command_ack_handler,
    mavlink.MAVLINK_MSG_ID_HEARTBEAT: heartbeat_handler,
    mavlink.MAVLINK_MSG_ID_MANUAL_CONTROL: manual_control_handler,
    mavlink.MAVLINK_MSG_ID_PARAM_REQUEST_LIST: param_request_list_handler,
    mavlink.MAVLINK_MSG_ID_PARAM_REQUEST_READ: param_request_read_handler,
    mavlink.MAVLINK_MSG_ID_PARAM_SET: param_set_handler,
}
