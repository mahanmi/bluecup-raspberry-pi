
from .mavlink import mavlink, client
from . import command_handler
from robot_core import robot


async def command_recv_handler(msg: mavlink.MAVLink_command_long_message | mavlink.MAVLink_command_int_message):
    if msg.command in command_handler.handlers:
        await command_handler.handlers[msg.command](msg)


async def command_ack_handler(msg: mavlink.MAVLink_command_ack_message):
    pass


async def manual_control_handler(msg: mavlink.MAVLink_manual_control_message):
    print(msg)


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
    print(msg)
    for param in parameters:
        await client.mav.param_value_send(**param)
    # await client.mav.statustext_send(6,b"ArduSub V4.6.0-dev",0,33)
    # print(f"{msg.target_system} {msg.target_component} requested parameters")
    # await client.mav.param_value_send(b"BTN0_FUNCTION",5,2,3,33)
    # print("sent")
    # await client.mav.param_value_send(b"BTN0_SFUNCTION",1,2,3,34)
    #
    #
    #
    # print("sent")
    # await client.mav.param_value_send(b"BTN1_FUNCTION",3,2,3,35)
    # print("sent")


    # if msg.target_system != client.source_system or (msg.target_component != client.source_component and msg.target_component != mavlink.MAV_COMP_ID_ALL):
    #     return
    # for param_id, value in params.items():
    #     await client.mav.param_value_send(
    #         param_id.encode(), value, mavlink.MAV_PARAM_TYPE_REAL32, len(params), params_index[param_id])


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


async def dc(msg: mavlink.MAVLink_message):
    await client.mav.mission_count_send(target_system=255,target_component=240,count=0,mission_type=0)
    await client.mav.mission_ack_send(255,240,1)





handlers = {
    mavlink.MAVLINK_MSG_ID_COMMAND_LONG: command_recv_handler,
    mavlink.MAVLINK_MSG_ID_COMMAND_INT: command_recv_handler,
    mavlink.MAVLINK_MSG_ID_COMMAND_ACK: command_ack_handler,
    mavlink.MAVLINK_MSG_ID_HEARTBEAT: heartbeat_handler,
    mavlink.MAVLINK_MSG_ID_MANUAL_CONTROL: manual_control_handler,
    mavlink.MAVLINK_MSG_ID_PARAM_REQUEST_LIST: param_request_list_handler,
    mavlink.MAVLINK_MSG_ID_PARAM_REQUEST_READ: param_request_read_handler,
    mavlink.MAVLINK_MSG_ID_PARAM_SET: param_set_handler,
    mavlink.MAVLINK_MSG_ID_MISSION_REQUEST_LIST: dc,
    mavlink.MAVLINK_MSG_ID_MISSION_ITEM_INT:dc,
}


parameters=[
#     mavlink.MAVLink_param_value_message(b"BTN0_FUNCTION",0,2,1308,33),
#     mavlink.MAVLink_param_value_message(b"BTN0_SFUNCTION",0,2,1308,34),
#     mavlink.MAVLink_param_value_message(b"BTN1_SFUNCTION",0,2,1308,35),
#     mavlink.MAVLink_param_value_message(b"BTN1_SFUNCTION",0,2,1308,36),
    {"param_id": b"BTN0_FUNCTION", "param_value": 2.0, "param_type": 2, "param_count": 1319, "param_index": 59},
    {"param_id": b"BTN0_SFUNCTION", "param_value": 8.0, "param_type": 2, "param_count": 1319, "param_index": 60},
    {"param_id": b"BTN1_FUNCTION", "param_value": 5.0, "param_type": 2, "param_count": 1319, "param_index": 61},
    {"param_id": b"BTN1_SFUNCTION", "param_value": 12.0, "param_type": 2, "param_count": 1319, "param_index": 62},
    {"param_id": b"BTN2_FUNCTION", "param_value": 7.0, "param_type": 2, "param_count": 1319, "param_index": 63},
    {"param_id": b"BTN2_SFUNCTION", "param_value": 63.0, "param_type": 2, "param_count": 1319, "param_index": 64},
    {"param_id": b"BTN3_FUNCTION", "param_value": 6.0, "param_type": 2, "param_count": 1319, "param_index": 65},
    {"param_id": b"BTN3_SFUNCTION", "param_value": 64.0, "param_type": 2, "param_count": 1319, "param_index": 66},
    {"param_id": b"BTN4_FUNCTION", "param_value": 4.0, "param_type": 2, "param_count": 1319, "param_index": 67},
    {"param_id": b"BTN4_SFUNCTION", "param_value": 53.0, "param_type": 2, "param_count": 1319, "param_index": 68},
    {"param_id": b"BTN5_FUNCTION", "param_value": 1.0, "param_type": 2, "param_count": 1319, "param_index": 69},
    {"param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 2, "param_count": 1319, "param_index": 70},
    {"param_id": b"BTN6_FUNCTION", "param_value": 3.0, "param_type": 2, "param_count": 1319, "param_index": 71},
    {"param_id": b"BTN6_SFUNCTION", "param_value": 0.0, "param_type": 2, "param_count": 1319, "param_index": 72},
    {"param_id": b"BTN7_FUNCTION", "param_value": 21.0, "param_type": 2, "param_count": 1319, "param_index": 73},
    {"param_id": b"BTN7_SFUNCTION", "param_value": 0.0, "param_type": 2, "param_count": 1319, "param_index": 74},
    {"param_id": b"BTN8_FUNCTION", "param_value": 48.0, "param_type": 2, "param_count": 1319, "param_index": 75},
    {"param_id": b"BTN8_SFUNCTION", "param_value": 0.0, "param_type": 2, "param_count": 1319, "param_index": 76},
    {"param_id": b"BTN9_FUNCTION", "param_value": 23.0, "param_type": 2, "param_count": 1319, "param_index": 77},
    {"param_id": b"BTN9_SFUNCTION", "param_value": 27.0, "param_type": 2, "param_count": 1319, "param_index": 78},
    {"param_id": b"BTN10_FUNCTION", "param_value": 22.0, "param_type": 2, "param_count": 1319, "param_index": 79},
    {"param_id": b"BTN10_SFUNCTION", "param_value": 26.0, "param_type": 2, "param_count": 1319, "param_index": 80},
    {"param_id": b"BTN11_FUNCTION", "param_value": 42.0, "param_type": 2, "param_count": 1319, "param_index": 81},
    {"param_id": b"BTN11_SFUNCTION", "param_value": 47.0, "param_type": 2, "param_count": 1319, "param_index": 82},
    {"param_id": b"BTN12_FUNCTION", "param_value": 43.0, "param_type": 2, "param_count": 1319, "param_index": 83},
    {"param_id": b"BTN12_SFUNCTION", "param_value": 46.0, "param_type": 2, "param_count": 1319, "param_index": 84},
    {"param_id": b"BTN13_FUNCTION", "param_value": 33.0, "param_type": 2, "param_count": 1319, "param_index": 85},
    {"param_id": b"BTN13_SFUNCTION", "param_value": 45.0, "param_type": 2, "param_count": 1319, "param_index": 86},
    {"param_id": b"BTN14_FUNCTION", "param_value": 32.0, "param_type": 2, "param_count": 1319, "param_index": 87},
    {"param_id": b"BTN14_SFUNCTION", "param_value": 44.0, "param_type": 2, "param_count": 1319, "param_index": 88},
    {"param_id": b"BTN15_FUNCTION", "param_value": 0.0, "param_type": 2, "param_count": 1319, "param_index": 89},
    {"param_id": b"BTN15_SFUNCTION", "param_value": 0.0, "param_type": 2, "param_count": 1319, "param_index": 90},
    {"param_id": b"BTN16_FUNCTION", "param_value": 0.0, "param_type": 2, "param_count": 1319, "param_index": 91},
    {"param_id": b"BTN16_SFUNCTION", "param_value": 0.0, "param_type": 2, "param_count": 1319, "param_index": 92},
    {"param_id": b"BTN17_FUNCTION", "param_value": 0.0, "param_type": 2, "param_count": 1319, "param_index": 93},
    {"param_id": b"BTN17_SFUNCTION", "param_value": 0.0, "param_type": 2, "param_count": 1319, "param_index": 94},
    {"param_id": b"BTN18_FUNCTION", "param_value": 0.0, "param_type": 2, "param_count": 1319, "param_index": 95},
    {"param_id": b"BTN18_SFUNCTION", "param_value": 0.0, "param_type": 2, "param_count": 1319, "param_index": 96},
    {"param_id": b"BTN19_FUNCTION", "param_value": 0.0, "param_type": 2, "param_count": 1319, "param_index": 97},
    {"param_id": b"BTN19_SFUNCTION", "param_value": 0.0, "param_type": 2, "param_count": 1319, "param_index": 98},
    {"param_id": b"BTN20_FUNCTION", "param_value": 0.0, "param_type": 2, "param_count": 1319, "param_index": 99},
    {"param_id": b"BTN20_SFUNCTION", "param_value": 0.0, "param_type": 2, "param_count": 1319, "param_index": 100},
    {"param_id": b"BTN21_FUNCTION", "param_value": 0.0, "param_type": 2, "param_count": 1319, "param_index": 101},
    {"param_id": b"BTN21_SFUNCTION", "param_value": 0.0, "param_type": 2, "param_count": 1319, "param_index": 102},
    {"param_id": b"BTN22_FUNCTION", "param_value": 0.0, "param_type": 2, "param_count": 1319, "param_index": 103},
    {"param_id": b"BTN22_SFUNCTION", "param_value": 0.0, "param_type": 2, "param_count": 1319, "param_index": 104},
    {"param_id": b"BTN23_FUNCTION", "param_value": 0.0, "param_type": 2, "param_count": 1319, "param_index": 105},
    {"param_id": b"BTN23_SFUNCTION", "param_value": 0.0, "param_type": 2, "param_count": 1319, "param_index": 106},
    {"param_id": b"BTN24_FUNCTION", "param_value": 0.0, "param_type": 2, "param_count": 1319, "param_index": 107},
    {"param_id": b"BTN24_SFUNCTION", "param_value": 0.0, "param_type": 2, "param_count": 1319, "param_index": 108},
    {"param_id": b"BTN25_FUNCTION", "param_value": 0.0, "param_type": 2, "param_count": 1319, "param_index": 109},
    {"param_id": b"BTN25_SFUNCTION", "param_value": 0.0, "param_type": 2, "param_count": 1319, "param_index": 110},
    {"param_id": b"BTN26_FUNCTION", "param_value": 0.0, "param_type": 2, "param_count": 1319, "param_index": 111},
    {"param_id": b"BTN26_SFUNCTION", "param_value": 0.0, "param_type": 2, "param_count": 1319, "param_index": 112},
    {"param_id": b"BTN27_FUNCTION", "param_value": 0.0, "param_type": 2, "param_count": 1319, "param_index": 113},
    {"param_id": b"BTN27_SFUNCTION", "param_value": 0.0, "param_type": 2, "param_count": 1319, "param_index": 114},
    {"param_id": b"BTN28_FUNCTION", "param_value": 0.0, "param_type": 2, "param_count": 1319, "param_index": 115},
    {"param_id": b"BTN28_SFUNCTION", "param_value": 0.0, "param_type": 2, "param_count": 1319, "param_index": 116},
    {"param_id": b"BTN29_FUNCTION", "param_value": 0.0, "param_type": 2, "param_count": 1319, "param_index": 117},
    {"param_id": b"BTN29_SFUNCTION", "param_value": 0.0, "param_type": 2, "param_count": 1319, "param_index": 118},
    {"param_id": b"BTN30_FUNCTION", "param_value": 0.0, "param_type": 2, "param_count": 1319, "param_index": 119},
    {"param_id": b"BTN30_SFUNCTION", "param_value": 0.0, "param_type": 2, "param_count": 1319, "param_index": 120},
    {"param_id": b"BTN31_FUNCTION", "param_value": 0.0, "param_type": 2, "param_count": 1319, "param_index": 121},
    {"param_id": b"BTN31_SFUNCTION", "param_value": 0.0, "param_type": 2, "param_count": 1319, "param_index": 122},
    {"target_system": 1, "target_component": 0, "param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 1},
    {"param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 2, "param_count": 1319, "param_index": 65535},
    {"target_system": 1, "target_component": 0, "param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 1},
    {"param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 2, "param_count": 1319, "param_index": 65535},
    {"target_system": 1, "target_component": 0, "param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 1},
    {"param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 2, "param_count": 1319, "param_index": 65535},
    {"target_system": 1, "target_component": 0, "param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 1},
    {"param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 2, "param_count": 1319, "param_index": 65535},
    {"target_system": 1, "target_component": 0, "param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 1},
    {"param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 2, "param_count": 1319, "param_index": 65535},
    {"target_system": 1, "target_component": 0, "param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 1},
    {"param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 2, "param_count": 1319, "param_index": 65535},
    {"target_system": 1, "target_component": 0, "param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 1},
    {"param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 2, "param_count": 1319, "param_index": 65535},
    {"target_system": 1, "target_component": 0, "param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 1},
    {"param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 2, "param_count": 1319, "param_index": 65535},
    {"target_system": 1, "target_component": 0, "param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 1},
    {"param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 2, "param_count": 1319, "param_index": 65535},
    {"target_system": 1, "target_component": 0, "param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 1},
    {"param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 2, "param_count": 1319, "param_index": 65535},
    {"target_system": 1, "target_component": 0, "param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 1},
    {"param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 2, "param_count": 1319, "param_index": 65535},
    {"target_system": 1, "target_component": 0, "param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 1},
    {"param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 2, "param_count": 1319, "param_index": 65535},
    {"target_system": 1, "target_component": 0, "param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 1},
    {"param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 2, "param_count": 1319, "param_index": 65535},
    {"target_system": 1, "target_component": 0, "param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 1},
    {"param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 2, "param_count": 1319, "param_index": 65535},
    {"target_system": 1, "target_component": 0, "param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 1},
    {"param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 2, "param_count": 1319, "param_index": 65535},
    {"target_system": 1, "target_component": 0, "param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 1},
    {"param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 2, "param_count": 1319, "param_index": 65535},
    {"target_system": 1, "target_component": 0, "param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 1},
    {"param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 2, "param_count": 1319, "param_index": 65535},
    {"target_system": 1, "target_component": 0, "param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 1},
    {"param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 2, "param_count": 1319, "param_index": 65535},
    {"target_system": 1, "target_component": 0, "param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 1},
    {"param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 2, "param_count": 1319, "param_index": 65535},
    {"param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 2, "param_count": 1319, "param_index": 65535},
    {"target_system": 1, "target_component": 0, "param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 1},
    {"param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 2, "param_count": 1319, "param_index": 65535},
    {"target_system": 1, "target_component": 0, "param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 1},
    {"param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 2, "param_count": 1319, "param_index": 65535},
    {"target_system": 1, "target_component": 0, "param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 1},
    {"param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 2, "param_count": 1319, "param_index": 65535},
    {"target_system": 1, "target_component": 0, "param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 1},
    {"param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 2, "param_count": 1319, "param_index": 65535},
    {"target_system": 1, "target_component": 0, "param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 1},
    {"param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 2, "param_count": 1319, "param_index": 65535},
    {"target_system": 1, "target_component": 0, "param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 1},
    {"param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 2, "param_count": 1319, "param_index": 65535},
    {"target_system": 1, "target_component": 0, "param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 1},
    {"param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 2, "param_count": 1319, "param_index": 65535},
    {"target_system": 1, "target_component": 0, "param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 1},
    {"param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 2, "param_count": 1319, "param_index": 65535},
    {"target_system": 1, "target_component": 0, "param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 1},
    {"param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 2, "param_count": 1319, "param_index": 65535},
    {"target_system": 1, "target_component": 0, "param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 1},
    {"param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 2, "param_count": 1319, "param_index": 65535},
    {"target_system": 1, "target_component": 0, "param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 1},
    {"param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 2, "param_count": 1319, "param_index": 65535},
    {"target_system": 1, "target_component": 0, "param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 1},
    {"param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 2, "param_count": 1319, "param_index": 65535},
    {"target_system": 1, "target_component": 0, "param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 1},
    {"param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 2, "param_count": 1319, "param_index": 65535},
    {"target_system": 1, "target_component": 0, "param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 1},
    {"param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 2, "param_count": 1319, "param_index": 65535},
    {"target_system": 1, "target_component": 0, "param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 1},
    {"param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 2, "param_count": 1319, "param_index": 65535},
    {"target_system": 1, "target_component": 0, "param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 1},
    {"param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 2, "param_count": 1319, "param_index": 65535},
    {"target_system": 1, "target_component": 0, "param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 1},
    {"param_id": b"BTN5_SFUNCTION", "param_value": 1.0, "param_type": 2, "param_count": 1319, "param_index": 65535},
]
