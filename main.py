from typing import Callable
from pymavlink.dialects.v20 import ardupilotmega as mavlink
from pymavlink import mavutil
import message_handler
import message_builder


client = mavutil.mavlink_connection(
    device='udpin:0.0.0.0:14604', dialect=mavlink.DIALECT)


class periodic_event_handler(mavutil.periodic_event):
    def __init__(self, frequency: int, handler):
        self.handler = handler
        super().__init__(frequency)


events_list: [tuple[int, mavutil.periodic_event, Callable[[], mavlink.MAVLink_message]]] = [
    (mavlink.MAVLINK_MSG_ID_HEARTBEAT,
     mavutil.periodic_event(1), message_builder.send_heartbeat),
    (mavlink.MAVLINK_MSG_ID_RAW_IMU, mavutil.periodic_event(
        2), message_builder.send_raw_imu),
    (mavlink.MAVLINK_MSG_ID_SCALED_IMU2,
     mavutil.periodic_event(2), message_builder.send_scaled_imu2),
    (mavlink.MAVLINK_MSG_ID_SCALED_IMU3,
     mavutil.periodic_event(2), message_builder.send_scaled_imu3),
    (mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE,
     mavutil.periodic_event(2), message_builder.send_scaled_pressure),
    (mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE2,
     mavutil.periodic_event(2), message_builder.send_scaled_pressure2),
    (mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE3,
     mavutil.periodic_event(2), message_builder.send_scaled_pressure3),
    (mavlink.MAVLINK_MSG_ID_SYS_STATUS,
     mavutil.periodic_event(2), message_builder.send_sys_status),
    (mavlink.MAVLINK_MSG_ID_POWER_STATUS,
     mavutil.periodic_event(2), message_builder.send_power_status),
    (mavlink.MAVLINK_MSG_ID_MEMINFO, mavutil.periodic_event(
        2), message_builder.send_meminfo),
    (mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,
     mavutil.periodic_event(2), message_builder.send_gps_raw_int),
    (mavlink.MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT,
     mavutil.periodic_event(2), message_builder.send_nav_controller_output),
    (mavlink.MAVLINK_MSG_ID_FENCE_STATUS,
     mavutil.periodic_event(2), message_builder.send_fence_status),
    (mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
     mavutil.periodic_event(3), message_builder.send_global_position_int),
    (mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,
     mavutil.periodic_event(2), message_builder.send_servo_output_raw),
    (mavlink.MAVLINK_MSG_ID_RC_CHANNELS,
     mavutil.periodic_event(2), message_builder.send_rc_channels),
    (mavlink.MAVLINK_MSG_ID_RC_CHANNELS_RAW,
     mavutil.periodic_event(2), message_builder.send_rc_channels_raw),
    (mavlink.MAVLINK_MSG_ID_ATTITUDE,
     mavutil.periodic_event(16), message_builder.send_attitude),
    (mavlink.MAVLINK_MSG_ID_SIMSTATE,
     mavutil.periodic_event(10), message_builder.send_simstate),
    (mavlink.MAVLINK_MSG_ID_AHRS2, mavutil.periodic_event(
        16), message_builder.send_ahrs2),
    (mavlink.MAVLINK_MSG_ID_PID_TUNING,
     mavutil.periodic_event(10), message_builder.send_pid_tuning),
    (mavlink.MAVLINK_MSG_ID_VFR_HUD, mavutil.periodic_event(
        16), message_builder.send_vfr_hud),
    (mavlink.MAVLINK_MSG_ID_AHRS, mavutil.periodic_event(
        16), message_builder.send_ahrs),
    (mavlink.MAVLINK_MSG_ID_SYSTEM_TIME,
     mavutil.periodic_event(3), message_builder.send_system_time),
    (mavlink.MAVLINK_MSG_ID_RANGEFINDER,
     mavutil.periodic_event(3), message_builder.send_rangefinder),
    (mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR,
     mavutil.periodic_event(3), message_builder.send_distance_sensor),
    (mavlink.MAVLINK_MSG_ID_BATTERY_STATUS,
     mavutil.periodic_event(3), message_builder.send_battery_status),
    (mavlink.MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS,
     mavutil.periodic_event(16), message_builder.send_gimbal_device_attitude_status),
    (mavlink.MAVLINK_MSG_ID_MAG_CAL_REPORT,
     mavutil.periodic_event(3), message_builder.send_mag_cal_report),
    (mavlink.MAVLINK_MSG_ID_MAG_CAL_PROGRESS,
     mavutil.periodic_event(3), message_builder.send_mag_cal_progress),
    (mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT,
     mavutil.periodic_event(3), message_builder.send_ekf_status_report),
    (mavlink.MAVLINK_MSG_ID_VIBRATION,
     mavutil.periodic_event(3), message_builder.send_vibration),
    (mavlink.MAVLINK_MSG_ID_PARAM_VALUE,
     mavutil.periodic_event(2), message_builder.send_param_value),
]

handlers = {
    mavlink.MAVLINK_MSG_ID_COMMAND_INT: message_handler.command_int_handler,
    mavlink.MAVLINK_MSG_ID_COMMAND_LONG: message_handler.command_long_handler,
    mavlink.MAVLINK_MSG_ID_COMMAND_ACK: message_handler.command_ack_handler,
}

while True:
    msg = client.recv_msg()
    if msg is not None and msg.message_type in handlers:
        handlers[msg.message_type](msg)

    for (id, event, action) in events_list:
        if event.trigger():
            client.mav.send(action())
