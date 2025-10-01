import asyncio
import sys
import os
import logging

# Configure logger for this module
logger = logging.getLogger(__name__)

# Smart import strategy - try src. prefix first, then without
try:
    from src.mission_planner import planner, missions
    from src.robot_core import robot
    from src.hardware_interface import light
except ImportError:
    # Running from src directory, use relative imports
    from mission_planner import planner, missions
    from robot_core import robot
    from hardware_interface import light

from .mavlink import mavlink, client, ButtonFunctions
from . import command_handler

mission_request_retries = 0
K_MODE_MANUAL = 0b0000000000000010

# Button state tracking flags
button_states = {
    'gear_up': False,
    'gear_down': False,
    'tilt_up': False,
    'tilt_down': False,
    'heave_up': False,
    'heave_down': False,
    'reset': False,
    'led_toggle': False,
    'led_increase_brightness': False,
    'led_decrease_brightness': False,
    'led_strobe': False,
    'led_pulse': False,
    'led_pwm_freq': False
}

# LED system state
led_system = {
    'is_on': False,
    'current_brightness': 0.0,
    'brightness_step': 0.2,  # 20% increments
    'min_brightness': 0.2,
    'max_brightness': 1.0,
    'current_mode': 'normal',  # normal, strobe, pulse
    'pwm_frequencies': ['low', 'medium', 'high'],
    'current_pwm_index': 1  # Start with 'medium'
}


async def command_recv_handler(msg: mavlink.MAVLink_command_long_message | mavlink.MAVLink_command_int_message):
    if msg.command in command_handler.handlers:
        await command_handler.handlers[msg.command](msg)


async def command_ack_handler(msg: mavlink.MAVLink_command_ack_message):
    pass


async def manual_control_handler(msg: mavlink.MAVLink_manual_control_message):
    global button_states

    x = msg.x / 1000.0  # Scale from -1000 to 1000 to -1.0 to 1.0
    y = msg.y / 1000.0  # Scale from -1000 to 1000 to -1.0 to 1.0
    z = (msg.z - 500) / 500.0  # Scale from 0 to 1000 to -1.0 to 1.0
    yaw = msg.r / 1000.0  # Scale from -1000 to 1000 to -1.0 to 1.0

    # Check for button press (transition from False to True)
    gear_up = msg.dpad_right == 1 and not button_states['gear_up']
    gear_down = msg.dpad_left == 1 and not button_states['gear_down']
    tilt_up = msg.dpad_up == 1 and not button_states['tilt_up']
    tilt_down = msg.dpad_down == 1 and not button_states['tilt_down']
    heave_up = msg.r2 == 1 and not button_states['heave_up']
    heave_down = msg.l2 == 1 and not button_states['heave_down']
    reset = msg.options == 1 and not button_states['reset']

    # Update button states
    button_states['gear_up'] = msg.dpad_right == 1
    button_states['gear_down'] = msg.dpad_left == 1
    button_states['tilt_up'] = msg.dpad_up == 1
    button_states['tilt_down'] = msg.dpad_down == 1
    button_states['heave_up'] = msg.r2 == 1
    button_states['heave_down'] = msg.l2 == 1
    button_states['reset'] = msg.options == 1

    robot.set_movement_targets(
        x=x, y=y, z=z, yaw=yaw, gear_up=gear_up, gear_down=gear_down, tilt_up=tilt_up, tilt_down=tilt_down, heave_up=heave_up, heave_down=heave_down, reset=reset)

    # LED Control System - Enhanced button mapping
    led_toggle = msg.square == 1 and not button_states['led_toggle']
    led_increase_brightness = msg.triangle == 1 and not button_states['led_increase_brightness']
    led_decrease_brightness = msg.cross == 1 and not button_states['led_decrease_brightness']
    led_strobe = msg.circle == 1 and not button_states['led_strobe']
    led_pulse = msg.l1 == 1 and not button_states['led_pulse']
    led_pwm_freq = msg.r1 == 1 and not button_states['led_pwm_freq']

    # Update button states
    button_states['led_toggle'] = msg.square == 1
    button_states['led_increase_brightness'] = msg.triangle == 1
    button_states['led_decrease_brightness'] = msg.cross == 1
    button_states['led_strobe'] = msg.circle == 1
    button_states['led_pulse'] = msg.l1 == 1
    button_states['led_pwm_freq'] = msg.r1 == 1

    # Handle LED controls (including reset)
    await handle_led_controls(led_toggle, led_increase_brightness, led_decrease_brightness,
                              led_strobe, led_pulse, led_pwm_freq, reset)

    pass


async def handle_led_controls(led_toggle, led_increase_brightness, led_decrease_brightness,
                              led_strobe, led_pulse, led_pwm_freq, led_reset):
    """
    Comprehensive LED control handler with advanced features

    Button Mapping:
    - Square (led_toggle): Toggle main light on/off
    - Triangle (led_increase_brightness): Increase brightness by 20%
    - Cross (led_decrease_brightness): Decrease brightness by 20%
    - Circle (led_strobe): Toggle strobe mode
    - L1 (led_pulse): Toggle pulse mode
    - R1 (led_pwm_freq): Cycle through PWM frequencies (low/medium/high)
    - Options (led_reset): Reset LED system to default state
    """
    global led_system

    try:
        # Initialize lights if not already done
        if not light.lights_initialized:
            success = light.initialize_lights()
            if not success:
                logger.error("Failed to initialize LED system")
                return

        # Handle LED reset (Options button) - Process first to override other states
        if led_reset:
            # Stop all effects and turn off lights
            light.main_light_stop_effects()
            light.main_light_off()
            
            # Reset PWM frequency to default (medium = 1kHz)
            light.set_main_light_pwm_frequency('medium')
            
            # Reset LED system state to defaults
            led_system['is_on'] = False
            led_system['current_brightness'] = 0.6  # Default 60% brightness
            led_system['current_mode'] = 'normal'
            led_system['current_pwm_index'] = 1  # Medium frequency
            
            logger.info("LED: System reset to defaults (OFF, 60% brightness, 1kHz PWM, normal mode)")
            return  # Exit early after reset

        # Handle LED toggle (Square button)
        if led_toggle:
            if led_system['is_on']:
                # Turn off and stop any effects
                light.main_light_stop_effects()
                light.main_light_off()
                led_system['is_on'] = False
                led_system['current_mode'] = 'normal'
                logger.info("LED: Main light OFF")
            else:
                # Turn on at current brightness or default
                brightness = led_system['current_brightness'] if led_system['current_brightness'] > 0 else 0.6
                light.main_light_on(brightness)
                led_system['is_on'] = True
                led_system['current_brightness'] = brightness
                led_system['current_mode'] = 'normal'
                logger.info(f"LED: Main light ON at {brightness:.1%} brightness")

        # Handle brightness increase (Triangle button)
        if led_increase_brightness and led_system['is_on']:
            new_brightness = min(led_system['current_brightness'] + led_system['brightness_step'],
                                 led_system['max_brightness'])
            if new_brightness != led_system['current_brightness']:
                led_system['current_brightness'] = new_brightness

                # If in normal mode, adjust brightness directly
                if led_system['current_mode'] == 'normal':
                    light.main_light_brightness(new_brightness)
                    logger.info(f"LED: Brightness increased to {new_brightness:.1%}")
                else:
                    logger.debug(
                        f"LED: Brightness level set to {new_brightness:.1%} (will apply when exiting effect mode)")

        # Handle brightness decrease (Cross button)
        if led_decrease_brightness and led_system['is_on']:
            new_brightness = max(led_system['current_brightness'] - led_system['brightness_step'],
                                 led_system['min_brightness'])
            if new_brightness != led_system['current_brightness']:
                led_system['current_brightness'] = new_brightness

                # If in normal mode, adjust brightness directly
                if led_system['current_mode'] == 'normal':
                    light.main_light_brightness(new_brightness)
                    logger.info(f"LED: Brightness decreased to {new_brightness:.1%}")
                else:
                    logger.debug(
                        f"LED: Brightness level set to {new_brightness:.1%} (will apply when exiting effect mode)")

        # Handle strobe mode (Circle button)
        if led_strobe and led_system['is_on']:
            if led_system['current_mode'] == 'strobe':
                # Exit strobe mode
                light.main_light_stop_effects()
                light.main_light_brightness(led_system['current_brightness'])
                led_system['current_mode'] = 'normal'
                logger.info("LED: Strobe mode OFF")
            else:
                # Enter strobe mode
                light.main_light_stop_effects()
                light.main_light_blink(
                    on_brightness=led_system['current_brightness'],
                    off_brightness=0.0,
                    on_time=0.1,
                    off_time=0.1
                )
                led_system['current_mode'] = 'strobe'
                logger.info(
                    f"LED: Strobe mode ON at {led_system['current_brightness']:.1%} brightness")

        # Handle pulse mode (L1 button)
        if led_pulse and led_system['is_on']:
            if led_system['current_mode'] == 'pulse':
                # Exit pulse mode
                light.main_light_stop_effects()
                light.main_light_brightness(led_system['current_brightness'])
                led_system['current_mode'] = 'normal'
                logger.info("LED: Pulse mode OFF")
            else:
                # Enter pulse mode
                light.main_light_stop_effects()
                light.main_light_pulse(
                    min_brightness=led_system['min_brightness'],
                    max_brightness=led_system['current_brightness'],
                    fade_in_time=1.0,
                    fade_out_time=1.0
                )
                led_system['current_mode'] = 'pulse'
                logger.info(
                    f"LED: Pulse mode ON (0.1 to {led_system['current_brightness']:.1%} brightness)")

        # Handle PWM frequency cycling (R1 button)
        if led_pwm_freq:
            # Cycle through PWM frequencies
            led_system['current_pwm_index'] = (
                led_system['current_pwm_index'] + 1) % len(led_system['pwm_frequencies'])
            new_freq = led_system['pwm_frequencies'][led_system['current_pwm_index']]

            success = light.set_main_light_pwm_frequency(new_freq)
            if success:
                freq_hz = light.PWM_FREQUENCIES[new_freq]
                logger.info(
                    f"LED: PWM frequency changed to {freq_hz}Hz ({new_freq})")
            else:
                logger.warning(f"LED: Failed to change PWM frequency to {new_freq}")

        # Log current LED status periodically (every 100th call approximately)
        import random
        if random.randint(1, 100) == 1:  # Roughly 1% chance for periodic status
            states = light.get_light_states()
            if states:
                main_state = states.get('main_light', {})
                logger.debug(f"LED Status: ON={led_system['is_on']}, Brightness={led_system['current_brightness']:.1%}, "
                      f"Mode={led_system['current_mode']}, PWM={light.get_main_light_pwm_frequency()}")

    except Exception as e:
        logger.error(f"LED Control Error: {e}")


async def heartbeat_handler(msg: mavlink.MAVLink_heartbeat_message):
    pass


# THis is just for example you should implement your own!

async def param_request_list_handler(msg: mavlink.MAVLink_param_request_list_message):
    param_count = len(parameters)
    for index, param in enumerate(parameters):
        logger.debug(f"Sending parameter: {param}")
        await client.mav.param_value_send(**param, param_count=param_count, param_index=index)


async def param_request_read_handler(msg: mavlink.MAVLink_param_request_read_message):
    logger.debug(f"{msg.target_system} {msg.target_component} requested parameter {msg}")
    param_id = msg.param_id.encode()
    if msg.target_system != client.source_system or (msg.target_component != client.source_component and msg.target_component != mavlink.MAV_COMP_ID_ALL):
        return
    for index, param in enumerate(parameters):
        if param['param_id'] == param_id:
            await client.mav.param_value_send(**param, param_count=len(parameters), param_index=index)


async def param_set_handler(msg: mavlink.MAVLink_param_set_message):
    print(msg)
    param_id = msg.param_id.encode()
    for index, param in enumerate(parameters):
        if param['param_id'] == param_id:
            param['param_value'] = msg.param_value
            param['param_type'] = msg.param_type
            await client.mav.param_value_send(**param, param_count=len(parameters), param_index=index)
            break
    else:
        new_object = {'param_id': msg.param_id.encode(
        ), 'param_value': msg.param_value, 'param_type': msg.param_type}
        parameters.append(new_object)
        await client.mav.param_value_send(**new_object, param_count=len(parameters), parama_index=len(parameters)-1)


async def mission_request_list_handler(msg: mavlink.MAVLink_mission_request_int_message):
    await client.mav.mission_count_send(target_system=msg.get_srcSystem(), target_component=msg.get_srcComponent(), count=planner.get_missions_count(), mission_type=0)


async def mission_request_int_handler(msg: mavlink.MAVLink_mission_request_int_message):
    mission = planner.get_mission_item(msg.seq)
    if not mission:
        return await client.mav.mission_request_int_send(
            target_system=msg.get_srcSystem(),
            target_component=msg.get_srcComponent(),
            seq=msg.seq
        )
        # return await client.mav.mission_ack_send(
        #     target_system=msg.get_srcSystem(),
        #     target_component=msg.get_srcComponent(),
        #     type=mavlink.MAV_MISSION_
        # )
    await client.mav.mission_item_int_send(target_system=msg.get_srcSystem(), target_component=msg.get_srcComponent(), seq=msg.seq, **mission.to_dict())


async def mission_item_int_handler(msg: mavlink.MAVLink_mission_item_int_message):
    global mission_request_retries
    mission_request_retries = 0
    mission = missions.from_message(msg)

    if not mission:
        return await client.mav.mission_ack_send(
            target_system=msg.get_srcSystem(),
            target_component=msg.get_srcComponent(),
            type=mavlink.MAV_MISSION_UNSUPPORTED
        )

    remaining = planner.store_mission(mission, seq=msg.seq)

    if remaining > 0:
        mission_request_retries = 5
        await client.mav.mission_request_int_send(
            target_system=msg.get_srcSystem(),
            target_component=msg.get_srcComponent(),
            seq=msg.seq+1
        )
    elif remaining == 0:
        await client.mav.mission_ack_send(
            target_system=msg.get_srcSystem(),
            target_component=msg.get_srcComponent(),
            type=mavlink.MAV_MISSION_ACCEPTED
        )
    else:
        await client.mav.mission_ack_send(
            target_system=msg.get_srcSystem(),
            target_component=msg.get_srcComponent(),
            type=mavlink.MAV_MISSION_ERROR,
        )


async def mission_count_handler(msg: mavlink.MAVLink_mission_count_message):
    planner.prepare_mission_download(msg.count)

    if msg.count > 0:
        await client.mav.mission_request_int_send(
            target_system=msg.get_srcSystem(),
            target_component=msg.get_srcComponent(),
            seq=0
        )


async def mission_ack_handler(msg: mavlink.MAVLink_mission_ack_message):
    print("Mission ACK received,", msg.type)


handlers = {
    mavlink.MAVLINK_MSG_ID_COMMAND_LONG: command_recv_handler,
    mavlink.MAVLINK_MSG_ID_COMMAND_INT: command_recv_handler,
    mavlink.MAVLINK_MSG_ID_COMMAND_ACK: command_ack_handler,
    mavlink.MAVLINK_MSG_ID_HEARTBEAT: heartbeat_handler,
    mavlink.MAVLINK_MSG_ID_MANUAL_CONTROL: manual_control_handler,
    mavlink.MAVLINK_MSG_ID_PARAM_REQUEST_LIST: param_request_list_handler,
    mavlink.MAVLINK_MSG_ID_PARAM_REQUEST_READ: param_request_read_handler,
    mavlink.MAVLINK_MSG_ID_PARAM_SET: param_set_handler,
    mavlink.MAVLINK_MSG_ID_MISSION_REQUEST_LIST: mission_request_list_handler,
    mavlink.MAVLINK_MSG_ID_MISSION_REQUEST_INT: mission_request_int_handler,
    mavlink.MAVLINK_MSG_ID_MISSION_ITEM_INT: mission_item_int_handler,
    mavlink.MAVLINK_MSG_ID_MISSION_COUNT: mission_count_handler,
    mavlink.MAVLINK_MSG_ID_MISSION_ACK: mission_ack_handler,
}


def btn_functions(id: int, normal_function: ButtonFunctions, shift_function: ButtonFunctions):
    return [
        {"param_id": f"BTN{id}_FUNCTION".encode(
        ), "param_value": normal_function.value, "param_type": 2},
        {"param_id": f"BTN{id}_SFUNCTION".encode(
        ), "param_value": shift_function.value, "param_type": 2},
    ]


# fmt: off
parameters = [
    *btn_functions(0, ButtonFunctions.Shift, ButtonFunctions.Shift),
    *btn_functions(1, ButtonFunctions.ModeManual, ButtonFunctions.KNone),
    *btn_functions(2, ButtonFunctions.ModeDepthHold, ButtonFunctions.ModePoshold),
    *btn_functions(3, ButtonFunctions.ModeStabilize, ButtonFunctions.ModeAcro),
    *btn_functions(4, ButtonFunctions.KNone, ButtonFunctions.KNone),
    *btn_functions(5, ButtonFunctions.KNone, ButtonFunctions.KNone),
    *btn_functions(6, ButtonFunctions.MountTiltDown, ButtonFunctions.Servo1Min),
    *btn_functions(7, ButtonFunctions.MountTiltUp, ButtonFunctions.Servo1Max),
    *btn_functions(8, ButtonFunctions.Disarm, ButtonFunctions.KNone),
    *btn_functions(9, ButtonFunctions.Arm, ButtonFunctions.KNone),
    *btn_functions(10, ButtonFunctions.MountCenter, ButtonFunctions.Relay1Toggle),
    *btn_functions(11, ButtonFunctions.InputHoldSet, ButtonFunctions.KNone),
    *btn_functions(12, ButtonFunctions.GainInc, ButtonFunctions.TrimPitchInc),
    *btn_functions(13, ButtonFunctions.GainDec, ButtonFunctions.TrimPitchDec),
    *btn_functions(14, ButtonFunctions.Lights1Dimmer, ButtonFunctions.TrimRollDec),
    *btn_functions(15, ButtonFunctions.Lights1Brighter, ButtonFunctions.TrimRollInc),
    *btn_functions(16, ButtonFunctions.KNone, ButtonFunctions.KNone),
    *btn_functions(17, ButtonFunctions.KNone, ButtonFunctions.KNone),
    *btn_functions(18, ButtonFunctions.KNone, ButtonFunctions.KNone),
    *btn_functions(19, ButtonFunctions.KNone, ButtonFunctions.KNone),
    *btn_functions(20, ButtonFunctions.KNone, ButtonFunctions.KNone),
    *btn_functions(21, ButtonFunctions.KNone, ButtonFunctions.KNone),
    *btn_functions(22, ButtonFunctions.KNone, ButtonFunctions.KNone),
    *btn_functions(23, ButtonFunctions.KNone, ButtonFunctions.KNone),
    *btn_functions(24, ButtonFunctions.KNone, ButtonFunctions.KNone),
    *btn_functions(25, ButtonFunctions.KNone, ButtonFunctions.KNone),
    *btn_functions(26, ButtonFunctions.KNone, ButtonFunctions.KNone),
    *btn_functions(27, ButtonFunctions.KNone, ButtonFunctions.KNone),
    *btn_functions(28, ButtonFunctions.KNone, ButtonFunctions.KNone),
    *btn_functions(29, ButtonFunctions.KNone, ButtonFunctions.KNone),
    *btn_functions(30, ButtonFunctions.KNone, ButtonFunctions.KNone),
    *btn_functions(31, ButtonFunctions.KNone, ButtonFunctions.KNone),
]
# fmt: on

# LED System Utility Functions


async def initialize_led_system():
    """Initialize the LED system at startup"""
    global led_system

    try:
        success = light.initialize_lights()
        if success:
            logger.info("LED System: Initialized successfully")

            # Set initial state
            led_system['is_on'] = False
            led_system['current_brightness'] = 0.5
            led_system['current_mode'] = 'normal'

            # Perform startup light test
            logger.info("LED System: Performing startup test...")
            light.main_light_on(0.3)
            await asyncio.sleep(0.5)
            light.main_light_off()
            await asyncio.sleep(0.2)
            light.main_light_on(0.3)
            await asyncio.sleep(0.3)
            light.main_light_off()

            logger.info("LED System: Ready for control")
            return True
        else:
            logger.error("LED System: Failed to initialize")
            return False
    except Exception as e:
        logger.error(f"LED System: Initialization error - {e}")
        return False


def cleanup_led_system():
    """Cleanup LED system on shutdown"""
    try:
        logger.info("LED System: Shutting down...")
        light.main_light_stop_effects()
        light.main_light_off()
        light.cleanup_lights()
        logger.info("LED System: Shutdown complete")
    except Exception as e:
        logger.error(f"LED System: Cleanup error - {e}")


def get_led_status():
    """Get current LED system status"""
    try:
        states = light.get_light_states()
        status = {
            'system_initialized': light.lights_initialized,
            'is_on': led_system['is_on'],
            'brightness': led_system['current_brightness'],
            'mode': led_system['current_mode'],
            'pwm_frequency': light.get_main_light_pwm_frequency(),
            'hardware_states': states
        }
        return status
    except Exception as e:
        logger.error(f"LED System: Status error - {e}")
        return None
