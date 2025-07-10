


#MPU 6050 , ICM
def fetch_raw_imu():


    return {
        "xacc":0,
        "yacc":0,
        "zacc":0,
        "xgyro":0,
        "ygyro":0,
        "zgyro":0,
        "xmag":0,
        "ymag":0,
        "zmag":0,
        # "id":0,
        # "temperature":0,
    }

# xacc	int16_t		X acceleration (raw)
# yacc	int16_t		Y acceleration (raw)
# zacc	int16_t		Z acceleration (raw)
# xgyro	int16_t		Angular speed around X axis (raw)
# ygyro	int16_t		Angular speed around Y axis (raw)
# zgyro	int16_t		Angular speed around Z axis (raw)
# xmag	int16_t		X Magnetic field (raw)
# ymag	int16_t		Y Magnetic field (raw)
# zmag	int16_t		Z Magnetic field (raw)
# id ++	uint8_t		Id. Ids are numbered from 0 and map to IMUs numbered from 1 (e.g. IMU1 will have a message with id=0)
# Messages with same value are from the same source (instance).
# temperature ++	int16_t	cdegC	Temperature, 0: IMU does not provide temperature values. If the IMU is at 0C it must send 1 (0.01C).

def fetch_gps_raw_int():
    return {
        "fix_type":3,

        "lat":0,
        "lon":0,
        "alt":0,
        "eph":65535,
        "epv":65535,
        "vel":65535,
        "cog":65535,
        "satellites_visible": 255,
        # "alt_ellipsoid":0,
        # "h_acc":0,
        # "v_acc":0,
        # "vel_acc":0,
        # "hdg_acc":0,
        # "yaw":0

    }


# fix_type	uint8_t			GPS_FIX_TYPE	GPS fix type.
# -----------------------------------------------------------------------
# 0	GPS_FIX_TYPE_NO_GPS	No GPS connected
# 1	GPS_FIX_TYPE_NO_FIX	No position information, GPS is connected
# 2	GPS_FIX_TYPE_2D_FIX	2D position
# 3	GPS_FIX_TYPE_3D_FIX	3D position
# 4	GPS_FIX_TYPE_DGPS	DGPS/SBAS aided 3D position
# 5	GPS_FIX_TYPE_RTK_FLOAT	RTK float, 3D position
# 6	GPS_FIX_TYPE_RTK_FIXED	RTK Fixed, 3D position
# 7	GPS_FIX_TYPE_STATIC	Static fixed, typically used for base stations
# 8	GPS_FIX_TYPE_PPP	PPP, 3D position.
# -------------------------------------------------------------------------
# lat	int32_t	degE7			Latitude (WGS84, EGM96 ellipsoid)
# lon	int32_t	degE7			Longitude (WGS84, EGM96 ellipsoid)
# alt	int32_t	mm			Altitude (MSL). Positive for up. Note that virtually all GPS modules provide the MSL altitude in addition to the WGS84 altitude.
# eph	uint16_t		1E-2	invalid:UINT16_MAX	GPS HDOP horizontal dilution of position (unitless * 100). If unknown, set to: UINT16_MAX
# epv	uint16_t		1E-2	invalid:UINT16_MAX	GPS VDOP vertical dilution of position (unitless * 100). If unknown, set to: UINT16_MAX
# vel	uint16_t	cm/s		invalid:UINT16_MAX	GPS ground speed. If unknown, set to: UINT16_MAX
# cog	uint16_t	cdeg		invalid:UINT16_MAX	Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
# satellites_visible	uint8_t			invalid:UINT8_MAX	Number of satellites visible. If unknown, set to UINT8_MAX
# alt_ellipsoid ++	int32_t	mm			Altitude (above WGS84, EGM96 ellipsoid). Positive for up.
# h_acc ++	uint32_t	mm			Position uncertainty.
# v_acc ++	uint32_t	mm			Altitude uncertainty.
# vel_acc ++	uint32_t	mm/s			Speed uncertainty.
# hdg_acc ++	uint32_t	degE5			Heading / track uncertainty
# yaw ++	uint16_t	cdeg		invalid:0	Yaw in earth frame from north. Use 0 if this GPS does not provide yaw. Use UINT16_MAX if this GPS is configured to provide yaw and is currently unable to provide it. Use 36000 for north.


def fetch_scaled_pressure():

    return {
        "press_abs":0,
        "press_diff":0,
        "temperature":0,
        # "temperature_press_diff":0
    }

# press_abs	float	hPa	Absolute pressure
# press_diff	float	hPa	Differential pressure 1
# temperature	int16_t	cdegC	Absolute pressure temperature
# temperature_press_diff ++	int16_t	cdegC	Differential pressure te


def fetch_scaled_pressure2():

    return {
        "press_abs":0,
        "press_diff":0,
        "temperature":0,
        # "temperature_press_diff":0
    }

# press_abs	float	hPa	Absolute pressure
# press_diff	float	hPa	Differential pressure 1
# temperature	int16_t	cdegC	Absolute pressure temperature
# temperature_press_diff ++	int16_t	cdegC	Differential pressure te


def fetch_scaled_pressure3():

    return {
        "press_abs":0,
        "press_diff":0,
        "temperature":0,
        # "temperature_press_diff":0
    }

# press_abs	float	hPa	Absolute pressure
# press_diff	float	hPa	Differential pressure 1
# temperature	int16_t	cdegC	Absolute pressure temperature
# temperature_press_diff ++	int16_t	cdegC	Differential pressure te



def fetch_ahrs2():
    return {
        "roll":0,
        "pitch":0,
        "yaw":0,
        "altitude":0,
        "lat":0,
        "lng":0
    }
# roll	float	rad	Roll angle.
# pitch	float	rad	Pitch angle.
# yaw	float	rad	Yaw angle.
# altitude	float	m	Altitude (MSL).
# lat	int32_t	degE7	Latitude.
# lng	int32_t	degE7	Longitude.

# ----------------------------------------------------------

def fetch_attitude():
    return {
        "roll":0,
        "pitch":0,
        "yaw":0,
        "rollspeed":0,
        "pitchspeed":0,
        "yawspeed":0

    }
# roll	float	rad	Roll angle (-pi..+pi)
# pitch	float	rad	Pitch angle (-pi..+pi)
# yaw	float	rad	Yaw angle (-pi..+pi)
# rollspeed	float	rad/s	Roll angular speed
# pitchspeed	float	rad/s	Pitch angular speed
# yawspeed	float	rad/s	Yaw angular speed

#---------------------------------------------------------------

def fetch_ekf_status_report():
    return {
        "flags":0,
        "velocity_variance":0,
        "pos_horiz_variance":0,
        "pos_vert_variance":0,
        "compass_variance":0,
        "terrain_alt_variance":0,


    }
# flags	uint16_t	EKF_STATUS_FLAGS	Flags.
# velocity_variance	float		Velocity variance.
# pos_horiz_variance	float		Horizontal Position variance.
# pos_vert_variance	float		Vertical Position variance.
# compass_variance	float		Compass variance.
# terrain_alt_variance	float		Terrain Altitude variance.

# ----------------------------------------------------------------

def fetch_global_position_int():
    return {
        "lat":0,
        "lon":0,
        "alt":0,
        "relative_alt":0,
        "vx":0,
        "vy":0,
        "vz":0,
        "hdg":65535

    }


# lat	int32_t	degE7	Latitude, expressed
# lon	int32_t	degE7	Longitude, expressed
# alt	int32_t	mm	Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.
# relative_alt	int32_t	mm	Altitude above home
# vx	int16_t	cm/s	Ground X Speed (Latitude, positive north)
# vy	int16_t	cm/s	Ground Y Speed (Longitude, positive east)
# vz	int16_t	cm/s	Ground Z Speed (Altitude, positive down)
# hdg	uint16_t	cdeg	Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX



def fetch_local_position_ned():
    return {
        "x":0,
        "y":0,
        "z":0,
        "vx":0,
        "vy":0,
        "vz":0

    }

# x	float	m	X Position
# y	float	m	Y Position
# z	float	m	Z Position
# vx	float	m/s	X Speed
# vy	float	m/s	Y Speed
# vz	float	m/s	Z Speed


def fetch_meminfo():
    return {
        "brkval":0,
        "freemem":0,
        # "freemem32":0
    }


# brkval	uint16_t		Heap top.
# freemem	uint16_t	bytes	Free memory.
# freemem32 ++	uint32_t	bytes	Free memory (32 bit).



def fetch_mission_current():
    return {
        "seq":0,
        "total":65535,
        "mission_state":0,
        "mission_mode":0,
        "mission_id":0,
        "fence_id":0,
        "rally_points_id":0
    }

# seq	uint16_t		Sequence
# total ++	uint16_t	invalid:UINT16_MAX	Total number of mission items on vehicle (on last item, sequence == total). If the autopilot stores its home location as part of the mission this will be excluded from the total. 0: Not supported, UINT16_MAX if no mission is present on the vehicle.
# mission_state ++	uint8_t	invalid:0 MISSION_STATE	Mission state machine state. MISSION_STATE_UNKNOWN if state reporting not supported.
# mission_mode ++	uint8_t	invalid:0	Vehicle is in a mode that can execute mission items or suspended. 0: Unknown, 1: In mission mode, 2: Suspended (not in mission mode).
# mission_id ++	uint32_t	invalid:0	Id of current on-vehicle mission plan, or 0 if IDs are not supported or there is no mission loaded. GCS can use this to track changes to the mission plan type. The same value is returned on mission upload (in the MISSION_ACK).
# fence_id ++	uint32_t	invalid:0	Id of current on-vehicle fence plan, or 0 if IDs are not supported or there is no fence loaded. GCS can use this to track changes to the fence plan type. The same value is returned on fence upload (in the MISSION_ACK).
# rally_points_id ++	uint32_t	invalid:0	Id of current on-vehicle rally point plan, or 0 if IDs are not supported or there are no rally points loaded. GCS can use this to track changes to the rally point plan type. The same value is returned on rally point upload (in the MISSION_ACK).

def fetch_named_value_float():
    return {
        "name":"",
        "value":0
    }

# name	char[10]		Name of the debug variable
# Messages with same value are from the same source (instance).
# value	float		Floating point value



def fetch_nav_controller_output():
    return {
        "nav_roll":0,
        "nav_pitch":0,
        "nav_bearing":0,
        "target_bearing":0,
        "wp_dist":0,
        "alt_error":0,
        "aspd_error":0,
        "xtrack_error":0
    }

# nav_roll	float	deg	Current desired roll
# nav_pitch	float	deg	Current desired pitch
# nav_bearing	int16_t	deg	Current desired heading
# target_bearing	int16_t	deg	Bearing to current waypoint/target
# wp_dist	uint16_t	m	Distance to active waypoint
# alt_error	float	m	Current altitude error
# aspd_error	float	m/s	Current airspeed error
# xtrack_error	float	m	Current crosstrack error on x-y plane


def fetch_power_status():
    return {
        "Vcc":0,
        "Vservo":0,
        "flags":0
    }

# Vcc	uint16_t	mV		5V rail voltage.
# Vservo	uint16_t	mV		Servo rail voltage.
# flags	uint16_t		MAV_POWER_STATUS	Bitmap of power supply status flags.

def motor_control(rc1,rc2,rc3,rc4,rc5,rc6,rc7,rc8,rc9,rc10,rc11,rc12,rc13,rc14,rc15,rc16,rc17,rc18):
    pass



def fetch_scaled_imu2_data():


    return {
        "xacc":0,
        "yacc":0,
        "zacc":0,
        "xgyro":0,
        "ygyro":0,
        "zgyro":0,
        "xmag":0,
        "ymag":0,
        "zmag":0,
        # "temperature":0
    }
# xacc	int16_t	mG	X acceleration
# yacc	int16_t	mG	Y acceleration
# zacc	int16_t	mG	Z acceleration
# xgyro	int16_t	mrad/s	Angular speed around X axis
# ygyro	int16_t	mrad/s	Angular speed around Y axis
# zgyro	int16_t	mrad/s	Angular speed around Z axis
# xmag	int16_t	mgauss	X Magnetic field
# ymag	int16_t	mgauss	Y Magnetic field
# zmag	int16_t	mgauss	Z Magnetic field
# temperature ++	int16_t	cdegC	Temperature, 0: IMU does not provide temperature values. If the IMU is at 0C it must send 1 (0.01C).

def fetch_scaled_imu3_data():


    return {
        "xacc":0,
        "yacc":0,
        "zacc":0,
        "xgyro":0,
        "ygyro":0,
        "zgyro":0,
        "xmag":0,
        "ymag":0,
        "zmag":0,
        # "temperature":0
    }

# xacc	int16_t	mG	X acceleration
# yacc	int16_t	mG	Y acceleration
# zacc	int16_t	mG	Z acceleration
# xgyro	int16_t	mrad/s	Angular speed around X axis
# ygyro	int16_t	mrad/s	Angular speed around Y axis
# zgyro	int16_t	mrad/s	Angular speed around Z axis
# xmag	int16_t	mgauss	X Magnetic field
# ymag	int16_t	mgauss	Y Magnetic field
# zmag	int16_t	mgauss	Z Magnetic field
# temperature ++	int16_t	cdegC	Temperature, 0: IMU does not provide temperature values. If the IMU is at 0C it must send 1 (0.01C).

def fetch_simstate():
    return {
        "roll":0,
        "pitch":0,
        "yaw":0,
        "xacc":0,
        "yacc":0,
        "zacc":0,
        "xgyro":0,
        "ygyro":0,
        "zgyro":0,
        "lat":0,
        "lon":0
    }


# roll	float	rad	Roll angle.
# pitch	float	rad	Pitch angle.
# yaw	float	rad	Yaw angle.
# xacc	float	m/s/s	X acceleration.
# yacc	float	m/s/s	Y acceleration.
# zacc	float	m/s/s	Z acceleration.
# xgyro	float	rad/s	Angular speed around X axis.
# ygyro	float	rad/s	Angular speed around Y axis.
# zgyro	float	rad/s	Angular speed around Z axis.
# lat	int32_t	degE7	Latitude.
# lng	int32_t	degE7	Longitude.

# def fetch_system_time():
#     return {
#         "time_unix_usec":None,
#     }

def fetch_sys_status():
    return {
        "onboard_control_sensors_present":0,
        "onboard_control_sensors_enabled":0,
        "onboard_control_sensors_health":0,
        "load":0,
        "voltage_battery":0,
        "current_battery":0,
        "battery_remaining":0,
        "drop_rate_comm":0,
        "errors_comm":0,
        "errors_count1":0,
        "errors_count2":0,
        "errors_count3":0,
        "errors_count4":0,
        # "onboard_control_sensors_present_extended":0,
        # "onboard_control_sensors_enabled_extended":0,
        # "onboard_control_sensors_health_extended":0
    }


#onboard_control_sensors_present	uint32_t		MAV_SYS_STATUS_SENSOR	Bitmap showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present.
# onboard_control_sensors_enabled	uint32_t		MAV_SYS_STATUS_SENSOR	Bitmap showing which onboard controllers and sensors are enabled: Value of 0: not enabled. Value of 1: enabled.
# onboard_control_sensors_health	uint32_t		MAV_SYS_STATUS_SENSOR	Bitmap showing which onboard controllers and sensors have an error (or are operational). Value of 0: error. Value of 1: healthy.
# load	uint16_t	d%		Maximum usage in percent of the mainloop time. Values: [0-1000] - should always be below 1000
# voltage_battery	uint16_t	mV	invalid:UINT16_MAX	Battery voltage, UINT16_MAX: Voltage not sent by autopilot
# current_battery	int16_t	cA	invalid:-1	Battery current, -1: Current not sent by autopilot
# battery_remaining	int8_t	%	invalid:-1	Battery energy remaining, -1: Battery remaining energy not sent by autopilot
# drop_rate_comm	uint16_t	c%		Communication drop rate, (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
# errors_comm	uint16_t			Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
# errors_count1	uint16_t			Autopilot-specific errors
# errors_count2	uint16_t			Autopilot-specific errors
# errors_count3	uint16_t			Autopilot-specific errors
# errors_count4	uint16_t			Autopilot-specific errors
# onboard_control_sensors_present_extended ++	uint32_t		MAV_SYS_STATUS_SENSOR_EXTENDED	Bitmap showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present.
# onboard_control_sensors_enabled_extended ++	uint32_t		MAV_SYS_STATUS_SENSOR_EXTENDED	Bitmap showing which onboard controllers and sensors are enabled: Value of 0: not enabled. Value of 1: enabled.
# onboard_control_sensors_health_extended ++	uint32_t		MAV_SYS_STATUS_SENSOR_EXTENDED	Bitmap showing which onboard controllers and sensors have an error (or are operational). Value of 0: error. Value of 1: healthy.



def fetch_terrain_report():
    return {
        "lat":None,
        "lon":None,
        "spacing":None,
        "terrain_height":None,
        "current_height":None,
        "pending":None,
        "loaded":None
    }

def fetch_vfr_hud():
    return {
        "airspeed":0,
        "groundspeed":0,
        "heading":0,
        "throttle":0,
        "alt":0,
        "climb":0
    }

# airspeed	float	m/s	Vehicle speed in form appropriate for vehicle type. For standard aircraft this is typically calibrated airspeed (CAS) or indicated airspeed (IAS) - either of which can be used by a pilot to estimate stall speed.
# groundspeed	float	m/s	Current ground speed.
# heading	int16_t	deg	Current heading in compass units (0-360, 0=north).
# throttle	uint16_t	%	Current throttle setting (0 to 100).
# alt	float	m	Current altitude (MSL).
# climb	float	m/s	Current climb rate.

def fetch_vibration():
    return {
        "vibration_x":0,
        "vibration_y":0,
        "vibration_z":0,
        "clipping_0":0,
        "clipping_1":0,
        "clipping_2":0,
    }



# vibration_x	float		Vibration levels on X-axis
# vibration_y	float		Vibration levels on Y-axis
# vibration_z	float		Vibration levels on Z-axis
# clipping_0	uint32_t		first accelerometer clipping count
# clipping_1	uint32_t		second accelerometer clipping count
# clipping_2	uint32_t		third accelerometer clipping count