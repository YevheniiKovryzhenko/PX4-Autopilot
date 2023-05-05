uint64 timestamp	# time since system start (microseconds)


####### actuator_armed #######
bool armed		# Set to true if system is armed
bool prearmed		# Set to true if the actuator safety is disabled but motors are not armed
bool ready_to_arm	# Set to true if system is ready to be armed
bool lockdown		# Set to true if actuators are forced to being disabled (due to emergency or HIL)
bool manual_lockdown    # Set to true if manual throttle kill switch is engaged
bool force_failsafe	# Set to true if the actuators are forced to the failsafe position
bool in_esc_calibration_mode # IO/FMU should ignore messages from the actuator controls topics
bool soft_stop # Set to true if we need to ESCs to remove the idle constraint

####### airspeed #######
float32 air_indicated_airspeed_m_s		# indicated airspeed in m/s

float32 air_true_airspeed_m_s		# true filtered airspeed in m/s

float32 air_temperature_celsius		# air temperature in degrees celsius, -1000 if unknown

float32 air_confidence			# confidence value from 0 to 1 for this sensor

####### adc report #######
uint32 adc_device_id		# unique device ID for the sensor that does not change between power cycles
int16[12] adc_channel_id		# ADC channel IDs, negative for non-existent, TODO: should be kept same as array index
int32[12] adc_raw_data		# ADC channel raw value, accept negative value, valid if channel ID is positive
uint32 adc_resolution		# ADC channel resolution
float32 adc_v_ref			# ADC channel voltage reference, use to calculate LSB voltage(lsb=scale/resolution)

####### battery status #######
bool bat_connected				# Whether or not a battery is connected, based on a voltage threshold
float32 bat_voltage_v			# Battery voltage in volts, 0 if unknown
float32 bat_voltage_filtered_v	# Battery voltage in volts, filtered, 0 if unknown
float32 bat_current_a			# Battery current in amperes, -1 if unknown
float32 bat_current_filtered_a	# Battery current in amperes, filtered, 0 if unknown
float32 bat_current_average_a	# Battery current average in amperes, -1 if unknown
float32 bat_discharged_mah		# Discharged amount in mAh, -1 if unknown
float32 bat_remaining			# From 1 to 0, -1 if unknown
float32 bat_scale				# Power scaling factor, >= 1, or -1 if unknown
float32 bat_temperature			# temperature of the battery. NaN if unknown
int32 bat_cell_count			# Number of cells

#uint8 BATTERY_SOURCE_POWER_MODULE = 0
#uint8 BATTERY_SOURCE_EXTERNAL = 1
#uint8 BATTERY_SOURCE_ESCS = 2
#uint8 bat_source				# Battery source
#uint8 bat_priority				# Zero based priority is the connection on the Power Controller V1..Vn AKA BrickN-1
#uint16 bat_capacity				# actual capacity of the battery
#uint16 bat_cycle_count			# number of discharge cycles the battery has experienced
#uint16 bat_run_time_to_empty	# predicted remaining battery capacity based on the present rate of discharge in min
#uint16 bat_average_time_to_empty	# predicted remaining battery capacity based on the average rate of discharge in min
#uint16 bat_serial_number		# serial number of the battery pack
#uint16 bat_manufacture_date		# manufacture date, part of serial number of the battery pack. formated as: Day + Month×32 + (Year–1980)×512
#uint16 bat_state_of_health		# state of health. FullChargeCapacity/DesignCapacity.
#uint16 bat_max_error			# max error, expected margin of error in % in the state-of-charge calculation with a range of 1 to 100%
#uint8 bat_id					# ID number of a battery. Should be unique and consistent for the lifetime of a vehicle. 1-indexed.
#uint16 bat_interface_error		# interface error counter

#float32[14] bat_voltage_cell_v		# Battery individual cell voltages
#float32 bat_max_cell_voltage_delta	# Max difference between individual cell voltages

#bool bat_is_powering_off			# Power off event imminent indication, false if unknown


#uint8 BATTERY_WARNING_NONE = 0		# no battery low voltage warning active
#uint8 BATTERY_WARNING_LOW = 1		# warning of low voltage
#uint8 BATTERY_WARNING_CRITICAL = 2	# critical voltage, return / abort immediately
#uint8 BATTERY_WARNING_EMERGENCY = 3	# immediate landing required
#uint8 BATTERY_WARNING_FAILED = 4	# the battery has failed completely

#uint8 bat_warning						# current battery warning


#uint8 MAX_INSTANCES = 4

#float32 bat_average_power               # The average power of the current discharge
#float32 bat_available_energy            # The predicted charge or energy remaining in the battery
#float32 bat_remaining_capacity          # The compensated battery capacity remaining
#float32 bat_design_capacity             # The design capacity of the battery
#uint16 bat_average_time_to_full         # The predicted remaining time until the battery reaches full charge, in minutes
#uint16 bat_over_discharge_count         # Number of battery overdischarge
#float32 bat_nominal_voltage             # Nominal voltage of the battery pack

####### Distance Sensor #######
# DISTANCE_SENSOR message data

#uint32 dist_device_id		# unique device ID for the sensor that does not change between power cycles

#float32 dist_min_distance		# Minimum distance the sensor can measure (in m)
#float32 dist_max_distance		# Maximum distance the sensor can measure (in m)
float32 dist_current_distance	# Current distance reading (in m)
#float32 dist_variance		# Measurement variance (in m^2), 0 for unknown / invalid readings
int8 dist_signal_quality		# Signal quality in percent (0...100%), where 0 = invalid signal, 100 = perfect signal, and -1 = unknown signal quality.

#uint8 dist_type			# Type from MAV_DISTANCE_SENSOR enum
#uint8 MAV_DISTANCE_SENSOR_LASER = 0
#uint8 MAV_DISTANCE_SENSOR_ULTRASOUND = 1
#uint8 MAV_DISTANCE_SENSOR_INFRARED = 2
#uint8 MAV_DISTANCE_SENSOR_RADAR = 3

#float32 dist_h_fov # Sensor horizontal field of view (rad)
#float32 dist_v_fov # Sensor vertical field of view (rad)
#float32[4] dist_q # Quaterion sensor orientation with respect to the vehicle body frame to specify the orientation ROTATION_CUSTOM

#uint8 dist_orientation		# Direction the sensor faces from MAV_SENSOR_ORIENTATION enum

#uint8 ROTATION_YAW_0		= 0 # MAV_SENSOR_ROTATION_NONE
#uint8 ROTATION_YAW_45		= 1 # MAV_SENSOR_ROTATION_YAW_45
#uint8 ROTATION_YAW_90		= 2 # MAV_SENSOR_ROTATION_YAW_90
#uint8 ROTATION_YAW_135		= 3 # MAV_SENSOR_ROTATION_YAW_135
#uint8 ROTATION_YAW_180		= 4 # MAV_SENSOR_ROTATION_YAW_180
#uint8 ROTATION_YAW_225		= 5 # MAV_SENSOR_ROTATION_YAW_225
#uint8 ROTATION_YAW_270		= 6 # MAV_SENSOR_ROTATION_YAW_270
#uint8 ROTATION_YAW_315		= 7 # MAV_SENSOR_ROTATION_YAW_315

#uint8 ROTATION_FORWARD_FACING	= 0 # MAV_SENSOR_ROTATION_NONE
#uint8 ROTATION_RIGHT_FACING	= 2 # MAV_SENSOR_ROTATION_YAW_90
#uint8 ROTATION_BACKWARD_FACING	= 4 # MAV_SENSOR_ROTATION_YAW_180
#uint8 ROTATION_LEFT_FACING	= 6 # MAV_SENSOR_ROTATION_YAW_270

#uint8 ROTATION_UPWARD_FACING   = 24 # MAV_SENSOR_ROTATION_PITCH_90
#uint8 ROTATION_DOWNWARD_FACING = 25 # MAV_SENSOR_ROTATION_PITCH_270

#uint8 ROTATION_CUSTOM          = 100 # MAV_SENSOR_ROTATION_CUSTOM


####### obstacle distance #######
# Obstacle distances in front of the sensor.

#uint8 obs_frame		#Coordinate frame of reference for the yaw rotation and offset of the sensor data. Defaults to MAV_FRAME_GLOBAL, which is North aligned. For body-mounted sensors use MAV_FRAME_BODY_FRD, which is vehicle front aligned.
#uint8 MAV_FRAME_GLOBAL = 0
#uint8 MAV_FRAME_LOCAL_NED = 1
#uint8 MAV_FRAME_BODY_FRD = 12

#uint8 obs_sensor_type # Type from MAV_DISTANCE_SENSOR enum.
#uint8 MAV_DISTANCE_SENSOR_LASER = 0
#uint8 MAV_DISTANCE_SENSOR_ULTRASOUND = 1
#uint8 MAV_DISTANCE_SENSOR_INFRARED = 2
#uint8 MAV_DISTANCE_SENSOR_RADAR = 3

#uint16[72] obs_distances # Distance of obstacles around the UAV with index 0 corresponding to local North. A value of 0 means that the obstacle is right in front of the sensor. A value of max_distance +1 means no obstacle is present. A value of UINT16_MAX for unknown/not used. In a array element, one unit corresponds to 1cm.

#float32 obs_increment # Angular width in degrees of each array element.

#uint16 obs_min_distance # Minimum distance the sensor can measure in centimeters.
#uint16 obs_max_distance # Maximum distance the sensor can measure in centimeters.

#float32 obs_angle_offset # Relative angle offset of the 0-index element in the distances array. Value of 0 corresponds to forward. Positive values are offsets to the right.

##### topi cs obstacle_distance obstacle_distance_fused


####### rc channels #######
#uint8 FUNCTION_THROTTLE   = 0
#uint8 FUNCTION_ROLL       = 1
#uint8 FUNCTION_PITCH      = 2
#uint8 FUNCTION_YAW        = 3
#uint8 FUNCTION_MODE       = 4
#uint8 FUNCTION_RETURN     = 5
#uint8 FUNCTION_POSCTL     = 6
#uint8 FUNCTION_LOITER     = 7
#uint8 FUNCTION_OFFBOARD   = 8
#uint8 FUNCTION_ACRO       = 9
#uint8 FUNCTION_FLAPS      = 10
#uint8 FUNCTION_AUX_1      = 11
#uint8 FUNCTION_AUX_2      = 12
#uint8 FUNCTION_AUX_3      = 13
#uint8 FUNCTION_AUX_4      = 14
#uint8 FUNCTION_AUX_5      = 15
#uint8 FUNCTION_PARAM_1    = 16
#uint8 FUNCTION_PARAM_2    = 17
#uint8 FUNCTION_PARAM_3_5  = 18
#uint8 FUNCTION_KILLSWITCH = 19
#uint8 FUNCTION_TRANSITION = 20
#uint8 FUNCTION_GEAR       = 21
#uint8 FUNCTION_ARMSWITCH  = 22
#uint8 FUNCTION_STAB       = 23
#uint8 FUNCTION_AUX_6      = 24
#uint8 FUNCTION_MAN        = 25

#uint64 rc_timestamp_last_valid					# Timestamp of last valid RC signal
#float32[18] rc_channels						# Scaled to -1..1 (throttle: 0..1)
#uint8 rc_channel_count						# Number of valid channels
#int8[26] rc_function						# Functions mapping
#uint8 rc_rssi							# Receive signal strength index
#bool rc_signal_lost						# Control signal lost, should be checked together with topic timeout
#uint32 rc_frame_drop_count						# Number of dropped frames


####### rc parameter map #######
#uint8 RC_PARAM_MAP_NCHAN = 3 # This limit is also hardcoded in the enum RC_CHANNELS_FUNCTION in rc_channels.h
#uint8 PARAM_ID_LEN = 16 # corresponds to MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN

#bool[3] rc_valid		#true for RC-Param channels which are mapped to a param
#int32[3] rc_param_index	# corresponding param index, this field is ignored if set to -1, in this case param_id will be used
#char[51] rc_param_id	# MAP_NCHAN * (ID_LEN + 1) chars, corresponding param id, null terminated
#float32[3] rc_scale		# scale to map the RC input [-1, 1] to a parameter value
#float32[3] rc_value0		# initial value around which the parameter value is changed
#float32[3] rc_value_min	# minimal parameter value
#float32[3] rc_value_max	# minimal parameter value


####### vehicle attitude #######
# This is similar to the mavlink message ATTITUDE_QUATERNION, but for onboard use

#uint64 veh_att_timestamp_sample     # the timestamp of the raw data (microseconds)

float32[4] veh_att_q			    # Quaternion rotation from the FRD body frame to the NED earth frame
#float32[4] veh_att_delta_q_reset 	# Amount by which quaternion has changed during last reset
#uint8 veh_att_quat_reset_counter	# Quaternion reset counter

##### topi cs vehicle_attitude vehicle_attitude_groundtruth vehicle_vision_attitude
##### topi cs estimator_attitude


####### vehicle global position #######
# Fused global position in WGS84.
# This struct contains global position estimation. It is not the raw GPS
# measurement (@see vehicle_gps_position). This topic is usually published by the position
# estimator, which will take more sources of information into account than just GPS,
# e.g. control inputs of the vehicle in a Kalman-filter implementation.
#

#uint64 veh_glob_timestamp_sample         # the timestamp of the raw data (microseconds)

float64 veh_glob_lat			# Latitude, (degrees)
float64 veh_glob_lon			# Longitude, (degrees)
float32 veh_glob_alt			# Altitude AMSL, (meters)
float32 veh_glob_alt_ellipsoid		# Altitude above ellipsoid, (meters)

#float32 veh_glob_delta_alt 	# Reset delta for altitude
#uint8 veh_glob_lat_lon_reset_counter	# Counter for reset events on horizontal position coordinates
#uint8 veh_glob_alt_reset_counter 	# Counter for reset events on altitude

#float32 veh_glob_eph			# Standard deviation of horizontal position error, (metres)
#float32 veh_glob_epv			# Standard deviation of vertical position error, (metres)

float32 veh_glob_terrain_alt		# Terrain altitude WGS84, (metres)
bool veh_glob_terrain_alt_valid		# Terrain altitude estimate is valid

bool veh_glob_dead_reckoning		# True if this position is estimated through dead-reckoning

##### topi cs vehicle_global_position vehicle_global_position_groundtruth
##### topi cs estimator_global_position



####### vehicle local position #######
# Fused local position in NED.

#uint64 veh_loc_timestamp_sample                 # the timestamp of the raw data (microseconds)

#bool veh_loc_xy_valid				# true if x and y are valid
#bool veh_loc_z_valid				# true if z is valid
#bool veh_loc_v_xy_valid				# true if vy and vy are valid
#bool veh_loc_v_z_valid				# true if vz is valid

# Position in local NED frame
float32 veh_loc_x				# North position in NED earth-fixed frame, (metres)
float32 veh_loc_y				# East position in NED earth-fixed frame, (metres)
float32 veh_loc_z				# Down position (negative altitude) in NED earth-fixed frame, (metres)

# Position reset delta
#float32[2] veh_loc_delta_xy
#uint8 veh_loc_xy_reset_counter

#float32 veh_loc_delta_z
#uint8 veh_loc_z_reset_counter

# Velocity in NED frame
float32 veh_loc_vx 				# North velocity in NED earth-fixed frame, (metres/sec)
float32 veh_loc_vy				# East velocity in NED earth-fixed frame, (metres/sec)
float32 veh_loc_vz				# Down velocity in NED earth-fixed frame, (metres/sec)
float32 veh_loc_z_deriv				# Down position time derivative in NED earth-fixed frame, (metres/sec)

# Velocity veh_loc_reset delta
#float32[2] veh_loc_delta_vxy
#uint8 veh_loc_vxy_reset_counter

#float32 veh_loc_delta_vz
#uint8 veh_loc_vz_reset_counter
# Acceleration in NED frame
float32 veh_loc_ax        # North velocity derivative in NED earth-fixed frame, (metres/sec^2)
float32 veh_loc_ay        # East velocity derivative in NED earth-fixed frame, (metres/sec^2)
float32 veh_loc_az        # Down velocity derivative in NED earth-fixed frame, (metres/sec^2)

float32 veh_loc_heading				# Euler yaw angle transforming the tangent plane relative to NED earth-fixed frame, -PI..+PI,  (radians)
#float32 veh_loc_delta_heading
#uint8 veh_loc_heading_reset_counter

# Position of reference point (local NED frame origin) in global (GPS / WGS84) frame
#bool veh_loc_xy_global				# true if position (x, y) has a valid global reference (ref_lat, ref_lon)
#bool veh_loc_z_global				# true if z has a valid global reference (ref_alt)
#uint64 veh_loc_ref_timestamp			# Time when reference position was set since system start, (microseconds)
#float64 veh_loc_ref_lat				# Reference point latitude, (degrees)
#float64 veh_loc_ref_lon				# Reference point longitude, (degrees)
#float32 veh_loc_ref_alt				# Reference altitude AMSL, (metres)

# Distance to surface
float32 veh_loc_dist_bottom			# Distance from from bottom surface to ground, (metres)
bool veh_loc_dist_bottom_valid			# true if distance to bottom surface is valid
#uint8 veh_loc_dist_bottom_sensor_bitfield	# bitfield indicating what type of sensor is used to estimate dist_bottom
#uint8 DIST_BOTTOM_SENSOR_NONE = 0
#uint8 DIST_BOTTOM_SENSOR_RANGE = 1	# (1 << 0) a range sensor is used to estimate dist_bottom field
#uint8 DIST_BOTTOM_SENSOR_FLOW = 2	# (1 << 1) a flow sensor is used to estimate dist_bottom field (mostly fixed-wing use case)

#float32 veh_loc_eph				# Standard deviation of horizontal position error, (metres)
#float32 veh_loc_epv				# Standard deviation of vertical position error, (metres)
#float32 veh_loc_evh				# Standard deviation of horizontal velocity error, (metres/sec)
#float32 veh_loc_evv				# Standard deviation of horizontal velocity error, (metres/sec)

# estimator specified vehicle limits
#float32 veh_loc_vxy_max				# maximum horizontal speed - set to 0 when limiting not required (meters/sec)
#float32 veh_loc_vz_max				# maximum vertical speed - set to 0 when limiting not required (meters/sec)
#float32 veh_loc_hagl_min			# minimum height above ground level - set to 0 when limiting not required (meters)
#float32 veh_loc_hagl_max			# maximum height above ground level - set to 0 when limiting not required (meters)

##### topi cs vehicle_local_position vehicle_local_position_groundtruth
##### topi cs estimator_local_position



####### vehicle odometry #######
# Vehicle odometry data. Fits ROS REP 147 for aerial vehicles

#uint64 odo_timestamp_sample

# Covariance matrix index constants
#uint8 COVARIANCE_MATRIX_X_VARIANCE=0
#uint8 COVARIANCE_MATRIX_Y_VARIANCE=6
#uint8 COVARIANCE_MATRIX_Z_VARIANCE=11
#uint8 COVARIANCE_MATRIX_ROLL_VARIANCE=15
#uint8 COVARIANCE_MATRIX_PITCH_VARIANCE=18
#uint8 COVARIANCE_MATRIX_YAW_VARIANCE=20
#uint8 COVARIANCE_MATRIX_VX_VARIANCE=0
#uint8 COVARIANCE_MATRIX_VY_VARIANCE=6
#uint8 COVARIANCE_MATRIX_VZ_VARIANCE=11
#uint8 COVARIANCE_MATRIX_ROLLRATE_VARIANCE=15
#uint8 COVARIANCE_MATRIX_PITCHRATE_VARIANCE=18
#uint8 COVARIANCE_MATRIX_YAWRATE_VARIANCE=20

# Position and linear velocity frame of reference constants
#uint8 LOCAL_FRAME_NED=0         # NED earth-fixed frame
#uint8 LOCAL_FRAME_FRD=1         # FRD earth-fixed frame, arbitrary heading reference
#uint8 LOCAL_FRAME_OTHER=2       # Not aligned with the std frames of reference
#uint8 BODY_FRAME_FRD=3          # FRD body-fixed frame

# Position and linear velocity local frame of reference
#uint8 odo_local_frame

# Position in meters. Frame of reference defined by local_frame. NaN if invalid/unknown
#float32 odo_x			# North position
#float32 odo_y			# East position
#float32 odo_z			# Down position

# Orientation quaternion. First value NaN if invalid/unknown
float32[4] odo_q			# Quaternion rotation from FRD body frame to refernce frame
float32[4] odo_q_offset		# Quaternion rotation from odometry reference frame to navigation frame

# Row-major representation of 6x6 pose cross-covariance matrix URT.
# NED earth-fixed frame.
# Order: x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis
# If position covariance invalid/unknown, first cell is NaN
# If orientation covariance invalid/unknown, 16th cell is NaN
#float32[21] odo_pose_covariance

# Reference frame of the velocity data
#uint8 odo_velocity_frame

# Velocity in meters/sec. Frame of reference defined by velocity_frame variable. NaN if invalid/unknown
#float32 odo_vx 			# North velocity
#float32 odo_vy			# East velocity
#float32 odo_vz			# Down velocity

# Angular rate in body-fixed frame (rad/s). NaN if invalid/unknown
float32 odo_rollspeed		# Angular velocity about X body axis
float32 odo_pitchspeed		# Angular velocity about Y body axis
float32 odo_yawspeed		# Angular velocity about Z body axis

# Row-major representation of 6x6 velocity cross-covariance matrix URT.
# Linear velocity in NED earth-fixed frame. Angular velocity in body-fixed frame.
# Order: vx, vy, vz, rotation rate about X axis, rotation rate about Y axis, rotation rate about Z axis
# If linear velocity covariance invalid/unknown, first cell is NaN
# If angular velocity covariance invalid/unknown, 16th cell is NaN
#float32[21] odo_velocity_covariance

##### topi cs vehicle_odometry vehicle_mocap_odometry vehicle_visual_odometry
##### topi cs estimator_odometry estimator_visual_odometry_aligned
