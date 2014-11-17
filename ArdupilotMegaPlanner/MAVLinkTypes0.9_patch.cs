    //                                                 {  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127,128,129,130,131,132,133,134,135,136,137,138,139,140,141,142,143,144,145,146,147,148,149,150,151,152,153,154,155,156,157,158,159,160,161,162,163,164,165,166,167,168,169,170,171,172,173,174,175,176,177,178,179,180,181,182,183,184,185,186,187,188,189,190,191,192,193,194,195,196,197,198,199,200,201,202,203,204,205,206,207,208,209,210,211,212,213,214,215,216,217,218,219,220,221,222,223,224,225,226,227,228,229,230,231,232,233,234,235,236,237,238,239,240,241,242,243,244,245,246,247,248,249,250,251,252,253,254,255}
    public byte[] MAVLINK_MESSAGE_LENGTHS = new byte[] {  5,  4,  8, 14,  8, 28,  3, 32,  0,  2,  3,  2,  2,  0,  0,  0,  0,  0,  0,  0, 19,  2, 23, 21,  0, 37, 26,101, 26, 16, 32, 32, 37, 32, 11, 17, 17, 16, 18, 36,  4,  4,  2,  2,  4,  2,  2,  3, 14, 12, 18, 16,  8, 27, 25, 18, 18, 24, 24,  0,  0,  0, 26, 16, 36,  5,  6, 56, 26, 21, 18,  0,  0, 18, 20, 20,  8,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 18,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 40, 72, 12, 33, 11,  8,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 36,  0,  0,  0,  0,  0,  0,  0,  0,  0, 42,  8,  4, 12, 15, 13,  6, 15, 14,  0, 12,  3,  8,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 30, 14, 14, 51,  5};
	public const byte MAVLINK_MSG_ID_ANALOG_RAW = 111;
	public const byte MAVLINK_MSG_ID_ANALOG_RAW_LEN = 40;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_analog_raw_t
	{
		public ushort chan01; /// adc channel_01
		public ushort chan02; /// adc channel_02
		public ushort chan03; /// adc channel_03
		public ushort chan04; /// adc channel_04
		public ushort chan05; /// adc channel_05
		public ushort chan06; /// adc channel_06
		public ushort chan07; /// adc channel_07
		public ushort chan08; /// adc channel_08
		public ushort chan09; /// adc channel_09
		public ushort chan10; /// adc channel_10
		public ushort chan11; /// adc channel_11
		public ushort chan12; /// adc channel_12
		public ushort chan13; /// adc channel_13
		public ushort chan14; /// adc channel_14
		public ushort chan15; /// adc channel_15
		public ushort chan16; /// adc channel_16
		public ulong usec; /// time
	};

	public const byte MAVLINK_MSG_ID_ANALOG_EU = 112;
	public const byte MAVLINK_MSG_ID_ANALOG_EU_LEN = 72;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_analog_eu_t
	{
		public float chan01; /// adc channel_01
		public float chan02; /// adc channel_02
		public float chan03; /// adc channel_03
		public float chan04; /// adc channel_04
		public float chan05; /// adc channel_05
		public float chan06; /// adc channel_06
		public float chan07; /// adc channel_07
		public float chan08; /// adc channel_08
		public float chan09; /// adc channel_09
		public float chan10; /// adc channel_10
		public float chan11; /// adc channel_11
		public float chan12; /// adc channel_12
		public float chan13; /// adc channel_13
		public float chan14; /// adc channel_14
		public float chan15; /// adc channel_15
		public float chan16; /// adc channel_16
		public ulong usec; /// time
	};

	public const byte MAVLINK_MSG_ID_CDNR_CONTROLLER = 113;
	public const byte MAVLINK_MSG_ID_CDNR_CONTROLLER_LEN = 12;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_cdnr_controller_t
	{
		public byte h_flag; /// enables/disables new heading command
		public byte s_flag; /// enables/disables new airspeed command
		public byte a_flag; /// enables/disables new altitude command
		public byte t_flag; /// enables/disables max time command
		public short new_heading; /// value for new heading
		public short new_airspeed; /// value for new airspeed
		public short new_altitude; /// value for new altitude
		public short max_time; /// maximum time to hold this command
	};

	public const byte MAVLINK_MSG_ID_TRAFFIC_DATA = 114;
	public const byte MAVLINK_MSG_ID_TRAFFIC_DATA_LEN = 33;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_traffic_data_t
	{
		[MarshalAs(
			UnmanagedType.ByValArray,
			SizeConst=8)]
		public byte[] ID; /// vehicle ID
		public float lat; /// latitude (deg)
		public float lon; /// longitude (deg)
		public float alt; /// altitude (meters)
		public float roll; /// roll (deg)
		public float pitch; /// pitch (deg)
		public float heading; /// heading (deg)
		public byte ownship_flag; /// identifies whether the aircraft is ownship (1) or not (0)
	};

	public const byte MAVLINK_MSG_ID_STATE_DATA = 115;
	public const byte MAVLINK_MSG_ID_STATE_DATA_LEN = 11;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_state_data_t
	{
		public byte num; /// Vehicle number, e.g. 1 for R1, 2 for R2, 3 for R3
		public byte atloiter; /// if 0, vehicle is not currently loitering, if 1, vehicle is loitering
		public byte mode; /// mode: MANUAL=0, CIRCLE=1, STABILIZE=2, FLY_BY_WIRE_A=5, FLY_BY_WIRE_B=6, FLY_BY_WIRE_C=7, AUTO=10, RTL=11, LOITER=12, GUIDED=15, INITIALISING=16
		public ulong usec; /// time
	};

	public const byte MAVLINK_MSG_ID_TRAFFIC_SIM_TIMING = 116;
	public const byte MAVLINK_MSG_ID_TRAFFIC_SIM_TIMING_LEN = 8;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_traffic_sim_timing_t
	{
		public float sim_time; /// Traffic simulation absolute time
		public float time_to_wpt; /// Absolute time to reach the next waypoint
	};

	public const byte MAVLINK_MSG_ID_HEARTBEAT = 0;
	public const byte MAVLINK_MSG_ID_HEARTBEAT_LEN = 5;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_heartbeat_t
	{
		public byte type; /// Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
		public byte autopilot; /// Type of the Autopilot: 0: Generic, 1: PIXHAWK, 2: SLUGS, 3: Ardupilot (up to 15 types), defined in MAV_AUTOPILOT_TYPE ENUM
		public byte mavlink_version; /// MAVLink version
		public byte vehicleID; /// Vehicle ID: 1: R1, 2: R2, 3: R3 (up to xx types), defined in "mydefines.h"
		public byte numWpts; /// Number of Waypoints in Flight Plan
	};

	public const byte MAVLINK_MSG_ID_BOOT = 1;
	public const byte MAVLINK_MSG_ID_BOOT_LEN = 4;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_boot_t
	{
		public uint version; /// The onboard software version
	};

	public const byte MAVLINK_MSG_ID_SYSTEM_TIME = 2;
	public const byte MAVLINK_MSG_ID_SYSTEM_TIME_LEN = 8;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_system_time_t
	{
		public ulong time_usec; /// Timestamp of the master clock in microseconds since UNIX epoch.
	};

	public const byte MAVLINK_MSG_ID_PING = 3;
	public const byte MAVLINK_MSG_ID_PING_LEN = 14;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_ping_t
	{
		public uint seq; /// PING sequence
		public byte target_system; /// 0: request ping from all receiving systems, if greater than 0: message is a ping response and number is the system id of the requesting system
		public byte target_component; /// 0: request ping from all receiving components, if greater than 0: message is a ping response and number is the system id of the requesting system
		public ulong time; /// Unix timestamp in microseconds
	};

	public const byte MAVLINK_MSG_ID_SYSTEM_TIME_UTC = 4;
	public const byte MAVLINK_MSG_ID_SYSTEM_TIME_UTC_LEN = 8;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_system_time_utc_t
	{
		public uint utc_date; /// GPS UTC date ddmmyy
		public uint utc_time; /// GPS UTC time hhmmss
	};

	public const byte MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL = 5;
	public const byte MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_LEN = 28;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_change_operator_control_t
	{
		public byte target_system; /// System the GCS requests control for
		public byte control_request; /// 0: request control of this MAV, 1: Release control of this MAV
		public byte version; /// 0: key as plaintext, 1-255: future, different hashing/encryption variants. The GCS should in general use the safest mode possible initially and then gradually move down the encryption level if it gets a NACK message indicating an encryption mismatch.
		[MarshalAs(
			UnmanagedType.ByValArray,
			SizeConst=25)]
		public byte[] passkey; /// Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The characters may involve A-Z, a-z, 0-9, and "!?,.-"
	};

	public const byte MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK = 6;
	public const byte MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK_LEN = 3;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_change_operator_control_ack_t
	{
		public byte gcs_system_id; /// ID of the GCS this message 
		public byte control_request; /// 0: request control of this MAV, 1: Release control of this MAV
		public byte ack; /// 0: ACK, 1: NACK: Wrong passkey, 2: NACK: Unsupported passkey encryption method, 3: NACK: Already under control
	};

	public const byte MAVLINK_MSG_ID_AUTH_KEY = 7;
	public const byte MAVLINK_MSG_ID_AUTH_KEY_LEN = 32;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_auth_key_t
	{
		[MarshalAs(
			UnmanagedType.ByValArray,
			SizeConst=32)]
		public byte[] key; /// key
	};

	public const byte MAVLINK_MSG_ID_ACTION_ACK = 9;
	public const byte MAVLINK_MSG_ID_ACTION_ACK_LEN = 2;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_action_ack_t
	{
		public byte action; /// The action id
		public byte result; /// 0: Action DENIED, 1: Action executed
	};

	public const byte MAVLINK_MSG_ID_ACTION = 10;
	public const byte MAVLINK_MSG_ID_ACTION_LEN = 3;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_action_t
	{
		public byte target; /// The system executing the action
		public byte target_component; /// The component executing the action
		public byte action; /// The action id
	};

	public const byte MAVLINK_MSG_ID_SET_MODE = 11;
	public const byte MAVLINK_MSG_ID_SET_MODE_LEN = 2;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_set_mode_t
	{
		public byte target; /// The system setting the mode
		public byte mode; /// The new mode
	};

	public const byte MAVLINK_MSG_ID_SET_NAV_MODE = 12;
	public const byte MAVLINK_MSG_ID_SET_NAV_MODE_LEN = 2;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_set_nav_mode_t
	{
		public byte target; /// The system setting the mode
		public byte nav_mode; /// The new navigation mode
	};

	public const byte MAVLINK_MSG_ID_PARAM_REQUEST_READ = 20;
	public const byte MAVLINK_MSG_ID_PARAM_REQUEST_READ_LEN = 19;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_param_request_read_t
	{
		public byte target_system; /// System ID
		public byte target_component; /// Component ID
		[MarshalAs(
			UnmanagedType.ByValArray,
			SizeConst=15)]
		public byte[] param_id; /// Onboard parameter id
		public short param_index; /// Parameter index. Send -1 to use the param ID field as identifier
	};

	public const byte MAVLINK_MSG_ID_PARAM_REQUEST_LIST = 21;
	public const byte MAVLINK_MSG_ID_PARAM_REQUEST_LIST_LEN = 2;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_param_request_list_t
	{
		public byte target_system; /// System ID
		public byte target_component; /// Component ID
	};

	public const byte MAVLINK_MSG_ID_PARAM_VALUE = 22;
	public const byte MAVLINK_MSG_ID_PARAM_VALUE_LEN = 23;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_param_value_t
	{
		[MarshalAs(
			UnmanagedType.ByValArray,
			SizeConst=15)]
		public byte[] param_id; /// Onboard parameter id
		public float param_value; /// Onboard parameter value
		public ushort param_count; /// Total number of onboard parameters
		public ushort param_index; /// Index of this onboard parameter
	};

	public const byte MAVLINK_MSG_ID_PARAM_SET = 23;
	public const byte MAVLINK_MSG_ID_PARAM_SET_LEN = 21;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_param_set_t
	{
		public byte target_system; /// System ID
		public byte target_component; /// Component ID
		[MarshalAs(
			UnmanagedType.ByValArray,
			SizeConst=15)]
		public byte[] param_id; /// Onboard parameter id
		public float param_value; /// Onboard parameter value
	};

	public const byte MAVLINK_MSG_ID_GPS_RAW_INT = 25;
	public const byte MAVLINK_MSG_ID_GPS_RAW_INT_LEN = 37;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_gps_raw_int_t
	{
		public ulong usec; /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
		public byte fix_type; /// 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
		public int lat; /// Latitude in 1E7 degrees
		public int lon; /// Longitude in 1E7 degrees
		public int alt; /// Altitude in 1E3 meters (millimeters)
		public float eph; /// GPS HDOP
		public float epv; /// GPS VDOP
		public float v; /// GPS ground speed (m/s)
		public float hdg; /// Compass heading in degrees, 0..360 degrees
	};

	public const byte MAVLINK_MSG_ID_SCALED_IMU = 26;
	public const byte MAVLINK_MSG_ID_SCALED_IMU_LEN = 26;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_scaled_imu_t
	{
		public ulong usec; /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
		public short xacc; /// X acceleration (mg)
		public short yacc; /// Y acceleration (mg)
		public short zacc; /// Z acceleration (mg)
		public short xgyro; /// Angular speed around X axis (millirad /sec)
		public short ygyro; /// Angular speed around Y axis (millirad /sec)
		public short zgyro; /// Angular speed around Z axis (millirad /sec)
		public short xmag; /// X Magnetic field (milli tesla)
		public short ymag; /// Y Magnetic field (milli tesla)
		public short zmag; /// Z Magnetic field (milli tesla)
	};

	public const byte MAVLINK_MSG_ID_GPS_STATUS = 27;
	public const byte MAVLINK_MSG_ID_GPS_STATUS_LEN = 101;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_gps_status_t
	{
		public byte satellites_visible; /// Number of satellites visible
		[MarshalAs(
			UnmanagedType.ByValArray,
			SizeConst=20)]
		public byte[] satellite_prn; /// Global satellite ID
		[MarshalAs(
			UnmanagedType.ByValArray,
			SizeConst=20)]
		public byte[] satellite_used; /// 0: Satellite not used, 1: used for localization
		[MarshalAs(
			UnmanagedType.ByValArray,
			SizeConst=20)]
		public byte[] satellite_elevation; /// Elevation (0: right on top of receiver, 90: on the horizon) of satellite
		[MarshalAs(
			UnmanagedType.ByValArray,
			SizeConst=20)]
		public byte[] satellite_azimuth; /// Direction of satellite, 0: 0 deg, 255: 360 deg.
		[MarshalAs(
			UnmanagedType.ByValArray,
			SizeConst=20)]
		public byte[] satellite_snr; /// Signal to noise ratio of satellite
	};

	public const byte MAVLINK_MSG_ID_RAW_IMU = 28;
	public const byte MAVLINK_MSG_ID_RAW_IMU_LEN = 26;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_raw_imu_t
	{
		public ulong usec; /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
		public short xacc; /// X acceleration (raw)
		public short yacc; /// Y acceleration (raw)
		public short zacc; /// Z acceleration (raw)
		public short xgyro; /// Angular speed around X axis (raw)
		public short ygyro; /// Angular speed around Y axis (raw)
		public short zgyro; /// Angular speed around Z axis (raw)
		public short xmag; /// X Magnetic field (raw)
		public short ymag; /// Y Magnetic field (raw)
		public short zmag; /// Z Magnetic field (raw)
	};

	public const byte MAVLINK_MSG_ID_RAW_PRESSURE = 29;
	public const byte MAVLINK_MSG_ID_RAW_PRESSURE_LEN = 16;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_raw_pressure_t
	{
		public ulong usec; /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
		public short press_abs; /// Absolute pressure (raw)
		public short press_diff1; /// Differential pressure 1 (raw)
		public short press_diff2; /// Differential pressure 2 (raw)
		public short temperature; /// Raw Temperature measurement (raw)
	};

	public const byte MAVLINK_MSG_ID_SCALED_PRESSURE = 38;
	public const byte MAVLINK_MSG_ID_SCALED_PRESSURE_LEN = 18;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_scaled_pressure_t
	{
		public ulong usec; /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
		public float press_abs; /// Absolute pressure (hectopascal)
		public float press_diff; /// Differential pressure 1 (hectopascal)
		public short temperature; /// Temperature measurement (0.01 degrees celsius)
	};

	public const byte MAVLINK_MSG_ID_ATTITUDE = 30;
	public const byte MAVLINK_MSG_ID_ATTITUDE_LEN = 32;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_attitude_t
	{
		public ulong usec; /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
		public float roll; /// Roll angle (rad)
		public float pitch; /// Pitch angle (rad)
		public float yaw; /// Yaw angle (rad)
		public float rollspeed; /// Roll angular speed (rad/s)
		public float pitchspeed; /// Pitch angular speed (rad/s)
		public float yawspeed; /// Yaw angular speed (rad/s)
	};

	public const byte MAVLINK_MSG_ID_LOCAL_POSITION = 31;
	public const byte MAVLINK_MSG_ID_LOCAL_POSITION_LEN = 32;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_local_position_t
	{
		public ulong usec; /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
		public float x; /// X Position
		public float y; /// Y Position
		public float z; /// Z Position
		public float vx; /// X Speed
		public float vy; /// Y Speed
		public float vz; /// Z Speed
	};

	public const byte MAVLINK_MSG_ID_GLOBAL_POSITION = 33;
	public const byte MAVLINK_MSG_ID_GLOBAL_POSITION_LEN = 32;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_global_position_t
	{
		public ulong usec; /// Timestamp (microseconds since unix epoch)
		public float lat; /// Latitude, in degrees
		public float lon; /// Longitude, in degrees
		public float alt; /// Absolute altitude, in meters
		public float vx; /// X Speed (in Latitude direction, positive: going north)
		public float vy; /// Y Speed (in Longitude direction, positive: going east)
		public float vz; /// Z Speed (in Altitude direction, positive: going up)
	};

	public const byte MAVLINK_MSG_ID_GPS_RAW = 32;
	public const byte MAVLINK_MSG_ID_GPS_RAW_LEN = 37;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_gps_raw_t
	{
		public ulong usec; /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
		public byte fix_type; /// 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
		public float lat; /// Latitude in degrees
		public float lon; /// Longitude in degrees
		public float alt; /// Altitude in meters
		public float eph; /// GPS HDOP
		public float epv; /// GPS VDOP
		public float v; /// GPS ground speed
		public float hdg; /// Compass heading in degrees, 0..360 degrees
	};

	public const byte MAVLINK_MSG_ID_SYS_STATUS = 34;
	public const byte MAVLINK_MSG_ID_SYS_STATUS_LEN = 11;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_sys_status_t
	{
		public byte mode; /// System mode, see MAV_MODE ENUM in mavlink/include/mavlink_types.h
		public byte nav_mode; /// Navigation mode, see MAV_NAV_MODE ENUM
		public byte status; /// System status flag, see MAV_STATUS ENUM
		public ushort load; /// Maximum usage in percent of the mainloop time, (0%%: 0, 100%%: 1000) should be always below 1000
		public ushort vbat; /// Battery voltage, in millivolts (1 = 1 millivolt)
		public ushort battery_remaining; /// Remaining battery energy: (0%%: 0, 100%%: 1000)
		public ushort packet_drop; /// Dropped packets (packets that were corrupted on reception on the MAV)
	};

	public const byte MAVLINK_MSG_ID_RC_CHANNELS_RAW = 35;
	public const byte MAVLINK_MSG_ID_RC_CHANNELS_RAW_LEN = 17;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_rc_channels_raw_t
	{
		public ushort chan1_raw; /// RC channel 1 value, in microseconds
		public ushort chan2_raw; /// RC channel 2 value, in microseconds
		public ushort chan3_raw; /// RC channel 3 value, in microseconds
		public ushort chan4_raw; /// RC channel 4 value, in microseconds
		public ushort chan5_raw; /// RC channel 5 value, in microseconds
		public ushort chan6_raw; /// RC channel 6 value, in microseconds
		public ushort chan7_raw; /// RC channel 7 value, in microseconds
		public ushort chan8_raw; /// RC channel 8 value, in microseconds
		public byte rssi; /// Receive signal strength indicator, 0: 0%%, 255: 100%%
	};

	public const byte MAVLINK_MSG_ID_RC_CHANNELS_SCALED = 36;
	public const byte MAVLINK_MSG_ID_RC_CHANNELS_SCALED_LEN = 17;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_rc_channels_scaled_t
	{
		public short chan1_scaled; /// RC channel 1 value scaled, (-100%%) -10000, (0%%) 0, (100%%) 10000
		public short chan2_scaled; /// RC channel 2 value scaled, (-100%%) -10000, (0%%) 0, (100%%) 10000
		public short chan3_scaled; /// RC channel 3 value scaled, (-100%%) -10000, (0%%) 0, (100%%) 10000
		public short chan4_scaled; /// RC channel 4 value scaled, (-100%%) -10000, (0%%) 0, (100%%) 10000
		public short chan5_scaled; /// RC channel 5 value scaled, (-100%%) -10000, (0%%) 0, (100%%) 10000
		public short chan6_scaled; /// RC channel 6 value scaled, (-100%%) -10000, (0%%) 0, (100%%) 10000
		public short chan7_scaled; /// RC channel 7 value scaled, (-100%%) -10000, (0%%) 0, (100%%) 10000
		public short chan8_scaled; /// RC channel 8 value scaled, (-100%%) -10000, (0%%) 0, (100%%) 10000
		public byte rssi; /// Receive signal strength indicator, 0: 0%%, 255: 100%%
	};

	public const byte MAVLINK_MSG_ID_SERVO_OUTPUT_RAW = 37;
	public const byte MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_LEN = 16;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_servo_output_raw_t
	{
		public ushort servo1_raw; /// Servo output 1 value, in microseconds
		public ushort servo2_raw; /// Servo output 2 value, in microseconds
		public ushort servo3_raw; /// Servo output 3 value, in microseconds
		public ushort servo4_raw; /// Servo output 4 value, in microseconds
		public ushort servo5_raw; /// Servo output 5 value, in microseconds
		public ushort servo6_raw; /// Servo output 6 value, in microseconds
		public ushort servo7_raw; /// Servo output 7 value, in microseconds
		public ushort servo8_raw; /// Servo output 8 value, in microseconds
	};

	public const byte MAVLINK_MSG_ID_WAYPOINT = 39;
	public const byte MAVLINK_MSG_ID_WAYPOINT_LEN = 36;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_waypoint_t
	{
		public byte target_system; /// System ID
		public byte target_component; /// Component ID
		public ushort seq; /// Sequence
		public byte frame; /// The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h
		public byte command; /// The scheduled action for the waypoint. see MAV_COMMAND in common.xml MAVLink specs
		public byte current; /// false:0, true:1
		public byte autocontinue; /// autocontinue to next wp
		public float param1; /// PARAM1 / For NAV command waypoints: Radius in which the waypoint is accepted as reached, in meters
		public float param2; /// PARAM2 / For NAV command waypoints: Time that the MAV should stay inside the PARAM1 radius before advancing, in milliseconds
		public float param3; /// PARAM3 / For LOITER command waypoints: Orbit to circle around the waypoint, in meters. If positive the orbit direction should be clockwise, if negative the orbit direction should be counter-clockwise.
		public float param4; /// PARAM4 / For NAV and LOITER command waypoints: Yaw orientation in degrees, [0..360] 0 = NORTH
		public float x; /// PARAM5 / local: x position, global: latitude
		public float y; /// PARAM6 / y position: global: longitude
		public float z; /// PARAM7 / z position: global: altitude
	};

	public const byte MAVLINK_MSG_ID_WAYPOINT_REQUEST = 40;
	public const byte MAVLINK_MSG_ID_WAYPOINT_REQUEST_LEN = 4;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_waypoint_request_t
	{
		public byte target_system; /// System ID
		public byte target_component; /// Component ID
		public ushort seq; /// Sequence
	};

	public const byte MAVLINK_MSG_ID_WAYPOINT_SET_CURRENT = 41;
	public const byte MAVLINK_MSG_ID_WAYPOINT_SET_CURRENT_LEN = 4;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_waypoint_set_current_t
	{
		public byte target_system; /// System ID
		public byte target_component; /// Component ID
		public ushort seq; /// Sequence
	};

	public const byte MAVLINK_MSG_ID_WAYPOINT_CURRENT = 42;
	public const byte MAVLINK_MSG_ID_WAYPOINT_CURRENT_LEN = 2;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_waypoint_current_t
	{
		public ushort seq; /// Sequence
	};

	public const byte MAVLINK_MSG_ID_WAYPOINT_REQUEST_LIST = 43;
	public const byte MAVLINK_MSG_ID_WAYPOINT_REQUEST_LIST_LEN = 2;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_waypoint_request_list_t
	{
		public byte target_system; /// System ID
		public byte target_component; /// Component ID
	};

	public const byte MAVLINK_MSG_ID_WAYPOINT_COUNT = 44;
	public const byte MAVLINK_MSG_ID_WAYPOINT_COUNT_LEN = 4;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_waypoint_count_t
	{
		public byte target_system; /// System ID
		public byte target_component; /// Component ID
		public ushort count; /// Number of Waypoints in the Sequence
	};

	public const byte MAVLINK_MSG_ID_WAYPOINT_CLEAR_ALL = 45;
	public const byte MAVLINK_MSG_ID_WAYPOINT_CLEAR_ALL_LEN = 2;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_waypoint_clear_all_t
	{
		public byte target_system; /// System ID
		public byte target_component; /// Component ID
	};

	public const byte MAVLINK_MSG_ID_WAYPOINT_REACHED = 46;
	public const byte MAVLINK_MSG_ID_WAYPOINT_REACHED_LEN = 2;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_waypoint_reached_t
	{
		public ushort seq; /// Sequence
	};

	public const byte MAVLINK_MSG_ID_WAYPOINT_ACK = 47;
	public const byte MAVLINK_MSG_ID_WAYPOINT_ACK_LEN = 3;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_waypoint_ack_t
	{
		public byte target_system; /// System ID
		public byte target_component; /// Component ID
		public byte type; /// 0: OK, 1: Error
	};

	public const byte MAVLINK_MSG_ID_GPS_SET_GLOBAL_ORIGIN = 48;
	public const byte MAVLINK_MSG_ID_GPS_SET_GLOBAL_ORIGIN_LEN = 14;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_gps_set_global_origin_t
	{
		public byte target_system; /// System ID
		public byte target_component; /// Component ID
		public int latitude; /// global position * 1E7
		public int longitude; /// global position * 1E7
		public int altitude; /// global position * 1000
	};

	public const byte MAVLINK_MSG_ID_GPS_LOCAL_ORIGIN_SET = 49;
	public const byte MAVLINK_MSG_ID_GPS_LOCAL_ORIGIN_SET_LEN = 12;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_gps_local_origin_set_t
	{
		public int latitude; /// Latitude (WGS84), expressed as * 1E7
		public int longitude; /// Longitude (WGS84), expressed as * 1E7
		public int altitude; /// Altitude(WGS84), expressed as * 1000
	};

	public const byte MAVLINK_MSG_ID_LOCAL_POSITION_SETPOINT_SET = 50;
	public const byte MAVLINK_MSG_ID_LOCAL_POSITION_SETPOINT_SET_LEN = 18;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_local_position_setpoint_set_t
	{
		public byte target_system; /// System ID
		public byte target_component; /// Component ID
		public float x; /// x position
		public float y; /// y position
		public float z; /// z position
		public float yaw; /// Desired yaw angle
	};

	public const byte MAVLINK_MSG_ID_LOCAL_POSITION_SETPOINT = 51;
	public const byte MAVLINK_MSG_ID_LOCAL_POSITION_SETPOINT_LEN = 16;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_local_position_setpoint_t
	{
		public float x; /// x position
		public float y; /// y position
		public float z; /// z position
		public float yaw; /// Desired yaw angle
	};

	public const byte MAVLINK_MSG_ID_CONTROL_STATUS = 52;
	public const byte MAVLINK_MSG_ID_CONTROL_STATUS_LEN = 8;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_control_status_t
	{
		public byte position_fix; /// Position fix: 0: lost, 2: 2D position fix, 3: 3D position fix 
		public byte vision_fix; /// Vision position fix: 0: lost, 1: 2D local position hold, 2: 2D global position fix, 3: 3D global position fix 
		public byte gps_fix; /// GPS position fix: 0: no reception, 1: Minimum 1 satellite, but no position fix, 2: 2D position fix, 3: 3D position fix 
		public byte ahrs_health; /// Attitude estimation health: 0: poor, 255: excellent
		public byte control_att; /// 0: Attitude control disabled, 1: enabled
		public byte control_pos_xy; /// 0: X, Y position control disabled, 1: enabled
		public byte control_pos_z; /// 0: Z position control disabled, 1: enabled
		public byte control_pos_yaw; /// 0: Yaw angle control disabled, 1: enabled
	};

	public const byte MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA = 53;
	public const byte MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA_LEN = 27;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_safety_set_allowed_area_t
	{
		public byte target_system; /// System ID
		public byte target_component; /// Component ID
		public byte frame; /// Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
		public float p1x; /// x position 1 / Latitude 1
		public float p1y; /// y position 1 / Longitude 1
		public float p1z; /// z position 1 / Altitude 1
		public float p2x; /// x position 2 / Latitude 2
		public float p2y; /// y position 2 / Longitude 2
		public float p2z; /// z position 2 / Altitude 2
	};

	public const byte MAVLINK_MSG_ID_SAFETY_ALLOWED_AREA = 54;
	public const byte MAVLINK_MSG_ID_SAFETY_ALLOWED_AREA_LEN = 25;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_safety_allowed_area_t
	{
		public byte frame; /// Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
		public float p1x; /// x position 1 / Latitude 1
		public float p1y; /// y position 1 / Longitude 1
		public float p1z; /// z position 1 / Altitude 1
		public float p2x; /// x position 2 / Latitude 2
		public float p2y; /// y position 2 / Longitude 2
		public float p2z; /// z position 2 / Altitude 2
	};

	public const byte MAVLINK_MSG_ID_SET_ROLL_PITCH_YAW_THRUST = 55;
	public const byte MAVLINK_MSG_ID_SET_ROLL_PITCH_YAW_THRUST_LEN = 18;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_set_roll_pitch_yaw_thrust_t
	{
		public byte target_system; /// System ID
		public byte target_component; /// Component ID
		public float roll; /// Desired roll angle in radians
		public float pitch; /// Desired pitch angle in radians
		public float yaw; /// Desired yaw angle in radians
		public float thrust; /// Collective thrust, normalized to 0 .. 1
	};

	public const byte MAVLINK_MSG_ID_SET_ROLL_PITCH_YAW_SPEED_THRUST = 56;
	public const byte MAVLINK_MSG_ID_SET_ROLL_PITCH_YAW_SPEED_THRUST_LEN = 18;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_set_roll_pitch_yaw_speed_thrust_t
	{
		public byte target_system; /// System ID
		public byte target_component; /// Component ID
		public float roll_speed; /// Desired roll angular speed in rad/s
		public float pitch_speed; /// Desired pitch angular speed in rad/s
		public float yaw_speed; /// Desired yaw angular speed in rad/s
		public float thrust; /// Collective thrust, normalized to 0 .. 1
	};

	public const byte MAVLINK_MSG_ID_ROLL_PITCH_YAW_THRUST_SETPOINT = 57;
	public const byte MAVLINK_MSG_ID_ROLL_PITCH_YAW_THRUST_SETPOINT_LEN = 24;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_roll_pitch_yaw_thrust_setpoint_t
	{
		public ulong time_us; /// Timestamp in micro seconds since unix epoch
		public float roll; /// Desired roll angle in radians
		public float pitch; /// Desired pitch angle in radians
		public float yaw; /// Desired yaw angle in radians
		public float thrust; /// Collective thrust, normalized to 0 .. 1
	};

	public const byte MAVLINK_MSG_ID_ROLL_PITCH_YAW_SPEED_THRUST_SETPOINT = 58;
	public const byte MAVLINK_MSG_ID_ROLL_PITCH_YAW_SPEED_THRUST_SETPOINT_LEN = 24;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_roll_pitch_yaw_speed_thrust_setpoint_t
	{
		public ulong time_us; /// Timestamp in micro seconds since unix epoch
		public float roll_speed; /// Desired roll angular speed in rad/s
		public float pitch_speed; /// Desired pitch angular speed in rad/s
		public float yaw_speed; /// Desired yaw angular speed in rad/s
		public float thrust; /// Collective thrust, normalized to 0 .. 1
	};

	public const byte MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT = 62;
	public const byte MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_LEN = 26;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_nav_controller_output_t
	{
		public float nav_roll; /// Current desired roll in degrees
		public float nav_pitch; /// Current desired pitch in degrees
		public short nav_bearing; /// Current desired heading in degrees
		public short target_bearing; /// Bearing to current waypoint/target in degrees
		public ushort wp_dist; /// Distance to active waypoint in meters
		public float alt_error; /// Current altitude error in meters
		public float aspd_error; /// Current airspeed error in meters/second
		public float xtrack_error; /// Current crosstrack error on x-y plane in meters
	};

	public const byte MAVLINK_MSG_ID_POSITION_TARGET = 63;
	public const byte MAVLINK_MSG_ID_POSITION_TARGET_LEN = 16;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_position_target_t
	{
		public float x; /// x position
		public float y; /// y position
		public float z; /// z position
		public float yaw; /// yaw orientation in radians, 0 = NORTH
	};

	public const byte MAVLINK_MSG_ID_STATE_CORRECTION = 64;
	public const byte MAVLINK_MSG_ID_STATE_CORRECTION_LEN = 36;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_state_correction_t
	{
		public float xErr; /// x position error
		public float yErr; /// y position error
		public float zErr; /// z position error
		public float rollErr; /// roll error (radians)
		public float pitchErr; /// pitch error (radians)
		public float yawErr; /// yaw error (radians)
		public float vxErr; /// x velocity
		public float vyErr; /// y velocity
		public float vzErr; /// z velocity
	};

	public const byte MAVLINK_MSG_ID_SET_ALTITUDE = 65;
	public const byte MAVLINK_MSG_ID_SET_ALTITUDE_LEN = 5;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_set_altitude_t
	{
		public byte target; /// The system setting the altitude
		public uint mode; /// The new altitude in meters
	};

	public const byte MAVLINK_MSG_ID_REQUEST_DATA_STREAM = 66;
	public const byte MAVLINK_MSG_ID_REQUEST_DATA_STREAM_LEN = 6;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_request_data_stream_t
	{
		public byte target_system; /// The target requested to send the message stream.
		public byte target_component; /// The target requested to send the message stream.
		public byte req_stream_id; /// The ID of the requested message type
		public ushort req_message_rate; /// Update rate in Hertz
		public byte start_stop; /// 1 to start sending, 0 to stop sending.
	};

	public const byte MAVLINK_MSG_ID_HIL_STATE = 67;
	public const byte MAVLINK_MSG_ID_HIL_STATE_LEN = 56;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_hil_state_t
	{
		public ulong usec; /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
		public float roll; /// Roll angle (rad)
		public float pitch; /// Pitch angle (rad)
		public float yaw; /// Yaw angle (rad)
		public float rollspeed; /// Roll angular speed (rad/s)
		public float pitchspeed; /// Pitch angular speed (rad/s)
		public float yawspeed; /// Yaw angular speed (rad/s)
		public int lat; /// Latitude, expressed as * 1E7
		public int lon; /// Longitude, expressed as * 1E7
		public int alt; /// Altitude in meters, expressed as * 1000 (millimeters)
		public short vx; /// Ground X Speed (Latitude), expressed as m/s * 100
		public short vy; /// Ground Y Speed (Longitude), expressed as m/s * 100
		public short vz; /// Ground Z Speed (Altitude), expressed as m/s * 100
		public short xacc; /// X acceleration (mg)
		public short yacc; /// Y acceleration (mg)
		public short zacc; /// Z acceleration (mg)
	};

	public const byte MAVLINK_MSG_ID_HIL_CONTROLS = 68;
	public const byte MAVLINK_MSG_ID_HIL_CONTROLS_LEN = 26;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_hil_controls_t
	{
		public ulong time_us; /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
		public float roll_ailerons; /// Control output -3 .. 1
		public float pitch_elevator; /// Control output -1 .. 1
		public float yaw_rudder; /// Control output -1 .. 1
		public float throttle; /// Throttle 0 .. 1
		public byte mode; /// System mode (MAV_MODE)
		public byte nav_mode; /// Navigation mode (MAV_NAV_MODE)
	};

	public const byte MAVLINK_MSG_ID_MANUAL_CONTROL = 69;
	public const byte MAVLINK_MSG_ID_MANUAL_CONTROL_LEN = 21;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_manual_control_t
	{
		public byte target; /// The system to be controlled
		public float roll; /// roll
		public float pitch; /// pitch
		public float yaw; /// yaw
		public float thrust; /// thrust
		public byte roll_manual; /// roll control enabled auto:0, manual:1
		public byte pitch_manual; /// pitch auto:0, manual:1
		public byte yaw_manual; /// yaw auto:0, manual:1
		public byte thrust_manual; /// thrust auto:0, manual:1
	};

	public const byte MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE = 70;
	public const byte MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN = 18;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_rc_channels_override_t
	{
		public byte target_system; /// System ID
		public byte target_component; /// Component ID
		public ushort chan1_raw; /// RC channel 1 value, in microseconds
		public ushort chan2_raw; /// RC channel 2 value, in microseconds
		public ushort chan3_raw; /// RC channel 3 value, in microseconds
		public ushort chan4_raw; /// RC channel 4 value, in microseconds
		public ushort chan5_raw; /// RC channel 5 value, in microseconds
		public ushort chan6_raw; /// RC channel 6 value, in microseconds
		public ushort chan7_raw; /// RC channel 7 value, in microseconds
		public ushort chan8_raw; /// RC channel 8 value, in microseconds
	};

	public const byte MAVLINK_MSG_ID_GLOBAL_POSITION_INT = 73;
	public const byte MAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN = 18;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_global_position_int_t
	{
		public int lat; /// Latitude, expressed as * 1E7
		public int lon; /// Longitude, expressed as * 1E7
		public int alt; /// Altitude in meters, expressed as * 1000 (millimeters)
		public short vx; /// Ground X Speed (Latitude), expressed as m/s * 100
		public short vy; /// Ground Y Speed (Longitude), expressed as m/s * 100
		public short vz; /// Ground Z Speed (Altitude), expressed as m/s * 100
	};

	public const byte MAVLINK_MSG_ID_VFR_HUD = 74;
	public const byte MAVLINK_MSG_ID_VFR_HUD_LEN = 20;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_vfr_hud_t
	{
		public float airspeed; /// Current airspeed in m/s
		public float groundspeed; /// Current ground speed in m/s
		public short heading; /// Current heading in degrees, in compass units (0..360, 0=north)
		public ushort throttle; /// Current throttle setting in integer percent, 0 to 100
		public float alt; /// Current altitude (MSL), in meters
		public float climb; /// Current climb rate in meters/second
	};

	public const byte MAVLINK_MSG_ID_COMMAND = 75;
	public const byte MAVLINK_MSG_ID_COMMAND_LEN = 20;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_command_t
	{
		public byte target_system; /// System which should execute the command
		public byte target_component; /// Component which should execute the command, 0 for all components
		public byte command; /// Command ID, as defined by MAV_CMD enum.
		public byte confirmation; /// 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
		public float param1; /// Parameter 1, as defined by MAV_CMD enum.
		public float param2; /// Parameter 2, as defined by MAV_CMD enum.
		public float param3; /// Parameter 3, as defined by MAV_CMD enum.
		public float param4; /// Parameter 4, as defined by MAV_CMD enum.
	};

	public const byte MAVLINK_MSG_ID_COMMAND_ACK = 76;
	public const byte MAVLINK_MSG_ID_COMMAND_ACK_LEN = 8;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_command_ack_t
	{
		public float command; /// Current airspeed in m/s
		public float result; /// 1: Action ACCEPTED and EXECUTED, 1: Action TEMPORARY REJECTED/DENIED, 2: Action PERMANENTLY DENIED, 3: Action UNKNOWN/UNSUPPORTED, 4: Requesting CONFIRMATION
	};

	public const byte MAVLINK_MSG_ID_OPTICAL_FLOW = 100;
	public const byte MAVLINK_MSG_ID_OPTICAL_FLOW_LEN = 18;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_optical_flow_t
	{
		public ulong time; /// Timestamp (UNIX)
		public byte sensor_id; /// Sensor ID
		public short flow_x; /// Flow in pixels in x-sensor direction
		public short flow_y; /// Flow in pixels in y-sensor direction
		public byte quality; /// Optical flow quality / confidence. 0: bad, 255: maximum quality
		public float ground_distance; /// Ground distance in meters
	};

	public const byte MAVLINK_MSG_ID_OBJECT_DETECTION_EVENT = 140;
	public const byte MAVLINK_MSG_ID_OBJECT_DETECTION_EVENT_LEN = 36;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_object_detection_event_t
	{
		public uint time; /// Timestamp in milliseconds since system boot
		public ushort object_id; /// Object ID
		public byte type; /// Object type: 0: image, 1: letter, 2: ground vehicle, 3: air vehicle, 4: surface vehicle, 5: sub-surface vehicle, 6: human, 7: animal
		[MarshalAs(
			UnmanagedType.ByValArray,
			SizeConst=20)]
		public byte[] name; /// Name of the object as defined by the detector
		public byte quality; /// Detection quality / confidence. 0: bad, 255: maximum confidence
		public float bearing; /// Angle of the object with respect to the body frame in NED coordinates in radians. 0: front
		public float distance; /// Ground distance in meters
	};

	public const byte MAVLINK_MSG_ID_DEBUG_VECT = 251;
	public const byte MAVLINK_MSG_ID_DEBUG_VECT_LEN = 30;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_debug_vect_t
	{
		[MarshalAs(
			UnmanagedType.ByValArray,
			SizeConst=10)]
		public byte[] name; /// Name
		public ulong usec; /// Timestamp
		public float x; /// x
		public float y; /// y
		public float z; /// z
	};

	public const byte MAVLINK_MSG_ID_NAMED_VALUE_FLOAT = 252;
	public const byte MAVLINK_MSG_ID_NAMED_VALUE_FLOAT_LEN = 14;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_named_value_float_t
	{
		[MarshalAs(
			UnmanagedType.ByValArray,
			SizeConst=10)]
		public byte[] name; /// Name of the debug variable
		public float value; /// Floating point value
	};

	public const byte MAVLINK_MSG_ID_NAMED_VALUE_INT = 253;
	public const byte MAVLINK_MSG_ID_NAMED_VALUE_INT_LEN = 14;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_named_value_int_t
	{
		[MarshalAs(
			UnmanagedType.ByValArray,
			SizeConst=10)]
		public byte[] name; /// Name of the debug variable
		public int value; /// Signed integer value
	};

	public const byte MAVLINK_MSG_ID_STATUSTEXT = 254;
	public const byte MAVLINK_MSG_ID_STATUSTEXT_LEN = 51;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_statustext_t
	{
		public byte severity; /// Severity of status, 0 = info message, 255 = critical fault
		[MarshalAs(
			UnmanagedType.ByValArray,
			SizeConst=50)]
		public byte[] text; /// Status text message, without null termination character
	};

	public const byte MAVLINK_MSG_ID_DEBUG = 255;
	public const byte MAVLINK_MSG_ID_DEBUG_LEN = 5;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_debug_t
	{
		public byte ind; /// index of debug variable
		public float value; /// DEBUG value
	};

	public const byte MAVLINK_MSG_ID_SENSOR_OFFSETS = 150;
	public const byte MAVLINK_MSG_ID_SENSOR_OFFSETS_LEN = 42;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_sensor_offsets_t
	{
		public short mag_ofs_x; /// magnetometer X offset
		public short mag_ofs_y; /// magnetometer Y offset
		public short mag_ofs_z; /// magnetometer Z offset
		public float mag_declination; /// magnetic declination (radians)
		public int raw_press; /// raw pressure from barometer
		public int raw_temp; /// raw temperature from barometer
		public float gyro_cal_x; /// gyro X calibration
		public float gyro_cal_y; /// gyro Y calibration
		public float gyro_cal_z; /// gyro Z calibration
		public float accel_cal_x; /// accel X calibration
		public float accel_cal_y; /// accel Y calibration
		public float accel_cal_z; /// accel Z calibration
	};

	public const byte MAVLINK_MSG_ID_SET_MAG_OFFSETS = 151;
	public const byte MAVLINK_MSG_ID_SET_MAG_OFFSETS_LEN = 8;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_set_mag_offsets_t
	{
		public byte target_system; /// System ID
		public byte target_component; /// Component ID
		public short mag_ofs_x; /// magnetometer X offset
		public short mag_ofs_y; /// magnetometer Y offset
		public short mag_ofs_z; /// magnetometer Z offset
	};

	public const byte MAVLINK_MSG_ID_MEMINFO = 152;
	public const byte MAVLINK_MSG_ID_MEMINFO_LEN = 4;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_meminfo_t
	{
		public ushort brkval; /// heap top
		public ushort freemem; /// free memory
	};

	public const byte MAVLINK_MSG_ID_AP_ADC = 153;
	public const byte MAVLINK_MSG_ID_AP_ADC_LEN = 12;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_ap_adc_t
	{
		public ushort adc1; /// ADC output 1
		public ushort adc2; /// ADC output 2
		public ushort adc3; /// ADC output 3
		public ushort adc4; /// ADC output 4
		public ushort adc5; /// ADC output 5
		public ushort adc6; /// ADC output 6
	};

	public const byte MAVLINK_MSG_ID_DIGICAM_CONFIGURE = 154;
	public const byte MAVLINK_MSG_ID_DIGICAM_CONFIGURE_LEN = 15;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_digicam_configure_t
	{
		public byte target_system; /// System ID
		public byte target_component; /// Component ID
		public byte mode; /// Mode enumeration from 1 to N //P, TV, AV, M, Etc (0 means ignore)
		public ushort shutter_speed; /// Divisor number //e.g. 1000 means 1/1000 (0 means ignore)
		public byte aperture; /// F stop number x 10 //e.g. 28 means 2.8 (0 means ignore)
		public byte iso; /// ISO enumeration from 1 to N //e.g. 80, 100, 200, Etc (0 means ignore)
		public byte exposure_type; /// Exposure type enumeration from 1 to N (0 means ignore)
		public byte command_id; /// Command Identity (incremental loop: 0 to 255)//A command sent multiple times will be executed or pooled just once
		public byte engine_cut_off; /// Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off)
		public byte extra_param; /// Extra parameters enumeration (0 means ignore)
		public float extra_value; /// Correspondent value to given extra_param
	};

	public const byte MAVLINK_MSG_ID_DIGICAM_CONTROL = 155;
	public const byte MAVLINK_MSG_ID_DIGICAM_CONTROL_LEN = 13;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_digicam_control_t
	{
		public byte target_system; /// System ID
		public byte target_component; /// Component ID
		public byte session; /// 0: stop, 1: start or keep it up //Session control e.g. show/hide lens
		public byte zoom_pos; /// 1 to N //Zoom's absolute position (0 means ignore)
		public byte zoom_step; /// -100 to 100 //Zooming step value to offset zoom from the current position
		public byte focus_lock; /// 0: unlock focus or keep unlocked, 1: lock focus or keep locked, 3: re-lock focus
		public byte shot; /// 0: ignore, 1: shot or start filming
		public byte command_id; /// Command Identity (incremental loop: 0 to 255)//A command sent multiple times will be executed or pooled just once
		public byte extra_param; /// Extra parameters enumeration (0 means ignore)
		public float extra_value; /// Correspondent value to given extra_param
	};

	public const byte MAVLINK_MSG_ID_MOUNT_CONFIGURE = 156;
	public const byte MAVLINK_MSG_ID_MOUNT_CONFIGURE_LEN = 6;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_mount_configure_t
	{
		public byte target_system; /// System ID
		public byte target_component; /// Component ID
		public byte mount_mode; /// mount operating mode (see MAV_MOUNT_MODE enum)
		public byte stab_roll; /// (1 = yes, 0 = no)
		public byte stab_pitch; /// (1 = yes, 0 = no)
		public byte stab_yaw; /// (1 = yes, 0 = no)
	};

	public const byte MAVLINK_MSG_ID_MOUNT_CONTROL = 157;
	public const byte MAVLINK_MSG_ID_MOUNT_CONTROL_LEN = 15;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_mount_control_t
	{
		public byte target_system; /// System ID
		public byte target_component; /// Component ID
		public int input_a; /// pitch(deg*100) or lat, depending on mount mode
		public int input_b; /// roll(deg*100) or lon depending on mount mode
		public int input_c; /// yaw(deg*100) or alt (in cm) depending on mount mode
		public byte save_position; /// if "1" it will save current trimmed position on EEPROM (just valid for NEUTRAL and LANDING)
	};

	public const byte MAVLINK_MSG_ID_MOUNT_STATUS = 158;
	public const byte MAVLINK_MSG_ID_MOUNT_STATUS_LEN = 14;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_mount_status_t
	{
		public byte target_system; /// System ID
		public byte target_component; /// Component ID
		public int pointing_a; /// pitch(deg*100) or lat, depending on mount mode
		public int pointing_b; /// roll(deg*100) or lon depending on mount mode
		public int pointing_c; /// yaw(deg*100) or alt (in cm) depending on mount mode
	};

	public const byte MAVLINK_MSG_ID_FENCE_POINT = 160;
	public const byte MAVLINK_MSG_ID_FENCE_POINT_LEN = 12;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_fence_point_t
	{
		public byte target_system; /// System ID
		public byte target_component; /// Component ID
		public byte idx; /// point index (first point is 1, 0 is for return point)
		public byte count; /// total number of points (for sanity checking)
		public float lat; /// Latitude of point
		public float lng; /// Longitude of point
	};

	public const byte MAVLINK_MSG_ID_FENCE_FETCH_POINT = 161;
	public const byte MAVLINK_MSG_ID_FENCE_FETCH_POINT_LEN = 3;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_fence_fetch_point_t
	{
		public byte target_system; /// System ID
		public byte target_component; /// Component ID
		public byte idx; /// point index (first point is 1, 0 is for return point)
	};

	public const byte MAVLINK_MSG_ID_FENCE_STATUS = 162;
	public const byte MAVLINK_MSG_ID_FENCE_STATUS_LEN = 8;
	[StructLayout(LayoutKind.Sequential,Pack=1)]
	public struct __mavlink_fence_status_t
	{
		public byte breach_status; /// 0 if currently inside fence, 1 if outside
		public ushort breach_count; /// number of fence breaches
		public byte breach_type; /// last breach type (see FENCE_BREACH_* enum)
		public uint breach_time; /// time of last breach in milliseconds since boot
	};

	Type[] mavstructs = new Type[] {
			typeof( __mavlink_heartbeat_t),			// message 0
			typeof( __mavlink_boot_t),			// message 1
			typeof( __mavlink_system_time_t),			// message 2
			typeof( __mavlink_ping_t),			// message 3
			typeof( __mavlink_system_time_utc_t),			// message 4
			typeof( __mavlink_change_operator_control_t),			// message 5
			typeof( __mavlink_change_operator_control_ack_t),			// message 6
			typeof( __mavlink_auth_key_t),			// message 7
			null,			 //message 8
			typeof( __mavlink_action_ack_t),			// message 9
			typeof( __mavlink_action_t),			// message 10
			typeof( __mavlink_set_mode_t),			// message 11
			typeof( __mavlink_set_nav_mode_t),			// message 12
			null,			 //message 13
			null,			 //message 14
			null,			 //message 15
			null,			 //message 16
			null,			 //message 17
			null,			 //message 18
			null,			 //message 19
			typeof( __mavlink_param_request_read_t),			// message 20
			typeof( __mavlink_param_request_list_t),			// message 21
			typeof( __mavlink_param_value_t),			// message 22
			typeof( __mavlink_param_set_t),			// message 23
			null,			 //message 24
			typeof( __mavlink_gps_raw_int_t),			// message 25
			typeof( __mavlink_scaled_imu_t),			// message 26
			typeof( __mavlink_gps_status_t),			// message 27
			typeof( __mavlink_raw_imu_t),			// message 28
			typeof( __mavlink_raw_pressure_t),			// message 29
			typeof( __mavlink_attitude_t),			// message 30
			typeof( __mavlink_local_position_t),			// message 31
			typeof( __mavlink_gps_raw_t),			// message 32
			typeof( __mavlink_global_position_t),			// message 33
			typeof( __mavlink_sys_status_t),			// message 34
			typeof( __mavlink_rc_channels_raw_t),			// message 35
			typeof( __mavlink_rc_channels_scaled_t),			// message 36
			typeof( __mavlink_servo_output_raw_t),			// message 37
			typeof( __mavlink_scaled_pressure_t),			// message 38
			typeof( __mavlink_waypoint_t),			// message 39
			typeof( __mavlink_waypoint_request_t),			// message 40
			typeof( __mavlink_waypoint_set_current_t),			// message 41
			typeof( __mavlink_waypoint_current_t),			// message 42
			typeof( __mavlink_waypoint_request_list_t),			// message 43
			typeof( __mavlink_waypoint_count_t),			// message 44
			typeof( __mavlink_waypoint_clear_all_t),			// message 45
			typeof( __mavlink_waypoint_reached_t),			// message 46
			typeof( __mavlink_waypoint_ack_t),			// message 47
			typeof( __mavlink_gps_set_global_origin_t),			// message 48
			typeof( __mavlink_gps_local_origin_set_t),			// message 49
			typeof( __mavlink_local_position_setpoint_set_t),			// message 50
			typeof( __mavlink_local_position_setpoint_t),			// message 51
			typeof( __mavlink_control_status_t),			// message 52
			typeof( __mavlink_safety_set_allowed_area_t),			// message 53
			typeof( __mavlink_safety_allowed_area_t),			// message 54
			typeof( __mavlink_set_roll_pitch_yaw_thrust_t),			// message 55
			typeof( __mavlink_set_roll_pitch_yaw_speed_thrust_t),			// message 56
			typeof( __mavlink_roll_pitch_yaw_thrust_setpoint_t),			// message 57
			typeof( __mavlink_roll_pitch_yaw_speed_thrust_setpoint_t),			// message 58
			null,			 //message 59
			null,			 //message 60
			null,			 //message 61
			typeof( __mavlink_nav_controller_output_t),			// message 62
			typeof( __mavlink_position_target_t),			// message 63
			typeof( __mavlink_state_correction_t),			// message 64
			typeof( __mavlink_set_altitude_t),			// message 65
			typeof( __mavlink_request_data_stream_t),			// message 66
			typeof( __mavlink_hil_state_t),			// message 67
			typeof( __mavlink_hil_controls_t),			// message 68
			typeof( __mavlink_manual_control_t),			// message 69
			typeof( __mavlink_rc_channels_override_t),			// message 70
			null,			 //message 71
			null,			 //message 72
			typeof( __mavlink_global_position_int_t),			// message 73
			typeof( __mavlink_vfr_hud_t),			// message 74
			typeof( __mavlink_command_t),			// message 75
			typeof( __mavlink_command_ack_t),			// message 76
			null,			 //message 77
			null,			 //message 78
			null,			 //message 79
			null,			 //message 80
			null,			 //message 81
			null,			 //message 82
			null,			 //message 83
			null,			 //message 84
			null,			 //message 85
			null,			 //message 86
			null,			 //message 87
			null,			 //message 88
			null,			 //message 89
			null,			 //message 90
			null,			 //message 91
			null,			 //message 92
			null,			 //message 93
			null,			 //message 94
			null,			 //message 95
			null,			 //message 96
			null,			 //message 97
			null,			 //message 98
			null,			 //message 99
			typeof( __mavlink_optical_flow_t),			// message 100
			null,			 //message 101
			null,			 //message 102
			null,			 //message 103
			null,			 //message 104
			null,			 //message 105
			null,			 //message 106
			null,			 //message 107
			null,			 //message 108
			null,			 //message 109
			null,			 //message 110
			typeof( __mavlink_analog_raw_t),			// message 111
			typeof( __mavlink_analog_eu_t),			// message 112
			typeof( __mavlink_cdnr_controller_t),			// message 113
			typeof( __mavlink_traffic_data_t),			// message 114
			typeof( __mavlink_state_data_t),			// message 115
			typeof( __mavlink_traffic_sim_timing_t),			// message 116
			null,			 //message 117
			null,			 //message 118
			null,			 //message 119
			null,			 //message 120
			null,			 //message 121
			null,			 //message 122
			null,			 //message 123
			null,			 //message 124
			null,			 //message 125
			null,			 //message 126
			null,			 //message 127
			null,			 //message 128
			null,			 //message 129
			null,			 //message 130
			null,			 //message 131
			null,			 //message 132
			null,			 //message 133
			null,			 //message 134
			null,			 //message 135
			null,			 //message 136
			null,			 //message 137
			null,			 //message 138
			null,			 //message 139
			typeof( __mavlink_object_detection_event_t),			// message 140
			null,			 //message 141
			null,			 //message 142
			null,			 //message 143
			null,			 //message 144
			null,			 //message 145
			null,			 //message 146
			null,			 //message 147
			null,			 //message 148
			null,			 //message 149
			typeof( __mavlink_sensor_offsets_t),			// message 150
			typeof( __mavlink_set_mag_offsets_t),			// message 151
			typeof( __mavlink_meminfo_t),			// message 152
			typeof( __mavlink_ap_adc_t),			// message 153
			typeof( __mavlink_digicam_configure_t),			// message 154
			typeof( __mavlink_digicam_control_t),			// message 155
			typeof( __mavlink_mount_configure_t),			// message 156
			typeof( __mavlink_mount_control_t),			// message 157
			typeof( __mavlink_mount_status_t),			// message 158
			null,			 //message 159
			typeof( __mavlink_fence_point_t),			// message 160
			typeof( __mavlink_fence_fetch_point_t),			// message 161
			typeof( __mavlink_fence_status_t),			// message 162
			null,			 //message 163
			null,			 //message 164
			null,			 //message 165
			null,			 //message 166
			null,			 //message 167
			null,			 //message 168
			null,			 //message 169
			null,			 //message 170
			null,			 //message 171
			null,			 //message 172
			null,			 //message 173
			null,			 //message 174
			null,			 //message 175
			null,			 //message 176
			null,			 //message 177
			null,			 //message 178
			null,			 //message 179
			null,			 //message 180
			null,			 //message 181
			null,			 //message 182
			null,			 //message 183
			null,			 //message 184
			null,			 //message 185
			null,			 //message 186
			null,			 //message 187
			null,			 //message 188
			null,			 //message 189
			null,			 //message 190
			null,			 //message 191
			null,			 //message 192
			null,			 //message 193
			null,			 //message 194
			null,			 //message 195
			null,			 //message 196
			null,			 //message 197
			null,			 //message 198
			null,			 //message 199
			null,			 //message 200
			null,			 //message 201
			null,			 //message 202
			null,			 //message 203
			null,			 //message 204
			null,			 //message 205
			null,			 //message 206
			null,			 //message 207
			null,			 //message 208
			null,			 //message 209
			null,			 //message 210
			null,			 //message 211
			null,			 //message 212
			null,			 //message 213
			null,			 //message 214
			null,			 //message 215
			null,			 //message 216
			null,			 //message 217
			null,			 //message 218
			null,			 //message 219
			null,			 //message 220
			null,			 //message 221
			null,			 //message 222
			null,			 //message 223
			null,			 //message 224
			null,			 //message 225
			null,			 //message 226
			null,			 //message 227
			null,			 //message 228
			null,			 //message 229
			null,			 //message 230
			null,			 //message 231
			null,			 //message 232
			null,			 //message 233
			null,			 //message 234
			null,			 //message 235
			null,			 //message 236
			null,			 //message 237
			null,			 //message 238
			null,			 //message 239
			null,			 //message 240
			null,			 //message 241
			null,			 //message 242
			null,			 //message 243
			null,			 //message 244
			null,			 //message 245
			null,			 //message 246
			null,			 //message 247
			null,			 //message 248
			null,			 //message 249
			null,			 //message 250
			typeof( __mavlink_debug_vect_t),			// message 251
			typeof( __mavlink_named_value_float_t),			// message 252
			typeof( __mavlink_named_value_int_t),			// message 253
			typeof( __mavlink_statustext_t),			// message 254
			typeof( __mavlink_debug_t),			// message 255
	};
