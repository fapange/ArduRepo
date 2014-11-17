// The following are the recommended settings for Xplane simulation. Remove the leading "/* and trailing "*/" to enable:

// Select Flight Mode
//#ifndef SLV_FLIGHTMODE
	#define SLV_FLIGHTMODE FLIGHT_MODE
//#endif

// AP Hardware Definition
#define CONFIG_APM_HARDWARE APM_HARDWARE_APM1

#if SLV_FLIGHTMODE == HILSIM_MODE
	// HIL_MODE SELECTION
	//
	// Mavlink supports
	// 1. HIL_MODE_ATTITUDE : simulated position, airspeed, and attitude
	// 2. HIL_MODE_SENSORS: full sensor simulation
	#define HIL_MODE            HIL_MODE_ATTITUDE
	#define HIL_PROTOCOL        HIL_PROTOCOL_MAVLINK
	#define HIL_PORT            0
	#define HIL_SERVOS	        1 // Enable servos while in HIL_MODE
#else
	#define HIL_MODE            HIL_MODE_DISABLED
#endif

#define GCS_PROTOCOL        GCS_PROTOCOL_MAVLINK
#define GCS_PORT            3
//#define SERIAL3_BAUD        115200

#define AUTO_TRIM           DISABLED // Do not re-trim radio

#define MAG_ORIENTATION		AP_COMPASS_COMPONENTS_UP_PINS_RIGHT
#define ALTITUDE_MIX			 1		// ratio: ALTITUDE_MIX*baro + (1-ALTITUDE_MIX)*gps
#define AIRSPEED_FBW_MIN		13		// m/s
#define AIRSPEED_CRUISE			20  	// m/s
#define AIRSPEED_FBW_MAX		40		// m/s
#define THROTTLE_MIN			 0		// percent
#define THROTTLE_CRUISE			50		// percent
#define THROTTLE_MAX			80		// percent
#define HEAD_MAX				40		// Degrees
#define PITCH_MAX				15		// Degrees
#define PITCH_MIN			   -10		// Degrees
#define WP_RADIUS_DEFAULT	   50		// meters
#define LOITER_RADIUS_DEFAULT  100		// meters
#define ALT_HOLD_HOME		   200		// meters


// Flight Modes
#define FLIGHT_MODE_1			MANUAL
#define FLIGHT_MODE_2			MANUAL
#define FLIGHT_MODE_3			MANUAL
#define FLIGHT_MODE_4			MANUAL
#define FLIGHT_MODE_5			MANUAL
#define FLIGHT_MODE_6			MANUAL

// Failsafe Actions
#define SHORT_FAILSAFE_ACTION		1
#define LONG_FAILSAFE_ACTION		1
#define GCS_HEARTBEAT_FAILSAFE		ENABLED

// Sensors
// All sensors are supported in all modes.
// The magnetometer is not used in 
// HIL_MODE_ATTITUDE but you may leave it
// enabled if you wish.
#define AIRSPEED_SENSOR     ENABLED
#define AIRSPEED_RATIO		1.0		// Note - this varies from the value in ArduPilot due to the difference in ADC resolution
#define MAGNETOMETER        ENABLED
#define THROTTLE_FAILSAFE   ENABLED

// GEOFENCE Parameters
#define GEOFENCE_ENABLED	ENABLED
#define FENCE_ENABLE_PWM	1000

//
#define ENABLE_STICK_MIXING	DISABLED
#define THROTTLE_OUT		ENABLED

//////////////////////////////////////////////////////////////////////////////
// Attitude control gains
//
#define SERVO_ROLL_P        0.2
#define SERVO_ROLL_I        0.025
#define SERVO_ROLL_D        0.0
#define SERVO_ROLL_INT_MAX  1
#define ROLL_SLEW_LIMIT     0

#define SERVO_PITCH_P       0.2
#define SERVO_PITCH_I       0.025
#define SERVO_PITCH_D       0.0
#define SERVO_PITCH_INT_MAX 1

#define SERVO_YAW_P         0.0
#define SERVO_YAW_I         0.0
#define SERVO_YAW_D         0.0
#define SERVO_YAW_INT_MAX   0

//////////////////////////////////////////////////////////////////////////////
// Navigation control gains
//
#define NAV_ROLL_P          0.7
#define NAV_ROLL_I          0.0
#define NAV_ROLL_D          0.0
#define NAV_ROLL_INT_MAX    5

#define NAV_PITCH_ASP_P     0.0
#define NAV_PITCH_ASP_I     0.0
#define NAV_PITCH_ASP_D     0.0
#define NAV_PITCH_ASP_INT_MAX 0

#define NAV_PITCH_ALT_P     0.7
#define NAV_PITCH_ALT_I     0.0
#define NAV_PITCH_ALT_D     0.0
#define NAV_PITCH_ALT_INT_MAX 5

//////////////////////////////////////////////////////////////////////////////
// Energy/Altitude control gains
//
#define THROTTLE_TE_P       0.50
#define THROTTLE_TE_I       0.0
#define THROTTLE_TE_D       0.0
#define THROTTLE_TE_INT_MAX 20

#define P_TO_T              0.1
#define T_TO_P              0
#define PITCH_COMP          0.0
#define RUDDER_MIX          0.0
#define PITCH_TARGET        0

//////////////////////////////////////////////////////////////////////////////
// Crosstrack compensation
//
#define XTRACK_GAIN         0 // deg/m
#define XTRACK_ENTRY_ANGLE  0 // deg


//////////////////////////////////////////////////////////////////////////////
// Radio channel limits
//
// Note that these are not called out in APM_Config.h.reference.
//
#define CH5_MIN	1499
#define CH5_MAX	1500
#define CH6_MIN	1053
#define CH6_MAX	1528
#define CH7_MIN	1124
#define CH7_MAX	1920
#define CH8_MIN	1161
#define CH8_MAX	1694

//////////////////////////////////////////////////////////////////////////////
// Vehicle Parameters
// ------------------
#define THISVEHICLE		"N802RE"
#define THISFLIGHTMODE	"Flight Mode"
#define AIRFRAME_NAME	"R2 Edge"

#define STALL_SPEED				14
#define AIRSPEED_CAL_GAIN		128.0
#define AIRSPEED_CAL_OFFSET		2.63

#define ROLL_ANGLE		1600
#define PITCH_ANGLE		700
#define RUDDER_ANGLE	1000
#define THROTTLE_RANGE	100
#define DEAD_ZONE		5

#define	CH_1_OFFSET  -10
#define	CH_2_OFFSET  -10
#define	CH_3_OFFSET  -10
#define	CH_4_OFFSET  -10
#define	CH_5_OFFSET  -15
#define	CH_6_OFFSET  -10
#define	CH_7_OFFSET  -10
#define	CH_8_OFFSET  -10

#define	CH_1_RC_OFF  1499
#define	CH_2_RC_OFF  1499
#define	CH_3_RC_OFF   999
#define	CH_4_RC_OFF  1499
#define	CH_5_RC_OFF  1499
#define	CH_6_RC_OFF  1499
#define	CH_7_RC_OFF  1499
#define	CH_8_RC_OFF  1499

#define	CH_1_RC_FAILSAFE  1569
#define	CH_2_RC_FAILSAFE  1249
#define	CH_3_RC_FAILSAFE  1250
#define	CH_4_RC_FAILSAFE  1530
#define	CH_5_RC_FAILSAFE  1499
#define	CH_6_RC_FAILSAFE  1528
#define	CH_7_RC_FAILSAFE  1499
#define	CH_8_RC_FAILSAFE  1198

#define ANALOG_EU_01        (float)(ac_ch[0])
#define ANALOG_EU_02        (float)RC_state
#define ANALOG_EU_03        (wind_dir/100.0f)
#define ANALOG_EU_04        (wind_vel/100.0f)
#define ANALOG_EU_05        (float)(WptRadius)
#define ANALOG_EU_06        (float)(time_left)
#define ANALOG_EU_07        (float)(ac_ch[6])
#define ANALOG_EU_08        (float)(ac_ch[7])
#define ANALOG_EU_09        (float)(ac_ch[8])
#define ANALOG_EU_10        (float)(ac_ch[9])
#define ANALOG_EU_11        (float)(ac_ch[10])
#define ANALOG_EU_12        (float)(ac_ch[11])
#define ANALOG_EU_13        (float)(ac_ch[12])
#define ANALOG_EU_14        (float)(ac_ch[13])
#define ANALOG_EU_15        (float)(ac_ch[14])
#define ANALOG_EU_16        (float)(ac_ch[15])
