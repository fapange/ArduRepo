
// Main Definitions
//#define MAVLINK10 // while MAVLINK10 is not defined it will use MavLink 0.9

#ifndef ENABLED
	#define ENABLED 1
	#define FLIGHT_MODE 1
#endif
#ifndef DISABLED
	#define DISABLED 0
	#define HILSIM_MODE 0
#endif

#define SLV_HILPLANE	 0
#define SLV_N801RE		 1
#define SLV_N802RE		 2
#define SLV_N803RE		 3
#define SLV_N381NA		81
#define SLV_N382NA		82
#define SLV_N383NA		83
#define SLV_N384NA		84
#define SLV_N385NA		85
#define SLV_N386NA		86
#define SLV_N387NA		87


// Select Aircraft
#ifdef MAV_SYSTEM_ID
	#undef MAV_SYSTEM_ID
#endif
#define MAV_SYSTEM_ID	(SLV_HILPLANE)
#define MAV_GCS_ID		(255-MAV_SYSTEM_ID)
#define MAV_JS_ID		(137)
#define MAV_BEAGLE_ID	(103)

// Basic Features
#ifndef SLV_ADDED
	#define SLV_ADDED			ENABLED
#endif

// Features being tested
//#ifndef SLV_USE_GROUNDSPEED
//	#define SLV_USE_GROUNDSPEED	ENABLED
//#endif

// Slew rate Calculations
#define P_SLEW_RATE ((long)g.pidNavPitchAirspeed.imax()*delta_ms_fast_loop/1000)	// ? degrees/sec at 50Hz
#define R_SLEW_RATE ((long)g.pidNavPitchAirspeed.imax()*delta_ms_fast_loop/1000)	// ? degrees/sec at 50Hz
#define T_SLEW_RATE ((float)g.kff_pitch_to_throttle*delta_ms_fast_loop/1000)	    // ? %throttle/sec at 50Hz

#define STALL_SPEED_BUFFER	2
#define ALTITUDE_THRESHOLD	5000
#define RC_OFF		((byte)0)
#define RC_ON		((byte)1)
#define RC_ENGINE_OFF	((byte)128)
#define RC_FAILSAFE	((byte)255)
