// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// This file is just a placeholder for your configuration file.  If you wish to change any of the setup parameters from
// their default values, place the appropriate #define statements here.

// For example if you wanted the Port 3 baud rate to be 38400 you would add a statement like the one below (uncommented)
//#define SERIAL3_BAUD        38400

// You may also put an include statement here to point at another configuration file.  This is convenient if you maintain
// different configuration files for different aircraft or HIL simulation.  See the examples below
//#include "APM_Config_mavlink_hil.h"
//#include "Skywalker.h"

#if MAV_SYSTEM_ID == SLV_HILPLANE
	#include "HIL_Plane.h"
#endif

#if MAV_SYSTEM_ID == SLV_N801RE
	#include "N801RE.h"
#endif
#if MAV_SYSTEM_ID == SLV_N802RE
	#include "N802RE.h"
#endif
#if MAV_SYSTEM_ID == SLV_N803RE
	#include "N803RE.h"
#endif

#if MAV_SYSTEM_ID == SLV_N381NA
	#include "N381NA.h"
#endif
#if MAV_SYSTEM_ID == SLV_N382NA
	#include "N382NA.h"
#endif
#if MAV_SYSTEM_ID == SLV_N383NA
	#include "N383NA.h"
#endif
#if MAV_SYSTEM_ID == SLV_N384NA
	#include "N384NA.h"
#endif
#if MAV_SYSTEM_ID == SLV_N385NA
	#include "N385NA.h"
#endif
#if MAV_SYSTEM_ID == SLV_N386NA
	#include "N386NA.h"
#endif
#if MAV_SYSTEM_ID == SLV_N387NA
	#include "N387NA.h"
#endif


// The following are the recommended settings for Xplane simulation. Remove the leading "/* and trailing "*/" to enable:

/*
#define HIL_MODE            HIL_MODE_ATTITUDE
*/

/*
// HIL_MODE SELECTION
//
// Mavlink supports
// 1. HIL_MODE_ATTITUDE : simulated position, airspeed, and attitude
// 2. HIL_MODE_SENSORS: full sensor simulation
#define HIL_MODE            HIL_MODE_ATTITUDE

// Sensors
// All sensors are supported in all modes.
// The magnetometer is not used in 
// HIL_MODE_ATTITUDE but you may leave it
// enabled if you wish.
#define AIRSPEED_SENSOR     ENABLED
#define MAGNETOMETER        ENABLED
#define AIRSPEED_CRUISE     25
#define THROTTLE_FAILSAFE   ENABLED
*/