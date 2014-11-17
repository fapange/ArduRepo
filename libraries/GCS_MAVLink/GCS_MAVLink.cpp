// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	GCS_MAVLink.cpp
/// @brief	Supporting bits for MAVLink.

#include "GCS_MAVLink.h"

BetterStream	*mavlink_comm_0_port;
BetterStream	*mavlink_comm_1_port;

// this might need to move to the flight software
mavlink_system_t mavlink_system = {7,1,0,0};

#ifdef MAVLINK10
# include "include_v1.0/mavlink_helpers.h"
#else
# include "include/mavlink_helpers.h"
#endif

int old3_mavlink_check_target(int APID, int SOURCEID)
{
	if (APID==MAV_SYSTEM_ID && SOURCEID==MAV_GCS_ID)    return 0;
	if (APID==MAV_SYSTEM_ID && SOURCEID==MAV_JS_ID)     return 0;
	if (APID==MAV_SYSTEM_ID && SOURCEID==MAV_BEAGLE_ID) return 0;
	if (APID==0 && SOURCEID==0) return 0;
	return 0;
}


/*
switch (APID)
	{
		case (int)MAV_SYSTEM_ID:
			switch (SOURCEID)
			{
				case (int)MAV_GCS_ID:
				case (int)MAV_JS_ID:
				case (int)MAV_BEAGLE_ID:
					return 0;
					break;
				case 0:
					if (APID==0)
						return 0;
					else
						return 1;
					break;
				default:
					return 1;
					break;
			}
			break;
		default:
			return 0;
			break;
	}
}
*/

uint8_t old2_mavlink_check_target(uint8_t sysid, uint8_t compid)
{
    if (!( (((uint8_t)sysid == (uint8_t)(MAV_SYSTEM_ID)) && (((uint8_t)compid == (uint8_t)(MAV_GCS_ID)) || ((uint8_t)compid == (uint8_t)(MAV_JS_ID)) || ((uint8_t)compid == (uint8_t)(MAV_BEAGLE_ID)))) || (((uint8_t)sysid == (uint8_t)0) && ((uint8_t)compid == (uint8_t)0)) ))
	{
		//gcs_send_text(PSTR("help"));
		//Serial.println(sysid);
		return 1;
	}
	return 0; // no error
}

uint8_t old_mavlink_check_target(uint8_t sysid, uint8_t compid)
{
    if (sysid != mavlink_system.sysid)
        return 1;
    // Currently we are not checking for correct compid since APM is not passing mavlink info to any subsystem
    // If it is addressed to our system ID we assume it is for us
    return 0; // no error
}
