// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Mavlink_compat.h"

// use this to prevent recursion during sensor init
static bool in_mavlink_delay;

// this costs us 51 bytes, but means that low priority
// messages don't block the CPU
static mavlink_statustext_t pending_status;

// true when we have received at least 1 MAVLink packet
static bool mavlink_active;

static void gcs_send_text_fmt(const prog_char_t *fmt, ...);

// check if a message will fit in the payload space available
#define CHECK_PAYLOAD_SIZE(id) if (payload_space < MAVLINK_MSG_ID_## id ##_LEN) return false

/*
  !!NOTE!!

  the use of NOINLINE separate functions for each message type avoids
  a compiler bug in gcc that would cause it to use far more stack
  space than is needed. Without the NOINLINE we use the sum of the
  stack needed for each message type. Please be careful to follow the
  pattern below when adding any new messages
 */

static NOINLINE void send_heartbeat(mavlink_channel_t chan)
{
#ifdef MAVLINK10
    uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    uint8_t system_status = MAV_STATE_ACTIVE;
    uint32_t custom_mode = control_mode;

    // work out the base_mode. This value is not very useful
    // for APM, but we calculate it as best we can so a generic
    // MAVLink enabled ground station can work out something about
    // what the MAV is up to. The actual bit values are highly
    // ambiguous for most of the APM flight modes. In practice, you
    // only get useful information from the custom_mode, which maps to
    // the APM flight mode and has a well defined meaning in the
    // ArduPlane documentation
    switch (control_mode) {
    case MANUAL:
        base_mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
        break;
    case STABILIZE:
    case FLY_BY_WIRE_A:
    case FLY_BY_WIRE_B:
    case FLY_BY_WIRE_C:
        base_mode = MAV_MODE_FLAG_STABILIZE_ENABLED;
        break;
    case AUTO:
    case RTL:
    case LOITER:
    case GUIDED:
    case CIRCLE:
        base_mode = MAV_MODE_FLAG_GUIDED_ENABLED |
                    MAV_MODE_FLAG_STABILIZE_ENABLED;
        // note that MAV_MODE_FLAG_AUTO_ENABLED does not match what
        // APM does in any mode, as that is defined as "system finds its own goal
        // positions", which APM does not currently do
        break;
    case INITIALISING:
        system_status = MAV_STATE_CALIBRATING;
        break;
    }

    if (control_mode != MANUAL && control_mode != INITIALISING) {
        // stabiliser of some form is enabled
        base_mode |= MAV_MODE_FLAG_STABILIZE_ENABLED;
    }

#if ENABLE_STICK_MIXING==ENABLED
    if (control_mode != INITIALISING) {
        // all modes except INITIALISING have some form of manual
        // override if stick mixing is enabled
        base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    }
#endif

#if HIL_MODE != HIL_MODE_DISABLED
    base_mode |= MAV_MODE_FLAG_HIL_ENABLED;
#endif

    // we are armed if we are not initialising
    if (control_mode != INITIALISING) {
        base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    // indicate we have set a custom mode
    base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

    mavlink_msg_heartbeat_send(
        chan,
        MAV_TYPE_FIXED_WING,
        MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode,
        custom_mode,
        system_status);
#else // MAVLINK10
    mavlink_msg_wID_heartbeat_send(
        chan,
		(uint8_t)MAV_SYSTEM_ID,
		(uint8_t)MAV_GCS_ID,
        mavlink_system.type,
        MAV_AUTOPILOT_ARDUPILOTMEGA,
		(uint8_t)2, // MavLink Version
		(uint8_t)MAV_SYSTEM_ID, // Sending the vehicle ID so the ground station can auto-configure
		(uint8_t)g.command_total.get());
#endif // MAVLINK10
	if ((airspeed < 100*(STALL_SPEED+STALL_SPEED_BUFFER)) & (current_loc.alt > RUDDER2STEER_ALT_THRESHOLD))
		gcs_send_text_fmt(PSTR("Approaching STALL"));
}

static NOINLINE void send_attitude(mavlink_channel_t chan)
{
    Vector3f omega = dcm.get_gyro();
    mavlink_msg_wID_attitude_send(
        chan,
		(uint8_t)MAV_SYSTEM_ID,
		(uint8_t)MAV_GCS_ID,
        micros(),
        dcm.roll,
        dcm.pitch-(float)ToRad(g.pitch_trim/100),
        dcm.yaw,
        omega.x,
        omega.y,
        omega.z);
}

#if GEOFENCE_ENABLED == ENABLED
static NOINLINE void send_fence_status(mavlink_channel_t chan)
{
    geofence_send_status(chan);
}
#endif


static NOINLINE void send_extended_status1(mavlink_channel_t chan, uint16_t packet_drops)
{
#ifdef MAVLINK10
    uint32_t control_sensors_present = 0;
    uint32_t control_sensors_enabled;
    uint32_t control_sensors_health;

    // first what sensors/controllers we have
    control_sensors_present |= (1<<0); // 3D gyro present
    control_sensors_present |= (1<<1); // 3D accelerometer present
    if (g.compass_enabled) {
        control_sensors_present |= (1<<2); // compass present
    }
    control_sensors_present |= (1<<3); // absolute pressure sensor present
    if (g_gps->fix) {
        control_sensors_present |= (1<<5); // GPS present
    }
    control_sensors_present |= (1<<10); // 3D angular rate control
    control_sensors_present |= (1<<11); // attitude stabilisation
    control_sensors_present |= (1<<12); // yaw position
    control_sensors_present |= (1<<13); // altitude control
    control_sensors_present |= (1<<14); // X/Y position control
    control_sensors_present |= (1<<15); // motor control

    // now what sensors/controllers are enabled

    // first the sensors
    control_sensors_enabled = control_sensors_present & 0x1FF;

    // now the controllers
    control_sensors_enabled = control_sensors_present & 0x1FF;

    switch (control_mode) {
    case MANUAL:
        break;

    case STABILIZE:
    case FLY_BY_WIRE_A:
        control_sensors_enabled |= (1<<10); // 3D angular rate control
        control_sensors_enabled |= (1<<11); // attitude stabilisation
        break;

    case FLY_BY_WIRE_B:
        control_sensors_enabled |= (1<<10); // 3D angular rate control
        control_sensors_enabled |= (1<<11); // attitude stabilisation
        control_sensors_enabled |= (1<<15); // motor control
        break;

    case FLY_BY_WIRE_C:
        control_sensors_enabled |= (1<<10); // 3D angular rate control
        control_sensors_enabled |= (1<<11); // attitude stabilisation
        control_sensors_enabled |= (1<<13); // altitude control
        control_sensors_enabled |= (1<<15); // motor control
        break;

    case AUTO:
    case RTL:
    case LOITER:
    case GUIDED:
    case CIRCLE:
        control_sensors_enabled |= (1<<10); // 3D angular rate control
        control_sensors_enabled |= (1<<11); // attitude stabilisation
        control_sensors_enabled |= (1<<12); // yaw position
        control_sensors_enabled |= (1<<13); // altitude control
        control_sensors_enabled |= (1<<14); // X/Y position control
        control_sensors_enabled |= (1<<15); // motor control
        break;

    case INITIALISING:
        break;
    }

    // at the moment all sensors/controllers are assumed healthy
    control_sensors_health = control_sensors_present;

    uint16_t battery_current = -1;
    uint8_t  battery_remaining = -1;

    if (current_total != 0 && g.pack_capacity != 0) {
        battery_remaining = (100.0 * (g.pack_capacity - current_total) / g.pack_capacity);
    }
    if (current_total != 0) {
        battery_current = current_amps * 100;
    }

    mavlink_msg_sys_status_send(
        chan,
        control_sensors_present,
        control_sensors_enabled,
        control_sensors_health,
        (uint16_t)(load * 1000),
        battery_voltage * 1000, // mV
        battery_current,        // in 10mA units
        battery_remaining,      // in %
        0, // comm drops %,
        0, // comm drops in pkts,
        0, 0, 0, 0);

#else // MAVLINK10
        uint8_t mode 	 = MAV_MODE_UNINIT;
        uint8_t nav_mode = MAV_NAV_VECTOR;

        switch(control_mode) {
        case MANUAL:
            mode 		= MAV_MODE_MANUAL;
            break;
        case STABILIZE:
            mode 		= MAV_MODE_TEST1;
            break;
        case FLY_BY_WIRE_A:
            mode 		= MAV_MODE_TEST2;
            nav_mode 	= 1;				//FBW nav_mode mapping;  1=A, 2=B, 3=C, etc.
            break;
        case FLY_BY_WIRE_B:
            mode 		= MAV_MODE_TEST2;
            nav_mode 	= 2;				//FBW nav_mode mapping;  1=A, 2=B, 3=C, etc.
            break;
        case GUIDED:
            mode 		= MAV_MODE_GUIDED;
            break;
        case AUTO:
            mode 		= MAV_MODE_AUTO;
            nav_mode 	= MAV_NAV_WAYPOINT;
            break;
        case RTL:
            mode 		= MAV_MODE_AUTO;
            nav_mode 	= MAV_NAV_RETURNING;
            break;
        case LOITER:
            mode 		= MAV_MODE_AUTO;
            nav_mode 	= MAV_NAV_LOITER;
            break;
        case INITIALISING:
            mode 		= MAV_MODE_UNINIT;
            nav_mode 	= MAV_NAV_GROUNDED;
            break;
        case CIRCLE:
            mode        = MAV_MODE_TEST3;
            break;
        }

        uint8_t status 		= MAV_STATE_ACTIVE;
        uint16_t battery_remaining = 1000.0 * (float)(g.pack_capacity - current_total)/(float)g.pack_capacity;	//Mavlink scaling 100% = 1000

        mavlink_msg_wID_sys_status_send(
            chan,
			(uint8_t)MAV_SYSTEM_ID,
			(uint8_t)MAV_GCS_ID,
            mode,
            nav_mode,
            status,
            load * 1000,
            battery_voltage * 1000,
            battery_remaining,
            packet_drops);
#endif // MAVLINK10
}

static void NOINLINE send_meminfo(mavlink_channel_t chan)
{
    extern unsigned __brkval;
    mavlink_msg_wID_meminfo_send(chan, (uint8_t)MAV_SYSTEM_ID, (uint8_t)MAV_GCS_ID, __brkval, memcheck_available_memory());
}

static void NOINLINE send_location(mavlink_channel_t chan)
{
#if SLV_ADDED == ENABLED
	Matrix3f rot = dcm.get_dcm_matrix(); // neglecting angle of attack for now
    mavlink_msg_wID_global_position_int_send(
        chan,
		(uint8_t)MAV_SYSTEM_ID,
		(uint8_t)MAV_GCS_ID,
        current_loc.lat,                // in 1E7 degrees
        current_loc.lng,                // in 1E7 degrees
        g_gps->altitude*10,             // millimeters above sea level
        g_gps->ground_speed * rot.a.x,  // X speed cm/s
        g_gps->ground_speed * rot.b.x,  // Y speed cm/s
        g_gps->ground_speed * rot.c.x);          // course in 1/100 degree
#else
	#error SLV_ADDED DISABLED.
	Matrix3f rot = dcm.get_dcm_matrix(); // neglecting angle of attack for now
    mavlink_msg_global_position_int_send(
        chan,
        millis(),
        current_loc.lat,                // in 1E7 degrees
        current_loc.lng,                // in 1E7 degrees
        g_gps->altitude*10,             // millimeters above sea level
        current_loc.alt * 10,           // millimeters above ground
        g_gps->ground_speed * rot.a.x,  // X speed cm/s
        g_gps->ground_speed * rot.b.x,  // Y speed cm/s
        g_gps->ground_speed * rot.c.x,
        g_gps->ground_course);          // course in 1/100 degree
#endif
}

static void NOINLINE send_nav_controller_output(mavlink_channel_t chan)
{
#if SLV_ADDED == ENABLED

		//Serial.print("Mav Bearing ");
		//Serial.println(nav_bearing);
		//Serial.println((double)target_altitude/100.0);

	mavlink_msg_wID_nav_controller_output_send(
        chan,
		(uint8_t)MAV_SYSTEM_ID,
		(uint8_t)MAV_GCS_ID,
        nav_roll / 1.0e2,
        nav_pitch / 1.0e2,
        nav_bearing / 1.0e2,
        target_bearing / 1.0e2,
        wp_distance,
        target_altitude / 1.0e2,
        target_airspeed / 1.0e2,
        crosstrack_error);
#else
	#error SLV_ADDED DISABLED.
	mavlink_msg_nav_controller_output_send(
        chan,
        nav_roll / 1.0e2,
        nav_pitch / 1.0e2,
        nav_bearing / 1.0e2,
        target_bearing / 1.0e2,
        wp_distance,
        altitude_error / 1.0e2,
        airspeed_error,
        crosstrack_error);
#endif
}

static void NOINLINE send_gps_raw(mavlink_channel_t chan)
{
#ifdef MAVLINK10
    uint8_t fix;
    if (g_gps->status() == 2) {
        fix = 3;
    } else {
        fix = 0;
    }

    mavlink_msg_gps_raw_int_send(
        chan,
        micros(),
        fix,
        g_gps->latitude,      // in 1E7 degrees
        g_gps->longitude,     // in 1E7 degrees
        g_gps->altitude * 10, // in mm
        g_gps->hdop,
        65535,
        g_gps->ground_speed,  // cm/s
        g_gps->ground_course, // 1/100 degrees,
        g_gps->num_sats);

#else // MAVLINK10
        mavlink_msg_wID_gps_raw_send(
            chan,
			(uint8_t)MAV_SYSTEM_ID,
			(uint8_t)MAV_GCS_ID,
            micros(),
            g_gps->status(),
            g_gps->latitude / 1.0e7,
            g_gps->longitude / 1.0e7,
            g_gps->altitude / 100.0,
            g_gps->hdop,
            0.0,
            g_gps->ground_speed / 100.0,
            g_gps->ground_course / 100.0);
#endif  // MAVLINK10
}

static void NOINLINE send_servo_out(mavlink_channel_t chan)
{
    const uint8_t rssi = 1;
    // normalized values scaled to -10000 to 10000
    // This is used for HIL.  Do not change without discussing with
    // HIL maintainers
#if SLV_ADDED == ENABLED
	mavlink_msg_wID_rc_channels_scaled_send(
        chan,
		(uint8_t)MAV_SYSTEM_ID,
		(uint8_t)MAV_GCS_ID,
        10000 * g.channel_roll.norm_output(),
        10000 * g.channel_pitch.norm_output(),
        10000 * g.channel_throttle.norm_output(),
        10000 * g.channel_rudder.norm_output(),
        10000 * g.rc_5.norm_output(),
	   -10000 * g.rc_6.norm_output(),
		10000 * g.rc_7.norm_output(),
        10000 * g.rc_8.norm_output(),
        rssi);
#else
	#error SLV_ADDED DISABLED.
    mavlink_msg_rc_channels_scaled_send(
        chan,
        millis(),
        0, // port 0
        10000 * g.channel_roll.norm_output(),
        10000 * g.channel_pitch.norm_output(),
        10000 * g.channel_throttle.norm_output(),
        10000 * g.channel_rudder.norm_output(),
        0,
        0,
        0,
        0,
        rssi);
#endif
}

static void NOINLINE send_radio_in(mavlink_channel_t chan)
{
#if SLV_ADDED == ENABLED
    uint8_t rssi = 1;
    mavlink_msg_wID_rc_channels_raw_send(
		chan,
		(uint8_t)MAV_SYSTEM_ID,
		(uint8_t)MAV_GCS_ID,
        g.channel_roll.radio_in,
        g.channel_pitch.radio_in,
        g.channel_throttle.radio_in,
        g.channel_rudder.radio_in,
        g.rc_5.radio_in,       // XXX currently only 4 RC channels defined
		g.rc_6.radio_in,
		g.rc_7.radio_in,
        g.rc_8.radio_in,
        rssi);
#else
	#error SLV_ADDED DISABLED.
    uint8_t rssi = 1;
    mavlink_msg_rc_channels_raw_send(
		chan,
        millis(),
        0, // port
        g.channel_roll.radio_in,
        g.channel_pitch.radio_in,
        g.channel_throttle.radio_in,
        g.channel_rudder.radio_in,
        g.rc_5.radio_in,       // XXX currently only 4 RC channels defined
        g.rc_6.radio_in,
        g.rc_7.radio_in,
        g.rc_8.radio_in,
        rssi);
#endif
}

#if SLV_ADDED == ENABLED
static void NOINLINE send_state_data(mavlink_channel_t chan)
{
	mavlink_msg_wID_state_data_send(
		chan,
		(uint8_t)MAV_SYSTEM_ID,
		(uint8_t)MAV_GCS_ID,
		(uint8_t)MAV_SYSTEM_ID,
		(loiter_sum>300)? (byte)1 : (byte)0,
		control_mode,
		micros());
}
#else
	#error SLV_ADDED DISABLED.
#endif


#if SLV_ADDED == ENABLED
static void NOINLINE send_cdnr_controller(mavlink_channel_t chan)
{
	mavlink_msg_wID_cdnr_controller_send(
		chan,
		(uint8_t)MAV_SYSTEM_ID,
		(uint8_t)MAV_GCS_ID,
		cdnr_hflag,
		cdnr_sflag,
		cdnr_aflag,
		cdnr_tflag,
		nav_bearing, 
		target_airspeed,
		target_altitude, 
		(int16_t)0);
}
#else
	#error SLV_ADDED DISABLED.
#endif


#if SLV_ADDED == ENABLED
static void NOINLINE send_analog_eu(mavlink_channel_t chan)
{
	mavlink_msg_wID_analog_eu_send(
        chan,
		(uint8_t)MAV_SYSTEM_ID,
		(uint8_t)MAV_GCS_ID,
        ANALOG_EU_01,
        ANALOG_EU_02,
        ANALOG_EU_03,
        ANALOG_EU_04,
        ANALOG_EU_05,
        ANALOG_EU_06,
        ANALOG_EU_07,
        ANALOG_EU_08,
        ANALOG_EU_09,
        ANALOG_EU_10,
        ANALOG_EU_11,
        ANALOG_EU_12,
        ANALOG_EU_13,
        ANALOG_EU_14,
        ANALOG_EU_15,
        ANALOG_EU_16,
        micros());
}
static void NOINLINE send_analog_raw(mavlink_channel_t chan)
{
    mavlink_msg_wID_analog_raw_send(
        chan,
		(uint8_t)MAV_SYSTEM_ID,
		(uint8_t)MAV_GCS_ID,
        ac_ch[0],
        ac_ch[1],
        ac_ch[2],
        ac_ch[3],
        ac_ch[4],
        ac_ch[5],
        ac_ch[6],
        ac_ch[7],
        ac_ch[8],
        ac_ch[9],
        ac_ch[10],
        ac_ch[11],
        ac_ch[12],
        ac_ch[13],
        ac_ch[14],
        ac_ch[15],
        micros());
}
#else
	#error SLV_ADDED DISABLED.
#endif

static void NOINLINE send_radio_out(mavlink_channel_t chan)
{
#if SLV_ADDED == ENABLED
	mavlink_msg_wID_servo_output_raw_send(
            chan,
			(uint8_t)MAV_SYSTEM_ID,
			(uint8_t)MAV_GCS_ID,
            g.channel_roll.radio_out,
            g.channel_pitch.radio_out,
            g.channel_throttle.radio_out,
            g.channel_rudder.radio_out,
            g.rc_5.radio_out,       // XXX currently only 4 RC channels defined
			g.rc_6.radio_out,
			g.rc_7.radio_out,
            g.rc_8.radio_out);
#else
	#error SLV_ADDED DISABLED.
	mavlink_msg_servo_output_raw_send(
            chan,
            micros(),
            0, // port
            g.channel_roll.radio_out,
            g.channel_pitch.radio_out,
            g.channel_throttle.radio_out,
            g.channel_rudder.radio_out,
            g.rc_5.radio_out,       // XXX currently only 4 RC channels defined
            g.rc_6.radio_out,
            g.rc_7.radio_out,
            g.rc_8.radio_out);
#endif
}

static void NOINLINE send_vfr_hud(mavlink_channel_t chan)
{
#ifdef SLV_ADDED
	mavlink_msg_wID_vfr_hud_send(
        chan,
		(uint8_t)MAV_SYSTEM_ID,
		(uint8_t)MAV_GCS_ID,
        (float)airspeed / 100.0,
        (float)g_gps->ground_speed / 100.0,
        (dcm.yaw_sensor / 100) % 360,
        (uint16_t)(100 * g.channel_throttle.norm_output()),
        (float)current_loc.alt/100.0f,
        baro_alt/100.0f);
#else
	mavlink_msg_vfr_hud_send(
        chan,
        (float)airspeed / 100.0,
        (float)g_gps->ground_speed / 100.0,
        (dcm.yaw_sensor / 100) % 360,
        (uint16_t)(100 * g.channel_throttle.norm_output()),
        current_loc.alt / 100.0,
        0);
#endif
}

static void NOINLINE send_raw_imu1(mavlink_channel_t chan)
{
    Vector3f accel = imu.get_accel();
    Vector3f gyro = imu.get_gyro();

    mavlink_msg_wID_raw_imu_send(
        chan,
		(uint8_t)MAV_SYSTEM_ID,
		(uint8_t)MAV_GCS_ID,
        micros(),
        accel.x * 1000.0 / gravity,
        accel.y * 1000.0 / gravity,
        accel.z * 1000.0 / gravity,
        gyro.x * 1000.0,
        gyro.y * 1000.0,
        gyro.z * 1000.0,
        compass.mag_x,
        compass.mag_y,
        compass.mag_z);
}

#if HIL_MODE != HIL_MODE_ATTITUDE
static void NOINLINE send_raw_imu2(mavlink_channel_t chan)
{
    int32_t pressure = barometer.get_pressure();
    mavlink_msg_wID_scaled_pressure_send(
        chan,
		(uint8_t)MAV_SYSTEM_ID,
		(uint8_t)MAV_GCS_ID,
        micros(),
        pressure/100.0,
        (pressure - g.ground_pressure)/100.0,
        barometer.get_temperature());
}

static void NOINLINE send_raw_imu3(mavlink_channel_t chan)
{
    Vector3f mag_offsets = compass.get_offsets();

    mavlink_msg_wID_sensor_offsets_send(chan,
									(uint8_t)MAV_SYSTEM_ID,
									(uint8_t)MAV_GCS_ID,
                                    mag_offsets.x,
                                    mag_offsets.y,
                                    mag_offsets.z,
                                    compass.get_declination(),
                                    barometer.get_raw_pressure(),
                                    barometer.get_raw_temp(),
                                    imu.gx(), imu.gy(), imu.gz(),
                                    imu.ax(), imu.ay(), imu.az());
}
#endif // HIL_MODE != HIL_MODE_ATTITUDE

static void NOINLINE send_gps_status(mavlink_channel_t chan)
{
    mavlink_msg_wID_gps_status_send(
        chan,
		(uint8_t)MAV_SYSTEM_ID,
		(uint8_t)MAV_GCS_ID,
        g_gps->num_sats,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL);
}

static void NOINLINE send_current_waypoint(mavlink_channel_t chan)
{
    mavlink_msg_wID_waypoint_current_send(
        chan,
		(uint8_t)MAV_SYSTEM_ID,
		(uint8_t)MAV_GCS_ID,
        g.command_index);
}

static void NOINLINE send_statustext(mavlink_channel_t chan)
{
    mavlink_msg_wID_statustext_send(
        chan,
		(uint8_t)MAV_SYSTEM_ID,
		(uint8_t)MAV_GCS_ID,
        pending_status.severity,
        pending_status.text);
}


// try to send a message, return false if it won't fit in the serial tx buffer
static bool mavlink_try_send_message(mavlink_channel_t chan, enum ap_message id, uint16_t packet_drops)
{
    int payload_space = comm_get_txspace(chan) - MAVLINK_NUM_NON_PAYLOAD_BYTES;

    if (chan == MAVLINK_COMM_1 && millis() < MAVLINK_TELEMETRY_PORT_DELAY) {
        // defer any messages on the telemetry port for 1 second after
        // bootup, to try to prevent bricking of Xbees
        return false;
    }

    switch (id) {
    case MSG_HEARTBEAT:
        CHECK_PAYLOAD_SIZE(HEARTBEAT);
        send_heartbeat(chan);
        return true;

    case MSG_EXTENDED_STATUS1:
        CHECK_PAYLOAD_SIZE(SYS_STATUS);
        send_extended_status1(chan, packet_drops);
        break;

    case MSG_EXTENDED_STATUS2:
        CHECK_PAYLOAD_SIZE(MEMINFO);
        send_meminfo(chan);
        break;

    case MSG_ATTITUDE:
        CHECK_PAYLOAD_SIZE(ATTITUDE);
        send_attitude(chan);
        break;

    case MSG_LOCATION:
        CHECK_PAYLOAD_SIZE(GLOBAL_POSITION_INT);
        send_location(chan);
        break;

    case MSG_NAV_CONTROLLER_OUTPUT:
#if SLV_ADDED == ENABLED
		CHECK_PAYLOAD_SIZE(NAV_CONTROLLER_OUTPUT);
        send_nav_controller_output(chan);
#else
		if (control_mode != MANUAL) {
            CHECK_PAYLOAD_SIZE(NAV_CONTROLLER_OUTPUT);
            send_nav_controller_output(chan);
        }
#endif
		break;

    case MSG_GPS_RAW:
#ifdef MAVLINK10
        CHECK_PAYLOAD_SIZE(GPS_RAW_INT);
#else
        CHECK_PAYLOAD_SIZE(GPS_RAW);
#endif
        send_gps_raw(chan);
        break;

    case MSG_SERVO_OUT:
        CHECK_PAYLOAD_SIZE(RC_CHANNELS_SCALED);
        send_servo_out(chan);
        break;

    case MSG_RADIO_IN:
        CHECK_PAYLOAD_SIZE(RC_CHANNELS_RAW);
        send_radio_in(chan);
        break;

#if SLV_ADDED == ENABLED
	case MSG_ANALOG_RAW:
        CHECK_PAYLOAD_SIZE(ANALOG_RAW);
        send_analog_raw(chan);
        break;
	case MSG_ANALOG_EU:
        CHECK_PAYLOAD_SIZE(ANALOG_EU);
        send_analog_eu(chan);
        break;
	case MSG_CDNR_CONTROLLER:
        CHECK_PAYLOAD_SIZE(CDNR_CONTROLLER);
        send_cdnr_controller(chan);
        break;
	case MSG_STATE_DATA:
        CHECK_PAYLOAD_SIZE(STATE_DATA);
        send_state_data(chan);
        break;
#else
	#error SLV_ADDED DISABLED.
#endif

    case MSG_RADIO_OUT:
        CHECK_PAYLOAD_SIZE(SERVO_OUTPUT_RAW);
        send_radio_out(chan);
        break;

    case MSG_VFR_HUD:
        CHECK_PAYLOAD_SIZE(VFR_HUD);
        send_vfr_hud(chan);
        break;

    case MSG_RAW_IMU1:
        CHECK_PAYLOAD_SIZE(RAW_IMU);
        send_raw_imu1(chan);
        break;

#if HIL_MODE != HIL_MODE_ATTITUDE
    case MSG_RAW_IMU2:
        CHECK_PAYLOAD_SIZE(SCALED_PRESSURE);
        send_raw_imu2(chan);

        break;
    case MSG_RAW_IMU3:
        CHECK_PAYLOAD_SIZE(SENSOR_OFFSETS);
        send_raw_imu3(chan);
        break;
#endif // HIL_MODE != HIL_MODE_ATTITUDE

    case MSG_GPS_STATUS:
        CHECK_PAYLOAD_SIZE(GPS_STATUS);
        send_gps_status(chan);
        break;

    case MSG_CURRENT_WAYPOINT:
        CHECK_PAYLOAD_SIZE(WAYPOINT_CURRENT);
        send_current_waypoint(chan);
        break;

    case MSG_NEXT_PARAM:
        CHECK_PAYLOAD_SIZE(PARAM_VALUE);
        if (chan == MAVLINK_COMM_0) {
            gcs0.queued_param_send();
        } else if (gcs3.initialised) {
            gcs3.queued_param_send();
        }
        break;

    case MSG_NEXT_WAYPOINT:
        CHECK_PAYLOAD_SIZE(WAYPOINT_REQUEST);
        if (chan == MAVLINK_COMM_0) {
            gcs0.queued_waypoint_send();
        } else if (gcs3.initialised) {
            gcs3.queued_waypoint_send();
        }
        break;

    case MSG_STATUSTEXT:
        CHECK_PAYLOAD_SIZE(STATUSTEXT);
        send_statustext(chan);
        break;

#if GEOFENCE_ENABLED == ENABLED
    case MSG_FENCE_STATUS:
        CHECK_PAYLOAD_SIZE(FENCE_STATUS);
        send_fence_status(chan);
        break;
#endif

    case MSG_RETRY_DEFERRED:
        break; // just here to prevent a warning
	}
    return true;
}


#define MAX_DEFERRED_MESSAGES MSG_RETRY_DEFERRED
static struct mavlink_queue {
    enum ap_message deferred_messages[MAX_DEFERRED_MESSAGES];
    uint8_t next_deferred_message;
    uint8_t num_deferred_messages;
} mavlink_queue[2];

// send a message using mavlink
static void mavlink_send_message(mavlink_channel_t chan, enum ap_message id, uint16_t packet_drops)
{
    uint8_t i, nextid;
    struct mavlink_queue *q = &mavlink_queue[(uint8_t)chan];

    // see if we can send the deferred messages, if any
    while (q->num_deferred_messages != 0) {
        if (!mavlink_try_send_message(chan,
                                      q->deferred_messages[q->next_deferred_message],
                                      packet_drops)) {
            break;
        }
        q->next_deferred_message++;
        if (q->next_deferred_message == MAX_DEFERRED_MESSAGES) {
            q->next_deferred_message = 0;
        }
        q->num_deferred_messages--;
    }

    if (id == MSG_RETRY_DEFERRED) {
        return;
    }

    // this message id might already be deferred
    for (i=0, nextid = q->next_deferred_message; i < q->num_deferred_messages; i++) {
        if (q->deferred_messages[nextid] == id) {
            // its already deferred, discard
            return;
        }
        nextid++;
        if (nextid == MAX_DEFERRED_MESSAGES) {
            nextid = 0;
        }
    }

    if (q->num_deferred_messages != 0 ||
        !mavlink_try_send_message(chan, id, packet_drops)) {
        // can't send it now, so defer it
        if (q->num_deferred_messages == MAX_DEFERRED_MESSAGES) {
            // the defer buffer is full, discard
            return;
        }
        nextid = q->next_deferred_message + q->num_deferred_messages;
        if (nextid >= MAX_DEFERRED_MESSAGES) {
            nextid -= MAX_DEFERRED_MESSAGES;
        }
        q->deferred_messages[nextid] = id;
        q->num_deferred_messages++;
    }
}

void mavlink_send_text(mavlink_channel_t chan, gcs_severity severity, const char *str)
{
    if (chan == MAVLINK_COMM_1 && millis() < MAVLINK_TELEMETRY_PORT_DELAY) {
        // don't send status MAVLink messages for 2 seconds after
        // bootup, to try to prevent Xbee bricking
        return;
    }

    if (severity == SEVERITY_LOW) {
        // send via the deferred queuing system
        pending_status.severity = (uint8_t)severity;
        strncpy((char *)pending_status.text, str, sizeof(pending_status.text));
        mavlink_send_message(chan, MSG_STATUSTEXT, 0);
    } else {
        // send immediately
        mavlink_msg_wID_statustext_send(chan, (uint8_t)MAV_SYSTEM_ID, (uint8_t)MAV_GCS_ID, severity, str);
    }
}


GCS_MAVLINK::GCS_MAVLINK(AP_Var::Key key) :
packet_drops(0),

// parameters
// note, all values not explicitly initialised here are zeroed
waypoint_send_timeout(1000), // 1 second
waypoint_receive_timeout(1000), // 1 second

// stream rates
_group	(key, key == Parameters::k_param_streamrates_port0 ? PSTR("SR0_"): PSTR("SR3_")),
				// AP_VAR					//ref	 //index, default, 	name
				streamRateRawSensors		(&_group, 0, 		0,		 PSTR("RAW_SENS")),
				streamRateExtendedStatus	(&_group, 1, 		0,		 PSTR("EXT_STAT")),
				streamRateRCChannels		(&_group, 2, 		0,		 PSTR("RC_CHAN")),
				streamRateRawController		(&_group, 3, 		0,		 PSTR("RAW_CTRL")),
				streamRatePosition			(&_group, 4, 		0,		 PSTR("POSITION")),
				streamRateExtra1			(&_group, 5, 		0,		 PSTR("EXTRA1")),
				streamRateExtra2			(&_group, 6, 		0,		 PSTR("EXTRA2")),
				streamRateExtra3			(&_group, 7, 		0,		 PSTR("EXTRA3"))
{

}

void
GCS_MAVLINK::init(FastSerial * port)
{
    GCS_Class::init(port);
    if (port == &Serial) {
        mavlink_comm_0_port = port;
        chan = MAVLINK_COMM_0;
    }else{
        mavlink_comm_1_port = port;
        chan = MAVLINK_COMM_1;
    }
	_queued_parameter = NULL;
}

void
GCS_MAVLINK::update(void)
{
    // receive new packets
    mavlink_message_t msg;
    mavlink_status_t status;
	status.packet_rx_drop_count = 0;

    // process received bytes
    while (comm_get_available(chan))
    {
        uint8_t c = comm_receive_ch(chan);

#if CLI_ENABLED == ENABLED
        /* allow CLI to be started by hitting enter 3 times, if no
           heartbeat packets have been received */
        if (mavlink_active == 0 && millis() < 20000) {
            if (c == '\n' || c == '\r') {
                crlf_count++;
            } else {
                crlf_count = 0;
            }
            if (crlf_count == 3) {
                run_cli();
            }
        }
#endif

        // Try to get a new message
        if (mavlink_parse_char(chan, c, &msg, &status)) {
            mavlink_active = 1;
            handleMessage(&msg);
        }
    }

    // Update packet drops counter
    packet_drops += status.packet_rx_drop_count;

    // send out queued params/ waypoints
    if (NULL != _queued_parameter) {
        send_message(MSG_NEXT_PARAM);
    }

    if (!waypoint_receiving && !waypoint_sending) {
        return;
    }

    uint32_t tnow = millis();

    if (waypoint_receiving &&
        waypoint_request_i <= (unsigned)g.command_total &&
        tnow > waypoint_timelast_request + 500) {
        waypoint_timelast_request = tnow;
        send_message(MSG_NEXT_WAYPOINT);
    }

    // stop waypoint sending if timeout
    if (waypoint_sending && (millis() - waypoint_timelast_send) > waypoint_send_timeout){
        waypoint_sending = false;
    }

    // stop waypoint receiving if timeout
    if (waypoint_receiving && (millis() - waypoint_timelast_receive) > waypoint_receive_timeout){
        waypoint_receiving = false;
    }
}

void
GCS_MAVLINK::data_stream_send(uint16_t freqMin, uint16_t freqMax)
{
	if (waypoint_sending == false && waypoint_receiving == false && _queued_parameter == NULL) {

		if (freqLoopMatch(streamRateRawSensors, freqMin, freqMax)){
			send_message(MSG_RAW_IMU1);
			send_message(MSG_RAW_IMU2);
			send_message(MSG_RAW_IMU3);
		}

		if (freqLoopMatch(streamRateExtendedStatus, freqMin, freqMax)) {
			send_message(MSG_EXTENDED_STATUS1);
			send_message(MSG_EXTENDED_STATUS2);
			send_message(MSG_GPS_STATUS);
			send_message(MSG_CURRENT_WAYPOINT);
			send_message(MSG_GPS_RAW);            // TODO - remove this message after location message is working
			send_message(MSG_NAV_CONTROLLER_OUTPUT);
			send_message(MSG_FENCE_STATUS);
		}

		if (freqLoopMatch(streamRatePosition, freqMin, freqMax)) {
			// sent with GPS read
			send_message(MSG_LOCATION);
		}

		if (freqLoopMatch(streamRateRawController, freqMin, freqMax)) {
			// This is used for HIL.  Do not change without discussing with HIL maintainers
			send_message(MSG_SERVO_OUT);
		}

		if (freqLoopMatch(streamRateRCChannels, freqMin, freqMax)) {
			send_message(MSG_RADIO_OUT);
			send_message(MSG_RADIO_IN);
		}

		if (freqLoopMatch(streamRateExtra1, freqMin, freqMax)){	 // Use Extra 1 for AHRS info
			send_message(MSG_ATTITUDE);
		}

		if (freqLoopMatch(streamRateExtra2, freqMin, freqMax)){		// Use Extra 2 for additional HIL info
			send_message(MSG_VFR_HUD);
		}

		if (freqLoopMatch(streamRateExtra3, freqMin, freqMax)){
#if SLV_ADDED == ENABLED
			send_message(MSG_ANALOG_EU);
			send_message(MSG_STATE_DATA);
#else
	#error SLV_ADDED DISABLED.
			// Available datastream
#endif
		}
	}
}



void
GCS_MAVLINK::send_message(enum ap_message id)
{
    mavlink_send_message(chan,id, packet_drops);
}

void
GCS_MAVLINK::send_text(gcs_severity severity, const char *str)
{
    mavlink_send_text(chan,severity,str);
}

void
GCS_MAVLINK::send_text(gcs_severity severity, const prog_char_t *str)
{
    mavlink_statustext_t m;
    uint8_t i;
    for (i=0; i<sizeof(m.text); i++) {
        m.text[i] = pgm_read_byte((const prog_char *)(str++));
    }
    if (i < sizeof(m.text)) m.text[i] = 0;
    mavlink_send_text(chan, severity, (const char *)m.text);
}

int mavlink_check_target(int APID, int SOURCEID)
{
	if (APID==MAV_SYSTEM_ID && SOURCEID==MAV_GCS_ID)
	{
		//gcs_send_text_fmt(PSTR("AP/GCS Match: [A%d G%d]==[A%d G%d]"),APID, SOURCEID,MAV_SYSTEM_ID,MAV_GCS_ID);
		return 0;
	}
	if (APID==MAV_SYSTEM_ID && SOURCEID==MAV_JS_ID)
	{
		gcs_send_text_fmt(PSTR("AP/JS Match: [A%d G%d]==[A%d G%d]"),APID, SOURCEID,MAV_SYSTEM_ID,MAV_JS_ID);
		return 0;
	}
	if (APID==MAV_SYSTEM_ID && SOURCEID==MAV_BEAGLE_ID)
	{
		gcs_send_text_fmt(PSTR("AP/BEAGLE Match: [A%d G%d]==[A%d G%d]"),APID, SOURCEID,MAV_SYSTEM_ID,MAV_BEAGLE_ID);
		return 0;
	}
	if (APID==0 && SOURCEID==0)
	{
		//gcs_send_text_fmt(PSTR("AP/NONE Match: [A%d G%d]==[A%d G%d]"),APID, SOURCEID,0,0);
		return 0;
	}
	gcs_send_text_fmt(PSTR("AP Reject: [A%d G%d]!=[A%d G%d]"),APID, SOURCEID,MAV_SYSTEM_ID,MAV_GCS_ID);
	return 1;
}

void GCS_MAVLINK::handleMessage(mavlink_message_t* msg)
{
    struct Location tell_command = {};		// command for telemetry
	static uint8_t mav_nav = (uint8_t)MAV_GCS_ID;	// For setting mode (some require receipt of 2 messages...)

	
	if (mavlink_check_target((int)msg->compid, (int)msg->sysid))
		return;

    switch (msg->msgid) {

	case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
        {
            // decode
            mavlink_request_data_stream_t packet;
            mavlink_msg_request_data_stream_decode(msg, &packet);

			if (mavlink_check_target(packet.target_system, packet.target_component))
				break;

            int freq = 0; // packet frequency

			if (packet.start_stop == 0)
				freq = 0; // stop sending
			else if (packet.start_stop == 1)
				freq = packet.req_message_rate; // start sending
			else
				break;

            switch(packet.req_stream_id){

                case MAV_DATA_STREAM_ALL:
                    streamRateRawSensors = freq;
                    streamRateExtendedStatus = freq;
                    streamRateRCChannels = freq;
                    streamRateRawController = freq;
                    streamRatePosition = freq;
                    streamRateExtra1 = freq;
                    streamRateExtra2 = freq;
                    streamRateExtra3.set_and_save(freq);	// We just do set and save on the last as it takes care of the whole group.
                    break;

                case MAV_DATA_STREAM_RAW_SENSORS:
                    streamRateRawSensors = freq;		// We do not set and save this one so that if HIL is shut down incorrectly
														// we will not continue to broadcast raw sensor data at 50Hz.
                    break;
                case MAV_DATA_STREAM_EXTENDED_STATUS:
                    streamRateExtendedStatus.set_and_save(freq);
                    break;

                case MAV_DATA_STREAM_RC_CHANNELS:
                    streamRateRCChannels.set_and_save(freq);
                    break;

                case MAV_DATA_STREAM_RAW_CONTROLLER:
                    streamRateRawController.set_and_save(freq);
                    break;

				//case MAV_DATA_STREAM_RAW_SENSOR_FUSION:
				//    streamRateRawSensorFusion.set_and_save(freq);
				//    break;

                case MAV_DATA_STREAM_POSITION:
                    streamRatePosition.set_and_save(freq);
                    break;

                case MAV_DATA_STREAM_EXTRA1:
                    streamRateExtra1.set_and_save(freq);
                    break;

                case MAV_DATA_STREAM_EXTRA2:
                    streamRateExtra2.set_and_save(freq);
                    break;

                case MAV_DATA_STREAM_EXTRA3:
                    streamRateExtra3.set_and_save(freq);
                    break;

                default:
                    break;
            }
            break;
        }

#ifdef MAVLINK10
    case MAVLINK_MSG_ID_COMMAND_LONG:
        {
            // decode
            mavlink_command_long_t packet;
            mavlink_msg_command_long_decode(msg, &packet);
            if (mavlink_check_target(packet.target_system, packet.target_component)) break;

            uint8_t result;

            // do command
            send_text(SEVERITY_LOW,PSTR("command received: "));

            switch(packet.command) {

            case MAV_CMD_NAV_LOITER_UNLIM:
                set_mode(LOITER);
                result = MAV_RESULT_ACCEPTED;
                break;

            case MAV_CMD_NAV_RETURN_TO_LAUNCH:
                set_mode(RTL);
                result = MAV_RESULT_ACCEPTED;
                break;

#if 0
                // not implemented yet, but could implement some of them
            case MAV_CMD_NAV_LAND:
            case MAV_CMD_NAV_TAKEOFF:
            case MAV_CMD_NAV_ROI:
            case MAV_CMD_NAV_PATHPLANNING:
                break;
#endif


            case MAV_CMD_PREFLIGHT_CALIBRATION:
                if (packet.param1 == 1 ||
                    packet.param2 == 1 ||
                    packet.param3 == 1) {
                    startup_IMU_ground();
                }
                if (packet.param4 == 1) {
                    trim_radio();
                }
                result = MAV_RESULT_ACCEPTED;
                break;


            default:
                result = MAV_RESULT_UNSUPPORTED;
                break;
            }

            mavlink_msg_command_ack_send(
                chan,
                packet.command,
                result);

            break;
        }

#else // MAVLINK10
    case MAVLINK_MSG_ID_ACTION:
        {
            // decode
            mavlink_action_t packet;
            mavlink_msg_action_decode(msg, &packet);
            if (mavlink_check_target(packet.target,packet.target_component)) break;

            uint8_t result = 0;

            // do action
            send_text(SEVERITY_LOW,PSTR("action received: "));
//Serial.println(packet.action);
            switch(packet.action){

				case MAV_ACTION_RESET_ACC_MAX_MIN:
					accel_min.x = 100;
					accel_min.y = 100;
					accel_min.z = 100;
					accel_max.x = -100;
					accel_max.y = -100;
					accel_max.z = -100;
					break;
				case MAV_ACTION_LAUNCH:
                    //set_mode(TAKEOFF);
                    break;

                case MAV_ACTION_RETURN:
                    set_mode(RTL);
                    result=1;
                    break;

                case MAV_ACTION_EMCY_LAND:
                    //set_mode(LAND);
                    break;

                case MAV_ACTION_HALT:
                    do_loiter_at_location();
                    result=1;
                    break;

                    /* No mappable implementation in APM 2.0
                case MAV_ACTION_MOTORS_START:
                case MAV_ACTION_CONFIRM_KILL:
                case MAV_ACTION_EMCY_KILL:
                case MAV_ACTION_MOTORS_STOP:
                case MAV_ACTION_SHUTDOWN:
                    break;
                */

                case MAV_ACTION_CONTINUE:
                    process_next_command();
                    result=1;
                    break;

                case MAV_ACTION_SET_MANUAL:
					b_4Dflag = false;
					time_left = 0;
					elapsed_time = 0;
                    set_mode(MANUAL);
                    result=1;
					if (b_ResetIntegratorsOnModeChange())	//if ((int)(g.airspeed_ratio/10)-100*((int)(g.airspeed_ratio/100)))
					{
						reset_I();
						//gcs_send_text_P(SEVERITY_HIGH,PSTR("Resetting Integrators on new Mode!"));
					}
                    break;

                case MAV_ACTION_SET_AUTO:
                    set_mode(AUTO);
                    result=1;
                    // clearing failsafe should not be needed
                    // here. Added based on some puzzling results in
                    // the simulator (tridge)
                    failsafe = FAILSAFE_NONE;
						if (b_ResetIntegratorsOnModeChange())	//if ((int)(g.airspeed_ratio/10)-100*((int)(g.airspeed_ratio/100)))
						{
							reset_I();
							//gcs_send_text_P(SEVERITY_HIGH,PSTR("Resetting Integrators on new Mode!"));
						}
                    break;

				case MAV_ACTION_STORAGE_READ:
                    AP_Var::load_all();
                    result=1;
                    break;

                case MAV_ACTION_STORAGE_WRITE:
                    AP_Var::save_all();
                    result=1;
                    break;

                case MAV_ACTION_CALIBRATE_RC: break;
                    trim_radio();
                    result=1;
                    break;

                case MAV_ACTION_CALIBRATE_GYRO:
                case MAV_ACTION_CALIBRATE_MAG:
                case MAV_ACTION_CALIBRATE_ACC:
                case MAV_ACTION_CALIBRATE_PRESSURE:
                case MAV_ACTION_REBOOT:  // this is a rough interpretation
                    startup_IMU_ground();
                    result=1;
                    break;

                /*    For future implemtation
                case MAV_ACTION_REC_START: break;
                case MAV_ACTION_REC_PAUSE: break;
                case MAV_ACTION_REC_STOP: break;
                */

                /* Takeoff is not an implemented flight mode in APM 2.0
                case MAV_ACTION_TAKEOFF:
                    set_mode(TAKEOFF);
                    break;
                */

                case MAV_ACTION_NAVIGATE:
                    set_mode(AUTO);
                    result=1;
						if (b_ResetIntegratorsOnModeChange())	//if ((int)(g.airspeed_ratio/10)-100*((int)(g.airspeed_ratio/100)))
						{
							reset_I();
							//gcs_send_text_P(SEVERITY_HIGH,PSTR("Resetting Integrators on new Mode!"));
						}
                    break;

                /* Land is not an implemented flight mode in APM 2.0
                case MAV_ACTION_LAND:
                    set_mode(LAND);
                    break;
                */

                case MAV_ACTION_LOITER:
                    set_mode(LOITER);
                    result=1;
						if (b_ResetIntegratorsOnModeChange())	//if ((int)(g.airspeed_ratio/10)-100*((int)(g.airspeed_ratio/100)))
						{
							reset_I();
							//gcs_send_text_P(SEVERITY_HIGH,PSTR("Resetting Integrators on new Mode!"));
						}
                    break;

                default: break;
                }

                mavlink_msg_wID_action_ack_send(
                    chan,
					packet.target,
					packet.target_component,
					packet.action,
                    result
                    );

            break;
        }
#endif

    case MAVLINK_MSG_ID_SET_MODE:
		{
            // decode
            mavlink_set_mode_t packet;
            mavlink_msg_set_mode_decode(msg, &packet);

#ifdef MAVLINK10
            if (!(packet.base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)) {
                // we ignore base_mode as there is no sane way to map
                // from that bitmap to a APM flight mode. We rely on
                // custom_mode instead.
                break;
            }
            switch (packet.custom_mode) {
            case MANUAL:
            case CIRCLE:
            case STABILIZE:
            case FLY_BY_WIRE_A:
            case FLY_BY_WIRE_B:
            case FLY_BY_WIRE_C:
            case AUTO:
            case RTL:
            case LOITER:
                set_mode(packet.custom_mode);
                break;
            }

#else // MAVLINK10

            switch(packet.mode){

                case MAV_MODE_MANUAL:
					b_4Dflag = false;
					time_left = 0;
					elapsed_time = 0;
					set_mode(MANUAL);
						if (b_ResetIntegratorsOnModeChange())	//if ((int)(g.airspeed_ratio/10)-100*((int)(g.airspeed_ratio/100)))
						{
							reset_I();
							//gcs_send_text_P(SEVERITY_HIGH,PSTR("Resetting Integrators on new Mode!"));
						}
					break;

				case MAV_MODE_GUIDED:
					set_mode(GUIDED);
						if (b_ResetIntegratorsOnModeChange())	//if ((int)(g.airspeed_ratio/10)-100*((int)(g.airspeed_ratio/100)))
						{
							reset_I();
							//gcs_send_text_P(SEVERITY_HIGH,PSTR("Resetting Integrators on new Mode!"));
						}
					break;

				case MAV_MODE_AUTO:
					if(mav_nav == (uint8_t)MAV_GCS_ID || mav_nav == MAV_NAV_WAYPOINT) 	{set_mode(AUTO);}
					if(mav_nav == MAV_NAV_RETURNING)					{set_mode(RTL);}
					if(mav_nav == MAV_NAV_LOITER)						{set_mode(LOITER);}
						if (b_ResetIntegratorsOnModeChange())	//if ((int)(g.airspeed_ratio/10)-100*((int)(g.airspeed_ratio/100)))
						{
							reset_I();
							//gcs_send_text_P(SEVERITY_HIGH,PSTR("Resetting Integrators on new Mode!"));
						}
					break;

                case MAV_MODE_TEST1:
					set_mode(STABILIZE);
						if (b_ResetIntegratorsOnModeChange())	//if ((int)(g.airspeed_ratio/10)-100*((int)(g.airspeed_ratio/100)))
						{
							reset_I();
							//gcs_send_text_P(SEVERITY_HIGH,PSTR("Resetting Integrators on new Mode!"));
						}
					break;

                case MAV_MODE_TEST2:
					if(mav_nav == 2)					{set_mode(FLY_BY_WIRE_B);}
					if(mav_nav == (uint8_t)MAV_GCS_ID || mav_nav == 1) 	{set_mode(FLY_BY_WIRE_A);}
						if (b_ResetIntegratorsOnModeChange())	//if ((int)(g.airspeed_ratio/10)-100*((int)(g.airspeed_ratio/100)))
						{
							reset_I();
							//gcs_send_text_P(SEVERITY_HIGH,PSTR("Resetting Integrators on new Mode!"));
						}
					break;

                case MAV_MODE_TEST3:
					set_mode(CIRCLE);
						if (b_ResetIntegratorsOnModeChange())	//if ((int)(g.airspeed_ratio/10)-100*((int)(g.airspeed_ratio/100)))
						{
							reset_I();
							//gcs_send_text_P(SEVERITY_HIGH,PSTR("Resetting Integrators on new Mode!"));
						}
					break;

			}
#endif
            break;
		}

#ifndef MAVLINK10
    case MAVLINK_MSG_ID_SET_NAV_MODE:
		{
            // decode
            mavlink_set_nav_mode_t packet;
            mavlink_msg_set_nav_mode_decode(msg, &packet);
			// To set some flight modes we must first receive a "set nav mode" message and then a "set mode" message
			mav_nav = packet.nav_mode;
			break;
		}
#endif // MAVLINK10


    case MAVLINK_MSG_ID_WAYPOINT_REQUEST_LIST:
        {
            // decode
            mavlink_waypoint_request_list_t packet;
            mavlink_msg_waypoint_request_list_decode(msg, &packet);
			if (mavlink_check_target(packet.target_system, packet.target_component))
				break;

            // Start sending waypoints
            mavlink_msg_wID_waypoint_count_send(
                chan,
				packet.target_system,
				packet.target_component,
				msg->sysid,
                msg->compid,
                g.command_total + 1); // + home

            waypoint_timelast_send   = millis();
            waypoint_sending         = true;
            waypoint_receiving       = false;
            waypoint_dest_sysid      = msg->sysid;
            waypoint_dest_compid     = msg->compid;
            break;
        }


	// XXX read a WP from EEPROM and send it to the GCS
    case MAVLINK_MSG_ID_WAYPOINT_REQUEST:
        {
            // Check if sending waypiont
            //if (!waypoint_sending) break;
			// 5/10/11 - We are trying out relaxing the requirement that we be in waypoint sending mode to respond to a waypoint request.  DEW

            // decode
            mavlink_waypoint_request_t packet;
            mavlink_msg_waypoint_request_decode(msg, &packet);

 			if (mavlink_check_target(packet.target_system, packet.target_component))
 				break;

            // send waypoint
            tell_command = get_cmd_with_index(packet.seq);

            // set frame of waypoint
            uint8_t frame;

			if (tell_command.options & MASK_OPTIONS_RELATIVE_ALT) {
                frame = MAV_FRAME_GLOBAL_RELATIVE_ALT; // reference frame
            } else {
                frame = MAV_FRAME_GLOBAL; // reference frame
            }

            float param1 = 0, param2 = 0 , param3 = 0, param4 = 0;

            // time that the mav should loiter in milliseconds
            uint8_t current = 0; // 1 (true), 0 (false)

			if (packet.seq == (uint16_t)g.command_index)
            	current = 1;

            uint8_t autocontinue = 1; // 1 (true), 0 (false)

            float x = 0, y = 0, z = 0;

            if (tell_command.id < MAV_CMD_NAV_LAST || tell_command.id == MAV_CMD_CONDITION_CHANGE_ALT) {
			// command needs scaling
                x = tell_command.lat/1.0e7; // local (x), global (latitude)
                y = tell_command.lng/1.0e7; // local (y), global (longitude)
                if (tell_command.options & MASK_OPTIONS_RELATIVE_ALT) {
                    //z = (tell_command.alt - home.alt) / 1.0e2; // because tell_command.alt already includes a += home.alt
                    z = (tell_command.alt) / 1.0e2; // because tell_command.alt already includes a += home.alt
                } else {
                    z = tell_command.alt/1.0e2; // local (z), global/relative (altitude)
                }
            }

			switch (tell_command.id) {				// Switch to map APM command fields inot MAVLink command fields
				case MAV_CMD_NAV_WAYPOINT:
				case MAV_CMD_NAV_LOITER_TURNS:
				case MAV_CMD_NAV_TAKEOFF:
				case MAV_CMD_DO_SET_HOME:
					param1 = tell_command.p1;
					break;

				case MAV_CMD_NAV_LOITER_TIME:
					param1 = tell_command.p1*10;    // APM loiter time is in ten second increments
					break;

				case MAV_CMD_CONDITION_YAW:
					param1 = tell_command.p1;
					x = tell_command.lat;
					y = tell_command.lng;
					break;

				case MAV_CMD_CONDITION_CHANGE_ALT:
					x=0;	// Clear fields loaded above that we don't want sent for this command
					y=0;
				case MAV_CMD_CONDITION_DELAY:
				case MAV_CMD_CONDITION_DISTANCE:
					param1 = tell_command.lat;
					break;

				case MAV_CMD_DO_JUMP:
					param2 = tell_command.lat;
					param1 = tell_command.p1;
					break;

				case MAV_CMD_DO_REPEAT_SERVO:
					param4 = tell_command.lng;
				case MAV_CMD_DO_REPEAT_RELAY:
				case MAV_CMD_DO_CHANGE_SPEED:
					param3 = tell_command.lat;
					param2 = tell_command.alt;
					param1 = tell_command.p1;
					break;

				case MAV_CMD_DO_SET_PARAMETER:
				case MAV_CMD_DO_SET_RELAY:
				case MAV_CMD_DO_SET_SERVO:
					param2 = tell_command.alt;
					param1 = tell_command.p1;
					break;
			}

			mavlink_msg_wID_waypoint_send(chan,
										packet.target_system, 
										packet.target_component,
										msg->sysid,
										msg->compid,
										packet.seq,
										frame,
										tell_command.id,
										current,
										autocontinue,
										param1,
										param2,
										param3,
										param4,
										x,
										y,
										z);

            // update last waypoint comm stamp
            waypoint_timelast_send = millis();
            break;
        }


    case MAVLINK_MSG_ID_WAYPOINT_ACK:
        {
            // decode
            mavlink_waypoint_ack_t packet;
            mavlink_msg_waypoint_ack_decode(msg, &packet);
            if (mavlink_check_target(packet.target_system,packet.target_component)) break;

            // turn off waypoint send
            waypoint_sending = false;
            break;
        }

    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
        {
			// decode
			//g.sysid_this_mav.set_and_save((int16_t)MAV_SYSTEM_ID);
			//g.sysid_my_gcs.set_and_save((int16_t)MAV_GCS_ID);

            mavlink_param_request_list_t packet;
            mavlink_msg_param_request_list_decode(msg, &packet);
            if (mavlink_check_target(packet.target_system,packet.target_component)) break;

            // Start sending parameters - next call to ::update will kick the first one out
            param_dest_sysid = packet.target_system;
            param_dest_compid = packet.target_component;

            _queued_parameter = AP_Var::first();
            _queued_parameter_index = 0;
            _queued_parameter_count = _count_parameters();
            break;
        }

    case MAVLINK_MSG_ID_WAYPOINT_CLEAR_ALL:
        {
            // decode
            mavlink_waypoint_clear_all_t packet;
            mavlink_msg_waypoint_clear_all_decode(msg, &packet);
			if (mavlink_check_target(packet.target_system, packet.target_component)) break;

            // clear all commands
            g.command_total.set_and_save(0);

            // note that we don't send multiple acks, as otherwise a
            // GCS that is doing a clear followed by a set may see
            // the additional ACKs as ACKs of the set operations
            mavlink_msg_wID_waypoint_ack_send(chan, 
										packet.target_system, 
										packet.target_component,
										msg->sysid,
										msg->compid, 
										MAV_MISSION_ACCEPTED);
            break;
        }

    case MAVLINK_MSG_ID_WAYPOINT_SET_CURRENT:
        {
            // decode
            mavlink_waypoint_set_current_t packet;
            mavlink_msg_waypoint_set_current_decode(msg, &packet);
            if (mavlink_check_target(packet.target_system,packet.target_component)) break;

            // set current command
            change_command(packet.seq);

            mavlink_msg_wID_waypoint_current_send(chan, 
										packet.target_system, 
										packet.target_component,
										g.command_index);
			/*
			struct Location temp = get_cmd_with_index(g.command_index);
			if (temp.id != MAV_CMD_NAV_WAYPOINT)
			{
				b_4Dflag = false;
				elapsed_time = 0;
				time_left = 0;
			}
			else
			{
				if (b_4DWaypointsEnabled() && (b_4Dflag==false) && (g.command_index>0))
				{
					b_4Dflag = true;
					elapsed_time = 0;
					time_left = (float)temp.p1;
					//time_left = (float)(next_WP.p1);
				}
 			}
			*/
			b_4Dflag = false;
			//cdnr_hflag = 0;
			//cdnr_sflag = 0;
			//cdnr_aflag = 0;
			//cdnr_tflag = 0;
			time_left = 0;
			elapsed_time = 0;
			Serial.printf_P (PSTR("Time %f  4D?  %d\n")
							,time_left
							,b_4Dflag);
           break;
        }

    case MAVLINK_MSG_ID_WAYPOINT_COUNT:
        {
            // decode
            mavlink_waypoint_count_t packet;
            mavlink_msg_waypoint_count_decode(msg, &packet);
            if (mavlink_check_target(packet.target_system,packet.target_component)) break;

            // start waypoint receiving
            if (packet.count > MAX_WAYPOINTS) {
                packet.count = MAX_WAYPOINTS;
            }
            g.command_total.set_and_save(packet.count - 1);

            waypoint_timelast_receive = millis();
            waypoint_timelast_request = 0;
            waypoint_receiving   = true;
            waypoint_sending     = false;
            waypoint_request_i   = 0;
            break;
        }

#ifdef MAVLINK_MSG_ID_SET_MAG_OFFSETS
    case MAVLINK_MSG_ID_SET_MAG_OFFSETS:
    {
        mavlink_set_mag_offsets_t packet;
        mavlink_msg_set_mag_offsets_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system,packet.target_component)) break;
        compass.set_offsets(Vector3f(packet.mag_ofs_x, packet.mag_ofs_y, packet.mag_ofs_z));
        break;
    }
#endif

	// XXX receive a WP from GCS and store in EEPROM
    case MAVLINK_MSG_ID_WAYPOINT:
        {
            // decode
            mavlink_waypoint_t packet;
            uint8_t result = MAV_MISSION_ACCEPTED;

            mavlink_msg_waypoint_decode(msg, &packet);
            if (mavlink_check_target(packet.target_system,packet.target_component)) break;

            // defaults
            tell_command.id = packet.command;

			switch (packet.frame)
			{
				case MAV_FRAME_MISSION:
				case MAV_FRAME_GLOBAL:
					{
						tell_command.lat = 1.0e7*packet.x; // in as DD converted to * t7
						tell_command.lng = 1.0e7*packet.y; // in as DD converted to * t7
						tell_command.alt = packet.z*1.0e2; // in as m converted to cm
						tell_command.options = 0; // absolute altitude
						tell_command.p1 = packet.param1;
						//tell_command.p2 = packet.param2;
						//tell_command.p3 = packet.param3;
						//tell_command.p4 = packet.param4;
						//Serial.println("CASE A");
						break;
					}

#ifdef MAV_FRAME_LOCAL_NED
				case MAV_FRAME_LOCAL_NED: // local (relative to home position)
					{
						tell_command.lat = 1.0e7*ToDeg(packet.x/
						(radius_of_earth*cos(ToRad(home.lat/1.0e7)))) + home.lat;
						tell_command.lng = 1.0e7*ToDeg(packet.y/radius_of_earth) + home.lng;
						tell_command.alt = -packet.z*1.0e2;
						tell_command.options = MASK_OPTIONS_RELATIVE_ALT;
						break;
					}
#endif

#ifdef MAV_FRAME_LOCAL
				case MAV_FRAME_LOCAL: // local (relative to home position)
					{
						tell_command.lat = 1.0e7*ToDeg(packet.x/
						(radius_of_earth*cos(ToRad(home.lat/1.0e7)))) + home.lat;
						tell_command.lng = 1.0e7*ToDeg(packet.y/radius_of_earth) + home.lng;
						tell_command.alt = packet.z*1.0e2;
						tell_command.options = MASK_OPTIONS_RELATIVE_ALT;
						break;
					}
#endif

				case MAV_FRAME_GLOBAL_RELATIVE_ALT: // absolute lat/lng, relative altitude
					{
						tell_command.lat = 1.0e7 * packet.x; // in as DD converted to * t7
						tell_command.lng = 1.0e7 * packet.y; // in as DD converted to * t7
						tell_command.alt = packet.z * 1.0e2;
						tell_command.options = MASK_OPTIONS_RELATIVE_ALT; // store altitude relative!! Always!!
						//Serial.println("CASE B");
						tell_command.p1 = packet.param1;
						break;
					}

            default:
                result = MAV_MISSION_UNSUPPORTED_FRAME;
                break;
			}

            
            if (result != MAV_MISSION_ACCEPTED) goto mission_failed;

            switch (tell_command.id) {                    // Switch to map APM command fields inot MAVLink command fields
            case MAV_CMD_NAV_WAYPOINT:
            case MAV_CMD_NAV_LOITER_UNLIM:
            case MAV_CMD_NAV_RETURN_TO_LAUNCH:
            case MAV_CMD_NAV_LAND:
                break;

            case MAV_CMD_NAV_LOITER_TURNS:
            case MAV_CMD_NAV_TAKEOFF:
            case MAV_CMD_DO_SET_HOME:
                tell_command.p1 = packet.param1;
                break;

			case MAV_CMD_CONDITION_YAW:
                tell_command.p1 = packet.param1;
                tell_command.lat = packet.x;
                tell_command.lng = packet.y;
                break;

            case MAV_CMD_CONDITION_CHANGE_ALT:
                tell_command.lat = packet.param1;
                break;

            case MAV_CMD_NAV_LOITER_TIME:
                tell_command.p1 = packet.param1; // / 10;    // APM loiter time is in ten second increments
                break;

            case MAV_CMD_CONDITION_DELAY:
            case MAV_CMD_CONDITION_DISTANCE:
                tell_command.lat = packet.param1;
                break;

            case MAV_CMD_DO_JUMP:
                tell_command.lat = packet.param2;
                tell_command.p1 = packet.param1;
                break;

            case MAV_CMD_DO_REPEAT_SERVO:
                tell_command.lng = packet.param4;
            case MAV_CMD_DO_REPEAT_RELAY:
            case MAV_CMD_DO_CHANGE_SPEED:
                tell_command.lat = packet.param3;
                tell_command.alt = packet.param2;
                tell_command.p1 = packet.param1;
                break;

            case MAV_CMD_DO_SET_PARAMETER:
            case MAV_CMD_DO_SET_RELAY:
            case MAV_CMD_DO_SET_SERVO:
                tell_command.alt = packet.param2;
                tell_command.p1 = packet.param1;
                break;

            default:
#ifdef MAVLINK10
                result = MAV_MISSION_UNSUPPORTED;
#endif
                break;
            }

            if (result != MAV_MISSION_ACCEPTED) goto mission_failed;

			if(packet.current == 2){ 				//current = 2 is a flag to tell us this is a "guided mode" waypoint and not for the mission
				guided_WP = tell_command;

				// make any new wp uploaded instant (in case we are already in Guided mode)
				set_guided_WP();
				set_mode(GUIDED);

				// verify we recevied the command
				mavlink_msg_wID_waypoint_ack_send(
						chan,
						packet.target_system, 
						packet.target_component,
						msg->sysid,
						msg->compid,
						0);

			} else {
				// packet.current == 255 means that ONLY a single waypoint is being updated
				// otherwise, the entire mission is being updated
				if (packet.current != 255)
				{
					// Check if receiving waypoints (mission upload expected)
					if (!waypoint_receiving) {
						send_text(SEVERITY_HIGH,PSTR("WayPoint Receiving FALSE"));
						result = MAV_MISSION_ERROR;
						goto mission_failed;
					}

					// check if this is the requested waypoint
					if (packet.seq != waypoint_request_i) {
						send_text(SEVERITY_HIGH,PSTR("Waypoint Invalid Sequence"));
						result = MAV_MISSION_INVALID_SEQUENCE;
						goto mission_failed;
					}
				}
				
				if (packet.seq == 0)
					home = tell_command;
				if (packet.seq == nav_command_index)
				{
					next_nav_command = tell_command;
					struct Location temp = get_cmd_with_index(nav_command_index);
					if (tell_command.p1 != temp.p1 || tell_command.lat != temp.lat || tell_command.lng != temp.lng || tell_command.alt != temp.alt)
					{
						//modify_next_WP(&next_nav_command,(tell_command.p1-temp.p1));
						modify_next_WP(&next_nav_command,(tell_command.p1-time_left));
						send_text(SEVERITY_LOW,PSTR("Current WayPoint MODIFIED"));
					}
					else
						send_text(SEVERITY_LOW,PSTR("Current WayPoint NOT MODIFIED"));
				}
				if (packet.seq == nav_command_index2)
					next_nav_command2 = tell_command;
				set_cmd_with_index(tell_command, packet.seq);

				// packet.current == 255 means that ONLY a single waypoint is being updated
				// otherwise, the entire mission is being updated
				if (packet.current != 255)
				{
					// update waypoint receiving state machine
					waypoint_timelast_receive = millis();
					waypoint_timelast_request = 0;
					waypoint_request_i++;

					if (waypoint_request_i > (uint16_t)g.command_total){
						mavlink_msg_wID_waypoint_ack_send(
							chan,
							packet.target_system, 
							packet.target_component,
							msg->sysid,
							msg->compid,
							result);

						send_text(SEVERITY_LOW,PSTR("flight plan received"));
						waypoint_receiving = false;
						// XXX ignores waypoint radius for individual waypoints, can
						// only set WP_RADIUS parameter
					}
				}
				else
				{
					mavlink_msg_wID_waypoint_ack_send(
						chan,
						packet.target_system, 
						packet.target_component,
						msg->sysid,
						msg->compid,
						result);

					send_text(SEVERITY_LOW,PSTR("flight plan received"));
				}
			}
            break;

        mission_failed:
            // we are rejecting the mission/waypoint
            mavlink_msg_wID_waypoint_ack_send(
                chan,
				packet.target_system, 
				packet.target_component,
                msg->sysid,
                msg->compid,
                result);
            break;
        }

#if GEOFENCE_ENABLED == ENABLED
	// receive a fence point from GCS and store in EEPROM
    case MAVLINK_MSG_ID_FENCE_POINT: {
        mavlink_fence_point_t packet;
        mavlink_msg_fence_point_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system, packet.target_component))
            break;
        if (g.fence_action != FENCE_ACTION_NONE) {
            send_text(SEVERITY_LOW,PSTR("fencing must be disabled"));
        } else if (packet.count != g.fence_total) {
            send_text(SEVERITY_LOW,PSTR("bad fence point"));
        } else {
            Vector2l point;
            point.x = packet.lat*1.0e7;
            point.y = packet.lng*1.0e7;
            set_fence_point_with_index(point, packet.idx);
        }
        break;
    }

	// send a fence point to GCS
    case MAVLINK_MSG_ID_FENCE_FETCH_POINT: {
        mavlink_fence_fetch_point_t packet;
        mavlink_msg_fence_fetch_point_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system, packet.target_component))
            break;
        if (packet.idx >= g.fence_total) {
            send_text(SEVERITY_LOW,PSTR("bad fence point"));
        } else {
            Vector2l point = get_fence_point_with_index(packet.idx);
            mavlink_msg_wID_fence_point_send(chan, 
										packet.target_system, 
										packet.target_component,
										0, 0, packet.idx, g.fence_total,
                                         point.x*1.0e-7, point.y*1.0e-7);
        }
        break;
    }
#endif // GEOFENCE_ENABLED

    case MAVLINK_MSG_ID_PARAM_SET:
        {
            AP_Var                  *vp;
            AP_Meta_class::Type_id  var_type;

            // decode
            mavlink_param_set_t packet;
            mavlink_msg_param_set_decode(msg, &packet);

			if (mavlink_check_target(packet.target_system, packet.target_component))
				break;

            // set parameter

            char key[ONBOARD_PARAM_NAME_LENGTH+1];
            strncpy(key, (char *)packet.param_id, ONBOARD_PARAM_NAME_LENGTH);
            key[ONBOARD_PARAM_NAME_LENGTH] = 0;

            // find the requested parameter
            vp = AP_Var::find(key);
            if ((NULL != vp) &&                             // exists
                    !isnan(packet.param_value) &&               // not nan
                    !isinf(packet.param_value)) {               // not inf

                // add a small amount before casting parameter values
                // from float to integer to avoid truncating to the
                // next lower integer value.
				float rounding_addition = 0.01;

                // fetch the variable type ID
                var_type = vp->meta_type_id();

                // handle variables with standard type IDs
                if (var_type == AP_Var::k_typeid_float) {
                    ((AP_Float *)vp)->set_and_save(packet.param_value);
                } else if (var_type == AP_Var::k_typeid_float16) {
                    ((AP_Float16 *)vp)->set_and_save(packet.param_value);
                } else if (var_type == AP_Var::k_typeid_int32) {
                    if (packet.param_value < 0) rounding_addition = -rounding_addition;
                    ((AP_Int32 *)vp)->set_and_save(packet.param_value+rounding_addition);
                } else if (var_type == AP_Var::k_typeid_int16) {
                    if (packet.param_value < 0) rounding_addition = -rounding_addition;
                    ((AP_Int16 *)vp)->set_and_save(packet.param_value+rounding_addition);
                } else if (var_type == AP_Var::k_typeid_int8) {
                    if (packet.param_value < 0) rounding_addition = -rounding_addition;
                    ((AP_Int8 *)vp)->set_and_save(packet.param_value+rounding_addition);
                } else {
                    // we don't support mavlink set on this parameter
                    break;
                }

                // Report back the new value if we accepted the change
                // we send the value we actually set, which could be
                // different from the value sent, in case someone sent
                // a fractional value to an integer type
                mavlink_msg_wID_param_value_send(
                    chan,
					packet.target_system, 
					packet.target_component,
                    key,
                    vp->cast_to_float(),
                    mav_var_type(vp->meta_type_id()),
                    _count_parameters(),
                    -1); // XXX we don't actually know what its index is...
            }

            break;
        } // end case

    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
        {
            // allow override of RC channel values for HIL
            // or for complete GCS control of switch position
            // and RC PWM values.
			if(msg->sysid != g.sysid_my_gcs) break;		// Only accept control from our gcs
            mavlink_rc_channels_override_t packet;
            int16_t v[8];
            mavlink_msg_rc_channels_override_decode(msg, &packet);

			if (mavlink_check_target(packet.target_system,packet.target_component))
				break;

            v[0] = packet.chan1_raw;
            v[1] = packet.chan2_raw;
            v[2] = packet.chan3_raw;
            v[3] = packet.chan4_raw;
            v[4] = packet.chan5_raw;
            v[5] = packet.chan6_raw;
            v[6] = packet.chan7_raw;
            v[7] = packet.chan8_raw;
            rc_override_active = APM_RC.setHIL(v);
			rc_override_fs_timer = millis();
            break;
        }

    case MAVLINK_MSG_ID_HEARTBEAT:
        {
            // We keep track of the last time we received a heartbeat from our GCS for failsafe purposes
			if(msg->sysid != g.sysid_my_gcs)
			{
				//Serial._printf_P(PSTR("msg ID %d does not match GSC ID %d\n"),msg->sysid, g.sysid_my_gcs);
				break;
			}
			rc_override_fs_timer = millis();
			pmTest1++;
            break;
        }

	#if HIL_MODE != HIL_MODE_DISABLED
        // This is used both as a sensor and to pass the location
        // in HIL_ATTITUDE mode.
#ifdef MAVLINK10
	case MAVLINK_MSG_ID_GPS_RAW_INT:
        {
            // decode
            mavlink_gps_raw_int_t packet;
            mavlink_msg_gps_raw_int_decode(msg, &packet);

            // set gps hil sensor
            g_gps->setHIL(packet.time_usec/1000.0,
                          packet.lat*1.0e-7, packet.lon*1.0e-7, packet.alt*1.0e-3,
                          packet.vel*1.0e-2, packet.cog*1.0e-2, 0, 0);
            break;
        }
#else // MAVLINK10
	case MAVLINK_MSG_ID_GPS_RAW:
        {
            // decode
            mavlink_gps_raw_t packet;

            if (waypoint_sending || waypoint_receiving) break;

			mavlink_msg_gps_raw_decode(msg, &packet);

            // set gps hil sensor
            g_gps->setHIL(packet.usec/1000.0,packet.lat,packet.lon,packet.alt,
            packet.v,packet.hdg,0,0);
            break;
        }
#endif // MAVLINK10

        //    Is this resolved? - MAVLink protocol change.....
    case MAVLINK_MSG_ID_VFR_HUD:
        {
            // decode
            mavlink_vfr_hud_t packet;
            mavlink_msg_vfr_hud_decode(msg, &packet);

            // set airspeed
            airspeed = 100*packet.airspeed;
            break;
        }
#ifdef MAVLINK10
	case MAVLINK_MSG_ID_HIL_STATE:
		{
			mavlink_hil_state_t packet;
			mavlink_msg_hil_state_decode(msg, &packet);
			
			float vel = sqrt((packet.vx * packet.vx) + (packet.vy * packet.vy));
			float cog = wrap_360(ToDeg(atan2(packet.vx, packet.vy)) * 100);
			
            // set gps hil sensor
            g_gps->setHIL(packet.time_usec/1000.0,
                          packet.lat*1.0e-7, packet.lon*1.0e-7, packet.alt*1.0e-3,
                          vel*1.0e-2, cog*1.0e-2, 0, 0);
			
			#if HIL_MODE == HIL_MODE_SENSORS
			
			// rad/sec
            Vector3f gyros;
            gyros.x = (float)packet.xgyro / 1000.0;
            gyros.y = (float)packet.ygyro / 1000.0;
            gyros.z = (float)packet.zgyro / 1000.0;
            // m/s/s
            Vector3f accels;
            accels.x = (float)packet.xacc / 1000.0;
            accels.y = (float)packet.yacc / 1000.0;
            accels.z = (float)packet.zacc / 1000.0;

            imu.set_gyro(gyros);

            imu.set_accel(accels);
			
			#else

			// set dcm hil sensor
            dcm.setHil(packet.roll,packet.pitch,packet.yaw,packet.rollspeed,
            packet.pitchspeed,packet.yawspeed);

			#endif

			break;
		}
#endif // MAVLINK10
#endif
#if HIL_MODE == HIL_MODE_ATTITUDE
    case MAVLINK_MSG_ID_ATTITUDE:
        {
            // decode
            mavlink_attitude_t packet;

            if (waypoint_sending || waypoint_receiving) break;

            mavlink_msg_attitude_decode(msg, &packet);

            // set dcm hil sensor
            dcm.setHil(packet.roll,packet.pitch,packet.yaw,packet.rollspeed,
            packet.pitchspeed,packet.yawspeed);
            break;
        }
#endif
#if HIL_MODE == HIL_MODE_SENSORS

    case MAVLINK_MSG_ID_RAW_IMU:
        {
            // decode
            mavlink_raw_imu_t packet;
            mavlink_msg_raw_imu_decode(msg, &packet);

            // set imu hil sensors
            // TODO: check scaling for temp/absPress
            float temp = 70;
            float absPress = 1;
                  //Serial.printf_P(PSTR("accel: %d %d %d\n"), packet.xacc, packet.yacc, packet.zacc);
                  //Serial.printf_P(PSTR("gyro: %d %d %d\n"), packet.xgyro, packet.ygyro, packet.zgyro);

            // rad/sec
            Vector3f gyros;
            gyros.x = (float)packet.xgyro / 1000.0;
            gyros.y = (float)packet.ygyro / 1000.0;
            gyros.z = (float)packet.zgyro / 1000.0;
            // m/s/s
            Vector3f accels;
            accels.x = (float)packet.xacc / 1000.0;
            accels.y = (float)packet.yacc / 1000.0;
            accels.z = (float)packet.zacc / 1000.0;

            imu.set_gyro(gyros);

            imu.set_accel(accels);

            compass.setHIL(packet.xmag,packet.ymag,packet.zmag);
            break;
        }

    case MAVLINK_MSG_ID_RAW_PRESSURE:
        {
            // decode
            mavlink_raw_pressure_t packet;
            mavlink_msg_raw_pressure_decode(msg, &packet);

            // set pressure hil sensor
            // TODO: check scaling
            float temp = 70;
            barometer.setHIL(temp,packet.press_diff1 + 101325);
            break;
        }
#endif // HIL_MODE
#if HIL_MODE == HIL_MODE_ATTITUDE

    case MAVLINK_MSG_ID_RAW_IMU:
        {
            // decode
            mavlink_raw_imu_t packet;

            if (waypoint_sending || waypoint_receiving) break;

            mavlink_msg_raw_imu_decode(msg, &packet);

            // set imu hil sensors
            // TODO: check scaling for temp/absPress
            float temp = 70;
            float absPress = 1;
                  //Serial.printf_P(PSTR("accel: %d %d %d\n"), packet.xacc, packet.yacc, packet.zacc);
                  //Serial.printf_P(PSTR("gyro: %d %d %d\n"), packet.xgyro, packet.ygyro, packet.zgyro);

            // rad/sec
            Vector3f gyros;
            gyros.x = (float)packet.xgyro / 1000.0;
            gyros.y = (float)packet.ygyro / 1000.0;
            gyros.z = (float)packet.zgyro / 1000.0;
            // m/s/s
            Vector3f accels;
            accels.x = (float)packet.xacc * gravity / 1000.0;
            accels.y = (float)packet.yacc * gravity / 1000.0;
            accels.z = (float)-packet.zacc * gravity / 1000.0;

            //imu.set_gyro(gyros);

            imu.set_accel(accels);

            //compass.setHIL(packet.xmag,packet.ymag,packet.zmag);
            break;
        }
#endif // HIL_MODE

#if MOUNT == ENABLED
    case MAVLINK_MSG_ID_MOUNT_CONFIGURE:
		{
			camera_mount.configure_msg(msg);
			break;
		}

    case MAVLINK_MSG_ID_MOUNT_CONTROL:
		{
			camera_mount.control_msg(msg);
			break;
		}

    case MAVLINK_MSG_ID_MOUNT_STATUS:
		{
			camera_mount.status_msg(msg);
			break;
		}
#endif // MOUNT == ENABLED

case MAVLINK_MSG_ID_CDNR_CONTROLLER:
		{
            // decode
            mavlink_cdnr_controller_t packet;
            mavlink_msg_cdnr_controller_decode(msg, &packet);

			cdnr_hflag = packet.h_flag;
			cdnr_sflag = packet.s_flag;
			cdnr_aflag = packet.a_flag;
			cdnr_tflag = packet.t_flag;

			if (!cdnr_hflag && !cdnr_sflag && !cdnr_aflag && !cdnr_tflag)
			{
				last_nav_heading = nav_bearing;
				nav_bearing = target_bearing;
				max_time = 0;
				Serial3.println("CDNR Cancelled");
			}

			if (cdnr_hflag)
			{
				if (packet.new_heading >= 0)
				{
					cdnr_bearing = 100*(long)packet.new_heading;
				}
			}

			if (cdnr_sflag)
			{
				if (packet.new_airspeed > 0)
					f_airspeed = 100*(long)packet.new_airspeed;
			}

			if (cdnr_aflag)
			{
				if (packet.new_altitude > 0)
					target_altitude = 100*(long)packet.new_altitude;
			}

			if (cdnr_tflag)
			{
				if (packet.max_time > 0)
					max_time    = packet.max_time;
			}

			//// If we are in the middle of a turn, turn it off
			//Serial.printf_P (PSTR("CDNR H[%d]=%d S[%d]=%d A[%d]=%d T[%d]=%d\n")
			//	, cdnr_hflag,packet.new_heading, cdnr_sflag,packet.new_airspeed, cdnr_aflag,packet.new_altitude, cdnr_tflag,packet.max_time);
			//Serial3.printf_P(PSTR("CDNR H[%d]=%d S[%d]=%d A[%d]=%d T[%d]=%d\n")
			//	, cdnr_hflag,packet.new_heading, cdnr_sflag,packet.new_airspeed, cdnr_aflag,packet.new_altitude, cdnr_tflag,packet.max_time);
		}
		break;

case MAVLINK_MSG_ID_TRAFFIC_SIM_TIMING:
		{
            // decode
            mavlink_traffic_sim_timing_t packet;
            mavlink_msg_traffic_sim_timing_decode(msg, &packet);

			time_left = packet.time_to_wpt - packet.sim_time;
			elapsed_time = 0;

			gcs_send_text((gcs_severity)1,"Traffic Simulator Time");
			//Serial.printf_P (PSTR("SIM time left\n"), time_left);
			//Serial3.printf_P (PSTR("SIM time left\n"), time_left);
		}
		break;

	} // end switch
} // end handle mavlink

uint16_t
GCS_MAVLINK::_count_parameters()
{
    // if we haven't cached the parameter count yet...
    if (0 == _parameter_count) {
        AP_Var  *vp;

        vp = AP_Var::first();
        do {
            // if a parameter responds to cast_to_float then we are going to be able to report it
            if (!isnan(vp->cast_to_float())) {
                _parameter_count++;
            }
        } while (NULL != (vp = vp->next()));
    }
    return _parameter_count;
}

AP_Var *
GCS_MAVLINK::_find_parameter(uint16_t index)
{
    AP_Var  *vp;

    vp = AP_Var::first();
    while (NULL != vp) {

        // if the parameter is reportable
        if (!(isnan(vp->cast_to_float()))) {
            // if we have counted down to the index we want
            if (0 == index) {
                // return the parameter
                return vp;
            }
            // count off this parameter, as it is reportable but not
            // the one we want
            index--;
        }
        // and move to the next parameter
        vp = vp->next();
    }
    return NULL;
}

/**
* @brief Send the next pending parameter, called from deferred message
* handling code
*/
void
GCS_MAVLINK::queued_param_send()
{
    // Check to see if we are sending parameters
    if (NULL == _queued_parameter) return;

    AP_Var      *vp;
    float       value;

    // copy the current parameter and prepare to move to the next
    vp = _queued_parameter;
    _queued_parameter = _queued_parameter->next();

    // if the parameter can be cast to float, report it here and break out of the loop
    value = vp->cast_to_float();
    if (!isnan(value)) {
        char param_name[ONBOARD_PARAM_NAME_LENGTH];         /// XXX HACK
        vp->copy_name(param_name, sizeof(param_name));

        mavlink_msg_wID_param_value_send(
            chan,
            param_dest_sysid,
            param_dest_compid,
            param_name,
            value,
            mav_var_type(vp->meta_type_id()),
            _queued_parameter_count,
            _queued_parameter_index);

        _queued_parameter_index++;
    }
}

/**
* @brief Send the next pending waypoint, called from deferred message
* handling code
*/
void
GCS_MAVLINK::queued_waypoint_send()
{
    if (waypoint_receiving &&
        waypoint_request_i <= (unsigned)g.command_total) {
        mavlink_msg_wID_waypoint_request_send(
            chan,
            (uint8_t)waypoint_dest_sysid,
            (uint8_t)waypoint_dest_compid,
            waypoint_dest_sysid,
            waypoint_dest_compid,
            waypoint_request_i);
    }
}

/*
 a delay() callback that processes MAVLink packets. We set this as the
 callback in long running library initialisation routines to allow
 MAVLink to process packets while waiting for the initialisation to
 complete
*/
static void mavlink_delay(unsigned long t)
{
    unsigned long tstart;
    static unsigned long last_1hz, last_50hz;

    if (in_mavlink_delay) {
        // this should never happen, but let's not tempt fate by
        // letting the stack grow too much
        delay(t);
        return;
    }

    in_mavlink_delay = true;

    tstart = millis();
    do {
        unsigned long tnow = millis();
        if (tnow - last_1hz > 1000) {
            last_1hz = tnow;
            gcs_send_message(MSG_HEARTBEAT);
            gcs_send_message(MSG_EXTENDED_STATUS1);
        }
        if (tnow - last_50hz > 20) {
            last_50hz = tnow;
            gcs_update();
        }
        delay(1);
#if USB_MUX_PIN > 0
        check_usb_mux();
#endif
    } while (millis() - tstart < t);

    in_mavlink_delay = false;
}

/*
  send a message on both GCS links
 */
static void gcs_send_message(enum ap_message id)
{
    gcs0.send_message(id);
    if (gcs3.initialised) {
        gcs3.send_message(id);
    }
}

/*
  send data streams in the given rate range on both links
 */
static void gcs_data_stream_send(uint16_t freqMin, uint16_t freqMax)
{
    gcs0.data_stream_send(freqMin, freqMax);
    if (gcs3.initialised) {
        gcs3.data_stream_send(freqMin, freqMax);
    }
}

/*
  look for incoming commands on the GCS links
 */
static void gcs_update(void)
{
	gcs0.update();
    if (gcs3.initialised) {
        gcs3.update();
    }
}

static void gcs_send_text(gcs_severity severity, const char *str)
{
    gcs0.send_text(severity, str);
    if (gcs3.initialised) {
        gcs3.send_text(severity, str);
    }
}

static void gcs_send_text_P(gcs_severity severity, const prog_char_t *str)
{
    gcs0.send_text(severity, str);
    if (gcs3.initialised) {
        gcs3.send_text(severity, str);
    }
}

/*
  send a low priority formatted message to the GCS
  only one fits in the queue, so if you send more than one before the
  last one gets into the serial buffer then the old one will be lost
 */
static void gcs_send_text_fmt(const prog_char_t *fmt, ...)
{
    char fmtstr[40];
    va_list ap;
    uint8_t i;
    for (i=0; i<sizeof(fmtstr)-1; i++) {
        fmtstr[i] = pgm_read_byte((const prog_char *)(fmt++));
        if (fmtstr[i] == 0) break;
    }
    fmtstr[i] = 0;
    pending_status.severity = (uint8_t)SEVERITY_LOW;
    va_start(ap, fmt);
    vsnprintf((char *)pending_status.text, sizeof(pending_status.text), fmtstr, ap);
    va_end(ap);
    mavlink_send_message(MAVLINK_COMM_0, MSG_STATUSTEXT, 0);
    if (gcs3.initialised) {
        mavlink_send_message(MAVLINK_COMM_1, MSG_STATUSTEXT, 0);
    }
}
