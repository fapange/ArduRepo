/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/********************************************************************************/
// Command Event Handlers
/********************************************************************************/
static void
handle_process_nav_cmd()
{
	// reset navigation integrators
	// -------------------------
	if (b_ResetIntegratorsOnWaypoint())	//if ((int)g.airspeed_ratio-10*((int)(g.airspeed_ratio/10)))
	{
		reset_I();
		//gcs_send_text_P(SEVERITY_HIGH,PSTR("Resetting Integrators on new Waypoint!"));
	}

	//gcs_send_text_fmt(PSTR("Executing command ID #%i"),next_nav_command.id);
	switch(next_nav_command.id){

		case MAV_CMD_NAV_TAKEOFF:
			do_takeoff();
			break;

		case MAV_CMD_NAV_WAYPOINT:	// Navigate to Waypoint
			do_nav_wp();
			break;

		case MAV_CMD_NAV_LAND:	// LAND to Waypoint
			do_land();
			break;

		case MAV_CMD_NAV_LOITER_UNLIM:	// Loiter indefinitely
			do_loiter_unlimited();
			break;

		case MAV_CMD_NAV_LOITER_TURNS:	// Loiter N Times
			do_loiter_turns();
			break;

		case MAV_CMD_NAV_LOITER_TIME:
			do_loiter_time();
			break;

		case MAV_CMD_NAV_RETURN_TO_LAUNCH:
			do_RTL();
			break;

		default:
			break;
	}
}

static void
handle_process_condition_command()
{
	//gcs_send_text_fmt(PSTR("Executing command ID #%i"),next_nonnav_command.id);
	switch(next_nonnav_command.id){

		case MAV_CMD_CONDITION_DELAY:
			do_wait_delay();
			break;

		case MAV_CMD_CONDITION_DISTANCE:
			do_within_distance();
			break;

		case MAV_CMD_CONDITION_CHANGE_ALT:
			do_change_alt();
			break;

		case MAV_CMD_CONDITION_YAW:
			do_change_yaw();
			break;

			/*	case MAV_CMD_NAV_LAND_OPTIONS:	//    TODO - Add the command or equiv to MAVLink (repair in verify_condition() also)
			gcs_send_text_P(SEVERITY_LOW,PSTR("Landing options set"));

			// pitch in deg, airspeed  m/s, throttle %, track WP 1 or 0
			landing_pitch 		= next_nav_command.lng * 100;
			g.airspeed_cruise   =  next_nav_command.alt * 100;
			g.throttle_cruise   = next_nav_command.lat;
			landing_distance 	= next_nav_command.p1;

			SendDebug_P("MSG: throttle_cruise = ");
			SendDebugln(g.throttle_cruise,DEC);
			break;
	*/

		default:
			break;
	}
}

static void handle_process_do_command()
{
	//gcs_send_text_fmt(PSTR("Executing command ID #%i"),next_nonnav_command.id);
	switch(next_nonnav_command.id){

		case MAV_CMD_DO_JUMP:
			do_jump();
			break;

		case MAV_CMD_DO_CHANGE_SPEED:
			do_change_speed();
			break;

		case MAV_CMD_DO_SET_HOME:
			do_set_home();
			break;

		case MAV_CMD_DO_SET_SERVO:
			do_set_servo();
			break;

		case MAV_CMD_DO_SET_RELAY:
			do_set_relay();
			break;

		case MAV_CMD_DO_REPEAT_SERVO:
			do_repeat_servo();
			break;

		case MAV_CMD_DO_REPEAT_RELAY:
			do_repeat_relay();
			break;

#if MOUNT == ENABLED
		// Sets the region of interest (ROI) for a sensor set or the
		// vehicle itself. This can then be used by the vehicles control
		// system to control the vehicle attitude and the attitude of various
		// devices such as cameras.
		//    |Region of interest mode. (see MAV_ROI enum)| Waypoint index/ target ID. (see MAV_ROI enum)| ROI index (allows a vehicle to manage multiple cameras etc.)| Empty| x the location of the fixed ROI (see MAV_FRAME)| y| z|
		case MAV_CMD_DO_SET_ROI:
			camera_mount.set_roi_cmd();
			break;

		case MAV_CMD_DO_MOUNT_CONFIGURE:	// Mission command to configure a camera mount |Mount operation mode (see MAV_CONFIGURE_MOUNT_MODE enum)| stabilize roll? (1 = yes, 0 = no)| stabilize pitch? (1 = yes, 0 = no)| stabilize yaw? (1 = yes, 0 = no)| Empty| Empty| Empty|
			camera_mount.configure_cmd();
			break;

		case MAV_CMD_DO_MOUNT_CONTROL:		// Mission command to control a camera mount |pitch(deg*100) or lat, depending on mount mode.| roll(deg*100) or lon depending on mount mode| yaw(deg*100) or alt (in cm) depending on mount mode| Empty| Empty| Empty| Empty|
			camera_mount.control_cmd();
			break;
#endif
		case MAV_CMD_DO_SET_PARAMETER:
			do_set_parameter();
			break;
		
		default:
			gcs_send_text_fmt(PSTR("Command ID #%i NOT RECOGNIZED"),next_nonnav_command.id);
			break;

	}
}

static void handle_no_commands()
{
	gcs_send_text_fmt(PSTR("Returning to Home"));
	next_nav_command = home;
	next_nav_command.alt = read_alt_to_hold();
	next_nav_command.id = MAV_CMD_NAV_LOITER_UNLIM;
	nav_command_ID = MAV_CMD_NAV_LOITER_UNLIM;
	non_nav_command_ID = WAIT_COMMAND;
	handle_process_nav_cmd();
}

/********************************************************************************/
// Verify command Handlers
/********************************************************************************/

static bool verify_nav_command()	// Returns true if command complete
{
	switch(nav_command_ID) {

		case MAV_CMD_NAV_TAKEOFF:
			return verify_takeoff();
			break;

		case MAV_CMD_NAV_LAND:
			return verify_land();
			break;

		case MAV_CMD_NAV_WAYPOINT:
			/*
			nav_bearing += nav_wind();
			nav_bearing += nav_crosstrack();
			nav_bearing = wrap_360(nav_bearing);
			nav_bearing += nav_geo_fence();
			nav_bearing = wrap_360(nav_bearing);
			*/
			return verify_nav_wp();
			break;

		case MAV_CMD_NAV_LOITER_UNLIM:
			return verify_loiter_unlim();
			break;

		case MAV_CMD_NAV_LOITER_TURNS:
			return verify_loiter_turns();
			break;

		case MAV_CMD_NAV_LOITER_TIME:
			return verify_loiter_time();
			break;

		case MAV_CMD_NAV_RETURN_TO_LAUNCH:
			return verify_RTL();
			break;

		default:
			//gcs_send_text_P(SEVERITY_HIGH,PSTR("verify_nav: Invalid or no current Nav cmd"));
			//gcs_send_text_P(SEVERITY_HIGH,PSTR("Alert: No Flight Plan"));
			return false;
			break;
	}
}

static bool verify_condition_command()		// Returns true if command complete
{
	switch(non_nav_command_ID) {
    case NO_COMMAND:
        break;

    case MAV_CMD_CONDITION_DELAY:
        return verify_wait_delay();
        break;

    case MAV_CMD_CONDITION_DISTANCE:
        return verify_within_distance();
        break;

    case MAV_CMD_CONDITION_CHANGE_ALT:
        return verify_change_alt();
        break;
        
		case MAV_CMD_CONDITION_YAW:
        return verify_change_yaw();
        break;
	
	case WAIT_COMMAND:
        return 0;
        break;
        

    default:
        gcs_send_text_P(SEVERITY_HIGH,PSTR("verify_conditon: Invalid or no current Condition cmd"));
        break;
	}
    return false;
}

/********************************************************************************/
//  Nav (Must) commands
/********************************************************************************/

static void do_RTL(void)
{
	control_mode 	= RTL;
	crash_timer 	= 0;
	next_WP 		= home;

	// Altitude to hold over home
	// Set by configuration tool
	// -------------------------
	next_WP.alt = read_alt_to_hold();

	if (g.log_bitmask & MASK_LOG_MODE)
		Log_Write_Mode(control_mode);
}

static void do_takeoff()
{
	set_next_WP(&next_nav_command);
	// pitch in deg, airspeed  m/s, throttle %, track WP 1 or 0
	takeoff_pitch 	 	= (int)next_nav_command.p1 * 100;
			//Serial.printf_P(PSTR("TO pitch:"));	Serial.println(takeoff_pitch);
			//Serial.printf_P(PSTR("home.alt:"));	Serial.println(home.alt);
	takeoff_altitude 	= next_nav_command.alt;
			//Serial.printf_P(PSTR("takeoff_altitude:"));	Serial.println(takeoff_altitude);
	next_WP.lat 		= home.lat + 1000;	// so we don't have bad calcs
	next_WP.lng 		= home.lng + 1000;	// so we don't have bad calcs
	takeoff_complete 	= false;			// set flag to use gps ground course during TO.  IMU will be doing yaw drift correction
											// Flag also used to override "on the ground" throttle disable
}

static void do_nav_wp()
{
	set_next_WP(&next_nav_command);
}

static void do_land()
{
	set_next_WP(&next_nav_command);
}

static void do_loiter_unlimited()
{
	set_next_WP(&next_nav_command);
}

static void do_loiter_turns()
{
	set_next_WP(&next_nav_command);
	loiter_total = next_nav_command.p1 * 360;
}

static void do_loiter_time()
{
	set_next_WP(&next_nav_command);
	//loiter_time = millis();
	loiter_time = 0;
	loiter_time_max = next_nav_command.p1*10; // units are (seconds * 10)
}

/********************************************************************************/
//  Verify Nav (Must) commands
/********************************************************************************/
static bool verify_takeoff()
{
	if (g_gps->ground_speed > 300){
		if(hold_course == -1){
			// save our current course to take off
			if(g.compass_enabled) {
				hold_course = dcm.yaw_sensor;
			} else {
				hold_course = g_gps->ground_course;
			}
		}
	}

	if(hold_course > -1){
		// recalc bearing error with hold_course;
		last_nav_heading = nav_bearing;
		nav_bearing = hold_course;
		// recalc bearing error
		calc_bearing_error();
	}

	if (current_loc.alt > takeoff_altitude)  {
		hold_course = -1;
		takeoff_complete = true;
		return true;
	} else {
		return false;
	}
}

static bool verify_land()
{
	// we don't verify landing - we never go to a new Nav command after Land
	if (((wp_distance > 0) && (wp_distance <= (2*g_gps->ground_speed/100)))
		|| (current_loc.alt <= next_WP.alt + 300)){

		land_complete = 1;		//Set land_complete if we are within 2 seconds distance or within 3 meters altitude

		if(hold_course == -1){
			// save our current course to land
			//hold_course = yaw_sensor;
			// save the course line of the runway to land
			hold_course = crosstrack_bearing;
		}
	}

	if(hold_course > -1){
		// recalc bearing error with hold_course;
		last_nav_heading = nav_bearing;
		nav_bearing = hold_course;
		// recalc bearing error
		calc_bearing_error();
	}

	update_crosstrack();
	return false;
}

static bool verify_nav_wp()
{
	hold_course = -1;
	update_crosstrack();

	float hangle = get_radians(&current_loc, &next_WP, &next_WP2)/2.0;
	float Speed = g_gps->ground_speed/100.0;
	float BankScale = constrain(1.1*exp(-pow((Speed-37)/34.57,2)),0.8,1.1);
	float turn_radius = g.pidNavPitchAirspeed.kP() * (Speed*Speed) / (9.3211*tan(BankScale*ToRad(g.roll_limit/100)));
	//float turn_radius = g.pidNavPitchAirspeed.kP() * (Speed*Speed) / 9.8;
	//float turn_radius = g.pidNavPitchAirspeed.kP()*(Speed*Speed/7.1073 - 1.7*Speed + 19.47);
	//float turn_radius = g.pidNavPitchAirspeed.kP()*(pow(g_gps->ground_speed/100,2)/7.1073 - 1.7*g_gps->ground_speed/100 + 19.47);
	//float turn_radius = g.pidNavPitchAirspeed.kP()*pow(g_gps->ground_speed/100,2)/9.7773;
	//float turn_radius = (2.5-0.025*g_gps->ground_speed/100)*pow(g_gps->ground_speed/100,2)/9.7773;
	//float turn_radius = pow(max((double)airspeed*cos(hcorr),(double)g_gps->ground_speed)/100,2)/9.7773;		//pow(airspeed/100,2)/(85.63*tan(ToRad(g.roll_limit/100)));
	//float turn_radius = pow(airspeed/100,2)/(9.7773*tan(ToRad(45)));	//pow(airspeed/100,2)/(85.63*tan(ToRad(g.roll_limit/100)));
	//float turn_radius = pow(max((double)airspeed,(double)g_gps->ground_speed)/100,2)/(9.7773*tan(ToRad(45)));	//pow(airspeed/100,2)/(85.63*tan(ToRad(g.roll_limit/100)));
	//(3*g_gps->ground_speed/100);
	
	long next_bearing = get_bearing(&next_WP, &next_WP2);
	long delta_Next = wrap_180(dcm.yaw_sensor - target_bearing)/100;
	long delta_Next2 = wrap_180(next_bearing - dcm.yaw_sensor)/100;
	float hcorr  = g.pidNavPitchAirspeed.kI() * ToRad(delta_Next*(delta_Next2/abs(delta_Next2)));
	float cangle = constrain(g.pidNavPitchAirspeed.kD()*(hangle+hcorr),ToRad(1), ToRad(90));

	//if (g.pidServoRudder.kI() > 0)
	//	WptRadius = g.pidServoRudder.kI();
	//else
	WptRadius = (9*WptRadius+turn_radius/tan(cangle))/10;

	/*
	Serial.print("D=");
	Serial.print(wp_distance);
	Serial.print(" R=");
	Serial.print(WptRadius);
	Serial.print(" gR=");
	Serial.print(g.waypoint_radius);
	Serial.print(" tr=");
	Serial.print((int)turn_radius);
	Serial.print(" Gs=");
	Serial.print(Speed);
	Serial.print(" Ba=");
	Serial.print((int)ToDeg(2*hangle));
	Serial.print(" hCorr=");
	Serial.print((int)ToDeg(hcorr));
	Serial.print(" trbs=");
	Serial.print(BankScale);
	Serial.print(" xTrack=");
	Serial.println((int)crosstrack_error);
	*/

	//WptRadius = constrain(WptRadius,(float)g.waypoint_radius,0.75*(float)(wp_totalDistance));
	if (g.waypoint_radius == 0)
		g.waypoint_radius = WP_RADIUS_DEFAULT;

	WptRadius = constrain(WptRadius,(float)g.waypoint_radius,0.75*(float)(wp_totalDistance-g.waypoint_radius));
	//WptRadius = constrain(WptRadius,(float)g.waypoint_radius,(float)3*g.waypoint_radius);

	if ((control_mode == AUTO) && (((wp_distance > 0) && (wp_distance <= WptRadius)) || location_past_point() || skip_wpt)) 
	{
		WptRadius = g.waypoint_radius;

		if (b_4DWaypointsEnabled() && b_4Dflag && (nav_command_index>0))
		{
			// 4D mode is active
			if (skip_wpt)
				gcs_send_text_fmt(PSTR("Skipped Waypoint %i in %d sec"),nav_command_index, (int)elapsed_time);
			else
				gcs_send_text_fmt(PSTR("Reached Waypoint %i in %d sec"),nav_command_index, (int)elapsed_time);
		}
		else
		{
			// 4D mode is not active
			if (skip_wpt)
				gcs_send_text_fmt(PSTR("Skipped Waypoint %i"),nav_command_index);
			else
				gcs_send_text_fmt(PSTR("Reached Waypoint %i"),nav_command_index);
		}

		skip_wpt = false;
		elapsed_time = 0;

		// It is Assumed that to reach this section of code, system
		// needs to be in AUTO mode
		if (b_4DWaypointsEnabled() && (b_4Dflag==false) && (nav_command_index>0))
		{
			time_left = 0;
			b_4Dflag = true;
		}
		//mode_change_counter = 20;
		return true;
	}
	// add in a more complex case
	// Doug to do
	if(loiter_sum > 300)
	{
		//mode_change_counter = 20;
		gcs_send_text_P(SEVERITY_MEDIUM,PSTR("Missed WP"));
		return true;
	}
	return false;
}

static bool location_past_point(void)
{
	Vector2f loc1(current_loc.lat,current_loc.lng);
	Vector2f pt1(prev_WP.lat,prev_WP.lng);
	Vector2f pt2(next_WP.lat,next_WP.lng);

	float angle = (float)acos(((loc1-pt2)*(pt1-pt2))/((loc1-pt2).length()*(pt1-pt2).length()));

	if (isinf(angle))
	{
		if (get_distance(&current_loc, &next_WP) == 0)
		{
			return true;
		}
		return false;
	}
	else if (angle == 0)
	{
		return get_distance(&current_loc, &prev_WP) > get_distance(&next_WP, &prev_WP);
	}
	if (degrees(angle) > 90)
	{
		return true;
	}
	return false;
}

static bool verify_loiter_unlim()
{
	update_loiter();
	calc_bearing_error();
	//gcs_send_text_fmt(PSTR("LOITER #%d)"),loiter_sum);
	return false;
}

static bool verify_loiter_time()
{
	update_loiter();
	calc_bearing_error();
	unsigned long loiter_dt = (loiter_time == 0) ? 0 : (millis() - loiter_time)/1000;
	if (loiter_dt > 0)
		gcs_send_text_fmt(PSTR("LOITER time #%d (ms)"),loiter_dt);
	//if ((millis() - loiter_time) > (unsigned long)loiter_time_max * 10000L) {		// scale loiter_time_max from (sec*10) to milliseconds
	if (loiter_dt > (unsigned long)loiter_time_max)
	{		// scale loiter_time_max from (sec*10) to milliseconds
		elapsed_time = 0;
		time_left = 0;
		gcs_send_text_P(SEVERITY_LOW,PSTR("verify_nav: LOITER time complete"));
		return true;
	}
	return false;
}

static bool verify_loiter_turns()
{
	update_loiter();
	calc_bearing_error();
	if(loiter_sum > loiter_total) 
	{
		elapsed_time = 0;
		time_left = 0;
		loiter_total = 0;
		gcs_send_text_P(SEVERITY_LOW,PSTR("verify_nav: LOITER orbits complete"));
		// clear the command queue;
		return true;
	}
	return false;
}

static bool verify_RTL()
{
	if (wp_distance <= g.waypoint_radius) {
		gcs_send_text_P(SEVERITY_LOW,PSTR("Reached home"));
		return true;
	}else{
		return false;
	}
}

/********************************************************************************/
//  Condition (May) commands
/********************************************************************************/

static void do_wait_delay()
{
	condition_start = millis();
	condition_value  = next_nonnav_command.lat * 1000;	// convert to milliseconds
}

static void do_change_alt()
{
	condition_rate		= next_nonnav_command.lat;
	condition_value 	= next_nonnav_command.alt;
	//target_altitude		= current_loc.alt - home.alt + (condition_rate / 10);		// Divide by ten for 10Hz update
	target_altitude		= current_loc.alt + (condition_rate / 10);		// Divide by ten for 10Hz update
	next_WP.alt 		= condition_value;								// For future nav calculations
	offset_altitude 	= 0;											// For future nav calculations
}

static void do_change_yaw()
{
	gcs_send_text_fmt(PSTR("In do_change_yaw().  Target heading: %i"),next_nonnav_command.lat);
	condition_rate		= next_nonnav_command.lng*100;
	if (next_nonnav_command.p1 == 1) // relative turn
		condition_value 	= wrap_360(dcm.yaw_sensor + next_nonnav_command.lat*100);
	else							 // absolute turn
		condition_value 	= next_nonnav_command.lat*100;

	last_nav_heading = nav_bearing;
	nav_bearing  		= wrap_360(dcm.yaw_sensor + (condition_rate / 10));		// Divide by ten for 10Hz update
//	next_WP.alt 		= condition_value;								// For future nav calculations
//	offset_altitude 	= 0;											// For future nav calculations
}

static void do_within_distance()
{
	condition_value  = next_nonnav_command.lat;
}

/********************************************************************************/
// Verify Condition (May) commands
/********************************************************************************/

static bool verify_wait_delay()
{
	if ((unsigned)(millis() - condition_start) > condition_value){
		condition_value 	= 0;
		return true;
	}
	return false;
}

static bool verify_change_alt()
{
	if( (condition_rate>=0 && current_loc.alt >= condition_value) || (condition_rate<=0 && current_loc.alt <= condition_value)) {
		condition_value = 0;
		return true;
	}
	target_altitude += condition_rate / 10;
	return false;
}

static bool verify_change_yaw()
{
	if( (dcm.yaw_sensor > wrap_360(condition_value-200)) && (dcm.yaw_sensor < wrap_360(condition_value+200))) {
		condition_value = 0;
		gcs_send_text_fmt(PSTR("In verify_change_yaw().  Nav Bearing: %i"),nav_bearing);
		return true;
	}
	gcs_send_text_fmt(PSTR("In verify_change_yaw().  Nav Bearing: %i"),nav_bearing);
	last_nav_heading = nav_bearing;
	nav_bearing += condition_rate / 10;
	nav_bearing = wrap_360(nav_bearing);
	return false;
}

static bool verify_within_distance()
{
	if (wp_distance < condition_value){
		condition_value = 0;
		return true;
	}
	return false;
}

/********************************************************************************/
//  Do (Now) commands
/********************************************************************************/

static void do_loiter_at_location()
{
	next_WP = current_loc;
}

static void do_jump()
{
	struct Location temp;
	//gcs_send_text_fmt(PSTR("Jumps to Waypoint %i left: %i"),next_nonnav_command.p1,next_nonnav_command.lat);
	
	if(next_nonnav_command.lat > 0) 
	{
		nav_command_ID		= NO_COMMAND;
		next_nav_command.id = NO_COMMAND;
		non_nav_command_ID 	= NO_COMMAND;
		
		temp 				= get_cmd_with_index(g.command_index);
		temp.lat 			= next_nonnav_command.lat - 1;					// Decrement repeat counter

		set_cmd_with_index(temp, g.command_index);
		//gcs_send_text_fmt(PSTR("setting command index: %i"),next_nonnav_command.p1 - 1);
		g.command_index.set_and_save(next_nonnav_command.p1 - 1);
		nav_command_index 	= next_nonnav_command.p1 - 1;
		next_WP = prev_WP;		// Need to back "next_WP" up as it was set to the next waypoint following the jump
		
		// commented on 3/23/15 to fix problem with jump commands not working correctly
		//process_next_command();
	} 
	else if (next_nonnav_command.lat == -1) 
	{								// A repeat count of -1 = repeat forever
		nav_command_ID 	= NO_COMMAND;
		non_nav_command_ID 	= NO_COMMAND;
		//gcs_send_text_fmt(PSTR("setting command index: %i"),next_nonnav_command.p1 - 1);
	    g.command_index.set_and_save(next_nonnav_command.p1 - 1);
		nav_command_index 	= next_nonnav_command.p1 - 1;
		next_WP = prev_WP;		// Need to back "next_WP" up as it was set to the next waypoint following the jump
		
		// commented on 3/23/15 to fix problem with jump commands not working correctly
		//process_next_command();
	}
}

static void do_set_parameter()
{
	switch (next_nonnav_command.p1)
	{
		case 178:	// k_param_throttle_min
			if(next_nonnav_command.alt > 0)
				g.throttle_min.set_and_save(next_nonnav_command.alt);
			break;
		case 179:	// k_param_throttle_max: 
				g.throttle_max.set_and_save(next_nonnav_command.alt);
			break;
		default:
			gcs_send_text_fmt(PSTR("Parameter ID #%i NOT RECOGNIZED"),next_nonnav_command.id);
			break;
	}

}

static void do_change_speed()
{
	switch (next_nonnav_command.p1)
	{
		case 0: // Airspeed
			if(next_nonnav_command.alt > 0)
				g.airspeed_cruise.set_and_save(next_nonnav_command.alt * 100);
			break;
		case 1: // Ground speed
			g.min_gndspeed.set_and_save(next_nonnav_command.alt * 100);
			break;
	}

	if(next_nonnav_command.lat > 0)
		g.throttle_cruise.set_and_save(next_nonnav_command.lat);
}

static void do_set_home()
{
	if(next_nonnav_command.p1 == 1 && GPS_enabled) {
		init_home();
	} else {
		home.id 	= MAV_CMD_NAV_WAYPOINT;
		home.lng 	= next_nonnav_command.lng;				// Lon * 10**7
		home.lat 	= next_nonnav_command.lat;				// Lat * 10**7
		home.alt 	= max(next_nonnav_command.alt, 0);
		home_is_set = true;
	}
}

static void do_set_servo()
{
	APM_RC.OutputCh(next_nonnav_command.p1 - 1, next_nonnav_command.alt);
}

static void do_set_relay()
{
	if (next_nonnav_command.p1 == 1) {
		relay.on();
	} else if (next_nonnav_command.p1 == 0) {
		relay.off();
	}else{
		relay.toggle();
	}
}

static void do_repeat_servo()
{
	event_id = next_nonnav_command.p1 - 1;

	if(next_nonnav_command.p1 >= CH_5 + 1 && next_nonnav_command.p1 <= CH_8 + 1) {

		event_timer 	= 0;
		event_delay 	= next_nonnav_command.lng * 500.0;	// /2 (half cycle time) * 1000 (convert to milliseconds)
		event_repeat 	= next_nonnav_command.lat * 2;
		event_value 	= next_nonnav_command.alt;

		switch(next_nonnav_command.p1) {
			case CH_5:
				event_undo_value = g.rc_5.radio_trim;
				break;
			case CH_6:
				event_undo_value = g.rc_6.radio_trim;
				break;
			case CH_7:
				event_undo_value = g.rc_7.radio_trim;
				break;
			case CH_8:
				event_undo_value = g.rc_8.radio_trim;
				break;
		}
		update_events();
	}
}

static void do_repeat_relay()
{
	event_id 		= RELAY_TOGGLE;
	event_timer 	= 0;
	event_delay 	= next_nonnav_command.lat * 500.0;	// /2 (half cycle time) * 1000 (convert to milliseconds)
	event_repeat	= next_nonnav_command.alt * 2;
	update_events();
}
