// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//****************************************************************
// Function that will calculate the desired direction to fly and distance
//****************************************************************
static void navigate()
{
	// do not navigate with corrupt data
	// ---------------------------------
	if (g_gps->fix == 0)
	{
		g_gps->new_data = false;
		return;
	}

	if(next_WP.lat == 0){
		return;
	}

	// waypoint distance from plane
	// ----------------------------
	wp_distance  = get_distance(&current_loc, &next_WP);
	wp_distance2 = get_distance(&current_loc, &next_WP2);

	if (wp_distance < 0){
		gcs_send_text_P(SEVERITY_HIGH,PSTR("<navigate> WP error - distance < 0"));
		//Serial.println(wp_distance,DEC);
		return;
	}

	// target_bearing is where we should be heading
	// --------------------------------------------
	target_bearing 	= get_bearing(&current_loc, &next_WP);

	// nav_bearing will includes xtrac correction
	// ------------------------------------------
	//nav_bearing 		= target_bearing;

	// check if we have missed the WP
	loiter_delta = (target_bearing - old_target_bearing)/100;

	// reset the old value
	old_target_bearing = target_bearing;

	// wrap values
	if (loiter_delta > 180) loiter_delta -= 360;
	if (loiter_delta < -180) loiter_delta += 360;
	loiter_sum += abs(loiter_delta);

	// control mode specific updates to nav_bearing
	// --------------------------------------------
	update_navigation();
}


static void calc_airspeed_errors()
{
	// Do not change target speed if in the middle of a turn
	if(!cdnr_sflag)
	{
		if (b_4DWaypointsEnabled() && (b_4Dflag==true))
		{
			if (control_mode == AUTO && next_WP.id != MAV_CMD_NAV_LOITER_UNLIM)
			{
				if ((time_left > 1) && ((wp_distance-WptRadius) > 0))
				{
					//if (abs(bearing_error) < 7500)
					{
						//float nTarget = constrain(100.0f*(float)(wp_distance-WptRadius)/time_left, (float)g.flybywire_airspeed_min.get()*100.0f, (float)g.flybywire_airspeed_max.get()*100.0f);
						float nTarget = 100.0f*(float)(wp_distance-WptRadius)/time_left;
						f_airspeed = ((99*f_airspeed) + (nTarget))/100;
					
						//Serial.printf_P (PSTR("target %d=(%d/%d)\n")
						//				,(int)(target_airspeed/100)
						//				,(int)wp_distance
						//				,(int)time_left
						//				,(int)((float)wp_distance/time_left));
					}
				}
			}
		}
		else
		{
			// Normal airspeed target
			f_airspeed = g.airspeed_cruise;

			// FBW_B airspeed target
			if (control_mode == FLY_BY_WIRE_B) 
			{
				f_airspeed = ((int)(g.flybywire_airspeed_max -
										 g.flybywire_airspeed_min) *
								   g.channel_throttle.servo_out) +
								  ((int)g.flybywire_airspeed_min * 100);
			}

			// Set target to current airspeed + ground speed undershoot,
			// but only when this is faster than the target airspeed commanded
			// above.
			/*
			if (control_mode >= FLY_BY_WIRE_B && (g.min_gndspeed > 0)) {
				long min_gnd_target_airspeed = airspeed + groundspeed_undershoot;
				if (min_gnd_target_airspeed > target_airspeed)
					f_airspeed = min_gnd_target_airspeed;
			}
			*/
		}
	}

	// Apply airspeed limit

	if (b_4DwGrounSpeedEnabled())
	{
		target_airspeed = (long)f_airspeed;
		airspeed_error = target_airspeed - g_gps->ground_speed;
		if (airspeed < (g.flybywire_airspeed_min.get() * 100))
		{
			if (airspeed_error < 0)
			{	airspeed_error = (g.flybywire_airspeed_min.get()*100 - airspeed); }
			else
			{	airspeed_error += (g.flybywire_airspeed_min.get()*100 - airspeed); }
		}
	}
	else
	{
		f_airspeed = constrain(f_airspeed, (float)g.flybywire_airspeed_min.get()*100.0f, (float)g.flybywire_airspeed_max.get()*100.0f);
		target_airspeed = (long)f_airspeed;
		airspeed_error = (float)(target_airspeed - airspeed);
	}

	airspeed_energy_error = ((target_airspeed * target_airspeed) - ((long)airspeed * (long)airspeed))/20000; //Changed 0.00005f * to / 20000 to avoid floating point calculation
}

static void calc_gndspeed_undershoot()
{
    // Function is overkill, but here in case we want to add filtering later
    groundspeed_undershoot = (g.min_gndspeed > 0) ? (g.min_gndspeed - g_gps->ground_speed) : 0;
}

static void calc_bearing_error()
{
	if(takeoff_complete == true  || g.compass_enabled == true) {
        /*
          most of the time we use the yaw sensor for heading, even if
          we don't have a compass. The yaw sensor is drift corrected
          in the DCM library. We only use the gps ground course
          directly if we haven't completed takeoff, as the yaw drift
          correction won't have had a chance to kick in. Drift
          correction using the GPS typically takes 10 seconds or so
          for a 180 degree correction.
         */
		bearing_error = wrap_180(nav_bearing - dcm.yaw_sensor);
	} else {

		// TODO: we need to use the Yaw gyro for in between GPS reads,
		// maybe as an offset from a saved gryo value.
		bearing_error = wrap_180(nav_bearing - g_gps->ground_course);
	}
}

static void calc_altitude_error()
{
	if (!cdnr_aflag)
	{
		if(control_mode == AUTO && offset_altitude != 0) 
		{
#if 0
			target_altitude = next_WP.alt;
			// climb immediately
			if (target_altitude != next_WP.alt)
			{
				if (next_WP.alt > pre_target_altitude)
				{
					if (next_WP.alt-pre_target_altitude > 10) 
						target_altitude = pre_target_altitude+40;
					else
						target_altitude = next_WP.alt;
				}
				else
				{
					if (pre_target_altitude-next_WP.alt > 10) 
						target_altitude = pre_target_altitude-40;
					else
						target_altitude = next_WP.alt;
				}
			}
			else
				target_altitude = next_WP.alt;

			pre_target_altitude = target_altitude;
#else
			// limit climb rates
			//target_altitude = next_WP.alt - ((float)((wp_distance - g.waypoint_radius) * offset_altitude) / (float)(wp_totalDistance - g.waypoint_radius));
			float ratio;
			float margin = 1.25f*(float)WptRadius;

			if (margin > (float)wp_totalDistance)
				ratio = 0.0f;
			else if (margin > (float)wp_distance)
				ratio = 0.0f;
			else
				ratio = (float)((float)wp_distance - margin) / (float)((float)wp_totalDistance - margin);

			target_altitude = (long)next_WP.alt - (long)(ratio * (float)offset_altitude);
#endif
			// stay within a certain range
			if(prev_WP.alt > next_WP.alt)
			{
				target_altitude = constrain(target_altitude, next_WP.alt, prev_WP.alt);
			}
			else
			{
				target_altitude = constrain(target_altitude, prev_WP.alt, next_WP.alt);
			}
		} 
		else if (control_mode != FLY_BY_WIRE_B && non_nav_command_ID != MAV_CMD_CONDITION_CHANGE_ALT) 
		{
			target_altitude = next_WP.alt;
		}
			//target_altitude_FBW_B = target_altitude;  // set FBW_B to what is used in auto
	}
	altitude_error 	= target_altitude - current_loc.alt; 
}

static long wrap_360(long error)
{
	if (error > 36000)	error -= 36000;
	if (error < 0)		error += 36000;
	return error;
}

static long wrap_180(long error)
{
	if (error > 18000)	error -= 36000;
	if (error < -18000)	error += 36000;
	return error;
}

	static void update_loiter()
	{
		float power;
		if(wp_distance <= g.loiter_radius){
			if (loiter_time == 0)
				loiter_time = millis();			// keep start time for loiter updating till we get within LOITER_RANGE of orbit
				
			power = float(wp_distance) / float(g.loiter_radius);
			power = constrain(power, 0.5, 1);
			last_nav_heading = nav_bearing;
			nav_bearing += (int)(g.crosstrack_gain * (2.0 + power));
		}else if(wp_distance < 2*g.loiter_radius){
			power = -((float)(wp_distance - 2*g.loiter_radius) / g.loiter_radius);
			power = constrain(power, 0.5, 1);			//power = constrain(power, 0, 1);
			last_nav_heading = nav_bearing;
			nav_bearing -= (int)(power * g.crosstrack_gain);

		}else{
			update_crosstrack();
			//loiter_time = millis();			// keep start time for loiter updating till we get within LOITER_RANGE of orbit
		
		}
	
		/*if (wp_distance < g.loiter_radius)
		{
			nav_bearing += g.crosstrack_gain;
		}
		else
		{
			nav_bearing -= 18000 * asin(g.loiter_radius / wp_distance) / M_PI;
		}

		update_crosstrack();
		loiter_time = millis();			// keep start time for loiter updating till we get within LOITER_RANGE of orbit
		*/
		nav_bearing = wrap_360(nav_bearing);
	}

static void dowindcalc()
{
    //Wind Fixed gain Observer
    //Ryan Beall 
    //8FEB10

    if (airspeed < 1 || g_gps->ground_speed < 1)
        return;

	double Kw = 0.010; // 0.01 // 0.10
    double Wn_error = airspeed * cos(ToRad(dcm.yaw_sensor/100)) * cos(ToRad((dcm.pitch_sensor - g.pitch_trim)/100)) - g_gps->ground_speed * cos(ToRad(g_gps->ground_course/100)) - Wn_fgo;
    double We_error = airspeed * sin(ToRad(dcm.yaw_sensor/100)) * cos(ToRad((dcm.pitch_sensor - g.pitch_trim)/100)) - g_gps->ground_speed * sin(ToRad(g_gps->ground_course/100)) - We_fgo;

    Wn_fgo = Wn_fgo + Kw * Wn_error;
    We_fgo = We_fgo + Kw * We_error;

    wind_dir = (float)wrap_360((long)ToDeg(atan2(We_fgo, Wn_fgo))*100);
    wind_vel = (float)sqrt(We_fgo*We_fgo + Wn_fgo*Wn_fgo);
}

static void update_crosstrack(void)
{
	long the_bearing;
		
	// Crosstrack Error
	// ----------------
	dowindcalc();

	// Crosstrack Error
	// ----------------

	if (cdnr_hflag)
		the_bearing = cdnr_bearing;
	else
		the_bearing = target_bearing;

	last_nav_heading = nav_bearing;
	nav_bearing = the_bearing;

	long the_speed;
	if (b_4DwGrounSpeedEnabled())
		the_speed = g_gps->ground_speed;
	else
		the_speed = airspeed;

	if (b_useWindCorrection())
	{
		double north = the_speed*cos(ToRad(the_bearing/100)) + wind_vel*cos(ToRad(wind_dir/100));
		double east  = the_speed*sin(ToRad(the_bearing/100)) + wind_vel*sin(ToRad(wind_dir/100));
		nav_bearing = wrap_360((long)ToDeg(atan2(east, north))*100);
	}

	crosstrack_error = sin(ToRad((the_bearing - crosstrack_bearing) / (float)100)) * (float)wp_distance;	 // Meters we are off track line
		
	int pidRes = 100*g.pidTeThrottle.get_pid(crosstrack_error,delta_ms_fast_loop);
	pidRes = constrain(pidRes, -g.crosstrack_entry_angle.get(), g.crosstrack_entry_angle.get());
	nav_bearing += pidRes;
	nav_bearing = wrap_360(nav_bearing);
	nav_bearing += nav_geo_fence();
	nav_bearing = wrap_360(nav_bearing);
}

static void reset_crosstrack()
{
	crosstrack_bearing 	= get_bearing(&prev_WP, &next_WP);	// Used for track following
}


static long nav_wind()
{
	return (g.crosstrack_entry_angle.get()*ToDeg(asin((wind_vel/(float)airspeed)* sin(ToRad(wrap_180((long)wind_dir - target_bearing)/100)))));
}

static long nav_crosstrack()
{
	return (constrain(100*g.pidTeThrottle.get_pid(crosstrack_error,delta_ms_fast_loop),0,4500));
}

/*
#define GEO_GAIN 4000.0f
static long nav_geo_fence()
{
    uint8_t i,j;
	int segn=0;
	float weight;
	long geoHeading;
	long resHeading;
	float distm;
	float mdist;
	float mweight;
	//float north;
	//float east;
	//float North;
	//float East;
	Vector2f v;
	Vector2f w;
	Vector2f p;
	Vector2f vp;
	Vector2f wp;
	Vector2f geo;
	Vector2f geoPoint;
	Vector2f xp,yp;
	Vector2f nav;
	Vector2f cur;
	Vector2f cur_xp;
	Vector2f nav_xp;
	Vector2f nav_yp;
	Vector2f geo_yp;

    if (geofence_state == NULL) 
		return (0L);
    
	// Current Location
	p = Vector2f((float)current_loc.lat/1e7f,(float)current_loc.lng/1e7f);
	// Current Heading
	cur = Vector2f((float)(cos(ToRad((float)dcm.yaw_sensor/100.0f))), (float)(sin(ToRad((float)dcm.yaw_sensor/100.0f))));
	// Nav Heading
	nav = Vector2f((float)(cos(ToRad((float)nav_bearing/100.0f))), (float)(sin(ToRad((float)nav_bearing/100.0f))));
	geo = Vector2f(0.0f,0.0f);
	//nav.normalize();

	mdist = 1e14f;
	for (i=1,j=i+1; i<geofence_state->num_points-1; j++, i++) 
	{
		v = Vector2f((float)geofence_state->boundary[i].x,(float)geofence_state->boundary[i].y)/1e7f;
		w = Vector2f((float)geofence_state->boundary[j].x,(float)geofence_state->boundary[j].y)/1e7f;
		geoPoint = find_closest_point(v, w, p);
		distm = get_distance(&p,&geoPoint);
		vp = v;
		wp = w;
		//gp = geoPoint;
		
		// Geofence repelling force
		//weight = ((float)g.pidNavPitchAirspeed.imax()/100.0f)*GEO_GAIN/(dist*dist);
		//weight = constrain(GEO_GAIN/(distm*distm),0.0f,1.0f);
		weight = 2.0f * (1.0f - 1.0f/(1.0f+(float)exp(-0.015f*(distm-0))));
		if (distm < mdist)
		{
			segn = i;
			mdist = distm;
			mweight = weight;
			xp = (w-v).normalized();			// Vector along fence
			yp = (p-geoPoint).normalized();		// Vector normal to fence
		}
	
		// If outside fence, reverse direction to attract airplane towards it so it can re-enter fence
		if (geofence_state->state == OUTSIDE) yp = -yp;

		geo += yp * weight;
	}

	cur_xp = cur.projected(xp);
	nav_xp = nav.projected(xp);
	nav_yp = nav.projected(yp)*(1.0f-mweight);
	geo_yp = geo.projected(yp);
	//Serial.printf_P(PSTR("tangent=%.2f ortho=%.2f geo=%.2f\n"),nav_xp.length(),nav_yp.length(),geo_yp.length());
	//Serial.printf_P(PSTR("sign=%.2f tangent=%.2f ortho=%.2f\n"),dot(cur_xp.normalized(),nav_xp.normalized()),dot(nav_xp,xp),dot(nav_yp+geo_yp,yp));
	if ((mdist < 100.0f) && (dot(cur_xp.normalized(),nav_xp.normalized()) < 0))
		nav_xp *= -1.0f;
	
	nav = nav_xp + nav_yp + geo_yp;
	//nav = nav_xp + geo;
	//nav += geo;

	geoHeading = (long)get_bearing(&p, &(p+nav));
	resHeading = constrain(wrap_180(geoHeading - nav_bearing), -18000, 18000);
	Serial.printf_P (PSTR("segn=%d dist=%.0f weight=%.3f angle=%d\n"),segn,mdist,mweight,resHeading/100);
	return (resHeading);
}
*/

static long nav_geo_fence()
{
    uint8_t i,j;
	int segn=0;
	long geoHeading;
	long navHeading;
	static long resHeading = 0;
	//static long mode_change_counter = 50;
	float distm;
	float mdist;
	//float pdist;
	float mweight;
	float dweight;
	//float aweight;
	//float pweight;
	//float direction;
	//const float q=0.75f;
	Vector2f v;
	Vector2f w;
	Vector2f p;
	//Vector2f vp;
	Vector2f wp;
	//Vector2f geo;
	Vector2f geoPoint;
	Vector2f xp,yp;
	Vector2f geoNav = Vector2f(0, 0);
	Vector2f nav;
	Vector2f head;
	//Vector2f pnav;
	//Vector2f cur;
	//Vector2f cur_xp;
	Vector2f nav_xp;
	Vector2f nav_yp;
	//Vector2f geo_yp;
	const long navSLEW = 2000;

    skip_wpt = false;
	
	if (geofence_state == NULL) 
		return (0L);
    
	// Current Location
	p = Vector2f((float)current_loc.lat,(float)current_loc.lng)/1e7f;
	
	// Waypoint Location
	wp = Vector2f((float)next_WP.lat,(float)next_WP.lng)/1e7f;
	
	// Nav Heading
	//nav = Vector2f((float)(cos(ToRad((float)nav_bearing/100.0f))), (float)(sin(ToRad((float)nav_bearing/100.0f))));
	nav = Vector2f((float)(cos(ToRad((float)nav_bearing/100.0f))), (float)(sin(ToRad((float)nav_bearing/100.0f))));
	head = Vector2f((float)(cos(ToRad((float)dcm.yaw_sensor/100.0f))), (float)(sin(ToRad((float)dcm.yaw_sensor/100.0f))));
	//pnav = Vector2f((float)(cos(ToRad((float)last_nav_heading/100.0f))), (float)(sin(ToRad((float)last_nav_heading/100.0f))));
	
	mdist = 1e14f;
	for (i=1,j=i+1; i<geofence_state->num_points-1; j++, i++) 
	{
		v = Vector2f((float)geofence_state->boundary[i].x,(float)geofence_state->boundary[i].y)/1e7f;
		w = Vector2f((float)geofence_state->boundary[j].x,(float)geofence_state->boundary[j].y)/1e7f;
		geoPoint = find_closest_point(v, w, p);
		distm = get_distance(&p,&geoPoint);
		//Serial.printf_P (PSTR("geo <%.2f,%.2f> %.2f\n"),geoPoint.x,geoPoint.y,distm);
		//if (distm > 0 && distm < mdist)
		if (distm > 0)
		{
			segn = i;
			xp = (w-v).normalized();			// Vector along fence
			//Serial.printf_P (PSTR("xp <%.2f,%.2f> "),xp.x,xp.y);
			if (geofence_state->state == OUTSIDE)
			{
				yp = (geoPoint-p).normalized();		// Vector normal to fence
				//Serial.printf_P (PSTR("yp_out <%.2f,%.2f> "),yp.x,yp.y);
			}
			else
			{
				yp = (p-geoPoint).normalized();		// Vector normal to fence
				//Serial.printf_P (PSTR("yp_in  <%.2f,%.2f> "),yp.x,yp.y);
			}
			if (distm < mdist) mdist = distm;

			// Calculate weights
			//aweight = (dot(pnav,-yp)+1.0f)/2.0f;
			//aweight = 0.0f;
			//dweight = 1.0f/(1.0f+exp(-0.05f*(150.0f+20.0f*aweight-mdist)));

#if 0
			// Using the vector xp
			float dotFactor = fabs(dot(nav,xp));
			float dt = (100.0f*(2.0f-dotFactor));
#else
			// Using the vector yp
			float dotFactor = dot(head,yp);
			float dt = (100.0f*(1.0f-dotFactor));
#endif
			dweight = 1.0f/(1.0f+exp((distm-dt)/20.0f));
			mweight = dweight;
			//if (mweight > 0.2) Serial.printf_P (PSTR("[%2.2d] "),segn);
			//if (mweight > 0.2) Serial.printf_P (PSTR("w=%.2f dt=%.2f  "),mweight,dt);

			//Serial.printf_P (PSTR("mdist=%.2f  weight=%.2f\n"),mdist,dweight);
			//mweight = dweight * (q + (1.0f-q)*aweight);

			// Correct Nav Bearing due to closeness to the geofence
			//nav_yp = yp * mweight + nav.projected(yp)*(1.0f-mweight);
			//nav_xp = nav.projected(xp) * (float)sqrt(1.0f-nav_yp.length_squared());
			//nav = (nav_xp + nav_yp).normalized();

			// Correct Nav Bearing due to closeness to the geofence
			nav_xp = nav.projected(xp)*(1.0f-0.75f*mweight);
			nav_yp = yp * mweight + nav.projected(yp)*(1.0f-mweight);
			//Serial.printf_P (PSTR("w=%.2f "),mweight);
			//if (mweight > 0.2) Serial.printf_P (PSTR("nXp<%.2f,%.2f> "),nav_xp.x,nav_xp.y);
			//if (mweight > 0.2) Serial.printf_P (PSTR("nYp<%.2f,%.2f> "),nav_yp.x,nav_yp.y);
			geoNav = ((nav_xp + nav_yp)*mweight + geoNav);
		}
	}
	geoNav.normalize();
	Serial.printf_P (PSTR("nav<%.2f,%.2f> "),nav.x,nav.y);
	Serial.printf_P (PSTR("geo<%.2f,%.2f> "),geoNav.x,geoNav.y);
	//Serial.printf_P (PSTR("nXp <%.2f,%.2f> "),nav_xp.x,nav_xp.y);
	//Serial.printf_P (PSTR("nYp <%.2f,%.2f> "),nav_yp.x,nav_yp.y);

	//Serial.printf_P (PSTR("segn=%d dist=%.0f dw=%.3f aw=%.3f weight=%.3f "),segn,mdist,dweight,aweight,mweight);

	// Correct Nav Bearing due to closeness to the Waypoint Transition Line
	// if avoiding geofence
	//geoPoint = find_closest_point(wp, wp+yp, p);
	//pdist = get_distance(&p,&geoPoint);
	//pweight = 2.0f/(1.0f+(float)exp(-0.015f*pdist)) - 1.0f;
	//Serial.printf_P (PSTR("wpdist=%.0f weight=%.3f "),pdist,pweight);
	//if ((mweight < 0.5) && (dot(nav_yp,yp) < 0.0f))
	//{
	//	nav_xp = nav.projected(xp)*pweight;
	//	nav_yp = yp * (float)sqrt(1-nav_xp.length_squared());
	//	nav = nav_xp + nav_yp;
	//}

	geoHeading = (long)get_bearing(&p, &(p+geoNav));
	navHeading = (long)get_bearing(&p, &(p+nav));
	//Serial.printf_P (PSTR("nav <%.2f,%.2f>  gH=%d nH=%d "),geoNav.x,geoNav.y,geoHeading/100,navHeading/100);
	Serial.printf_P (PSTR("gH=%d "),geoHeading/100);
	Serial.printf_P (PSTR("nH=%d "),navHeading/100);
	//resHeading = wrap_180(last_nav_heading-nav_bearing) + constrain(wrap_180(geoHeading-last_nav_heading),-navSLEW,navSLEW);
	//resHeading = constrain(wrap_180(geoHeading - nav_bearing), -18000, 18000);
	//resHeading = constrain(wrap_180(geoHeading - navHeading), -navSLEW, navSLEW);
	resHeading = wrap_180(geoHeading - navHeading);
	Serial.printf_P (PSTR("Geo=%d\n"),resHeading/100);

	if (mode_change_counter > 0) 
		mode_change_counter = mode_change_counter-1;
	
	if (geofence_enabled())
	{
		//Serial.printf_P (PSTR("Geo ON"));
		if ((mode_change_counter == 0) && (abs(resHeading) > 9000))
		{
			//Serial.printf_P (PSTR("[%2.2d] "),mode_change_counter);
			Serial.printf_P (PSTR("Geo=%d\n"),resHeading/100);
			//skip_wpt = true;
			//mode_change_counter = 10;
		}

		return (resHeading);
	}
	else
	{
		//Serial.printf_P (PSTR("Geo OFF"));
		return (0L);
	}
}

static Vector2f find_closest_point(Vector2f v, Vector2f w, Vector2f p)
{
	float t;
	Vector2f projection;

	// Consider the line extending the segment, parameterized as v + t (w - v).
	// We find projection of point p onto the line. 
	// It falls where t = [(p-v) . (w-v)] / |w-v|^2
	
	t = dot(w-v,p-v)/dot(w-v,w-v);
	if (t>0.0f && t<1.0f)
		projection = v + (w - v) * t;  // Projection falls on the segment
	else
		projection = v*0.0f;
	return (projection);
}

static float dot(Vector2f a, Vector2f b)
{
	return ((a.x * b.x) + (a.y * b.y));
}

static long get_distance(struct Location *loc1, struct Location *loc2)
{
	return ((long)get_distance((float)loc1->lat, (float)loc1->lng, (float)loc2->lat, (float)loc2->lng));
}
static long get_distance(int32_t x1, int32_t y1, int32_t x2, int32_t y2)
{
	if(x1 == 0 || y1 == 0)
		return -1;
	if(x2 == 0 || y2 == 0)
		return -1;
	float dlat 		= (float)(x2 - x1);
	float dlong		= ((float)(y2 - y1)) * scaleLongDown;
	return (long)(sqrt(sq(dlat) + sq(dlong)) * .01113195);
}

static float get_distance(Vector2f *loc1, Vector2f *loc2)
{
	return (get_distance(1e7f*loc1->x, 1e7f*loc1->y, 1e7f*loc2->x, 1e7f*loc2->y));
}
static float get_distance(float x1, float y1, float x2, float y2)
{
	if(x1 == 0 || y1 == 0)
		return -1;
	if(x2 == 0 || y2 == 0)
		return -1;
	//Serial.printf_P(PSTR("P1(%.0f,%.0f) P2(%.0f,%.0f)\n"),x1,y1,x2,y2);
	float dlat 		= (float)(x2 - x1);
	float dlong		= ((float)(y2 - y1)) * scaleLongDown;
	return (float)(sqrt(sq(dlat) + sq(dlong)) * .01113195);
}


static long get_bearing(struct Location *loc1, struct Location *loc2)
{
	return (get_bearing(loc1->lat, loc1->lng, loc2->lat, loc2->lng));
}
static long get_bearing(int32_t x1, int32_t y1, int32_t x2, int32_t y2)
{
	long off_x = y2 - y1;
	long off_y = (x2 - x1) * scaleLongUp;
	long bearing =	9000 + atan2(-off_y, off_x) * 5729.57795;
	if (bearing < 0) bearing += 36000;
	
	return wrap_360(bearing);
}

static float get_bearing(Vector2f *loc1, Vector2f *loc2)
{
	return (get_bearing(loc1->x, loc1->y, loc2->x, loc2->y));
}
static float get_bearing(float x1, float y1, float x2, float y2)
{
	float off_x = y2 - y1;
	float off_y = (x2 - x1) * scaleLongUp;
	float bearing =	9000 + atan2(-off_y, off_x) * 5729.57795;
	if (bearing < 0) bearing += 36000;
	
	return (bearing);
}


static float get_radians(struct Location *loc1, struct Location *loc2, struct Location *loc3)
{
	float deg_angle;
	long bearing1 = get_bearing(loc2,loc1)/100;
	long bearing2 = get_bearing(loc2,loc3)/100;
	if (bearing1 > bearing2)
		deg_angle = bearing1 - bearing2;
	else
		deg_angle = bearing2 - bearing1;

	if (deg_angle > 180)
		deg_angle = 360-deg_angle;
	
	return ToRad(deg_angle);
}
