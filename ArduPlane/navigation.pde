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

static long sign_180(long error)
{
	if (error < 0) return  -1;
	return 1;
}
static long comp_360(long error)
{
	if (error < 0) return  (error+36000);
	return (error-36000);
}

	static void update_loiter()
	{
		float power;
		if(wp_distance <= g.loiter_radius)
		{
			if (loiter_time == 0)
				loiter_time = millis();			// keep start time for loiter updating till we get within LOITER_RANGE of orbit
				
			power = float(wp_distance) / float(g.loiter_radius);
			power = constrain(power, 0.5, 1);
			last_nav_heading = nav_bearing;
			nav_bearing += (int)(g.crosstrack_gain * (2.0 + power));
			nav_bearing = wrap_360(nav_bearing);
			nav_bearing += nav_geo_fence();
			nav_bearing = wrap_360(nav_bearing);
		}
		else if(wp_distance < 2*g.loiter_radius)
		{
			power = -((float)(wp_distance - 2*g.loiter_radius) / g.loiter_radius);
			power = constrain(power, 0.5, 1);			//power = constrain(power, 0, 1);
			last_nav_heading = nav_bearing;
			nav_bearing -= (int)(power * g.crosstrack_gain);
			nav_bearing = wrap_360(nav_bearing);
			nav_bearing += nav_geo_fence();
			nav_bearing = wrap_360(nav_bearing);
		}
		else
		{
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

	// if wind_vel is less than 0.25 m/s set the wind_dir to zero
	if (wind_vel < 25.0f) wind_dir = 0.0f;
}

#define headSLEW  18000

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
	nav_bearing = last_nav_heading + constrain(wrap_180(nav_bearing - last_nav_heading), -headSLEW, headSLEW);
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

//#define lowD	50.0f
//#define highD	200.0f
#define lowD	30.0f
#define highD	150.0f
#define refSpeed 3000.0f
//#define minhLim ((highD-lowD)/2.0f + lowD)

//---------------------------------------------------------------------
// This function calculates the effect of a geoContainment area  to the 
// navHeading of the airplane by superposition of the individual
// effects of all boundary segments
//---------------------------------------------------------------------
static long nav_geo_fence()
{
    uint8_t i,j;
	int segn=0;
	long navHeading;
	static long delHeading = 0;
	float distm;
	float d_weight;
	float v_weight;
	float t_weight;
	float s_weight;
	float v_ratio;
	float hLim;
	float lLim;
	Vector2f v;
	Vector2f w;
	Vector2f p;
	Vector2f wp;
	Vector2f geoPoint;
	Vector2f yp;
	Vector2f geoNav = Vector2f(0, 0);
	Vector2f nav;
	Vector2f tgt;
	Vector2f gnd;

    skip_wpt = false;
	
	if (geofence_state == NULL) 
		return (0L);
    
	// Current Location
	p = Vector2f((float)current_loc.lat,(float)current_loc.lng)/1e7f;
	
	// Waypoint Location vector
	wp = Vector2f((float)next_WP.lat,(float)next_WP.lng)/1e7f;
	// Nav Heading vector
	nav  = Vector2f((float)(cos(ToRad((float)nav_bearing/100.0f))), (float)(sin(ToRad((float)nav_bearing/100.0f))));
	// Nav Heading vector
	tgt  = Vector2f((float)(cos(ToRad((float)target_bearing/100.0f))), (float)(sin(ToRad((float)target_bearing/100.0f))));
	// Ground track vector
	gnd  = Vector2f((float)(cos(ToRad((float)g_gps->ground_course/100.0f))), (float)(sin(ToRad((float)g_gps->ground_course/100.0f))));

	// gain based on the ground speed magnitude
	//v_ratio = ((float)g_gps->ground_speed/(float)g.flybywire_airspeed_max)/100.0f;
	v_ratio = (float)g_gps->ground_speed/refSpeed;

	//Serial.printf_P (PSTR("v_r=%.2f "),v_ratio);

	// Cycle through fence segments and find their contribution towards geoContainment
	//Serial.printf_P (PSTR("[ "));
	for (i=1,j=i+1; i<geofence_state->num_points-1; j++, i++) 
	{
		// the two endpoints of the segment
		v = Vector2f((float)geofence_state->boundary[i].x,(float)geofence_state->boundary[i].y)/1e7f;
		w = Vector2f((float)geofence_state->boundary[j].x,(float)geofence_state->boundary[j].y)/1e7f;
		// defines point in segment closest to airplane
		geoPoint = find_closest_point(v, w, p);
		distm = get_distance(&p,&geoPoint);
		// consider the segment only if point lies within the segment
		if (distm > 0)
		{
			// point lies within segment
			segn = i;
			if (geofence_state->state == OUTSIDE)	yp = (geoPoint-p).normalized();	// Normal vector from airplane to fence 
			else									yp = (p-geoPoint).normalized();	// Normal vector from fence to airplane
			
			// gain based on the direction of ground track relative to fence segment
			// +v_ratio if moving toward fence segment,
			//    0     if moving parallel to fence segment,
			// -v_ratio if moving away from fence segment
			v_weight = -dot(gnd, yp);
			v_weight = (v_weight+0.5f)/1.5f;
			if (v_weight < 0.0f) v_weight = 0.0f;

			// adjustment to distance thresholds based on the velocity gain
			// the faster the approach of the airplane, and the more perpendicular
			// the approach to the segment, the farthest away that the distance 
			// gain starts acting, 
			float sfactor = (1.0f + 0.5f*v_weight*v_ratio);
			hLim = highD * sfactor;
			lLim = lowD  * sfactor;
			//Serial.printf_P (PSTR("hL=%.2f "),hLim);
			//if (hLim < minhLim) hLim = minhLim;

			if		(distm < lLim)	d_weight = 1.0f;
			else if (distm > hLim)	d_weight = 0.0f;
			else					d_weight = (hLim-distm)/(hLim-lLim);
			
			//t_weight = 1.0f+0.25f*pow(dot(tgt, yp),2);
			t_weight = 1.0f + pow(dot(tgt, yp),2);

			//s_weight = d_weight * (v_weight + t_weight)/2.0f;
			s_weight = d_weight * (v_weight + t_weight)/3.0f;
			//s_weight = (d_weight + v_weight);
			//s_weight = pow(Vector2f(d_weight,d_weight*v_weight).length_squared(),2);

			//if (segn == 1)
			{
				//Serial.printf_P (PSTR("%03.0f "),distm);
				//Serial.printf_P (PSTR("%d"),segn);
				//Serial.printf_P (PSTR("%.2f "),d_weight);
				//Serial.printf_P (PSTR("%.2f "),v_ratio);
				//Serial.printf_P (PSTR("%.2f "),v_weight);
				//Serial.printf_P (PSTR("%.2f "),t_weight);
				//Serial.printf_P (PSTR("%.2f "),s_weight);
			}

			geoNav = (yp * s_weight + geoNav);
		}
	}
	//Serial.printf_P (PSTR("] "));

	geoWeight = geoNav.length();
	if (geoWeight > 1.0f)
	{
		//Serial.printf_P (PSTR(">"));
		geoWeight = 1.0f;
	}
	else if (geoWeight == 0.0f) 
	{
		//Serial.printf_P (PSTR("0"));
		geoNav = nav;
	}
	else
	{
		//Serial.printf_P (PSTR("*"));
		geoNav.normalize();
		geoNav = geoNav * geoWeight + nav * (1.0f - geoWeight);
	}
	//Serial.printf_P (PSTR(" %.2f"),geoWeight);
	//Serial.printf_P (PSTR("\n"));

	navHeading = (long)get_bearing(&p, &(p+nav));
	geoHeading = (long)get_bearing(&p, &(p+geoNav));

	delHeading = wrap_180(geoHeading - navHeading);
	//Serial.printf_P (PSTR(" Geo=%d "),delHeading/100);
	
	if (geofence_enabled())
	{
		geoHeading = wrap_360(delHeading);
		return (delHeading);
	}
	else
	{
		geoWeight = 0.0f;
		geoHeading = 0L;
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
