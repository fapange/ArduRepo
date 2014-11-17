// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
  geo-fencing support
  Andrew Tridgell, December 2011
 */

/*
  The state of geo-fencing. This structure is dynamically allocated
  the first time it is used. This means we only pay for the pointer
  and not the structure on systems where geo-fencing is not being
  used.

  We store a copy of the boundary in memory as we need to access it
  very quickly at runtime
 */
enum fence_state
{
	INSIDE = 0,
	OUTSIDE,
	EXITING,
	ENTERING
};

static struct geofence_state {
    uint8_t num_points;
    bool boundary_uptodate;
    bool fence_triggered;
    uint16_t breach_count;
    uint8_t breach_type;
    uint32_t breach_time;
    byte old_switch_position;
	byte prev_mode;
	fence_state state;
	fence_state pre_state;
    /* point 0 is the return point */
    Vector2l boundary[MAX_FENCEPOINTS];
} *geofence_state;


/*
	returns status of geofence
*/
/*bool geofenceTriggered()
{
	if (geofence_state == NULL)
	{
		Serial.printf_P (PSTR("GeoFence is NULL"));
		Serial3.printf_P(PSTR("GeoFence is NULL"));
		return false;
	}
	Serial.printf_P (PSTR("GeoFence Status %d"),(int)geofence_state->fence_triggered);
	Serial3.printf_P(PSTR("GeoFence Status %d"),(int)geofence_state->fence_triggered);
	return geofence_state->fence_triggered;
}*/

/*
  fence boundaries fetch/store
 */
static Vector2l get_fence_point_with_index(unsigned i)
{
    uint32_t mem;
    Vector2l ret;

    if (i > (unsigned)g.fence_total) 
	{
        return Vector2l(0,0);
    }

    // read fence point
    mem = FENCE_START_BYTE + (i * FENCE_WP_SIZE);
    ret.x = eeprom_read_dword((uint32_t *)mem);
    mem += sizeof(uint32_t);
    ret.y = eeprom_read_dword((uint32_t *)mem);

    return ret;
}

// save a fence point
static void set_fence_point_with_index(Vector2l &point, unsigned i)
{
    uint32_t mem;

    if (i >= (unsigned)g.fence_total.get()) 
	{
        // not allowed
        return;
    }

    mem = FENCE_START_BYTE + (i * FENCE_WP_SIZE);

    eeprom_write_dword((uint32_t *)mem, point.x);
    mem += sizeof(uint32_t);
    eeprom_write_dword((uint32_t *)mem, point.y);

    if (geofence_state != NULL) 
	{
        geofence_state->boundary_uptodate = false;
    }
}

/*
  allocate and fill the geofence state structure
 */
static void geofence_load(void)
{
    uint8_t i;

    if (geofence_state == NULL) 
	{
        if (memcheck_available_memory() < 512 + sizeof(struct geofence_state)) 
		{
            // too risky to enable as we could run out of stack
            goto failed;
        }
        geofence_state = (struct geofence_state *)calloc(1, sizeof(struct geofence_state));
        if (geofence_state == NULL) 
		{
            // not much we can do here except disable it
            goto failed;
        }
    }

	geofence_state->state = INSIDE;
	geofence_state->pre_state = INSIDE;

	if (g.fence_total > MAX_FENCEPOINTS)
	{
		goto failed;
	}

    for (i=0; i<g.fence_total; i++) 
	{
        geofence_state->boundary[i] = get_fence_point_with_index(i);
    }
    geofence_state->num_points = i;

    if (!Polygon_complete(&geofence_state->boundary[1], geofence_state->num_points-1)) 
	{
        // first point and last point must be the same
        goto failed;
    }
    if (Polygon_outside(geofence_state->boundary[0], &geofence_state->boundary[1], geofence_state->num_points-1)) 
	{
        // return point needs to be inside the fence
        goto failed;
    }

    geofence_state->boundary_uptodate = true;
    //geofence_state->fence_triggered = false;

    gcs_send_text_P(SEVERITY_LOW,PSTR("geo-fence loaded"));
    gcs_send_message(MSG_FENCE_STATUS);
    return;

failed:
    g.fence_action.set(FENCE_ACTION_NONE);
    gcs_send_text_P(SEVERITY_HIGH,PSTR("geo-fence setup error"));
	return;
}

/*
  return true if geo-fencing is enabled
 */
static bool geofence_enabled(void)
{
    if (g.fence_action == FENCE_ACTION_NONE ||
        g.fence_channel == 0 ||
        APM_RC.InputCh(g.fence_channel-1) < FENCE_ENABLE_PWM) 
	{
        // geo-fencing is disabled
        //if (geofence_state != NULL) 
		//{
        //    // re-arm for when the channel trigger is switched on
        //    geofence_state->fence_triggered = false;
        //}
        return false;
    }

    if (!g_gps->fix) 
	{
        // we can't do much without a GPS fix
        return false;
    }

    return true;
}


/*
  return true if we have breached the geo-fence minimum altiude
 */
static bool geofence_check_minalt(void)
{
    if (g.fence_maxalt <= g.fence_minalt) 
	{
        return false;
    }
    if (g.fence_minalt == 0) 
	{
        return false;
    }
    //return (current_loc.alt < (g.fence_minalt*100) + home.alt);
	//breach_type = FENCE_BREACH_MINALT;
    return (current_loc.alt < (g.fence_minalt*100));
}

/*
  return true if we have breached the geo-fence maximum altiude
 */
static bool geofence_check_maxalt(void)
{
    if (g.fence_maxalt <= g.fence_minalt) 
	{
        return false;
    }
    if (g.fence_maxalt == 0) 
	{
        return false;
    }
    //return (current_loc.alt > (g.fence_maxalt*100) + home.alt);
    //breach_type = FENCE_BREACH_MAXALT;
    return (current_loc.alt > (g.fence_maxalt*100));
}


static bool geofence_check_boundaries(void)
{
    bool outside = false;
    Vector2l location;
    location.x = current_loc.lat;
    location.y = current_loc.lng;

    /* allocate the geo-fence state if need be */
    if (geofence_state == NULL || !geofence_state->boundary_uptodate) 
	{
        geofence_load();
    }

    if (geofence_state == NULL) return outside;

	geofence_state->fence_triggered = Polygon_outside(location, &geofence_state->boundary[1], geofence_state->num_points-1);
	if (geofence_state->fence_triggered)
	{
		// Breached fence
		geofence_state->breach_type = FENCE_BREACH_BOUNDARY;
		return geofence_state->fence_triggered;
	}
	
	geofence_state->fence_triggered = geofence_check_minalt();
	if (geofence_state->fence_triggered)
	{
		// Breached minimum altitude
		geofence_state->breach_type = FENCE_BREACH_MINALT;
		return geofence_state->fence_triggered;
	}

	geofence_state->fence_triggered = geofence_check_maxalt();
	if (geofence_state->fence_triggered)
	{
		// Breached maximum altitude
		geofence_state->breach_type = FENCE_BREACH_MAXALT;
		return geofence_state->fence_triggered;
	}
	
	// Inside Geofence boundaries
    geofence_state->breach_type = FENCE_BREACH_NONE;
	return geofence_state->fence_triggered;
}

static void geofence_state_machine(void)
{
	bool breach = geofence_check_boundaries();
	switch (geofence_state->state)
	{
		case INSIDE:
			if (breach)
			{
				geofence_state->pre_state = geofence_state->state;
				geofence_state->state = EXITING;
			}
			break;
		case EXITING:
			geofence_state->prev_mode = control_mode;
			geofence_state->breach_count++;
			geofence_state->breach_time = millis();
			geofence_state->pre_state = geofence_state->state;
			geofence_state->state = OUTSIDE;
			if (geofence_enabled())
			{
				guided_WP.id = 0;
				guided_WP.p1  = 0;
				guided_WP.options = 0;
				guided_WP.lat = geofence_state->boundary[0].x;
				guided_WP.lng = geofence_state->boundary[0].y;
				set_mode(GUIDED);
			}
			break;
		case OUTSIDE:
			if (!breach)
			{
				geofence_state->pre_state = geofence_state->state;
				geofence_state->state = ENTERING;
			}
			break;
		case ENTERING:
			set_mode(geofence_state->prev_mode);
			geofence_state->pre_state = geofence_state->state;
			geofence_state->state = INSIDE;
			break;
		default:
			gcs_send_message(MSG_FENCE_STATUS);
			geofence_state->state = INSIDE;
			gcs_send_text_P(SEVERITY_HIGH,PSTR("UNKNOWN GEOFENCE STATE"));
			break;
	}
}

/*
  check if we have breached the geo-fence
 */
/*
static void geofence_check(bool altitude_check_only)
{
    bool outside = false;
    uint8_t breach_type = FENCE_BREACH_NONE;

    // allocate the geo-fence state if need be //
    if (geofence_state == NULL || !geofence_state->boundary_uptodate) 
	{
        geofence_load();
    }

    if (geofence_check_minalt()) 
	{
        outside = true;
        breach_type = FENCE_BREACH_MINALT;
    } 
	else if (geofence_check_maxalt()) 
	{
        outside = true;
        breach_type = FENCE_BREACH_MAXALT;
    } 
	else if (!altitude_check_only) 
	{
        Vector2l location;
        location.x = current_loc.lat;
        location.y = current_loc.lng;
		if (geofence_state != NULL)
		{
			outside = Polygon_outside(location, &geofence_state->boundary[1], geofence_state->num_points-1);
			if (outside) 
			{
				breach_type = FENCE_BREACH_BOUNDARY;
			}
		}
    }

    if (geofence_state->fence_triggered != outside)
	{
		if (outside) 
		{
			// we are outside, and have not previously triggered.
			geofence_state->breach_type = breach_type;
			geofence_state->breach_count++;
			geofence_state->breach_time = millis();

			if (geofence_enabled())
			{
				// see what action the user wants
				switch (g.fence_action) 
				{
					case FENCE_ACTION_GUIDED:
						guided_WP.id = 0;
						guided_WP.p1  = 0;
						guided_WP.options = 0;
						guided_WP.lat = geofence_state->boundary[0].x;
						guided_WP.lng = geofence_state->boundary[0].y;

						geofence_state->old_switch_position = oldSwitchPosition;
						geofence_state->prev_mode = control_mode;
						if (control_mode == MANUAL && g.auto_trim) 
						{
							// make sure we don't auto trim the surfaces on this change
							control_mode = STABILIZE;
						}
						set_mode(GUIDED);
						break;
				}
			}

			//gcs_send_text_P(SEVERITY_LOW,PSTR("geo-fence Triggered"));
			#if FENCE_TRIGGERED_PIN > 0
			digitalWrite(FENCE_TRIGGERED_PIN, HIGH);
			#endif
			//gcs_send_message(MSG_FENCE_STATUS);
		}
		else
		{
			// we have moved back inside the fence
			geofence_state->breach_type = breach_type;

			// restore AP to previous mode
			if (geofence_enabled())
			{
				set_mode(geofence_state->prev_mode);
			}

			//gcs_send_text_P(SEVERITY_LOW,PSTR("geo-fence OK"));
			#if FENCE_TRIGGERED_PIN > 0
			digitalWrite(FENCE_TRIGGERED_PIN, LOW);
			#endif
			//gcs_send_message(MSG_FENCE_STATUS);
		}
	}
	geofence_state->fence_triggered = outside;
}
*/

/*
  return true if geofencing allows stick mixing. When we have
  triggered failsafe and are in GUIDED mode then stick mixing is
  disabled. Otherwise the aircraft may not be able to recover from
  a breach of the geo-fence
 */
static bool geofence_stickmixing(void) 
{
    if (geofence_enabled() &&
        geofence_state != NULL &&
        geofence_state->fence_triggered &&
        control_mode == GUIDED) 
	{
        // don't mix in user input
        return false;
    }
    // normal mixing rules
    return true;
}

/*
static void oldgeofence_send_status(mavlink_channel_t chan)
{
    if (geofence_enabled() && geofence_state != NULL) 
	{
        mavlink_msg_wID_fence_status_send(chan,
									  (uint8_t)MAV_SYSTEM_ID,
									  (uint8_t)MAV_GCS_ID,
                                      (int8_t)geofence_state->fence_triggered,
                                      geofence_state->breach_count,
                                      geofence_state->breach_type,
                                      geofence_state->breach_time);
    }
}
*/
static void geofence_send_status(mavlink_channel_t chan)
{
    if (geofence_state != NULL) 
	{
        mavlink_msg_wID_fence_status_send(chan,
									  (uint8_t)MAV_SYSTEM_ID,
									  (uint8_t)MAV_GCS_ID,
                                      (int8_t)geofence_state->fence_triggered,
                                      geofence_state->breach_count,
                                      geofence_state->breach_type,
                                      geofence_state->breach_time);
    }
}
