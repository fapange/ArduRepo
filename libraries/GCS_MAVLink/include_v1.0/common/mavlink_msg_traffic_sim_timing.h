// MESSAGE TRAFFIC_SIM_TIMING PACKING
// Generated from parse_APM_XML.m on 19-Aug-2013 09:01:20

#define MAVLINK_MSG_ID_TRAFFIC_SIM_TIMING 116 

typedef struct __mavlink_traffic_sim_timing_t 
{ 
  float sim_time;  ///< Traffic simulation absolute time
  float time_to_wpt;  ///< Absolute time to reach the next waypoint
} mavlink_traffic_sim_timing_t;

#define MAVLINK_MSG_ID_TRAFFIC_SIM_TIMING_LEN 8
#define MAVLINK_MSG_ID_116_LEN 8

#define MAVLINK_MESSAGE_INFO_TRAFFIC_SIM_TIMING \
{ \
  "TRAFFIC_SIM_TIMING", \
  2, \
  { \
    { "sim_time", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_traffic_sim_timing_t, sim_time) }, \
    { "time_to_wpt", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_traffic_sim_timing_t, time_to_wpt) }, \
  } \
}

/**
 * @brief Pack a traffic_sim_timing message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param sim_time Traffic simulation absolute time
 * @param time_to_wpt Absolute time to reach the next waypoint
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_traffic_sim_timing_pack(
	 uint8_t system_id
	,uint8_t component_id
	,mavlink_message_t* msg
	,float sim_time
	,float time_to_wpt
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TRAFFIC_SIM_TIMING_LEN];
	_mav_put_float(buf, 0, sim_time);
	_mav_put_float(buf, 4, time_to_wpt);

	memcpy(_MAV_PAYLOAD(msg), buf, MAVLINK_MSG_ID_TRAFFIC_SIM_TIMING_LEN);
#else
	mavlink_traffic_sim_timing_t packet;
	packet.sim_time = sim_time;
	packet.time_to_wpt = time_to_wpt;

	memcpy(_MAV_PAYLOAD(msg), &packet, MAVLINK_MSG_ID_TRAFFIC_SIM_TIMING_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_TRAFFIC_SIM_TIMING;
	return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TRAFFIC_SIM_TIMING_LEN);
}

/**
 * @brief Pack a traffic_sim_timing message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 *
 * @param sim_time Traffic simulation absolute time
 * @param time_to_wpt Absolute time to reach the next waypoint
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_traffic_sim_timing_pack_chan(
	 uint8_t system_id
	,uint8_t component_id
	,uint8_t chan
	,mavlink_message_t* msg
	,float sim_time
	,float time_to_wpt
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TRAFFIC_SIM_TIMING_LEN];
	_mav_put_float(buf, 0, sim_time);
	_mav_put_float(buf, 4, time_to_wpt);

	memcpy(_MAV_PAYLOAD(msg), buf, MAVLINK_MSG_ID_TRAFFIC_SIM_TIMING_LEN);
#else
	mavlink_traffic_sim_timing_t packet;
	packet.sim_time = sim_time;
	packet.time_to_wpt = time_to_wpt;

	memcpy(_MAV_PAYLOAD(msg), &packet, MAVLINK_MSG_ID_TRAFFIC_SIM_TIMING_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_TRAFFIC_SIM_TIMING;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TRAFFIC_SIM_TIMING_LEN);
}

/**
 * @brief Encode a traffic_sim_timing struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param vfr_hud C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_traffic_sim_timing_encode(
	 uint8_t system_id
	,uint8_t component_id
	,mavlink_message_t* msg
	,const mavlink_traffic_sim_timing_t* traffic_sim_timing
	)
{
	return mavlink_msg_traffic_sim_timing_pack(
	 system_id
	,component_id
	,msg
	,traffic_sim_timing->sim_time
	,traffic_sim_timing->time_to_wpt
	);
}

/**
 * @brief Send a traffic_sim_timing message
 * @param chan The MAVLink channel to send this message
 *
 * @param sim_time Traffic simulation absolute time
 * @param time_to_wpt Absolute time to reach the next waypoint
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_traffic_sim_timing_send(
	 mavlink_channel_t chan
	,float sim_time
	,float time_to_wpt
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TRAFFIC_SIM_TIMING_LEN];
	_mav_put_float(buf, 0, sim_time);
	_mav_put_float(buf, 4, time_to_wpt);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRAFFIC_SIM_TIMING, buf, MAVLINK_MSG_ID_TRAFFIC_SIM_TIMING_LEN);
#else
	mavlink_traffic_sim_timing_t packet;
	packet.sim_time = sim_time;
	packet.time_to_wpt = time_to_wpt;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRAFFIC_SIM_TIMING, (const char*)&packet, MAVLINK_MSG_ID_TRAFFIC_SIM_TIMING_LEN);
#endif
}

#endif // MAVLINK_USE_CONVENIENCE_FUNCTIONS

// MESSAGE TRAFFIC_SIM_TIMING UNPACKING

/**
 * @brief Get field sim_time from traffic_sim_timing message
 *
 * @return Traffic simulation absolute time
 */
static inline float mavlink_msg_traffic_sim_timing_get_sim_time(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,0);
}

/**
 * @brief Get field time_to_wpt from traffic_sim_timing message
 *
 * @return Absolute time to reach the next waypoint
 */
static inline float mavlink_msg_traffic_sim_timing_get_time_to_wpt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,4);
}

/**
 * @brief Decode a traffic_sim_timing message into a struct
 *
 * @param msg The message to decode
 * @param traffic_sim_timing C-struct to decode the message contents into
 */
static inline void mavlink_msg_traffic_sim_timing_decode(const mavlink_message_t* msg, mavlink_traffic_sim_timing_t* traffic_sim_timing)
{
#if MAVLINK_NEED_BYTE_SWAP
	traffic_sim_timing->sim_time = mavlink_msg_traffic_sim_timing_get_sim_time(msg);
	traffic_sim_timing->time_to_wpt = mavlink_msg_traffic_sim_timing_get_time_to_wpt(msg);
#else
	memcpy(traffic_sim_timing, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_TRAFFIC_SIM_TIMING_LEN);
#endif
}
