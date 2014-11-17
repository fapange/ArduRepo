// MESSAGE CDNR_CONTROLLER PACKING
// Generated from parse_APM_XML.m on 19-Aug-2013 09:01:23

#define MAVLINK_MSG_ID_CDNR_CONTROLLER 113 

typedef struct __mavlink_cdnr_controller_t 
{ 
  int8_t h_flag;  ///< enables/disables new heading command
  int8_t s_flag;  ///< enables/disables new airspeed command
  int8_t a_flag;  ///< enables/disables new altitude command
  int8_t t_flag;  ///< enables/disables max time command
  int16_t new_heading;  ///< value for new heading
  int16_t new_airspeed;  ///< value for new airspeed
  int16_t new_altitude;  ///< value for new altitude
  int16_t max_time;  ///< maximum time to hold this command
} mavlink_cdnr_controller_t;

#define MAVLINK_MSG_ID_CDNR_CONTROLLER_LEN 12
#define MAVLINK_MSG_ID_113_LEN 12

#define MAVLINK_MESSAGE_INFO_CDNR_CONTROLLER \
{ \
  "CDNR_CONTROLLER", \
  8, \
  { \
    { "h_flag", NULL, MAVLINK_TYPE_INT8_T, 0, 0, offsetof(mavlink_cdnr_controller_t, h_flag) }, \
    { "s_flag", NULL, MAVLINK_TYPE_INT8_T, 0, 1, offsetof(mavlink_cdnr_controller_t, s_flag) }, \
    { "a_flag", NULL, MAVLINK_TYPE_INT8_T, 0, 2, offsetof(mavlink_cdnr_controller_t, a_flag) }, \
    { "t_flag", NULL, MAVLINK_TYPE_INT8_T, 0, 3, offsetof(mavlink_cdnr_controller_t, t_flag) }, \
    { "new_heading", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_cdnr_controller_t, new_heading) }, \
    { "new_airspeed", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_cdnr_controller_t, new_airspeed) }, \
    { "new_altitude", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_cdnr_controller_t, new_altitude) }, \
    { "max_time", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_cdnr_controller_t, max_time) }, \
  } \
}

/**
 * @brief Pack a cdnr_controller message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param h_flag enables/disables new heading command
 * @param s_flag enables/disables new airspeed command
 * @param a_flag enables/disables new altitude command
 * @param t_flag enables/disables max time command
 * @param new_heading value for new heading
 * @param new_airspeed value for new airspeed
 * @param new_altitude value for new altitude
 * @param max_time maximum time to hold this command
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_cdnr_controller_pack(
	 uint8_t system_id
	,uint8_t component_id
	,mavlink_message_t* msg
	,int8_t h_flag
	,int8_t s_flag
	,int8_t a_flag
	,int8_t t_flag
	,int16_t new_heading
	,int16_t new_airspeed
	,int16_t new_altitude
	,int16_t max_time
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CDNR_CONTROLLER_LEN];
	_mav_put_int8_t(buf, 0, h_flag);
	_mav_put_int8_t(buf, 1, s_flag);
	_mav_put_int8_t(buf, 2, a_flag);
	_mav_put_int8_t(buf, 3, t_flag);
	_mav_put_int16_t(buf, 4, new_heading);
	_mav_put_int16_t(buf, 6, new_airspeed);
	_mav_put_int16_t(buf, 8, new_altitude);
	_mav_put_int16_t(buf, 10, max_time);

	memcpy(_MAV_PAYLOAD(msg), buf, MAVLINK_MSG_ID_CDNR_CONTROLLER_LEN);
#else
	mavlink_cdnr_controller_t packet;
	packet.h_flag = h_flag;
	packet.s_flag = s_flag;
	packet.a_flag = a_flag;
	packet.t_flag = t_flag;
	packet.new_heading = new_heading;
	packet.new_airspeed = new_airspeed;
	packet.new_altitude = new_altitude;
	packet.max_time = max_time;

	memcpy(_MAV_PAYLOAD(msg), &packet, MAVLINK_MSG_ID_CDNR_CONTROLLER_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_CDNR_CONTROLLER;
	return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CDNR_CONTROLLER_LEN);
}

/**
 * @brief Pack a cdnr_controller message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 *
 * @param h_flag enables/disables new heading command
 * @param s_flag enables/disables new airspeed command
 * @param a_flag enables/disables new altitude command
 * @param t_flag enables/disables max time command
 * @param new_heading value for new heading
 * @param new_airspeed value for new airspeed
 * @param new_altitude value for new altitude
 * @param max_time maximum time to hold this command
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_cdnr_controller_pack_chan(
	 uint8_t system_id
	,uint8_t component_id
	,uint8_t chan
	,mavlink_message_t* msg
	,int8_t h_flag
	,int8_t s_flag
	,int8_t a_flag
	,int8_t t_flag
	,int16_t new_heading
	,int16_t new_airspeed
	,int16_t new_altitude
	,int16_t max_time
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CDNR_CONTROLLER_LEN];
	_mav_put_int8_t(buf, 0, h_flag);
	_mav_put_int8_t(buf, 1, s_flag);
	_mav_put_int8_t(buf, 2, a_flag);
	_mav_put_int8_t(buf, 3, t_flag);
	_mav_put_int16_t(buf, 4, new_heading);
	_mav_put_int16_t(buf, 6, new_airspeed);
	_mav_put_int16_t(buf, 8, new_altitude);
	_mav_put_int16_t(buf, 10, max_time);

	memcpy(_MAV_PAYLOAD(msg), buf, MAVLINK_MSG_ID_CDNR_CONTROLLER_LEN);
#else
	mavlink_cdnr_controller_t packet;
	packet.h_flag = h_flag;
	packet.s_flag = s_flag;
	packet.a_flag = a_flag;
	packet.t_flag = t_flag;
	packet.new_heading = new_heading;
	packet.new_airspeed = new_airspeed;
	packet.new_altitude = new_altitude;
	packet.max_time = max_time;

	memcpy(_MAV_PAYLOAD(msg), &packet, MAVLINK_MSG_ID_CDNR_CONTROLLER_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_CDNR_CONTROLLER;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CDNR_CONTROLLER_LEN);
}

/**
 * @brief Encode a cdnr_controller struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param vfr_hud C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_cdnr_controller_encode(
	 uint8_t system_id
	,uint8_t component_id
	,mavlink_message_t* msg
	,const mavlink_cdnr_controller_t* cdnr_controller
	)
{
	return mavlink_msg_cdnr_controller_pack(
	 system_id
	,component_id
	,msg
	,cdnr_controller->h_flag
	,cdnr_controller->s_flag
	,cdnr_controller->a_flag
	,cdnr_controller->t_flag
	,cdnr_controller->new_heading
	,cdnr_controller->new_airspeed
	,cdnr_controller->new_altitude
	,cdnr_controller->max_time
	);
}

/**
 * @brief Send a cdnr_controller message
 * @param chan The MAVLink channel to send this message
 *
 * @param h_flag enables/disables new heading command
 * @param s_flag enables/disables new airspeed command
 * @param a_flag enables/disables new altitude command
 * @param t_flag enables/disables max time command
 * @param new_heading value for new heading
 * @param new_airspeed value for new airspeed
 * @param new_altitude value for new altitude
 * @param max_time maximum time to hold this command
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_cdnr_controller_send(
	 mavlink_channel_t chan
	,int8_t h_flag
	,int8_t s_flag
	,int8_t a_flag
	,int8_t t_flag
	,int16_t new_heading
	,int16_t new_airspeed
	,int16_t new_altitude
	,int16_t max_time
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CDNR_CONTROLLER_LEN];
	_mav_put_int8_t(buf, 0, h_flag);
	_mav_put_int8_t(buf, 1, s_flag);
	_mav_put_int8_t(buf, 2, a_flag);
	_mav_put_int8_t(buf, 3, t_flag);
	_mav_put_int16_t(buf, 4, new_heading);
	_mav_put_int16_t(buf, 6, new_airspeed);
	_mav_put_int16_t(buf, 8, new_altitude);
	_mav_put_int16_t(buf, 10, max_time);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CDNR_CONTROLLER, buf, MAVLINK_MSG_ID_CDNR_CONTROLLER_LEN);
#else
	mavlink_cdnr_controller_t packet;
	packet.h_flag = h_flag;
	packet.s_flag = s_flag;
	packet.a_flag = a_flag;
	packet.t_flag = t_flag;
	packet.new_heading = new_heading;
	packet.new_airspeed = new_airspeed;
	packet.new_altitude = new_altitude;
	packet.max_time = max_time;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CDNR_CONTROLLER, (const char*)&packet, MAVLINK_MSG_ID_CDNR_CONTROLLER_LEN);
#endif
}

#endif // MAVLINK_USE_CONVENIENCE_FUNCTIONS

// MESSAGE CDNR_CONTROLLER UNPACKING

/**
 * @brief Get field h_flag from cdnr_controller message
 *
 * @return enables/disables new heading command
 */
static inline int8_t mavlink_msg_cdnr_controller_get_h_flag(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int8_t(msg,0);
}

/**
 * @brief Get field s_flag from cdnr_controller message
 *
 * @return enables/disables new airspeed command
 */
static inline int8_t mavlink_msg_cdnr_controller_get_s_flag(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int8_t(msg,1);
}

/**
 * @brief Get field a_flag from cdnr_controller message
 *
 * @return enables/disables new altitude command
 */
static inline int8_t mavlink_msg_cdnr_controller_get_a_flag(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int8_t(msg,2);
}

/**
 * @brief Get field t_flag from cdnr_controller message
 *
 * @return enables/disables max time command
 */
static inline int8_t mavlink_msg_cdnr_controller_get_t_flag(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int8_t(msg,3);
}

/**
 * @brief Get field new_heading from cdnr_controller message
 *
 * @return value for new heading
 */
static inline int16_t mavlink_msg_cdnr_controller_get_new_heading(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,4);
}

/**
 * @brief Get field new_airspeed from cdnr_controller message
 *
 * @return value for new airspeed
 */
static inline int16_t mavlink_msg_cdnr_controller_get_new_airspeed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,6);
}

/**
 * @brief Get field new_altitude from cdnr_controller message
 *
 * @return value for new altitude
 */
static inline int16_t mavlink_msg_cdnr_controller_get_new_altitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,8);
}

/**
 * @brief Get field max_time from cdnr_controller message
 *
 * @return maximum time to hold this command
 */
static inline int16_t mavlink_msg_cdnr_controller_get_max_time(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,10);
}

/**
 * @brief Decode a cdnr_controller message into a struct
 *
 * @param msg The message to decode
 * @param cdnr_controller C-struct to decode the message contents into
 */
static inline void mavlink_msg_cdnr_controller_decode(const mavlink_message_t* msg, mavlink_cdnr_controller_t* cdnr_controller)
{
#if MAVLINK_NEED_BYTE_SWAP
	cdnr_controller->h_flag = mavlink_msg_cdnr_controller_get_h_flag(msg);
	cdnr_controller->s_flag = mavlink_msg_cdnr_controller_get_s_flag(msg);
	cdnr_controller->a_flag = mavlink_msg_cdnr_controller_get_a_flag(msg);
	cdnr_controller->t_flag = mavlink_msg_cdnr_controller_get_t_flag(msg);
	cdnr_controller->new_heading = mavlink_msg_cdnr_controller_get_new_heading(msg);
	cdnr_controller->new_airspeed = mavlink_msg_cdnr_controller_get_new_airspeed(msg);
	cdnr_controller->new_altitude = mavlink_msg_cdnr_controller_get_new_altitude(msg);
	cdnr_controller->max_time = mavlink_msg_cdnr_controller_get_max_time(msg);
#else
	memcpy(cdnr_controller, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_CDNR_CONTROLLER_LEN);
#endif
}
