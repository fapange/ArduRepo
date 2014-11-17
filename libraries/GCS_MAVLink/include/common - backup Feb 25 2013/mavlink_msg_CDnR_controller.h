//------------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------------//

// SLV_GCS_MAVLINK_NEWMESSAGE
#include "../../ArduPlane/mydefines.h"

#if SLV_GCS_MAVLINK_NEWMESSAGE == ENABLED
// MESSAGE CDnR_CONTROLLER PACKING

#define MAVLINK_MSG_ID_CDnR_CONTROLLER 113

typedef struct __mavlink_CDnR_controller_t
{
 uint8_t h_flag;       ///< New Heading Alarm Flag
 uint8_t s_flag;       ///< New Airspeed Alarm Flag
 uint8_t a_flag;       ///< New Altitude Alarm Flag
 uint8_t t_flag;       ///< Max Time Alarm Flag
 int16_t new_heading;  ///< Desired heading to avoid collision
 int16_t new_airspeed; ///< Desired airspeed to avoid collision
 int16_t new_altitude; ///< Desired altitude to avoid collision
 int16_t max_time;     ///< Max number of seconds to maintain new course before resuming waypoint
} mavlink_CDnR_controller_t;

#define MAVLINK_MSG_ID_CDnR_CONTROLLER_LEN 12
#define MAVLINK_MSG_ID_113_LEN MAVLINK_MSG_ID_CDnR_CONTROLLER_LEN


#define MAVLINK_MESSAGE_INFO_CDnR_CONTROLLER { \
	"CDnR_CONTROLLER", \
	8, \
	{  { "h_flag"      , NULL, MAVLINK_TYPE_INT8_T  , 0,  0, offsetof(mavlink_CDnR_controller_t, h_flag)      }, \
       { "s_flag"      , NULL, MAVLINK_TYPE_INT8_T  , 0,  1, offsetof(mavlink_CDnR_controller_t, s_flag)      }, \
       { "a_flag"      , NULL, MAVLINK_TYPE_INT8_T  , 0,  2, offsetof(mavlink_CDnR_controller_t, a_flag)      }, \
       { "t_flag"      , NULL, MAVLINK_TYPE_INT8_T  , 0,  3, offsetof(mavlink_CDnR_controller_t, t_flag)      }, \
       { "new_heading" , NULL, MAVLINK_TYPE_INT16_T , 0,  4, offsetof(mavlink_CDnR_controller_t, new_heading) }, \
       { "new_airspeed", NULL, MAVLINK_TYPE_INT16_T , 0,  6, offsetof(mavlink_CDnR_controller_t, new_airspeed)}, \
       { "new_altitude", NULL, MAVLINK_TYPE_INT16_T , 0,  8, offsetof(mavlink_CDnR_controller_t, new_altitude) }, \
       { "max_time"    , NULL, MAVLINK_TYPE_INT16_T , 0, 10, offsetof(mavlink_CDnR_controller_t, max_time)    }, \
       } \
}


/**
 * @brief Pack a CDnR_controller message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param alarm Collision Alarm Flag
 * @param new_heading Desired heading to avoid collision
 * @param new_altitude Desired altitude to avoid collision
 * @param max_time Max number of seconds to maintain new course before resuming waypoint
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_CDnR_controller_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
							   uint8_t h_flag, uint8_t s_flag, uint8_t a_flag, uint8_t t_flag, 
							   int16_t new_heading, int16_t new_airspeed, int16_t new_altitude, int16_t max_time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CDnR_CONTROLLER_LEN];
	_mav_put_uint8_t (buf,  0, h_flag);
	_mav_put_uint8_t (buf,  1, s_flag);
	_mav_put_uint8_t (buf,  2, a_flag);
	_mav_put_uint8_t (buf,  3, t_flag);
	_mav_put_int16_t (buf,  4, new_heading);
	_mav_put_int16_t (buf,  6, new_airspeed);
	_mav_put_int16_t (buf,  8, new_altitude);
	_mav_put_int16_t (buf, 10, max_time);

        memcpy(_MAV_PAYLOAD(msg), buf, MAVLINK_MSG_ID_CDnR_CONTROLLER_LEN);
#else
	mavlink_CDnR_controller_t packet;
	packet.h_flag = h_flag;
	packet.s_flag = s_flag;
	packet.a_flag = a_flag;
	packet.t_flag = t_flag;
	packet.new_heading = new_heading;
	packet.new_airspeed = new_airspeed;
	packet.new_altitude = new_altitude;
	packet.max_time = max_time;

        memcpy(_MAV_PAYLOAD(msg), &packet, MAVLINK_MSG_ID_CDnR_CONTROLLER_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_CDnR_CONTROLLER;
	return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CDnR_CONTROLLER_LEN);
}

/**
 * @brief Pack a CDnR_controller message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param alarm Collision Alarm Flag
 * @param new_heading Desired heading to avoid collision
 * @param new_altitude Desired altitude to avoid collision
 * @param max_time Max number of seconds to maintain new course before resuming waypoint
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_CDnR_controller_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
							   uint8_t h_flag, uint8_t s_flag, uint8_t a_flag, uint8_t t_flag, 
							   int16_t new_heading, int16_t new_airspeed, int16_t new_altitude, int16_t max_time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CDnR_CONTROLLER_LEN];
	_mav_put_uint8_t (buf,  0, h_flag);
	_mav_put_uint8_t (buf,  1, s_flag);
	_mav_put_uint8_t (buf,  2, a_flag);
	_mav_put_uint8_t (buf,  3, t_flag);
	_mav_put_int16_t (buf,  4, new_heading);
	_mav_put_int16_t (buf,  6, new_airspeed);
	_mav_put_int16_t (buf,  8, new_altitude);
	_mav_put_int16_t (buf, 10, max_time);

        memcpy(_MAV_PAYLOAD(msg), buf, MAVLINK_MSG_ID_CDnR_CONTROLLER_LEN);
#else
	mavlink_CDnR_controller_t packet;
	packet.h_flag = h_flag;
	packet.s_flag = s_flag;
	packet.a_flag = a_flag;
	packet.t_flag = t_flag;
	packet.new_heading = new_heading;
	packet.new_airspeed = new_airspeed;
	packet.new_altitude = new_altitude;
	packet.max_time = max_time;

        memcpy(_MAV_PAYLOAD(msg), &packet, MAVLINK_MSG_ID_CDnR_CONTROLLER_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_CDnR_CONTROLLER;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CDnR_CONTROLLER_LEN);
}

/**
 * @brief Encode a CDnR_controller struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param CDnR_controller C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_CDnR_controller_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_CDnR_controller_t* CDnR_controller)
{
	return mavlink_msg_CDnR_controller_pack(system_id, component_id, msg, 
							   CDnR_controller->h_flag, CDnR_controller->s_flag, CDnR_controller->a_flag, CDnR_controller->t_flag, 
							   CDnR_controller->new_heading, CDnR_controller->new_airspeed, CDnR_controller->new_altitude, CDnR_controller->max_time);
}

/**
 * @brief Send a CDnR_controller message
 * @param chan MAVLink channel to send the message
 *
 * @param alarm Collision Alarm Flag
 * @param new_heading Desired heading to avoid collision
 * @param new_altitude Desired altitude to avoid collision
 * @param max_time Max number of seconds to maintain new course before resuming waypoint
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_CDnR_controller_send(mavlink_channel_t chan, 
							   uint8_t h_flag, uint8_t s_flag, uint8_t a_flag, uint8_t t_flag, 
							   int16_t new_heading, int16_t new_airspeed, int16_t new_altitude, int16_t max_time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CDnR_CONTROLLER_LEN];
	_mav_put_uint8_t (buf,  0, h_flag);
	_mav_put_uint8_t (buf,  1, s_flag);
	_mav_put_uint8_t (buf,  2, a_flag);
	_mav_put_uint8_t (buf,  3, t_flag);
	_mav_put_int16_t (buf,  4, new_heading);
	_mav_put_int16_t (buf,  6, new_airspeed);
	_mav_put_int16_t (buf,  8, new_altitude);
	_mav_put_int16_t (buf, 10, max_time);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CDnR_CONTROLLER, buf, MAVLINK_MSG_ID_CDnR_CONTROLLER_LEN);
#else
	mavlink_CDnR_controller_t packet;
	packet.h_flag = h_flag;
	packet.s_flag = s_flag;
	packet.a_flag = a_flag;
	packet.t_flag = t_flag;
	packet.new_heading = new_heading;
	packet.new_airspeed = new_airspeed;
	packet.new_altitude = new_altitude;
	packet.max_time = max_time;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CDnR_CONTROLLER, (const char *)&packet, MAVLINK_MSG_ID_CDnR_CONTROLLER_LEN);
#endif
}

#endif

// MESSAGE CDnR_CONTROLLER UNPACKING

/**
 * @brief Get field alarm from CDnR_controller message
 *
 * @return Collision Alarm Flag
 */
static inline uint8_t mavlink_msg_CDnR_controller_get_h_flag(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field alarm from CDnR_controller message
 *
 * @return Collision Alarm Flag
 */
static inline uint8_t mavlink_msg_CDnR_controller_get_s_flag(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field alarm from CDnR_controller message
 *
 * @return Collision Alarm Flag
 */
static inline uint8_t mavlink_msg_CDnR_controller_get_a_flag(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field alarm from CDnR_controller message
 *
 * @return Collision Alarm Flag
 */
static inline uint8_t mavlink_msg_CDnR_controller_get_t_flag(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field new_heading from CDnR_controller message
 *
 * @return Desired heading to avoid collision
 */
static inline int16_t mavlink_msg_CDnR_controller_get_new_heading(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  4);
}

/**
 * @brief Get field new_heading from CDnR_controller message
 *
 * @return Desired heading to avoid collision
 */
static inline int16_t mavlink_msg_CDnR_controller_get_new_airspeed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  6);
}

/**
 * @brief Get field new_altitude from CDnR_controller message
 *
 * @return Desired altitude to avoid collision
 */
static inline int16_t mavlink_msg_CDnR_controller_get_new_altitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  8);
}

/**
 * @brief Get field max_time from CDnR_controller message
 *
 * @return Max number of seconds to maintain new course before resuming waypoint
 */
static inline int16_t mavlink_msg_CDnR_controller_get_max_time(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg, 10);
}

/**
 * @brief Decode a CDnR_controller message into a struct
 *
 * @param msg The message to decode
 * @param CDnR_controller C-struct to decode the message contents into
 */
static inline void mavlink_msg_CDnR_controller_decode(const mavlink_message_t* msg, mavlink_CDnR_controller_t* CDnR_controller)
{
#if MAVLINK_NEED_BYTE_SWAP
	CDnR_controller->h_flag       = mavlink_msg_CDnR_controller_get_h_flag(msg);
	CDnR_controller->s_flag       = mavlink_msg_CDnR_controller_get_s_flag(msg);
	CDnR_controller->a_flag       = mavlink_msg_CDnR_controller_get_a_flag(msg);
	CDnR_controller->t_flag       = mavlink_msg_CDnR_controller_get_t_flag(msg);
	CDnR_controller->new_heading = mavlink_msg_CDnR_controller_get_new_heading(msg);
	CDnR_controller->new_airspeed = mavlink_msg_CDnR_controller_get_new_airspeed(msg);
	CDnR_controller->new_altitude = mavlink_msg_CDnR_controller_get_new_altitude(msg);
	CDnR_controller->max_time    = mavlink_msg_CDnR_controller_get_max_time(msg);
#else
	memcpy(CDnR_controller, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_CDnR_CONTROLLER_LEN);
#endif
}
#else
	#error SLV_GCS_MAVLINK_NEWMESSAGE DISABLED.
#endif
