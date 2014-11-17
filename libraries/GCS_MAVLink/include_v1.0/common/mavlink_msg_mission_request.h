// MESSAGE MISSION_REQUEST PACKING
// Generated from parse_APM_XML.m on 19-Aug-2013 09:01:20

#define MAVLINK_MSG_ID_MISSION_REQUEST 40 

typedef struct __mavlink_mission_request_t 
{ 
  uint8_t target_system;  ///< System ID
  uint8_t target_component;  ///< Component ID
  uint16_t seq;  ///< Sequence
} mavlink_mission_request_t;

#define MAVLINK_MSG_ID_MISSION_REQUEST_LEN 4
#define MAVLINK_MSG_ID_40_LEN 4

#define MAVLINK_MESSAGE_INFO_MISSION_REQUEST \
{ \
  "MISSION_REQUEST", \
  3, \
  { \
    { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_mission_request_t, target_system) }, \
    { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_mission_request_t, target_component) }, \
    { "seq", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_mission_request_t, seq) }, \
  } \
}

/**
 * @brief Pack a mission_request message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param seq Sequence
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mission_request_pack(
	 uint8_t system_id
	,uint8_t component_id
	,mavlink_message_t* msg
	,uint8_t target_system
	,uint8_t target_component
	,uint16_t seq
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MISSION_REQUEST_LEN];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_uint16_t(buf, 2, seq);

	memcpy(_MAV_PAYLOAD(msg), buf, MAVLINK_MSG_ID_MISSION_REQUEST_LEN);
#else
	mavlink_mission_request_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.seq = seq;

	memcpy(_MAV_PAYLOAD(msg), &packet, MAVLINK_MSG_ID_MISSION_REQUEST_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MISSION_REQUEST;
	return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MISSION_REQUEST_LEN);
}

/**
 * @brief Pack a mission_request message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param seq Sequence
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mission_request_pack_chan(
	 uint8_t system_id
	,uint8_t component_id
	,uint8_t chan
	,mavlink_message_t* msg
	,uint8_t target_system
	,uint8_t target_component
	,uint16_t seq
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MISSION_REQUEST_LEN];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_uint16_t(buf, 2, seq);

	memcpy(_MAV_PAYLOAD(msg), buf, MAVLINK_MSG_ID_MISSION_REQUEST_LEN);
#else
	mavlink_mission_request_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.seq = seq;

	memcpy(_MAV_PAYLOAD(msg), &packet, MAVLINK_MSG_ID_MISSION_REQUEST_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MISSION_REQUEST;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MISSION_REQUEST_LEN);
}

/**
 * @brief Encode a mission_request struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param vfr_hud C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mission_request_encode(
	 uint8_t system_id
	,uint8_t component_id
	,mavlink_message_t* msg
	,const mavlink_mission_request_t* mission_request
	)
{
	return mavlink_msg_mission_request_pack(
	 system_id
	,component_id
	,msg
	,mission_request->target_system
	,mission_request->target_component
	,mission_request->seq
	);
}

/**
 * @brief Send a mission_request message
 * @param chan The MAVLink channel to send this message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param seq Sequence
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mission_request_send(
	 mavlink_channel_t chan
	,uint8_t target_system
	,uint8_t target_component
	,uint16_t seq
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MISSION_REQUEST_LEN];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_uint16_t(buf, 2, seq);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_REQUEST, buf, MAVLINK_MSG_ID_MISSION_REQUEST_LEN);
#else
	mavlink_mission_request_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.seq = seq;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_REQUEST, (const char*)&packet, MAVLINK_MSG_ID_MISSION_REQUEST_LEN);
#endif
}

#endif // MAVLINK_USE_CONVENIENCE_FUNCTIONS

// MESSAGE MISSION_REQUEST UNPACKING

/**
 * @brief Get field target_system from mission_request message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_mission_request_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,0);
}

/**
 * @brief Get field target_component from mission_request message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_mission_request_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,1);
}

/**
 * @brief Get field seq from mission_request message
 *
 * @return Sequence
 */
static inline uint16_t mavlink_msg_mission_request_get_seq(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,2);
}

/**
 * @brief Decode a mission_request message into a struct
 *
 * @param msg The message to decode
 * @param mission_request C-struct to decode the message contents into
 */
static inline void mavlink_msg_mission_request_decode(const mavlink_message_t* msg, mavlink_mission_request_t* mission_request)
{
#if MAVLINK_NEED_BYTE_SWAP
	mission_request->target_system = mavlink_msg_mission_request_get_target_system(msg);
	mission_request->target_component = mavlink_msg_mission_request_get_target_component(msg);
	mission_request->seq = mavlink_msg_mission_request_get_seq(msg);
#else
	memcpy(mission_request, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_MISSION_REQUEST_LEN);
#endif
}
