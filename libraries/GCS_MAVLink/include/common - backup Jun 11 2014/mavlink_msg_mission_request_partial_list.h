// MESSAGE MISSION_REQUEST_PARTIAL_LIST PACKING
// Generated from parse_APM_XML.m on 06-Jun-2013 10:07:52

#define MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST 37 

typedef struct __mavlink_mission_request_partial_list_t 
{ 
  uint8_t target_system;  ///< System ID
  uint8_t target_component;  ///< Component ID
  int16_t start_index;  ///< Start index, 0 by default
  int16_t end_index;  ///< End index, -1 by default (-1: send list to end). Else a valid index of the list
} mavlink_mission_request_partial_list_t;

#define MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN 6
#define MAVLINK_MSG_ID_37_LEN 6

#define MAVLINK_MESSAGE_INFO_MISSION_REQUEST_PARTIAL_LIST \
{ \
  "MISSION_REQUEST_PARTIAL_LIST", \
  4, \
  { \
    { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_mission_request_partial_list_t, target_system) }, \
    { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_mission_request_partial_list_t, target_component) }, \
    { "start_index", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_mission_request_partial_list_t, start_index) }, \
    { "end_index", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_mission_request_partial_list_t, end_index) }, \
  } \
}

/**
 * @brief Pack a mission_request_partial_list message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param start_index Start index, 0 by default
 * @param end_index End index, -1 by default (-1: send list to end). Else a valid index of the list
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mission_request_partial_list_pack(
	 uint8_t system_id
	,uint8_t component_id
	,mavlink_message_t* msg
	,uint8_t target_system
	,uint8_t target_component
	,int16_t start_index
	,int16_t end_index
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_int16_t(buf, 2, start_index);
	_mav_put_int16_t(buf, 4, end_index);

	memcpy(_MAV_PAYLOAD(msg), buf, MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN);
#else
	mavlink_mission_request_partial_list_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.start_index = start_index;
	packet.end_index = end_index;

	memcpy(_MAV_PAYLOAD(msg), &packet, MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST;
	return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN);
}

/**
 * @brief Pack a mission_request_partial_list message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param start_index Start index, 0 by default
 * @param end_index End index, -1 by default (-1: send list to end). Else a valid index of the list
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mission_request_partial_list_pack_chan(
	 uint8_t system_id
	,uint8_t component_id
	,uint8_t chan
	,mavlink_message_t* msg
	,uint8_t target_system
	,uint8_t target_component
	,int16_t start_index
	,int16_t end_index
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_int16_t(buf, 2, start_index);
	_mav_put_int16_t(buf, 4, end_index);

	memcpy(_MAV_PAYLOAD(msg), buf, MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN);
#else
	mavlink_mission_request_partial_list_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.start_index = start_index;
	packet.end_index = end_index;

	memcpy(_MAV_PAYLOAD(msg), &packet, MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN);
}

/**
 * @brief Encode a mission_request_partial_list struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param vfr_hud C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mission_request_partial_list_encode(
	 uint8_t system_id
	,uint8_t component_id
	,mavlink_message_t* msg
	,const mavlink_mission_request_partial_list_t* mission_request_partial_list
	)
{
	return mavlink_msg_mission_request_partial_list_pack(
	 system_id
	,component_id
	,msg
	,mission_request_partial_list->target_system
	,mission_request_partial_list->target_component
	,mission_request_partial_list->start_index
	,mission_request_partial_list->end_index
	);
}

/**
 * @brief Send a mission_request_partial_list message
 * @param chan The MAVLink channel to send this message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param start_index Start index, 0 by default
 * @param end_index End index, -1 by default (-1: send list to end). Else a valid index of the list
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mission_request_partial_list_send(
	 mavlink_channel_t chan
	,uint8_t target_system
	,uint8_t target_component
	,int16_t start_index
	,int16_t end_index
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_int16_t(buf, 2, start_index);
	_mav_put_int16_t(buf, 4, end_index);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST, buf, MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN);
#else
	mavlink_mission_request_partial_list_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.start_index = start_index;
	packet.end_index = end_index;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST, (const char*)&packet, MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN);
#endif
}

#endif // MAVLINK_USE_CONVENIENCE_FUNCTIONS

// MESSAGE MISSION_REQUEST_PARTIAL_LIST UNPACKING

/**
 * @brief Get field target_system from mission_request_partial_list message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_mission_request_partial_list_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,0);
}

/**
 * @brief Get field target_component from mission_request_partial_list message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_mission_request_partial_list_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,1);
}

/**
 * @brief Get field start_index from mission_request_partial_list message
 *
 * @return Start index, 0 by default
 */
static inline int16_t mavlink_msg_mission_request_partial_list_get_start_index(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,2);
}

/**
 * @brief Get field end_index from mission_request_partial_list message
 *
 * @return End index, -1 by default (-1: send list to end). Else a valid index of the list
 */
static inline int16_t mavlink_msg_mission_request_partial_list_get_end_index(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,4);
}

/**
 * @brief Decode a mission_request_partial_list message into a struct
 *
 * @param msg The message to decode
 * @param mission_request_partial_list C-struct to decode the message contents into
 */
static inline void mavlink_msg_mission_request_partial_list_decode(const mavlink_message_t* msg, mavlink_mission_request_partial_list_t* mission_request_partial_list)
{
#if MAVLINK_NEED_BYTE_SWAP
	mission_request_partial_list->target_system = mavlink_msg_mission_request_partial_list_get_target_system(msg);
	mission_request_partial_list->target_component = mavlink_msg_mission_request_partial_list_get_target_component(msg);
	mission_request_partial_list->start_index = mavlink_msg_mission_request_partial_list_get_start_index(msg);
	mission_request_partial_list->end_index = mavlink_msg_mission_request_partial_list_get_end_index(msg);
#else
	memcpy(mission_request_partial_list, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN);
#endif
}
