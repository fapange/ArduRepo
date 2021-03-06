// MESSAGE DEBUG PACKING
// Generated from parse_APM_XML.m on 24-Feb-2014 13:50:53

#define MAVLINK_MSG_ID_DEBUG 255 

typedef struct __mavlink_debug_t 
{ 
  uint8_t ind;  ///< index of debug variable
  float value;  ///< DEBUG value
} mavlink_debug_t;

#define MAVLINK_MSG_ID_DEBUG_LEN 5
#define MAVLINK_MSG_ID_255_LEN 5

#define MAVLINK_MESSAGE_INFO_DEBUG \
{ \
  "DEBUG", \
  2, \
  { \
    { "ind", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_debug_t, ind) }, \
    { "value", NULL, MAVLINK_TYPE_FLOAT, 0, 1, offsetof(mavlink_debug_t, value) }, \
  } \
}

/**
 * @brief Pack a debug message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param ind index of debug variable
 * @param value DEBUG value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_debug_pack(
	 uint8_t system_id
	,uint8_t component_id
	,mavlink_message_t* msg
	,uint8_t ind
	,float value
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_DEBUG_LEN];
	_mav_put_uint8_t(buf, 0, ind);
	_mav_put_float(buf, 1, value);

	memcpy(_MAV_PAYLOAD(msg), buf, MAVLINK_MSG_ID_DEBUG_LEN);
#else
	mavlink_debug_t packet;
	packet.ind = ind;
	packet.value = value;

	memcpy(_MAV_PAYLOAD(msg), &packet, MAVLINK_MSG_ID_DEBUG_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_DEBUG;
	return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DEBUG_LEN);
}

/**
 * @brief Pack a debug message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 *
 * @param ind index of debug variable
 * @param value DEBUG value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_debug_pack_chan(
	 uint8_t system_id
	,uint8_t component_id
	,uint8_t chan
	,mavlink_message_t* msg
	,uint8_t ind
	,float value
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_DEBUG_LEN];
	_mav_put_uint8_t(buf, 0, ind);
	_mav_put_float(buf, 1, value);

	memcpy(_MAV_PAYLOAD(msg), buf, MAVLINK_MSG_ID_DEBUG_LEN);
#else
	mavlink_debug_t packet;
	packet.ind = ind;
	packet.value = value;

	memcpy(_MAV_PAYLOAD(msg), &packet, MAVLINK_MSG_ID_DEBUG_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_DEBUG;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DEBUG_LEN);
}

/**
 * @brief Encode a debug struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param debug C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_debug_encode(
	 uint8_t system_id
	,uint8_t component_id
	,mavlink_message_t* msg
	,const mavlink_debug_t* debug
	)
{
	return mavlink_msg_debug_pack(
	 system_id
	,component_id
	,msg
	,debug->ind
	,debug->value
	);
}

/**
 * @brief Send a debug message
 * @param chan The MAVLink channel to send this message
 *
 * @param ind index of debug variable
 * @param value DEBUG value
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_debug_send(
	 mavlink_channel_t chan
	,uint8_t ind
	,float value
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_DEBUG_LEN];
	_mav_put_uint8_t(buf, 0, ind);
	_mav_put_float(buf, 1, value);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEBUG, buf, MAVLINK_MSG_ID_DEBUG_LEN);
#else
	mavlink_debug_t packet;
	packet.ind = ind;
	packet.value = value;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEBUG, (const char*)&packet, MAVLINK_MSG_ID_DEBUG_LEN);
#endif
}

static inline void mavlink_msg_wID_debug_send(
	 mavlink_channel_t chan
	,uint8_t sID
	,uint8_t cID
	,uint8_t ind
	,float value
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_DEBUG_LEN];
	_mav_put_uint8_t(buf, 0, ind);
	_mav_put_float(buf, 1, value);

	_mav_wID_finalize_message_chan_send(chan, sID, cID, MAVLINK_MSG_ID_DEBUG, buf, MAVLINK_MSG_ID_DEBUG_LEN);
#else
	mavlink_debug_t packet;
	packet.ind = ind;
	packet.value = value;

	_mav_wID_finalize_message_chan_send(chan, sID, cID, MAVLINK_MSG_ID_DEBUG, (const char*)&packet, MAVLINK_MSG_ID_DEBUG_LEN);
#endif
}

#endif // MAVLINK_USE_CONVENIENCE_FUNCTIONS

// MESSAGE DEBUG UNPACKING

/**
 * @brief Get field ind from debug message
 *
 * @return index of debug variable
 */
static inline uint8_t mavlink_msg_debug_get_ind(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,0);
}

/**
 * @brief Get field value from debug message
 *
 * @return DEBUG value
 */
static inline float mavlink_msg_debug_get_value(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,1);
}

/**
 * @brief Decode a debug message into a struct
 *
 * @param msg The message to decode
 * @param debug C-struct to decode the message contents into
 */
static inline void mavlink_msg_debug_decode(const mavlink_message_t* msg, mavlink_debug_t* debug)
{
#if MAVLINK_NEED_BYTE_SWAP
	debug->ind = mavlink_msg_debug_get_ind(msg);
	debug->value = mavlink_msg_debug_get_value(msg);
#else
	memcpy(debug, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_DEBUG_LEN);
#endif
}
