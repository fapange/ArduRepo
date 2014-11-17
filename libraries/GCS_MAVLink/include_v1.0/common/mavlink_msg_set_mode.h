// MESSAGE SET_MODE PACKING
// Generated from parse_APM_XML.m on 19-Aug-2013 09:01:20

#define MAVLINK_MSG_ID_SET_MODE 11 

typedef struct __mavlink_set_mode_t 
{ 
  uint8_t target_system;  ///< The system setting the mode
  uint8_t base_mode;  ///< The new base mode
  uint32_t custom_mode;  ///< The new autopilot-specific mode. This field can be ignored by an autopilot.
} mavlink_set_mode_t;

#define MAVLINK_MSG_ID_SET_MODE_LEN 6
#define MAVLINK_MSG_ID_11_LEN 6

#define MAVLINK_MESSAGE_INFO_SET_MODE \
{ \
  "SET_MODE", \
  3, \
  { \
    { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_set_mode_t, target_system) }, \
    { "base_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_set_mode_t, base_mode) }, \
    { "custom_mode", NULL, MAVLINK_TYPE_UINT32_T, 0, 2, offsetof(mavlink_set_mode_t, custom_mode) }, \
  } \
}

/**
 * @brief Pack a set_mode message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system The system setting the mode
 * @param base_mode The new base mode
 * @param custom_mode The new autopilot-specific mode. This field can be ignored by an autopilot.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_mode_pack(
	 uint8_t system_id
	,uint8_t component_id
	,mavlink_message_t* msg
	,uint8_t target_system
	,uint8_t base_mode
	,uint32_t custom_mode
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SET_MODE_LEN];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, base_mode);
	_mav_put_uint32_t(buf, 2, custom_mode);

	memcpy(_MAV_PAYLOAD(msg), buf, MAVLINK_MSG_ID_SET_MODE_LEN);
#else
	mavlink_set_mode_t packet;
	packet.target_system = target_system;
	packet.base_mode = base_mode;
	packet.custom_mode = custom_mode;

	memcpy(_MAV_PAYLOAD(msg), &packet, MAVLINK_MSG_ID_SET_MODE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SET_MODE;
	return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_MODE_LEN);
}

/**
 * @brief Pack a set_mode message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system The system setting the mode
 * @param base_mode The new base mode
 * @param custom_mode The new autopilot-specific mode. This field can be ignored by an autopilot.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_mode_pack_chan(
	 uint8_t system_id
	,uint8_t component_id
	,uint8_t chan
	,mavlink_message_t* msg
	,uint8_t target_system
	,uint8_t base_mode
	,uint32_t custom_mode
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SET_MODE_LEN];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, base_mode);
	_mav_put_uint32_t(buf, 2, custom_mode);

	memcpy(_MAV_PAYLOAD(msg), buf, MAVLINK_MSG_ID_SET_MODE_LEN);
#else
	mavlink_set_mode_t packet;
	packet.target_system = target_system;
	packet.base_mode = base_mode;
	packet.custom_mode = custom_mode;

	memcpy(_MAV_PAYLOAD(msg), &packet, MAVLINK_MSG_ID_SET_MODE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SET_MODE;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_MODE_LEN);
}

/**
 * @brief Encode a set_mode struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param vfr_hud C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_mode_encode(
	 uint8_t system_id
	,uint8_t component_id
	,mavlink_message_t* msg
	,const mavlink_set_mode_t* set_mode
	)
{
	return mavlink_msg_set_mode_pack(
	 system_id
	,component_id
	,msg
	,set_mode->target_system
	,set_mode->base_mode
	,set_mode->custom_mode
	);
}

/**
 * @brief Send a set_mode message
 * @param chan The MAVLink channel to send this message
 *
 * @param target_system The system setting the mode
 * @param base_mode The new base mode
 * @param custom_mode The new autopilot-specific mode. This field can be ignored by an autopilot.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_mode_send(
	 mavlink_channel_t chan
	,uint8_t target_system
	,uint8_t base_mode
	,uint32_t custom_mode
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SET_MODE_LEN];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, base_mode);
	_mav_put_uint32_t(buf, 2, custom_mode);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_MODE, buf, MAVLINK_MSG_ID_SET_MODE_LEN);
#else
	mavlink_set_mode_t packet;
	packet.target_system = target_system;
	packet.base_mode = base_mode;
	packet.custom_mode = custom_mode;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_MODE, (const char*)&packet, MAVLINK_MSG_ID_SET_MODE_LEN);
#endif
}

#endif // MAVLINK_USE_CONVENIENCE_FUNCTIONS

// MESSAGE SET_MODE UNPACKING

/**
 * @brief Get field target_system from set_mode message
 *
 * @return The system setting the mode
 */
static inline uint8_t mavlink_msg_set_mode_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,0);
}

/**
 * @brief Get field base_mode from set_mode message
 *
 * @return The new base mode
 */
static inline uint8_t mavlink_msg_set_mode_get_base_mode(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,1);
}

/**
 * @brief Get field custom_mode from set_mode message
 *
 * @return The new autopilot-specific mode. This field can be ignored by an autopilot.
 */
static inline uint32_t mavlink_msg_set_mode_get_custom_mode(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,2);
}

/**
 * @brief Decode a set_mode message into a struct
 *
 * @param msg The message to decode
 * @param set_mode C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_mode_decode(const mavlink_message_t* msg, mavlink_set_mode_t* set_mode)
{
#if MAVLINK_NEED_BYTE_SWAP
	set_mode->target_system = mavlink_msg_set_mode_get_target_system(msg);
	set_mode->base_mode = mavlink_msg_set_mode_get_base_mode(msg);
	set_mode->custom_mode = mavlink_msg_set_mode_get_custom_mode(msg);
#else
	memcpy(set_mode, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SET_MODE_LEN);
#endif
}
