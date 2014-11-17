// MESSAGE PARAM_VALUE PACKING
// Generated from parse_APM_XML.m on 19-Aug-2013 09:01:23

#define MAVLINK_MSG_ID_PARAM_VALUE 22 

typedef struct __mavlink_param_value_t 
{ 
  int8_t param_id[15];  ///< Onboard parameter id
  float param_value;  ///< Onboard parameter value
  uint16_t param_count;  ///< Total number of onboard parameters
  uint16_t param_index;  ///< Index of this onboard parameter
} mavlink_param_value_t;

#define MAVLINK_MSG_ID_PARAM_VALUE_LEN 23
#define MAVLINK_MSG_ID_22_LEN 23

#define MAVLINK_MESSAGE_INFO_PARAM_VALUE \
{ \
  "PARAM_VALUE", \
  4, \
  { \
    { "param_id", NULL, MAVLINK_TYPE_INT8_T, 15, 0, offsetof(mavlink_param_value_t, param_id) }, \
    { "param_value", NULL, MAVLINK_TYPE_FLOAT, 0, 15, offsetof(mavlink_param_value_t, param_value) }, \
    { "param_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 19, offsetof(mavlink_param_value_t, param_count) }, \
    { "param_index", NULL, MAVLINK_TYPE_UINT16_T, 0, 21, offsetof(mavlink_param_value_t, param_index) }, \
  } \
}

/**
 * @brief Pack a param_value message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param param_id Onboard parameter id
 * @param param_value Onboard parameter value
 * @param param_count Total number of onboard parameters
 * @param param_index Index of this onboard parameter
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_param_value_pack(
	 uint8_t system_id
	,uint8_t component_id
	,mavlink_message_t* msg
	,const int8_t* param_id
	,float param_value
	,uint16_t param_count
	,uint16_t param_index
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PARAM_VALUE_LEN];
	_mav_put_int8_t_array(buf, 0, param_id, 15);
	_mav_put_float(buf, 15, param_value);
	_mav_put_uint16_t(buf, 19, param_count);
	_mav_put_uint16_t(buf, 21, param_index);

	memcpy(_MAV_PAYLOAD(msg), buf, MAVLINK_MSG_ID_PARAM_VALUE_LEN);
#else
	mavlink_param_value_t packet;
	mav_array_memcpy(packet.param_id, param_id, sizeof(int8_t)*15);
	packet.param_value = param_value;
	packet.param_count = param_count;
	packet.param_index = param_index;

	memcpy(_MAV_PAYLOAD(msg), &packet, MAVLINK_MSG_ID_PARAM_VALUE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_PARAM_VALUE;
	return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PARAM_VALUE_LEN);
}

/**
 * @brief Pack a param_value message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 *
 * @param param_id Onboard parameter id
 * @param param_value Onboard parameter value
 * @param param_count Total number of onboard parameters
 * @param param_index Index of this onboard parameter
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_param_value_pack_chan(
	 uint8_t system_id
	,uint8_t component_id
	,uint8_t chan
	,mavlink_message_t* msg
	,const int8_t* param_id
	,float param_value
	,uint16_t param_count
	,uint16_t param_index
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PARAM_VALUE_LEN];
	_mav_put_int8_t_array(buf, 0, param_id, 15);
	_mav_put_float(buf, 15, param_value);
	_mav_put_uint16_t(buf, 19, param_count);
	_mav_put_uint16_t(buf, 21, param_index);

	memcpy(_MAV_PAYLOAD(msg), buf, MAVLINK_MSG_ID_PARAM_VALUE_LEN);
#else
	mavlink_param_value_t packet;
	mav_array_memcpy(packet.param_id, param_id, sizeof(int8_t)*15);
	packet.param_value = param_value;
	packet.param_count = param_count;
	packet.param_index = param_index;

	memcpy(_MAV_PAYLOAD(msg), &packet, MAVLINK_MSG_ID_PARAM_VALUE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_PARAM_VALUE;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PARAM_VALUE_LEN);
}

/**
 * @brief Encode a param_value struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param vfr_hud C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_param_value_encode(
	 uint8_t system_id
	,uint8_t component_id
	,mavlink_message_t* msg
	,const mavlink_param_value_t* param_value
	)
{
	return mavlink_msg_param_value_pack(
	 system_id
	,component_id
	,msg
	,param_value->param_id
	,param_value->param_value
	,param_value->param_count
	,param_value->param_index
	);
}

/**
 * @brief Send a param_value message
 * @param chan The MAVLink channel to send this message
 *
 * @param param_id Onboard parameter id
 * @param param_value Onboard parameter value
 * @param param_count Total number of onboard parameters
 * @param param_index Index of this onboard parameter
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_param_value_send(
	 mavlink_channel_t chan
	,const int8_t* param_id
	,float param_value
	,uint16_t param_count
	,uint16_t param_index
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PARAM_VALUE_LEN];
	_mav_put_int8_t_array(buf, 0, param_id, 15);
	_mav_put_float(buf, 15, param_value);
	_mav_put_uint16_t(buf, 19, param_count);
	_mav_put_uint16_t(buf, 21, param_index);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_VALUE, buf, MAVLINK_MSG_ID_PARAM_VALUE_LEN);
#else
	mavlink_param_value_t packet;
	mav_array_memcpy(packet.param_id, param_id, sizeof(int8_t)*15);
	packet.param_value = param_value;
	packet.param_count = param_count;
	packet.param_index = param_index;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_VALUE, (const char*)&packet, MAVLINK_MSG_ID_PARAM_VALUE_LEN);
#endif
}

#endif // MAVLINK_USE_CONVENIENCE_FUNCTIONS

// MESSAGE PARAM_VALUE UNPACKING

/**
 * @brief Get field param_id from param_value message
 *
 * @return Onboard parameter id
 */
static inline int8_t mavlink_msg_param_value_get_param_id(const mavlink_message_t* msg, int8_t* param_id)
{
	return _MAV_RETURN_int8_t_array(msg,param_id,15,0);
}

/**
 * @brief Get field param_value from param_value message
 *
 * @return Onboard parameter value
 */
static inline float mavlink_msg_param_value_get_param_value(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,15);
}

/**
 * @brief Get field param_count from param_value message
 *
 * @return Total number of onboard parameters
 */
static inline uint16_t mavlink_msg_param_value_get_param_count(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,19);
}

/**
 * @brief Get field param_index from param_value message
 *
 * @return Index of this onboard parameter
 */
static inline uint16_t mavlink_msg_param_value_get_param_index(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,21);
}

/**
 * @brief Decode a param_value message into a struct
 *
 * @param msg The message to decode
 * @param param_value C-struct to decode the message contents into
 */
static inline void mavlink_msg_param_value_decode(const mavlink_message_t* msg, mavlink_param_value_t* param_value)
{
#if MAVLINK_NEED_BYTE_SWAP
	mavlink_msg_param_value_get_param_id(msg, param_value->param_id);
	param_value->param_value = mavlink_msg_param_value_get_param_value(msg);
	param_value->param_count = mavlink_msg_param_value_get_param_count(msg);
	param_value->param_index = mavlink_msg_param_value_get_param_index(msg);
#else
	memcpy(param_value, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_PARAM_VALUE_LEN);
#endif
}
