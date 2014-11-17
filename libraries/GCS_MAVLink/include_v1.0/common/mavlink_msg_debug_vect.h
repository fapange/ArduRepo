// MESSAGE DEBUG_VECT PACKING
// Generated from parse_APM_XML.m on 19-Aug-2013 09:01:21

#define MAVLINK_MSG_ID_DEBUG_VECT 250 

typedef struct __mavlink_debug_vect_t 
{ 
  char name[10];  ///< Name
  uint64_t time_usec;  ///< Timestamp
  float x;  ///< x
  float y;  ///< y
  float z;  ///< z
} mavlink_debug_vect_t;

#define MAVLINK_MSG_ID_DEBUG_VECT_LEN 30
#define MAVLINK_MSG_ID_250_LEN 30

#define MAVLINK_MESSAGE_INFO_DEBUG_VECT \
{ \
  "DEBUG_VECT", \
  5, \
  { \
    { "name", NULL, MAVLINK_TYPE_CHAR, 10, 0, offsetof(mavlink_debug_vect_t, name) }, \
    { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 10, offsetof(mavlink_debug_vect_t, time_usec) }, \
    { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 18, offsetof(mavlink_debug_vect_t, x) }, \
    { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 22, offsetof(mavlink_debug_vect_t, y) }, \
    { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 26, offsetof(mavlink_debug_vect_t, z) }, \
  } \
}

/**
 * @brief Pack a debug_vect message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param name Name
 * @param time_usec Timestamp
 * @param x x
 * @param y y
 * @param z z
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_debug_vect_pack(
	 uint8_t system_id
	,uint8_t component_id
	,mavlink_message_t* msg
	,const char* name
	,uint64_t time_usec
	,float x
	,float y
	,float z
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_DEBUG_VECT_LEN];
	_mav_put_char_array(buf, 0, name, 10);
	_mav_put_uint64_t(buf, 10, time_usec);
	_mav_put_float(buf, 18, x);
	_mav_put_float(buf, 22, y);
	_mav_put_float(buf, 26, z);

	memcpy(_MAV_PAYLOAD(msg), buf, MAVLINK_MSG_ID_DEBUG_VECT_LEN);
#else
	mavlink_debug_vect_t packet;
	mav_array_memcpy(packet.name, name, sizeof(char)*10);
	packet.time_usec = time_usec;
	packet.x = x;
	packet.y = y;
	packet.z = z;

	memcpy(_MAV_PAYLOAD(msg), &packet, MAVLINK_MSG_ID_DEBUG_VECT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_DEBUG_VECT;
	return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DEBUG_VECT_LEN);
}

/**
 * @brief Pack a debug_vect message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 *
 * @param name Name
 * @param time_usec Timestamp
 * @param x x
 * @param y y
 * @param z z
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_debug_vect_pack_chan(
	 uint8_t system_id
	,uint8_t component_id
	,uint8_t chan
	,mavlink_message_t* msg
	,const char* name
	,uint64_t time_usec
	,float x
	,float y
	,float z
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_DEBUG_VECT_LEN];
	_mav_put_char_array(buf, 0, name, 10);
	_mav_put_uint64_t(buf, 10, time_usec);
	_mav_put_float(buf, 18, x);
	_mav_put_float(buf, 22, y);
	_mav_put_float(buf, 26, z);

	memcpy(_MAV_PAYLOAD(msg), buf, MAVLINK_MSG_ID_DEBUG_VECT_LEN);
#else
	mavlink_debug_vect_t packet;
	mav_array_memcpy(packet.name, name, sizeof(char)*10);
	packet.time_usec = time_usec;
	packet.x = x;
	packet.y = y;
	packet.z = z;

	memcpy(_MAV_PAYLOAD(msg), &packet, MAVLINK_MSG_ID_DEBUG_VECT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_DEBUG_VECT;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DEBUG_VECT_LEN);
}

/**
 * @brief Encode a debug_vect struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param vfr_hud C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_debug_vect_encode(
	 uint8_t system_id
	,uint8_t component_id
	,mavlink_message_t* msg
	,const mavlink_debug_vect_t* debug_vect
	)
{
	return mavlink_msg_debug_vect_pack(
	 system_id
	,component_id
	,msg
	,debug_vect->name
	,debug_vect->time_usec
	,debug_vect->x
	,debug_vect->y
	,debug_vect->z
	);
}

/**
 * @brief Send a debug_vect message
 * @param chan The MAVLink channel to send this message
 *
 * @param name Name
 * @param time_usec Timestamp
 * @param x x
 * @param y y
 * @param z z
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_debug_vect_send(
	 mavlink_channel_t chan
	,const char* name
	,uint64_t time_usec
	,float x
	,float y
	,float z
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_DEBUG_VECT_LEN];
	_mav_put_char_array(buf, 0, name, 10);
	_mav_put_uint64_t(buf, 10, time_usec);
	_mav_put_float(buf, 18, x);
	_mav_put_float(buf, 22, y);
	_mav_put_float(buf, 26, z);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEBUG_VECT, buf, MAVLINK_MSG_ID_DEBUG_VECT_LEN);
#else
	mavlink_debug_vect_t packet;
	mav_array_memcpy(packet.name, name, sizeof(char)*10);
	packet.time_usec = time_usec;
	packet.x = x;
	packet.y = y;
	packet.z = z;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEBUG_VECT, (const char*)&packet, MAVLINK_MSG_ID_DEBUG_VECT_LEN);
#endif
}

#endif // MAVLINK_USE_CONVENIENCE_FUNCTIONS

// MESSAGE DEBUG_VECT UNPACKING

/**
 * @brief Get field name from debug_vect message
 *
 * @return Name
 */
static inline char mavlink_msg_debug_vect_get_name(const mavlink_message_t* msg, char* name)
{
	return _MAV_RETURN_char_array(msg,name,10,0);
}

/**
 * @brief Get field time_usec from debug_vect message
 *
 * @return Timestamp
 */
static inline uint64_t mavlink_msg_debug_vect_get_time_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,10);
}

/**
 * @brief Get field x from debug_vect message
 *
 * @return x
 */
static inline float mavlink_msg_debug_vect_get_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,18);
}

/**
 * @brief Get field y from debug_vect message
 *
 * @return y
 */
static inline float mavlink_msg_debug_vect_get_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,22);
}

/**
 * @brief Get field z from debug_vect message
 *
 * @return z
 */
static inline float mavlink_msg_debug_vect_get_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,26);
}

/**
 * @brief Decode a debug_vect message into a struct
 *
 * @param msg The message to decode
 * @param debug_vect C-struct to decode the message contents into
 */
static inline void mavlink_msg_debug_vect_decode(const mavlink_message_t* msg, mavlink_debug_vect_t* debug_vect)
{
#if MAVLINK_NEED_BYTE_SWAP
	mavlink_msg_debug_vect_get_name(msg, debug_vect->name);
	debug_vect->time_usec = mavlink_msg_debug_vect_get_time_usec(msg);
	debug_vect->x = mavlink_msg_debug_vect_get_x(msg);
	debug_vect->y = mavlink_msg_debug_vect_get_y(msg);
	debug_vect->z = mavlink_msg_debug_vect_get_z(msg);
#else
	memcpy(debug_vect, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_DEBUG_VECT_LEN);
#endif
}
