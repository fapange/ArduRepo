// MESSAGE SET_GPS_GLOBAL_ORIGIN PACKING
// Generated from parse_APM_XML.m on 06-Jun-2013 10:07:52

#define MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN 48 

typedef struct __mavlink_set_gps_global_origin_t 
{ 
  uint8_t target_system;  ///< System ID
  int32_t latitude;  ///< global position * 1E7
  int32_t longitude;  ///< global position * 1E7
  int32_t altitude;  ///< global position * 1000
} mavlink_set_gps_global_origin_t;

#define MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN_LEN 13
#define MAVLINK_MSG_ID_48_LEN 13

#define MAVLINK_MESSAGE_INFO_SET_GPS_GLOBAL_ORIGIN \
{ \
  "SET_GPS_GLOBAL_ORIGIN", \
  4, \
  { \
    { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_set_gps_global_origin_t, target_system) }, \
    { "latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 1, offsetof(mavlink_set_gps_global_origin_t, latitude) }, \
    { "longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 5, offsetof(mavlink_set_gps_global_origin_t, longitude) }, \
    { "altitude", NULL, MAVLINK_TYPE_INT32_T, 0, 9, offsetof(mavlink_set_gps_global_origin_t, altitude) }, \
  } \
}

/**
 * @brief Pack a set_gps_global_origin message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param latitude global position * 1E7
 * @param longitude global position * 1E7
 * @param altitude global position * 1000
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_gps_global_origin_pack(
	 uint8_t system_id
	,uint8_t component_id
	,mavlink_message_t* msg
	,uint8_t target_system
	,int32_t latitude
	,int32_t longitude
	,int32_t altitude
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN_LEN];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_int32_t(buf, 1, latitude);
	_mav_put_int32_t(buf, 5, longitude);
	_mav_put_int32_t(buf, 9, altitude);

	memcpy(_MAV_PAYLOAD(msg), buf, MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN_LEN);
#else
	mavlink_set_gps_global_origin_t packet;
	packet.target_system = target_system;
	packet.latitude = latitude;
	packet.longitude = longitude;
	packet.altitude = altitude;

	memcpy(_MAV_PAYLOAD(msg), &packet, MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN;
	return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN_LEN);
}

/**
 * @brief Pack a set_gps_global_origin message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param latitude global position * 1E7
 * @param longitude global position * 1E7
 * @param altitude global position * 1000
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_gps_global_origin_pack_chan(
	 uint8_t system_id
	,uint8_t component_id
	,uint8_t chan
	,mavlink_message_t* msg
	,uint8_t target_system
	,int32_t latitude
	,int32_t longitude
	,int32_t altitude
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN_LEN];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_int32_t(buf, 1, latitude);
	_mav_put_int32_t(buf, 5, longitude);
	_mav_put_int32_t(buf, 9, altitude);

	memcpy(_MAV_PAYLOAD(msg), buf, MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN_LEN);
#else
	mavlink_set_gps_global_origin_t packet;
	packet.target_system = target_system;
	packet.latitude = latitude;
	packet.longitude = longitude;
	packet.altitude = altitude;

	memcpy(_MAV_PAYLOAD(msg), &packet, MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN_LEN);
}

/**
 * @brief Encode a set_gps_global_origin struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param vfr_hud C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_gps_global_origin_encode(
	 uint8_t system_id
	,uint8_t component_id
	,mavlink_message_t* msg
	,const mavlink_set_gps_global_origin_t* set_gps_global_origin
	)
{
	return mavlink_msg_set_gps_global_origin_pack(
	 system_id
	,component_id
	,msg
	,set_gps_global_origin->target_system
	,set_gps_global_origin->latitude
	,set_gps_global_origin->longitude
	,set_gps_global_origin->altitude
	);
}

/**
 * @brief Send a set_gps_global_origin message
 * @param chan The MAVLink channel to send this message
 *
 * @param target_system System ID
 * @param latitude global position * 1E7
 * @param longitude global position * 1E7
 * @param altitude global position * 1000
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_gps_global_origin_send(
	 mavlink_channel_t chan
	,uint8_t target_system
	,int32_t latitude
	,int32_t longitude
	,int32_t altitude
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN_LEN];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_int32_t(buf, 1, latitude);
	_mav_put_int32_t(buf, 5, longitude);
	_mav_put_int32_t(buf, 9, altitude);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN, buf, MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN_LEN);
#else
	mavlink_set_gps_global_origin_t packet;
	packet.target_system = target_system;
	packet.latitude = latitude;
	packet.longitude = longitude;
	packet.altitude = altitude;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN, (const char*)&packet, MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN_LEN);
#endif
}

#endif // MAVLINK_USE_CONVENIENCE_FUNCTIONS

// MESSAGE SET_GPS_GLOBAL_ORIGIN UNPACKING

/**
 * @brief Get field target_system from set_gps_global_origin message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_set_gps_global_origin_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,0);
}

/**
 * @brief Get field latitude from set_gps_global_origin message
 *
 * @return global position * 1E7
 */
static inline int32_t mavlink_msg_set_gps_global_origin_get_latitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,1);
}

/**
 * @brief Get field longitude from set_gps_global_origin message
 *
 * @return global position * 1E7
 */
static inline int32_t mavlink_msg_set_gps_global_origin_get_longitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,5);
}

/**
 * @brief Get field altitude from set_gps_global_origin message
 *
 * @return global position * 1000
 */
static inline int32_t mavlink_msg_set_gps_global_origin_get_altitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,9);
}

/**
 * @brief Decode a set_gps_global_origin message into a struct
 *
 * @param msg The message to decode
 * @param set_gps_global_origin C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_gps_global_origin_decode(const mavlink_message_t* msg, mavlink_set_gps_global_origin_t* set_gps_global_origin)
{
#if MAVLINK_NEED_BYTE_SWAP
	set_gps_global_origin->target_system = mavlink_msg_set_gps_global_origin_get_target_system(msg);
	set_gps_global_origin->latitude = mavlink_msg_set_gps_global_origin_get_latitude(msg);
	set_gps_global_origin->longitude = mavlink_msg_set_gps_global_origin_get_longitude(msg);
	set_gps_global_origin->altitude = mavlink_msg_set_gps_global_origin_get_altitude(msg);
#else
	memcpy(set_gps_global_origin, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN_LEN);
#endif
}
