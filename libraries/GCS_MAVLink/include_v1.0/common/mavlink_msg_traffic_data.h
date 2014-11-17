// MESSAGE TRAFFIC_DATA PACKING
// Generated from parse_APM_XML.m on 19-Aug-2013 09:01:20

#define MAVLINK_MSG_ID_TRAFFIC_DATA 114 

typedef struct __mavlink_traffic_data_t 
{ 
  char ID[8];  ///< vehicle ID
  float lat;  ///< latitude (deg)
  float lon;  ///< longitude (deg)
  float alt;  ///< altitude (meters)
  float roll;  ///< roll (deg)
  float pitch;  ///< pitch (deg)
  float heading;  ///< heading (deg)
  uint8_t ownship_flag;  ///< identifies whether the aircraft is ownship (1) or not (0)
} mavlink_traffic_data_t;

#define MAVLINK_MSG_ID_TRAFFIC_DATA_LEN 33
#define MAVLINK_MSG_ID_114_LEN 33

#define MAVLINK_MESSAGE_INFO_TRAFFIC_DATA \
{ \
  "TRAFFIC_DATA", \
  8, \
  { \
    { "ID", NULL, MAVLINK_TYPE_CHAR, 8, 0, offsetof(mavlink_traffic_data_t, ID) }, \
    { "lat", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_traffic_data_t, lat) }, \
    { "lon", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_traffic_data_t, lon) }, \
    { "alt", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_traffic_data_t, alt) }, \
    { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_traffic_data_t, roll) }, \
    { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_traffic_data_t, pitch) }, \
    { "heading", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_traffic_data_t, heading) }, \
    { "ownship_flag", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_traffic_data_t, ownship_flag) }, \
  } \
}

/**
 * @brief Pack a traffic_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param ID vehicle ID
 * @param lat latitude (deg)
 * @param lon longitude (deg)
 * @param alt altitude (meters)
 * @param roll roll (deg)
 * @param pitch pitch (deg)
 * @param heading heading (deg)
 * @param ownship_flag identifies whether the aircraft is ownship (1) or not (0)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_traffic_data_pack(
	 uint8_t system_id
	,uint8_t component_id
	,mavlink_message_t* msg
	,const char* ID
	,float lat
	,float lon
	,float alt
	,float roll
	,float pitch
	,float heading
	,uint8_t ownship_flag
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TRAFFIC_DATA_LEN];
	_mav_put_char_array(buf, 0, ID, 8);
	_mav_put_float(buf, 8, lat);
	_mav_put_float(buf, 12, lon);
	_mav_put_float(buf, 16, alt);
	_mav_put_float(buf, 20, roll);
	_mav_put_float(buf, 24, pitch);
	_mav_put_float(buf, 28, heading);
	_mav_put_uint8_t(buf, 32, ownship_flag);

	memcpy(_MAV_PAYLOAD(msg), buf, MAVLINK_MSG_ID_TRAFFIC_DATA_LEN);
#else
	mavlink_traffic_data_t packet;
	mav_array_memcpy(packet.ID, ID, sizeof(char)*8);
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.heading = heading;
	packet.ownship_flag = ownship_flag;

	memcpy(_MAV_PAYLOAD(msg), &packet, MAVLINK_MSG_ID_TRAFFIC_DATA_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_TRAFFIC_DATA;
	return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TRAFFIC_DATA_LEN);
}

/**
 * @brief Pack a traffic_data message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 *
 * @param ID vehicle ID
 * @param lat latitude (deg)
 * @param lon longitude (deg)
 * @param alt altitude (meters)
 * @param roll roll (deg)
 * @param pitch pitch (deg)
 * @param heading heading (deg)
 * @param ownship_flag identifies whether the aircraft is ownship (1) or not (0)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_traffic_data_pack_chan(
	 uint8_t system_id
	,uint8_t component_id
	,uint8_t chan
	,mavlink_message_t* msg
	,const char* ID
	,float lat
	,float lon
	,float alt
	,float roll
	,float pitch
	,float heading
	,uint8_t ownship_flag
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TRAFFIC_DATA_LEN];
	_mav_put_char_array(buf, 0, ID, 8);
	_mav_put_float(buf, 8, lat);
	_mav_put_float(buf, 12, lon);
	_mav_put_float(buf, 16, alt);
	_mav_put_float(buf, 20, roll);
	_mav_put_float(buf, 24, pitch);
	_mav_put_float(buf, 28, heading);
	_mav_put_uint8_t(buf, 32, ownship_flag);

	memcpy(_MAV_PAYLOAD(msg), buf, MAVLINK_MSG_ID_TRAFFIC_DATA_LEN);
#else
	mavlink_traffic_data_t packet;
	mav_array_memcpy(packet.ID, ID, sizeof(char)*8);
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.heading = heading;
	packet.ownship_flag = ownship_flag;

	memcpy(_MAV_PAYLOAD(msg), &packet, MAVLINK_MSG_ID_TRAFFIC_DATA_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_TRAFFIC_DATA;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TRAFFIC_DATA_LEN);
}

/**
 * @brief Encode a traffic_data struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param vfr_hud C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_traffic_data_encode(
	 uint8_t system_id
	,uint8_t component_id
	,mavlink_message_t* msg
	,const mavlink_traffic_data_t* traffic_data
	)
{
	return mavlink_msg_traffic_data_pack(
	 system_id
	,component_id
	,msg
	,traffic_data->ID
	,traffic_data->lat
	,traffic_data->lon
	,traffic_data->alt
	,traffic_data->roll
	,traffic_data->pitch
	,traffic_data->heading
	,traffic_data->ownship_flag
	);
}

/**
 * @brief Send a traffic_data message
 * @param chan The MAVLink channel to send this message
 *
 * @param ID vehicle ID
 * @param lat latitude (deg)
 * @param lon longitude (deg)
 * @param alt altitude (meters)
 * @param roll roll (deg)
 * @param pitch pitch (deg)
 * @param heading heading (deg)
 * @param ownship_flag identifies whether the aircraft is ownship (1) or not (0)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_traffic_data_send(
	 mavlink_channel_t chan
	,const char* ID
	,float lat
	,float lon
	,float alt
	,float roll
	,float pitch
	,float heading
	,uint8_t ownship_flag
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TRAFFIC_DATA_LEN];
	_mav_put_char_array(buf, 0, ID, 8);
	_mav_put_float(buf, 8, lat);
	_mav_put_float(buf, 12, lon);
	_mav_put_float(buf, 16, alt);
	_mav_put_float(buf, 20, roll);
	_mav_put_float(buf, 24, pitch);
	_mav_put_float(buf, 28, heading);
	_mav_put_uint8_t(buf, 32, ownship_flag);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRAFFIC_DATA, buf, MAVLINK_MSG_ID_TRAFFIC_DATA_LEN);
#else
	mavlink_traffic_data_t packet;
	mav_array_memcpy(packet.ID, ID, sizeof(char)*8);
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.heading = heading;
	packet.ownship_flag = ownship_flag;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRAFFIC_DATA, (const char*)&packet, MAVLINK_MSG_ID_TRAFFIC_DATA_LEN);
#endif
}

#endif // MAVLINK_USE_CONVENIENCE_FUNCTIONS

// MESSAGE TRAFFIC_DATA UNPACKING

/**
 * @brief Get field ID from traffic_data message
 *
 * @return vehicle ID
 */
static inline char mavlink_msg_traffic_data_get_ID(const mavlink_message_t* msg, char* ID)
{
	return _MAV_RETURN_char_array(msg,ID,8,0);
}

/**
 * @brief Get field lat from traffic_data message
 *
 * @return latitude (deg)
 */
static inline float mavlink_msg_traffic_data_get_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,8);
}

/**
 * @brief Get field lon from traffic_data message
 *
 * @return longitude (deg)
 */
static inline float mavlink_msg_traffic_data_get_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,12);
}

/**
 * @brief Get field alt from traffic_data message
 *
 * @return altitude (meters)
 */
static inline float mavlink_msg_traffic_data_get_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,16);
}

/**
 * @brief Get field roll from traffic_data message
 *
 * @return roll (deg)
 */
static inline float mavlink_msg_traffic_data_get_roll(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,20);
}

/**
 * @brief Get field pitch from traffic_data message
 *
 * @return pitch (deg)
 */
static inline float mavlink_msg_traffic_data_get_pitch(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,24);
}

/**
 * @brief Get field heading from traffic_data message
 *
 * @return heading (deg)
 */
static inline float mavlink_msg_traffic_data_get_heading(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,28);
}

/**
 * @brief Get field ownship_flag from traffic_data message
 *
 * @return identifies whether the aircraft is ownship (1) or not (0)
 */
static inline uint8_t mavlink_msg_traffic_data_get_ownship_flag(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,32);
}

/**
 * @brief Decode a traffic_data message into a struct
 *
 * @param msg The message to decode
 * @param traffic_data C-struct to decode the message contents into
 */
static inline void mavlink_msg_traffic_data_decode(const mavlink_message_t* msg, mavlink_traffic_data_t* traffic_data)
{
#if MAVLINK_NEED_BYTE_SWAP
	mavlink_msg_traffic_data_get_ID(msg, traffic_data->ID);
	traffic_data->lat = mavlink_msg_traffic_data_get_lat(msg);
	traffic_data->lon = mavlink_msg_traffic_data_get_lon(msg);
	traffic_data->alt = mavlink_msg_traffic_data_get_alt(msg);
	traffic_data->roll = mavlink_msg_traffic_data_get_roll(msg);
	traffic_data->pitch = mavlink_msg_traffic_data_get_pitch(msg);
	traffic_data->heading = mavlink_msg_traffic_data_get_heading(msg);
	traffic_data->ownship_flag = mavlink_msg_traffic_data_get_ownship_flag(msg);
#else
	memcpy(traffic_data, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_TRAFFIC_DATA_LEN);
#endif
}
