// MESSAGE RC_CHANNELS_SCALED PACKING
// Generated from parse_APM_XML.m on 19-Aug-2013 09:01:20

#define MAVLINK_MSG_ID_RC_CHANNELS_SCALED 34 

typedef struct __mavlink_rc_channels_scaled_t 
{ 
  uint32_t time_boot_ms;  ///< Timestamp (milliseconds since system boot)
  uint8_t port;  ///< Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows to encode more than 8 servos.
  int16_t chan1_scaled;  ///< RC channel 1 value scaled, (-100%%) -10000, (0%%) 0, (100%%) 10000
  int16_t chan2_scaled;  ///< RC channel 2 value scaled, (-100%%) -10000, (0%%) 0, (100%%) 10000
  int16_t chan3_scaled;  ///< RC channel 3 value scaled, (-100%%) -10000, (0%%) 0, (100%%) 10000
  int16_t chan4_scaled;  ///< RC channel 4 value scaled, (-100%%) -10000, (0%%) 0, (100%%) 10000
  int16_t chan5_scaled;  ///< RC channel 5 value scaled, (-100%%) -10000, (0%%) 0, (100%%) 10000
  int16_t chan6_scaled;  ///< RC channel 6 value scaled, (-100%%) -10000, (0%%) 0, (100%%) 10000
  int16_t chan7_scaled;  ///< RC channel 7 value scaled, (-100%%) -10000, (0%%) 0, (100%%) 10000
  int16_t chan8_scaled;  ///< RC channel 8 value scaled, (-100%%) -10000, (0%%) 0, (100%%) 10000
  uint8_t rssi;  ///< Receive signal strength indicator, 0: 0%%, 255: 100%%
} mavlink_rc_channels_scaled_t;

#define MAVLINK_MSG_ID_RC_CHANNELS_SCALED_LEN 22
#define MAVLINK_MSG_ID_34_LEN 22

#define MAVLINK_MESSAGE_INFO_RC_CHANNELS_SCALED \
{ \
  "RC_CHANNELS_SCALED", \
  11, \
  { \
    { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_rc_channels_scaled_t, time_boot_ms) }, \
    { "port", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_rc_channels_scaled_t, port) }, \
    { "chan1_scaled", NULL, MAVLINK_TYPE_INT16_T, 0, 5, offsetof(mavlink_rc_channels_scaled_t, chan1_scaled) }, \
    { "chan2_scaled", NULL, MAVLINK_TYPE_INT16_T, 0, 7, offsetof(mavlink_rc_channels_scaled_t, chan2_scaled) }, \
    { "chan3_scaled", NULL, MAVLINK_TYPE_INT16_T, 0, 9, offsetof(mavlink_rc_channels_scaled_t, chan3_scaled) }, \
    { "chan4_scaled", NULL, MAVLINK_TYPE_INT16_T, 0, 11, offsetof(mavlink_rc_channels_scaled_t, chan4_scaled) }, \
    { "chan5_scaled", NULL, MAVLINK_TYPE_INT16_T, 0, 13, offsetof(mavlink_rc_channels_scaled_t, chan5_scaled) }, \
    { "chan6_scaled", NULL, MAVLINK_TYPE_INT16_T, 0, 15, offsetof(mavlink_rc_channels_scaled_t, chan6_scaled) }, \
    { "chan7_scaled", NULL, MAVLINK_TYPE_INT16_T, 0, 17, offsetof(mavlink_rc_channels_scaled_t, chan7_scaled) }, \
    { "chan8_scaled", NULL, MAVLINK_TYPE_INT16_T, 0, 19, offsetof(mavlink_rc_channels_scaled_t, chan8_scaled) }, \
    { "rssi", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_rc_channels_scaled_t, rssi) }, \
  } \
}

/**
 * @brief Pack a rc_channels_scaled message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param port Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows to encode more than 8 servos.
 * @param chan1_scaled RC channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan2_scaled RC channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan3_scaled RC channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan4_scaled RC channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan5_scaled RC channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan6_scaled RC channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan7_scaled RC channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan8_scaled RC channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param rssi Receive signal strength indicator, 0: 0%, 255: 100%
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rc_channels_scaled_pack(
	 uint8_t system_id
	,uint8_t component_id
	,mavlink_message_t* msg
	,uint32_t time_boot_ms
	,uint8_t port
	,int16_t chan1_scaled
	,int16_t chan2_scaled
	,int16_t chan3_scaled
	,int16_t chan4_scaled
	,int16_t chan5_scaled
	,int16_t chan6_scaled
	,int16_t chan7_scaled
	,int16_t chan8_scaled
	,uint8_t rssi
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_RC_CHANNELS_SCALED_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_uint8_t(buf, 4, port);
	_mav_put_int16_t(buf, 5, chan1_scaled);
	_mav_put_int16_t(buf, 7, chan2_scaled);
	_mav_put_int16_t(buf, 9, chan3_scaled);
	_mav_put_int16_t(buf, 11, chan4_scaled);
	_mav_put_int16_t(buf, 13, chan5_scaled);
	_mav_put_int16_t(buf, 15, chan6_scaled);
	_mav_put_int16_t(buf, 17, chan7_scaled);
	_mav_put_int16_t(buf, 19, chan8_scaled);
	_mav_put_uint8_t(buf, 21, rssi);

	memcpy(_MAV_PAYLOAD(msg), buf, MAVLINK_MSG_ID_RC_CHANNELS_SCALED_LEN);
#else
	mavlink_rc_channels_scaled_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.port = port;
	packet.chan1_scaled = chan1_scaled;
	packet.chan2_scaled = chan2_scaled;
	packet.chan3_scaled = chan3_scaled;
	packet.chan4_scaled = chan4_scaled;
	packet.chan5_scaled = chan5_scaled;
	packet.chan6_scaled = chan6_scaled;
	packet.chan7_scaled = chan7_scaled;
	packet.chan8_scaled = chan8_scaled;
	packet.rssi = rssi;

	memcpy(_MAV_PAYLOAD(msg), &packet, MAVLINK_MSG_ID_RC_CHANNELS_SCALED_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_RC_CHANNELS_SCALED;
	return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RC_CHANNELS_SCALED_LEN);
}

/**
 * @brief Pack a rc_channels_scaled message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param port Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows to encode more than 8 servos.
 * @param chan1_scaled RC channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan2_scaled RC channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan3_scaled RC channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan4_scaled RC channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan5_scaled RC channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan6_scaled RC channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan7_scaled RC channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan8_scaled RC channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param rssi Receive signal strength indicator, 0: 0%, 255: 100%
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rc_channels_scaled_pack_chan(
	 uint8_t system_id
	,uint8_t component_id
	,uint8_t chan
	,mavlink_message_t* msg
	,uint32_t time_boot_ms
	,uint8_t port
	,int16_t chan1_scaled
	,int16_t chan2_scaled
	,int16_t chan3_scaled
	,int16_t chan4_scaled
	,int16_t chan5_scaled
	,int16_t chan6_scaled
	,int16_t chan7_scaled
	,int16_t chan8_scaled
	,uint8_t rssi
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_RC_CHANNELS_SCALED_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_uint8_t(buf, 4, port);
	_mav_put_int16_t(buf, 5, chan1_scaled);
	_mav_put_int16_t(buf, 7, chan2_scaled);
	_mav_put_int16_t(buf, 9, chan3_scaled);
	_mav_put_int16_t(buf, 11, chan4_scaled);
	_mav_put_int16_t(buf, 13, chan5_scaled);
	_mav_put_int16_t(buf, 15, chan6_scaled);
	_mav_put_int16_t(buf, 17, chan7_scaled);
	_mav_put_int16_t(buf, 19, chan8_scaled);
	_mav_put_uint8_t(buf, 21, rssi);

	memcpy(_MAV_PAYLOAD(msg), buf, MAVLINK_MSG_ID_RC_CHANNELS_SCALED_LEN);
#else
	mavlink_rc_channels_scaled_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.port = port;
	packet.chan1_scaled = chan1_scaled;
	packet.chan2_scaled = chan2_scaled;
	packet.chan3_scaled = chan3_scaled;
	packet.chan4_scaled = chan4_scaled;
	packet.chan5_scaled = chan5_scaled;
	packet.chan6_scaled = chan6_scaled;
	packet.chan7_scaled = chan7_scaled;
	packet.chan8_scaled = chan8_scaled;
	packet.rssi = rssi;

	memcpy(_MAV_PAYLOAD(msg), &packet, MAVLINK_MSG_ID_RC_CHANNELS_SCALED_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_RC_CHANNELS_SCALED;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RC_CHANNELS_SCALED_LEN);
}

/**
 * @brief Encode a rc_channels_scaled struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param vfr_hud C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rc_channels_scaled_encode(
	 uint8_t system_id
	,uint8_t component_id
	,mavlink_message_t* msg
	,const mavlink_rc_channels_scaled_t* rc_channels_scaled
	)
{
	return mavlink_msg_rc_channels_scaled_pack(
	 system_id
	,component_id
	,msg
	,rc_channels_scaled->time_boot_ms
	,rc_channels_scaled->port
	,rc_channels_scaled->chan1_scaled
	,rc_channels_scaled->chan2_scaled
	,rc_channels_scaled->chan3_scaled
	,rc_channels_scaled->chan4_scaled
	,rc_channels_scaled->chan5_scaled
	,rc_channels_scaled->chan6_scaled
	,rc_channels_scaled->chan7_scaled
	,rc_channels_scaled->chan8_scaled
	,rc_channels_scaled->rssi
	);
}

/**
 * @brief Send a rc_channels_scaled message
 * @param chan The MAVLink channel to send this message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param port Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows to encode more than 8 servos.
 * @param chan1_scaled RC channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan2_scaled RC channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan3_scaled RC channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan4_scaled RC channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan5_scaled RC channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan6_scaled RC channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan7_scaled RC channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan8_scaled RC channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param rssi Receive signal strength indicator, 0: 0%, 255: 100%
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rc_channels_scaled_send(
	 mavlink_channel_t chan
	,uint32_t time_boot_ms
	,uint8_t port
	,int16_t chan1_scaled
	,int16_t chan2_scaled
	,int16_t chan3_scaled
	,int16_t chan4_scaled
	,int16_t chan5_scaled
	,int16_t chan6_scaled
	,int16_t chan7_scaled
	,int16_t chan8_scaled
	,uint8_t rssi
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_RC_CHANNELS_SCALED_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_uint8_t(buf, 4, port);
	_mav_put_int16_t(buf, 5, chan1_scaled);
	_mav_put_int16_t(buf, 7, chan2_scaled);
	_mav_put_int16_t(buf, 9, chan3_scaled);
	_mav_put_int16_t(buf, 11, chan4_scaled);
	_mav_put_int16_t(buf, 13, chan5_scaled);
	_mav_put_int16_t(buf, 15, chan6_scaled);
	_mav_put_int16_t(buf, 17, chan7_scaled);
	_mav_put_int16_t(buf, 19, chan8_scaled);
	_mav_put_uint8_t(buf, 21, rssi);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_CHANNELS_SCALED, buf, MAVLINK_MSG_ID_RC_CHANNELS_SCALED_LEN);
#else
	mavlink_rc_channels_scaled_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.port = port;
	packet.chan1_scaled = chan1_scaled;
	packet.chan2_scaled = chan2_scaled;
	packet.chan3_scaled = chan3_scaled;
	packet.chan4_scaled = chan4_scaled;
	packet.chan5_scaled = chan5_scaled;
	packet.chan6_scaled = chan6_scaled;
	packet.chan7_scaled = chan7_scaled;
	packet.chan8_scaled = chan8_scaled;
	packet.rssi = rssi;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_CHANNELS_SCALED, (const char*)&packet, MAVLINK_MSG_ID_RC_CHANNELS_SCALED_LEN);
#endif
}

#endif // MAVLINK_USE_CONVENIENCE_FUNCTIONS

// MESSAGE RC_CHANNELS_SCALED UNPACKING

/**
 * @brief Get field time_boot_ms from rc_channels_scaled message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline uint32_t mavlink_msg_rc_channels_scaled_get_time_boot_ms(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,0);
}

/**
 * @brief Get field port from rc_channels_scaled message
 *
 * @return Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows to encode more than 8 servos.
 */
static inline uint8_t mavlink_msg_rc_channels_scaled_get_port(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,4);
}

/**
 * @brief Get field chan1_scaled from rc_channels_scaled message
 *
 * @return RC channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
static inline int16_t mavlink_msg_rc_channels_scaled_get_chan1_scaled(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,5);
}

/**
 * @brief Get field chan2_scaled from rc_channels_scaled message
 *
 * @return RC channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
static inline int16_t mavlink_msg_rc_channels_scaled_get_chan2_scaled(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,7);
}

/**
 * @brief Get field chan3_scaled from rc_channels_scaled message
 *
 * @return RC channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
static inline int16_t mavlink_msg_rc_channels_scaled_get_chan3_scaled(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,9);
}

/**
 * @brief Get field chan4_scaled from rc_channels_scaled message
 *
 * @return RC channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
static inline int16_t mavlink_msg_rc_channels_scaled_get_chan4_scaled(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,11);
}

/**
 * @brief Get field chan5_scaled from rc_channels_scaled message
 *
 * @return RC channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
static inline int16_t mavlink_msg_rc_channels_scaled_get_chan5_scaled(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,13);
}

/**
 * @brief Get field chan6_scaled from rc_channels_scaled message
 *
 * @return RC channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
static inline int16_t mavlink_msg_rc_channels_scaled_get_chan6_scaled(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,15);
}

/**
 * @brief Get field chan7_scaled from rc_channels_scaled message
 *
 * @return RC channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
static inline int16_t mavlink_msg_rc_channels_scaled_get_chan7_scaled(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,17);
}

/**
 * @brief Get field chan8_scaled from rc_channels_scaled message
 *
 * @return RC channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
static inline int16_t mavlink_msg_rc_channels_scaled_get_chan8_scaled(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,19);
}

/**
 * @brief Get field rssi from rc_channels_scaled message
 *
 * @return Receive signal strength indicator, 0: 0%, 255: 100%
 */
static inline uint8_t mavlink_msg_rc_channels_scaled_get_rssi(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,21);
}

/**
 * @brief Decode a rc_channels_scaled message into a struct
 *
 * @param msg The message to decode
 * @param rc_channels_scaled C-struct to decode the message contents into
 */
static inline void mavlink_msg_rc_channels_scaled_decode(const mavlink_message_t* msg, mavlink_rc_channels_scaled_t* rc_channels_scaled)
{
#if MAVLINK_NEED_BYTE_SWAP
	rc_channels_scaled->time_boot_ms = mavlink_msg_rc_channels_scaled_get_time_boot_ms(msg);
	rc_channels_scaled->port = mavlink_msg_rc_channels_scaled_get_port(msg);
	rc_channels_scaled->chan1_scaled = mavlink_msg_rc_channels_scaled_get_chan1_scaled(msg);
	rc_channels_scaled->chan2_scaled = mavlink_msg_rc_channels_scaled_get_chan2_scaled(msg);
	rc_channels_scaled->chan3_scaled = mavlink_msg_rc_channels_scaled_get_chan3_scaled(msg);
	rc_channels_scaled->chan4_scaled = mavlink_msg_rc_channels_scaled_get_chan4_scaled(msg);
	rc_channels_scaled->chan5_scaled = mavlink_msg_rc_channels_scaled_get_chan5_scaled(msg);
	rc_channels_scaled->chan6_scaled = mavlink_msg_rc_channels_scaled_get_chan6_scaled(msg);
	rc_channels_scaled->chan7_scaled = mavlink_msg_rc_channels_scaled_get_chan7_scaled(msg);
	rc_channels_scaled->chan8_scaled = mavlink_msg_rc_channels_scaled_get_chan8_scaled(msg);
	rc_channels_scaled->rssi = mavlink_msg_rc_channels_scaled_get_rssi(msg);
#else
	memcpy(rc_channels_scaled, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_RC_CHANNELS_SCALED_LEN);
#endif
}
