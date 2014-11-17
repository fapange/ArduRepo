// MESSAGE ANALOG_EU PACKING
// Generated from parse_APM_XML.m on 19-Aug-2013 09:01:23

#define MAVLINK_MSG_ID_ANALOG_EU 112 

typedef struct __mavlink_analog_eu_t 
{ 
  float chan01;  ///< adc channel_01
  float chan02;  ///< adc channel_02
  float chan03;  ///< adc channel_03
  float chan04;  ///< adc channel_04
  float chan05;  ///< adc channel_05
  float chan06;  ///< adc channel_06
  float chan07;  ///< adc channel_07
  float chan08;  ///< adc channel_08
  float chan09;  ///< adc channel_09
  float chan10;  ///< adc channel_10
  float chan11;  ///< adc channel_11
  float chan12;  ///< adc channel_12
  float chan13;  ///< adc channel_13
  float chan14;  ///< adc channel_14
  float chan15;  ///< adc channel_15
  float chan16;  ///< adc channel_16
  uint64_t usec;  ///< time
} mavlink_analog_eu_t;

#define MAVLINK_MSG_ID_ANALOG_EU_LEN 72
#define MAVLINK_MSG_ID_112_LEN 72

#define MAVLINK_MESSAGE_INFO_ANALOG_EU \
{ \
  "ANALOG_EU", \
  17, \
  { \
    { "chan01", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_analog_eu_t, chan01) }, \
    { "chan02", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_analog_eu_t, chan02) }, \
    { "chan03", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_analog_eu_t, chan03) }, \
    { "chan04", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_analog_eu_t, chan04) }, \
    { "chan05", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_analog_eu_t, chan05) }, \
    { "chan06", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_analog_eu_t, chan06) }, \
    { "chan07", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_analog_eu_t, chan07) }, \
    { "chan08", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_analog_eu_t, chan08) }, \
    { "chan09", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_analog_eu_t, chan09) }, \
    { "chan10", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_analog_eu_t, chan10) }, \
    { "chan11", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_analog_eu_t, chan11) }, \
    { "chan12", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_analog_eu_t, chan12) }, \
    { "chan13", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_analog_eu_t, chan13) }, \
    { "chan14", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_analog_eu_t, chan14) }, \
    { "chan15", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_analog_eu_t, chan15) }, \
    { "chan16", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_analog_eu_t, chan16) }, \
    { "usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 64, offsetof(mavlink_analog_eu_t, usec) }, \
  } \
}

/**
 * @brief Pack a analog_eu message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param chan01 adc channel_01
 * @param chan02 adc channel_02
 * @param chan03 adc channel_03
 * @param chan04 adc channel_04
 * @param chan05 adc channel_05
 * @param chan06 adc channel_06
 * @param chan07 adc channel_07
 * @param chan08 adc channel_08
 * @param chan09 adc channel_09
 * @param chan10 adc channel_10
 * @param chan11 adc channel_11
 * @param chan12 adc channel_12
 * @param chan13 adc channel_13
 * @param chan14 adc channel_14
 * @param chan15 adc channel_15
 * @param chan16 adc channel_16
 * @param usec time
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_analog_eu_pack(
	 uint8_t system_id
	,uint8_t component_id
	,mavlink_message_t* msg
	,float chan01
	,float chan02
	,float chan03
	,float chan04
	,float chan05
	,float chan06
	,float chan07
	,float chan08
	,float chan09
	,float chan10
	,float chan11
	,float chan12
	,float chan13
	,float chan14
	,float chan15
	,float chan16
	,uint64_t usec
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ANALOG_EU_LEN];
	_mav_put_float(buf, 0, chan01);
	_mav_put_float(buf, 4, chan02);
	_mav_put_float(buf, 8, chan03);
	_mav_put_float(buf, 12, chan04);
	_mav_put_float(buf, 16, chan05);
	_mav_put_float(buf, 20, chan06);
	_mav_put_float(buf, 24, chan07);
	_mav_put_float(buf, 28, chan08);
	_mav_put_float(buf, 32, chan09);
	_mav_put_float(buf, 36, chan10);
	_mav_put_float(buf, 40, chan11);
	_mav_put_float(buf, 44, chan12);
	_mav_put_float(buf, 48, chan13);
	_mav_put_float(buf, 52, chan14);
	_mav_put_float(buf, 56, chan15);
	_mav_put_float(buf, 60, chan16);
	_mav_put_uint64_t(buf, 64, usec);

	memcpy(_MAV_PAYLOAD(msg), buf, MAVLINK_MSG_ID_ANALOG_EU_LEN);
#else
	mavlink_analog_eu_t packet;
	packet.chan01 = chan01;
	packet.chan02 = chan02;
	packet.chan03 = chan03;
	packet.chan04 = chan04;
	packet.chan05 = chan05;
	packet.chan06 = chan06;
	packet.chan07 = chan07;
	packet.chan08 = chan08;
	packet.chan09 = chan09;
	packet.chan10 = chan10;
	packet.chan11 = chan11;
	packet.chan12 = chan12;
	packet.chan13 = chan13;
	packet.chan14 = chan14;
	packet.chan15 = chan15;
	packet.chan16 = chan16;
	packet.usec = usec;

	memcpy(_MAV_PAYLOAD(msg), &packet, MAVLINK_MSG_ID_ANALOG_EU_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ANALOG_EU;
	return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ANALOG_EU_LEN);
}

/**
 * @brief Pack a analog_eu message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 *
 * @param chan01 adc channel_01
 * @param chan02 adc channel_02
 * @param chan03 adc channel_03
 * @param chan04 adc channel_04
 * @param chan05 adc channel_05
 * @param chan06 adc channel_06
 * @param chan07 adc channel_07
 * @param chan08 adc channel_08
 * @param chan09 adc channel_09
 * @param chan10 adc channel_10
 * @param chan11 adc channel_11
 * @param chan12 adc channel_12
 * @param chan13 adc channel_13
 * @param chan14 adc channel_14
 * @param chan15 adc channel_15
 * @param chan16 adc channel_16
 * @param usec time
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_analog_eu_pack_chan(
	 uint8_t system_id
	,uint8_t component_id
	,uint8_t chan
	,mavlink_message_t* msg
	,float chan01
	,float chan02
	,float chan03
	,float chan04
	,float chan05
	,float chan06
	,float chan07
	,float chan08
	,float chan09
	,float chan10
	,float chan11
	,float chan12
	,float chan13
	,float chan14
	,float chan15
	,float chan16
	,uint64_t usec
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ANALOG_EU_LEN];
	_mav_put_float(buf, 0, chan01);
	_mav_put_float(buf, 4, chan02);
	_mav_put_float(buf, 8, chan03);
	_mav_put_float(buf, 12, chan04);
	_mav_put_float(buf, 16, chan05);
	_mav_put_float(buf, 20, chan06);
	_mav_put_float(buf, 24, chan07);
	_mav_put_float(buf, 28, chan08);
	_mav_put_float(buf, 32, chan09);
	_mav_put_float(buf, 36, chan10);
	_mav_put_float(buf, 40, chan11);
	_mav_put_float(buf, 44, chan12);
	_mav_put_float(buf, 48, chan13);
	_mav_put_float(buf, 52, chan14);
	_mav_put_float(buf, 56, chan15);
	_mav_put_float(buf, 60, chan16);
	_mav_put_uint64_t(buf, 64, usec);

	memcpy(_MAV_PAYLOAD(msg), buf, MAVLINK_MSG_ID_ANALOG_EU_LEN);
#else
	mavlink_analog_eu_t packet;
	packet.chan01 = chan01;
	packet.chan02 = chan02;
	packet.chan03 = chan03;
	packet.chan04 = chan04;
	packet.chan05 = chan05;
	packet.chan06 = chan06;
	packet.chan07 = chan07;
	packet.chan08 = chan08;
	packet.chan09 = chan09;
	packet.chan10 = chan10;
	packet.chan11 = chan11;
	packet.chan12 = chan12;
	packet.chan13 = chan13;
	packet.chan14 = chan14;
	packet.chan15 = chan15;
	packet.chan16 = chan16;
	packet.usec = usec;

	memcpy(_MAV_PAYLOAD(msg), &packet, MAVLINK_MSG_ID_ANALOG_EU_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ANALOG_EU;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ANALOG_EU_LEN);
}

/**
 * @brief Encode a analog_eu struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param vfr_hud C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_analog_eu_encode(
	 uint8_t system_id
	,uint8_t component_id
	,mavlink_message_t* msg
	,const mavlink_analog_eu_t* analog_eu
	)
{
	return mavlink_msg_analog_eu_pack(
	 system_id
	,component_id
	,msg
	,analog_eu->chan01
	,analog_eu->chan02
	,analog_eu->chan03
	,analog_eu->chan04
	,analog_eu->chan05
	,analog_eu->chan06
	,analog_eu->chan07
	,analog_eu->chan08
	,analog_eu->chan09
	,analog_eu->chan10
	,analog_eu->chan11
	,analog_eu->chan12
	,analog_eu->chan13
	,analog_eu->chan14
	,analog_eu->chan15
	,analog_eu->chan16
	,analog_eu->usec
	);
}

/**
 * @brief Send a analog_eu message
 * @param chan The MAVLink channel to send this message
 *
 * @param chan01 adc channel_01
 * @param chan02 adc channel_02
 * @param chan03 adc channel_03
 * @param chan04 adc channel_04
 * @param chan05 adc channel_05
 * @param chan06 adc channel_06
 * @param chan07 adc channel_07
 * @param chan08 adc channel_08
 * @param chan09 adc channel_09
 * @param chan10 adc channel_10
 * @param chan11 adc channel_11
 * @param chan12 adc channel_12
 * @param chan13 adc channel_13
 * @param chan14 adc channel_14
 * @param chan15 adc channel_15
 * @param chan16 adc channel_16
 * @param usec time
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_analog_eu_send(
	 mavlink_channel_t chan
	,float chan01
	,float chan02
	,float chan03
	,float chan04
	,float chan05
	,float chan06
	,float chan07
	,float chan08
	,float chan09
	,float chan10
	,float chan11
	,float chan12
	,float chan13
	,float chan14
	,float chan15
	,float chan16
	,uint64_t usec
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ANALOG_EU_LEN];
	_mav_put_float(buf, 0, chan01);
	_mav_put_float(buf, 4, chan02);
	_mav_put_float(buf, 8, chan03);
	_mav_put_float(buf, 12, chan04);
	_mav_put_float(buf, 16, chan05);
	_mav_put_float(buf, 20, chan06);
	_mav_put_float(buf, 24, chan07);
	_mav_put_float(buf, 28, chan08);
	_mav_put_float(buf, 32, chan09);
	_mav_put_float(buf, 36, chan10);
	_mav_put_float(buf, 40, chan11);
	_mav_put_float(buf, 44, chan12);
	_mav_put_float(buf, 48, chan13);
	_mav_put_float(buf, 52, chan14);
	_mav_put_float(buf, 56, chan15);
	_mav_put_float(buf, 60, chan16);
	_mav_put_uint64_t(buf, 64, usec);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ANALOG_EU, buf, MAVLINK_MSG_ID_ANALOG_EU_LEN);
#else
	mavlink_analog_eu_t packet;
	packet.chan01 = chan01;
	packet.chan02 = chan02;
	packet.chan03 = chan03;
	packet.chan04 = chan04;
	packet.chan05 = chan05;
	packet.chan06 = chan06;
	packet.chan07 = chan07;
	packet.chan08 = chan08;
	packet.chan09 = chan09;
	packet.chan10 = chan10;
	packet.chan11 = chan11;
	packet.chan12 = chan12;
	packet.chan13 = chan13;
	packet.chan14 = chan14;
	packet.chan15 = chan15;
	packet.chan16 = chan16;
	packet.usec = usec;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ANALOG_EU, (const char*)&packet, MAVLINK_MSG_ID_ANALOG_EU_LEN);
#endif
}

#endif // MAVLINK_USE_CONVENIENCE_FUNCTIONS

// MESSAGE ANALOG_EU UNPACKING

/**
 * @brief Get field chan01 from analog_eu message
 *
 * @return adc channel_01
 */
static inline float mavlink_msg_analog_eu_get_chan01(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,0);
}

/**
 * @brief Get field chan02 from analog_eu message
 *
 * @return adc channel_02
 */
static inline float mavlink_msg_analog_eu_get_chan02(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,4);
}

/**
 * @brief Get field chan03 from analog_eu message
 *
 * @return adc channel_03
 */
static inline float mavlink_msg_analog_eu_get_chan03(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,8);
}

/**
 * @brief Get field chan04 from analog_eu message
 *
 * @return adc channel_04
 */
static inline float mavlink_msg_analog_eu_get_chan04(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,12);
}

/**
 * @brief Get field chan05 from analog_eu message
 *
 * @return adc channel_05
 */
static inline float mavlink_msg_analog_eu_get_chan05(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,16);
}

/**
 * @brief Get field chan06 from analog_eu message
 *
 * @return adc channel_06
 */
static inline float mavlink_msg_analog_eu_get_chan06(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,20);
}

/**
 * @brief Get field chan07 from analog_eu message
 *
 * @return adc channel_07
 */
static inline float mavlink_msg_analog_eu_get_chan07(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,24);
}

/**
 * @brief Get field chan08 from analog_eu message
 *
 * @return adc channel_08
 */
static inline float mavlink_msg_analog_eu_get_chan08(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,28);
}

/**
 * @brief Get field chan09 from analog_eu message
 *
 * @return adc channel_09
 */
static inline float mavlink_msg_analog_eu_get_chan09(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,32);
}

/**
 * @brief Get field chan10 from analog_eu message
 *
 * @return adc channel_10
 */
static inline float mavlink_msg_analog_eu_get_chan10(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,36);
}

/**
 * @brief Get field chan11 from analog_eu message
 *
 * @return adc channel_11
 */
static inline float mavlink_msg_analog_eu_get_chan11(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,40);
}

/**
 * @brief Get field chan12 from analog_eu message
 *
 * @return adc channel_12
 */
static inline float mavlink_msg_analog_eu_get_chan12(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,44);
}

/**
 * @brief Get field chan13 from analog_eu message
 *
 * @return adc channel_13
 */
static inline float mavlink_msg_analog_eu_get_chan13(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,48);
}

/**
 * @brief Get field chan14 from analog_eu message
 *
 * @return adc channel_14
 */
static inline float mavlink_msg_analog_eu_get_chan14(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,52);
}

/**
 * @brief Get field chan15 from analog_eu message
 *
 * @return adc channel_15
 */
static inline float mavlink_msg_analog_eu_get_chan15(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,56);
}

/**
 * @brief Get field chan16 from analog_eu message
 *
 * @return adc channel_16
 */
static inline float mavlink_msg_analog_eu_get_chan16(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,60);
}

/**
 * @brief Get field usec from analog_eu message
 *
 * @return time
 */
static inline uint64_t mavlink_msg_analog_eu_get_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,64);
}

/**
 * @brief Decode a analog_eu message into a struct
 *
 * @param msg The message to decode
 * @param analog_eu C-struct to decode the message contents into
 */
static inline void mavlink_msg_analog_eu_decode(const mavlink_message_t* msg, mavlink_analog_eu_t* analog_eu)
{
#if MAVLINK_NEED_BYTE_SWAP
	analog_eu->chan01 = mavlink_msg_analog_eu_get_chan01(msg);
	analog_eu->chan02 = mavlink_msg_analog_eu_get_chan02(msg);
	analog_eu->chan03 = mavlink_msg_analog_eu_get_chan03(msg);
	analog_eu->chan04 = mavlink_msg_analog_eu_get_chan04(msg);
	analog_eu->chan05 = mavlink_msg_analog_eu_get_chan05(msg);
	analog_eu->chan06 = mavlink_msg_analog_eu_get_chan06(msg);
	analog_eu->chan07 = mavlink_msg_analog_eu_get_chan07(msg);
	analog_eu->chan08 = mavlink_msg_analog_eu_get_chan08(msg);
	analog_eu->chan09 = mavlink_msg_analog_eu_get_chan09(msg);
	analog_eu->chan10 = mavlink_msg_analog_eu_get_chan10(msg);
	analog_eu->chan11 = mavlink_msg_analog_eu_get_chan11(msg);
	analog_eu->chan12 = mavlink_msg_analog_eu_get_chan12(msg);
	analog_eu->chan13 = mavlink_msg_analog_eu_get_chan13(msg);
	analog_eu->chan14 = mavlink_msg_analog_eu_get_chan14(msg);
	analog_eu->chan15 = mavlink_msg_analog_eu_get_chan15(msg);
	analog_eu->chan16 = mavlink_msg_analog_eu_get_chan16(msg);
	analog_eu->usec = mavlink_msg_analog_eu_get_usec(msg);
#else
	memcpy(analog_eu, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_ANALOG_EU_LEN);
#endif
}
