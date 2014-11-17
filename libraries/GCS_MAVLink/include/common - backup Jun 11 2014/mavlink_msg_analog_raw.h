// MESSAGE ANALOG_RAW PACKING
// Generated from parse_APM_XML.m on 19-Aug-2013 09:01:23

#define MAVLINK_MSG_ID_ANALOG_RAW 111 

typedef struct __mavlink_analog_raw_t 
{ 
  uint16_t chan01;  ///< adc channel_01
  uint16_t chan02;  ///< adc channel_02
  uint16_t chan03;  ///< adc channel_03
  uint16_t chan04;  ///< adc channel_04
  uint16_t chan05;  ///< adc channel_05
  uint16_t chan06;  ///< adc channel_06
  uint16_t chan07;  ///< adc channel_07
  uint16_t chan08;  ///< adc channel_08
  uint16_t chan09;  ///< adc channel_09
  uint16_t chan10;  ///< adc channel_10
  uint16_t chan11;  ///< adc channel_11
  uint16_t chan12;  ///< adc channel_12
  uint16_t chan13;  ///< adc channel_13
  uint16_t chan14;  ///< adc channel_14
  uint16_t chan15;  ///< adc channel_15
  uint16_t chan16;  ///< adc channel_16
  uint64_t usec;  ///< time
} mavlink_analog_raw_t;

#define MAVLINK_MSG_ID_ANALOG_RAW_LEN 40
#define MAVLINK_MSG_ID_111_LEN 40

#define MAVLINK_MESSAGE_INFO_ANALOG_RAW \
{ \
  "ANALOG_RAW", \
  17, \
  { \
    { "chan01", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_analog_raw_t, chan01) }, \
    { "chan02", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_analog_raw_t, chan02) }, \
    { "chan03", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_analog_raw_t, chan03) }, \
    { "chan04", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_analog_raw_t, chan04) }, \
    { "chan05", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_analog_raw_t, chan05) }, \
    { "chan06", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_analog_raw_t, chan06) }, \
    { "chan07", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_analog_raw_t, chan07) }, \
    { "chan08", NULL, MAVLINK_TYPE_UINT16_T, 0, 14, offsetof(mavlink_analog_raw_t, chan08) }, \
    { "chan09", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_analog_raw_t, chan09) }, \
    { "chan10", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_analog_raw_t, chan10) }, \
    { "chan11", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_analog_raw_t, chan11) }, \
    { "chan12", NULL, MAVLINK_TYPE_UINT16_T, 0, 22, offsetof(mavlink_analog_raw_t, chan12) }, \
    { "chan13", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_analog_raw_t, chan13) }, \
    { "chan14", NULL, MAVLINK_TYPE_UINT16_T, 0, 26, offsetof(mavlink_analog_raw_t, chan14) }, \
    { "chan15", NULL, MAVLINK_TYPE_UINT16_T, 0, 28, offsetof(mavlink_analog_raw_t, chan15) }, \
    { "chan16", NULL, MAVLINK_TYPE_UINT16_T, 0, 30, offsetof(mavlink_analog_raw_t, chan16) }, \
    { "usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 32, offsetof(mavlink_analog_raw_t, usec) }, \
  } \
}

/**
 * @brief Pack a analog_raw message
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
static inline uint16_t mavlink_msg_analog_raw_pack(
	 uint8_t system_id
	,uint8_t component_id
	,mavlink_message_t* msg
	,uint16_t chan01
	,uint16_t chan02
	,uint16_t chan03
	,uint16_t chan04
	,uint16_t chan05
	,uint16_t chan06
	,uint16_t chan07
	,uint16_t chan08
	,uint16_t chan09
	,uint16_t chan10
	,uint16_t chan11
	,uint16_t chan12
	,uint16_t chan13
	,uint16_t chan14
	,uint16_t chan15
	,uint16_t chan16
	,uint64_t usec
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ANALOG_RAW_LEN];
	_mav_put_uint16_t(buf, 0, chan01);
	_mav_put_uint16_t(buf, 2, chan02);
	_mav_put_uint16_t(buf, 4, chan03);
	_mav_put_uint16_t(buf, 6, chan04);
	_mav_put_uint16_t(buf, 8, chan05);
	_mav_put_uint16_t(buf, 10, chan06);
	_mav_put_uint16_t(buf, 12, chan07);
	_mav_put_uint16_t(buf, 14, chan08);
	_mav_put_uint16_t(buf, 16, chan09);
	_mav_put_uint16_t(buf, 18, chan10);
	_mav_put_uint16_t(buf, 20, chan11);
	_mav_put_uint16_t(buf, 22, chan12);
	_mav_put_uint16_t(buf, 24, chan13);
	_mav_put_uint16_t(buf, 26, chan14);
	_mav_put_uint16_t(buf, 28, chan15);
	_mav_put_uint16_t(buf, 30, chan16);
	_mav_put_uint64_t(buf, 32, usec);

	memcpy(_MAV_PAYLOAD(msg), buf, MAVLINK_MSG_ID_ANALOG_RAW_LEN);
#else
	mavlink_analog_raw_t packet;
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

	memcpy(_MAV_PAYLOAD(msg), &packet, MAVLINK_MSG_ID_ANALOG_RAW_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ANALOG_RAW;
	return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ANALOG_RAW_LEN);
}

/**
 * @brief Pack a analog_raw message on a channel
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
static inline uint16_t mavlink_msg_analog_raw_pack_chan(
	 uint8_t system_id
	,uint8_t component_id
	,uint8_t chan
	,mavlink_message_t* msg
	,uint16_t chan01
	,uint16_t chan02
	,uint16_t chan03
	,uint16_t chan04
	,uint16_t chan05
	,uint16_t chan06
	,uint16_t chan07
	,uint16_t chan08
	,uint16_t chan09
	,uint16_t chan10
	,uint16_t chan11
	,uint16_t chan12
	,uint16_t chan13
	,uint16_t chan14
	,uint16_t chan15
	,uint16_t chan16
	,uint64_t usec
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ANALOG_RAW_LEN];
	_mav_put_uint16_t(buf, 0, chan01);
	_mav_put_uint16_t(buf, 2, chan02);
	_mav_put_uint16_t(buf, 4, chan03);
	_mav_put_uint16_t(buf, 6, chan04);
	_mav_put_uint16_t(buf, 8, chan05);
	_mav_put_uint16_t(buf, 10, chan06);
	_mav_put_uint16_t(buf, 12, chan07);
	_mav_put_uint16_t(buf, 14, chan08);
	_mav_put_uint16_t(buf, 16, chan09);
	_mav_put_uint16_t(buf, 18, chan10);
	_mav_put_uint16_t(buf, 20, chan11);
	_mav_put_uint16_t(buf, 22, chan12);
	_mav_put_uint16_t(buf, 24, chan13);
	_mav_put_uint16_t(buf, 26, chan14);
	_mav_put_uint16_t(buf, 28, chan15);
	_mav_put_uint16_t(buf, 30, chan16);
	_mav_put_uint64_t(buf, 32, usec);

	memcpy(_MAV_PAYLOAD(msg), buf, MAVLINK_MSG_ID_ANALOG_RAW_LEN);
#else
	mavlink_analog_raw_t packet;
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

	memcpy(_MAV_PAYLOAD(msg), &packet, MAVLINK_MSG_ID_ANALOG_RAW_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ANALOG_RAW;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ANALOG_RAW_LEN);
}

/**
 * @brief Encode a analog_raw struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param vfr_hud C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_analog_raw_encode(
	 uint8_t system_id
	,uint8_t component_id
	,mavlink_message_t* msg
	,const mavlink_analog_raw_t* analog_raw
	)
{
	return mavlink_msg_analog_raw_pack(
	 system_id
	,component_id
	,msg
	,analog_raw->chan01
	,analog_raw->chan02
	,analog_raw->chan03
	,analog_raw->chan04
	,analog_raw->chan05
	,analog_raw->chan06
	,analog_raw->chan07
	,analog_raw->chan08
	,analog_raw->chan09
	,analog_raw->chan10
	,analog_raw->chan11
	,analog_raw->chan12
	,analog_raw->chan13
	,analog_raw->chan14
	,analog_raw->chan15
	,analog_raw->chan16
	,analog_raw->usec
	);
}

/**
 * @brief Send a analog_raw message
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

static inline void mavlink_msg_analog_raw_send(
	 mavlink_channel_t chan
	,uint16_t chan01
	,uint16_t chan02
	,uint16_t chan03
	,uint16_t chan04
	,uint16_t chan05
	,uint16_t chan06
	,uint16_t chan07
	,uint16_t chan08
	,uint16_t chan09
	,uint16_t chan10
	,uint16_t chan11
	,uint16_t chan12
	,uint16_t chan13
	,uint16_t chan14
	,uint16_t chan15
	,uint16_t chan16
	,uint64_t usec
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ANALOG_RAW_LEN];
	_mav_put_uint16_t(buf, 0, chan01);
	_mav_put_uint16_t(buf, 2, chan02);
	_mav_put_uint16_t(buf, 4, chan03);
	_mav_put_uint16_t(buf, 6, chan04);
	_mav_put_uint16_t(buf, 8, chan05);
	_mav_put_uint16_t(buf, 10, chan06);
	_mav_put_uint16_t(buf, 12, chan07);
	_mav_put_uint16_t(buf, 14, chan08);
	_mav_put_uint16_t(buf, 16, chan09);
	_mav_put_uint16_t(buf, 18, chan10);
	_mav_put_uint16_t(buf, 20, chan11);
	_mav_put_uint16_t(buf, 22, chan12);
	_mav_put_uint16_t(buf, 24, chan13);
	_mav_put_uint16_t(buf, 26, chan14);
	_mav_put_uint16_t(buf, 28, chan15);
	_mav_put_uint16_t(buf, 30, chan16);
	_mav_put_uint64_t(buf, 32, usec);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ANALOG_RAW, buf, MAVLINK_MSG_ID_ANALOG_RAW_LEN);
#else
	mavlink_analog_raw_t packet;
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

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ANALOG_RAW, (const char*)&packet, MAVLINK_MSG_ID_ANALOG_RAW_LEN);
#endif
}

#endif // MAVLINK_USE_CONVENIENCE_FUNCTIONS

// MESSAGE ANALOG_RAW UNPACKING

/**
 * @brief Get field chan01 from analog_raw message
 *
 * @return adc channel_01
 */
static inline uint16_t mavlink_msg_analog_raw_get_chan01(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,0);
}

/**
 * @brief Get field chan02 from analog_raw message
 *
 * @return adc channel_02
 */
static inline uint16_t mavlink_msg_analog_raw_get_chan02(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,2);
}

/**
 * @brief Get field chan03 from analog_raw message
 *
 * @return adc channel_03
 */
static inline uint16_t mavlink_msg_analog_raw_get_chan03(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,4);
}

/**
 * @brief Get field chan04 from analog_raw message
 *
 * @return adc channel_04
 */
static inline uint16_t mavlink_msg_analog_raw_get_chan04(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,6);
}

/**
 * @brief Get field chan05 from analog_raw message
 *
 * @return adc channel_05
 */
static inline uint16_t mavlink_msg_analog_raw_get_chan05(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,8);
}

/**
 * @brief Get field chan06 from analog_raw message
 *
 * @return adc channel_06
 */
static inline uint16_t mavlink_msg_analog_raw_get_chan06(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,10);
}

/**
 * @brief Get field chan07 from analog_raw message
 *
 * @return adc channel_07
 */
static inline uint16_t mavlink_msg_analog_raw_get_chan07(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,12);
}

/**
 * @brief Get field chan08 from analog_raw message
 *
 * @return adc channel_08
 */
static inline uint16_t mavlink_msg_analog_raw_get_chan08(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,14);
}

/**
 * @brief Get field chan09 from analog_raw message
 *
 * @return adc channel_09
 */
static inline uint16_t mavlink_msg_analog_raw_get_chan09(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,16);
}

/**
 * @brief Get field chan10 from analog_raw message
 *
 * @return adc channel_10
 */
static inline uint16_t mavlink_msg_analog_raw_get_chan10(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,18);
}

/**
 * @brief Get field chan11 from analog_raw message
 *
 * @return adc channel_11
 */
static inline uint16_t mavlink_msg_analog_raw_get_chan11(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,20);
}

/**
 * @brief Get field chan12 from analog_raw message
 *
 * @return adc channel_12
 */
static inline uint16_t mavlink_msg_analog_raw_get_chan12(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,22);
}

/**
 * @brief Get field chan13 from analog_raw message
 *
 * @return adc channel_13
 */
static inline uint16_t mavlink_msg_analog_raw_get_chan13(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,24);
}

/**
 * @brief Get field chan14 from analog_raw message
 *
 * @return adc channel_14
 */
static inline uint16_t mavlink_msg_analog_raw_get_chan14(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,26);
}

/**
 * @brief Get field chan15 from analog_raw message
 *
 * @return adc channel_15
 */
static inline uint16_t mavlink_msg_analog_raw_get_chan15(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,28);
}

/**
 * @brief Get field chan16 from analog_raw message
 *
 * @return adc channel_16
 */
static inline uint16_t mavlink_msg_analog_raw_get_chan16(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,30);
}

/**
 * @brief Get field usec from analog_raw message
 *
 * @return time
 */
static inline uint64_t mavlink_msg_analog_raw_get_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,32);
}

/**
 * @brief Decode a analog_raw message into a struct
 *
 * @param msg The message to decode
 * @param analog_raw C-struct to decode the message contents into
 */
static inline void mavlink_msg_analog_raw_decode(const mavlink_message_t* msg, mavlink_analog_raw_t* analog_raw)
{
#if MAVLINK_NEED_BYTE_SWAP
	analog_raw->chan01 = mavlink_msg_analog_raw_get_chan01(msg);
	analog_raw->chan02 = mavlink_msg_analog_raw_get_chan02(msg);
	analog_raw->chan03 = mavlink_msg_analog_raw_get_chan03(msg);
	analog_raw->chan04 = mavlink_msg_analog_raw_get_chan04(msg);
	analog_raw->chan05 = mavlink_msg_analog_raw_get_chan05(msg);
	analog_raw->chan06 = mavlink_msg_analog_raw_get_chan06(msg);
	analog_raw->chan07 = mavlink_msg_analog_raw_get_chan07(msg);
	analog_raw->chan08 = mavlink_msg_analog_raw_get_chan08(msg);
	analog_raw->chan09 = mavlink_msg_analog_raw_get_chan09(msg);
	analog_raw->chan10 = mavlink_msg_analog_raw_get_chan10(msg);
	analog_raw->chan11 = mavlink_msg_analog_raw_get_chan11(msg);
	analog_raw->chan12 = mavlink_msg_analog_raw_get_chan12(msg);
	analog_raw->chan13 = mavlink_msg_analog_raw_get_chan13(msg);
	analog_raw->chan14 = mavlink_msg_analog_raw_get_chan14(msg);
	analog_raw->chan15 = mavlink_msg_analog_raw_get_chan15(msg);
	analog_raw->chan16 = mavlink_msg_analog_raw_get_chan16(msg);
	analog_raw->usec = mavlink_msg_analog_raw_get_usec(msg);
#else
	memcpy(analog_raw, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_ANALOG_RAW_LEN);
#endif
}
