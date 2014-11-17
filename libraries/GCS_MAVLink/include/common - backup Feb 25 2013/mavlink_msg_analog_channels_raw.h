//------------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------------//

// SLV_GCS_MAVLINK_NEWMESSAGE
#include "../../ArduPlane/mydefines.h"

#if SLV_GCS_MAVLINK_NEWMESSAGE == ENABLED
// MESSAGE ANALOG_CHANNELS_RAW PACKING

#define MAVLINK_MSG_ID_ANALOG_CHANNELS_RAW 111

typedef struct __mavlink_analog_channels_raw_t 
{
	uint16_t ch01; ///< ANALOG channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
	uint16_t ch02; ///< ANALOG channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
	uint16_t ch03; ///< ANALOG channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
	uint16_t ch04; ///< ANALOG channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
	uint16_t ch05; ///< ANALOG channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
	uint16_t ch06; ///< ANALOG channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
	uint16_t ch07; ///< ANALOG channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
	uint16_t ch08; ///< ANALOG channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
	uint16_t ch09; ///< ANALOG channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
	uint16_t ch10; ///< ANALOG channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
	uint16_t ch11; ///< ANALOG channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
	uint16_t ch12; ///< ANALOG channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
	uint16_t ch13; ///< ANALOG channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
	uint16_t ch14; ///< ANALOG channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
	uint16_t ch15; ///< ANALOG channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
	uint16_t ch16; ///< ANALOG channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
	uint64_t usec;
	//uint8_t rssi; ///< Receive signal strength indicator, 0: 0%, 255: 100%

} mavlink_analog_channels_raw_t;

#define MAVLINK_MSG_ID_ANALOG_CHANNELS_RAW_LEN 40
#define MAVLINK_MSG_ID_111_LEN 40

#define MAVLINK_MESSAGE_INFO_ANALOG_CHANNELS_RAW { \
	"ANALOG_CHANNELS_RAW", \
	17, \
	{  { "ch01", NULL, MAVLINK_TYPE_UINT16_T, 0,  0, offsetof(mavlink_rc_channels_raw_t, ch01) }, \
       { "ch02", NULL, MAVLINK_TYPE_UINT16_T, 0,  2, offsetof(mavlink_rc_channels_raw_t, ch02) }, \
       { "ch03", NULL, MAVLINK_TYPE_UINT16_T, 0,  4, offsetof(mavlink_rc_channels_raw_t, ch03) }, \
       { "ch04", NULL, MAVLINK_TYPE_UINT16_T, 0,  6, offsetof(mavlink_rc_channels_raw_t, ch04) }, \
       { "ch05", NULL, MAVLINK_TYPE_UINT16_T, 0,  8, offsetof(mavlink_rc_channels_raw_t, ch05) }, \
       { "ch06", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_rc_channels_raw_t, ch06) }, \
       { "ch07", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_rc_channels_raw_t, ch07) }, \
       { "ch08", NULL, MAVLINK_TYPE_UINT16_T, 0, 14, offsetof(mavlink_rc_channels_raw_t, ch08) }, \
       { "ch09", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_rc_channels_raw_t, ch09) }, \
       { "ch10", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_rc_channels_raw_t, ch10) }, \
       { "ch11", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_rc_channels_raw_t, ch11) }, \
       { "ch12", NULL, MAVLINK_TYPE_UINT16_T, 0, 22, offsetof(mavlink_rc_channels_raw_t, ch12) }, \
       { "ch13", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_rc_channels_raw_t, ch13) }, \
       { "ch14", NULL, MAVLINK_TYPE_UINT16_T, 0, 26, offsetof(mavlink_rc_channels_raw_t, ch14) }, \
       { "ch15", NULL, MAVLINK_TYPE_UINT16_T, 0, 28, offsetof(mavlink_rc_channels_raw_t, ch15) }, \
       { "ch16", NULL, MAVLINK_TYPE_UINT16_T, 0, 30, offsetof(mavlink_rc_channels_raw_t, ch16) }, \
       { "usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 32, offsetof(mavlink_rc_channels_raw_t, usec) }, \
    } \
}




/**
 * @brief Pack a analog_channels_raw message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param ch01 ANALOG channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param ch02 ANALOG channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param ch03 ANALOG channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param ch04 ANALOG channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param ch05 ANALOG channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param ch06 ANALOG channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param ch07 ANALOG channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param ch08 ANALOG channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param rssi Receive signal strength indicator, 0: 0%, 255: 100%
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_analog_channels_raw_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, 
	uint16_t ch01, uint16_t ch02, uint16_t ch03, uint16_t ch04, 
	uint16_t ch05, uint16_t ch06, uint16_t ch07, uint16_t ch08, 
	uint16_t ch09, uint16_t ch10, uint16_t ch11, uint16_t ch12, 
	uint16_t ch13, uint16_t ch14, uint16_t ch15, uint16_t ch16, 
	uint64_t usec) //uint8_t rssi)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[40];
	_mav_put_uint16_t(buf,  0, ch01);
	_mav_put_uint16_t(buf,  2, ch02);
	_mav_put_uint16_t(buf,  4, ch03);
	_mav_put_uint16_t(buf,  6, ch04);
	_mav_put_uint16_t(buf,  8, ch05);
	_mav_put_uint16_t(buf, 10, ch06);
	_mav_put_uint16_t(buf, 12, ch07);
	_mav_put_uint16_t(buf, 14, ch08);
	_mav_put_uint16_t(buf, 16, ch09);
	_mav_put_uint16_t(buf, 18, ch10);
	_mav_put_uint16_t(buf, 20, ch11);
	_mav_put_uint16_t(buf, 22, ch12);
	_mav_put_uint16_t(buf, 24, ch13);
	_mav_put_uint16_t(buf, 26, ch14);
	_mav_put_uint16_t(buf, 28, ch15);
	_mav_put_uint16_t(buf, 30, ch16);
	_mav_put_uint64_t(buf, 32, usec);

        memcpy(_MAV_PAYLOAD(msg), buf, 40);
#else
	mavlink_rc_channels_raw_t packet;
	packet.ch01 = ch01;
	packet.ch02 = ch02;
	packet.ch03 = ch03;
	packet.ch04 = ch04;
	packet.ch05 = ch05;
	packet.ch06 = ch06;
	packet.ch07 = ch07;
	packet.ch08 = ch08;
	packet.ch09 = ch09;
	packet.ch10 = ch10;
	packet.ch11 = ch11;
	packet.ch12 = ch12;
	packet.ch13 = ch13;
	packet.ch14 = ch14;
	packet.ch15 = ch15;
	packet.ch16 = ch16;
	packet.usec = usec;

        memcpy(_MAV_PAYLOAD(msg), &packet, 40);
#endif

	msg->msgid = MAVLINK_MSG_ID_ANALOG_CHANNELS_RAW;
	return mavlink_finalize_message(msg, system_id, component_id, 40);
}

/**
 * @brief Pack a analog_channels_raw message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param ch01 ANALOG channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param ch02 ANALOG channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param ch03 ANALOG channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param ch04 ANALOG channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param ch05 ANALOG channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param ch06 ANALOG channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param ch07 ANALOG channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param ch08 ANALOG channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param rssi Receive signal strength indicator, 0: 0%, 255: 100%
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_analog_channels_raw_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, 
	uint16_t ch01, uint16_t ch02, uint16_t ch03, uint16_t ch04, 
	uint16_t ch05, uint16_t ch06, uint16_t ch07, uint16_t ch08, 
	uint16_t ch09, uint16_t ch10, uint16_t ch11, uint16_t ch12, 
	uint16_t ch13, uint16_t ch14, uint16_t ch15, uint16_t ch16, 
	uint64_t usec) //uint8_t rssi)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[40];
	_mav_put_uint16_t(buf,  0, ch01);
	_mav_put_uint16_t(buf,  2, ch02);
	_mav_put_uint16_t(buf,  4, ch03);
	_mav_put_uint16_t(buf,  6, ch04);
	_mav_put_uint16_t(buf,  8, ch05);
	_mav_put_uint16_t(buf, 10, ch06);
	_mav_put_uint16_t(buf, 12, ch07);
	_mav_put_uint16_t(buf, 14, ch08);
	_mav_put_uint16_t(buf, 16, ch09);
	_mav_put_uint16_t(buf, 18, ch10);
	_mav_put_uint16_t(buf, 20, ch11);
	_mav_put_uint16_t(buf, 22, ch12);
	_mav_put_uint16_t(buf, 24, ch13);
	_mav_put_uint16_t(buf, 26, ch14);
	_mav_put_uint16_t(buf, 28, ch15);
	_mav_put_uint16_t(buf, 30, ch16);
	_mav_put_uint64_t(buf, 32, usec);

        memcpy(_MAV_PAYLOAD(msg), buf, 40);
#else
	mavlink_rc_channels_raw_t packet;
	packet.ch01 = ch01;
	packet.ch02 = ch02;
	packet.ch03 = ch03;
	packet.ch04 = ch04;
	packet.ch05 = ch05;
	packet.ch06 = ch06;
	packet.ch07 = ch07;
	packet.ch08 = ch08;
	packet.ch09 = ch09;
	packet.ch10 = ch10;
	packet.ch11 = ch11;
	packet.ch12 = ch12;
	packet.ch13 = ch13;
	packet.ch14 = ch14;
	packet.ch15 = ch15;
	packet.ch16 = ch16;
	packet.usec = usec;

        memcpy(_MAV_PAYLOAD(msg), &packet, 40);
#endif

	msg->msgid = MAVLINK_MSG_ID_ANALOG_CHANNELS_RAW;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 40);
}

/**
 * @brief Encode a analog_channels_raw struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param analog_channels_raw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_analog_channels_raw_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_analog_channels_raw_t* analog_channels_raw)
{
	return mavlink_msg_analog_channels_raw_pack(system_id, component_id, msg, 
		analog_channels_raw->ch01, 
		analog_channels_raw->ch02, 
		analog_channels_raw->ch03, 
		analog_channels_raw->ch04, 
		analog_channels_raw->ch05, 
		analog_channels_raw->ch06, 
		analog_channels_raw->ch07, 
		analog_channels_raw->ch08, 
		analog_channels_raw->ch09, 
		analog_channels_raw->ch10, 
		analog_channels_raw->ch11, 
		analog_channels_raw->ch12, 
		analog_channels_raw->ch13, 
		analog_channels_raw->ch14, 
		analog_channels_raw->ch15, 
		analog_channels_raw->ch16, 
		analog_channels_raw->usec);
}

/**
 * @brief Send a analog_channels_raw message
 * @param chan MAVLink channel to send the message
 *
 * @param ch01 ANALOG channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param ch02 ANALOG channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param ch03 ANALOG channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param ch04 ANALOG channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param ch05 ANALOG channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param ch06 ANALOG channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param ch07 ANALOG channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param ch08 ANALOG channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param rssi Receive signal strength indicator, 0: 0%, 255: 100%
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_analog_channels_raw_send(mavlink_channel_t chan, 
	uint16_t ch01, uint16_t ch02, uint16_t ch03, uint16_t ch04, 
	uint16_t ch05, uint16_t ch06, uint16_t ch07, uint16_t ch08, 
	uint16_t ch09, uint16_t ch10, uint16_t ch11, uint16_t ch12, 
	uint16_t ch13, uint16_t ch14, uint16_t ch15, uint16_t ch16, 
	uint64_t usec) //uint8_t rssi)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[40];
	_mav_put_uint16_t(buf,  0, ch01);
	_mav_put_uint16_t(buf,  2, ch02);
	_mav_put_uint16_t(buf,  4, ch03);
	_mav_put_uint16_t(buf,  6, ch04);
	_mav_put_uint16_t(buf,  8, ch05);
	_mav_put_uint16_t(buf, 10, ch06);
	_mav_put_uint16_t(buf, 12, ch07);
	_mav_put_uint16_t(buf, 14, ch08);
	_mav_put_uint16_t(buf, 16, ch09);
	_mav_put_uint16_t(buf, 18, ch10);
	_mav_put_uint16_t(buf, 20, ch11);
	_mav_put_uint16_t(buf, 22, ch12);
	_mav_put_uint16_t(buf, 24, ch13);
	_mav_put_uint16_t(buf, 26, ch14);
	_mav_put_uint16_t(buf, 28, ch15);
	_mav_put_uint16_t(buf, 30, ch16);
	_mav_put_uint64_t(buf, 32, usec);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ANALOG_CHANNELS_RAW, buf, 40);
#else
	mavlink_rc_channels_raw_t packet;
	packet.ch01 = ch01;
	packet.ch02 = ch02;
	packet.ch03 = ch03;
	packet.ch04 = ch04;
	packet.ch05 = ch05;
	packet.ch06 = ch06;
	packet.ch07 = ch07;
	packet.ch08 = ch08;
	packet.ch09 = ch09;
	packet.ch10 = ch10;
	packet.ch11 = ch11;
	packet.ch12 = ch12;
	packet.ch13 = ch13;
	packet.ch14 = ch14;
	packet.ch15 = ch15;
	packet.ch16 = ch16;
	packet.usec = usec;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ANALOG_CHANNELS_RAW, (const char *)&packet, 40);
#endif
}

#endif
// MESSAGE ANALOG_CHANNELS_RAW UNPACKING

/**
 * @brief Get field ch01 from analog_channels_raw message
 *
 * @return ANALOG channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
static inline uint16_t mavlink_msg_analog_channels_raw_get_ch01(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field ch02 from analog_channels_raw message
 *
 * @return ANALOG channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
static inline uint16_t mavlink_msg_analog_channels_raw_get_ch02(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field chan3 from analog_channels_raw message
 *
 * @return ANALOG channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
static inline uint16_t mavlink_msg_analog_channels_raw_get_ch03(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field chan4 from analog_channels_raw message
 *
 * @return ANALOG channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
static inline uint16_t mavlink_msg_analog_channels_raw_get_ch04(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Get field chan5 from analog_channels_raw message
 *
 * @return ANALOG channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
static inline uint16_t mavlink_msg_analog_channels_raw_get_ch05(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field chan6 from analog_channels_raw message
 *
 * @return ANALOG channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
static inline uint16_t mavlink_msg_analog_channels_raw_get_ch06(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  10);
}

/**
 * @brief Get field chan7 from analog_channels_raw message
 *
 * @return ANALOG channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
static inline uint16_t mavlink_msg_analog_channels_raw_get_ch07(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Get field chan8 from analog_channels_raw message
 *
 * @return ANALOG channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
static inline uint16_t mavlink_msg_analog_channels_raw_get_ch08(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  14);
}

/**
 * @brief Get field chan9 from analog_channels_raw message
 *
 * @return ANALOG channel 9 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
static inline uint16_t mavlink_msg_analog_channels_raw_get_ch09(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  16);
}

/**
 * @brief Get field chan10 from analog_channels_raw message
 *
 * @return ANALOG channel 10 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
static inline uint16_t mavlink_msg_analog_channels_raw_get_ch10(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  18);
}

/**
 * @brief Get field chan11 from analog_channels_raw message
 *
 * @return ANALOG channel 11 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
static inline uint16_t mavlink_msg_analog_channels_raw_get_ch11(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  20);
}

/**
 * @brief Get field chan12 from analog_channels_raw message
 *
 * @return ANALOG channel 12 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
static inline uint16_t mavlink_msg_analog_channels_raw_get_ch12(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  22);
}

/**
 * @brief Get field chan13 from analog_channels_raw message
 *
 * @return ANALOG channel 13 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
static inline uint16_t mavlink_msg_analog_channels_raw_get_ch13(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  24);
}

/**
 * @brief Get field chan14 from analog_channels_raw message
 *
 * @return ANALOG channel 14 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
static inline uint16_t mavlink_msg_analog_channels_raw_get_ch14(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  26);
}

/**
 * @brief Get field chan15 from analog_channels_raw message
 *
 * @return ANALOG channel 15 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
static inline uint16_t mavlink_msg_analog_channels_raw_get_ch15(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  28);
}

/**
 * @brief Get field chan16 from analog_channels_raw message
 *
 * @return ANALOG channel 16 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
static inline uint16_t mavlink_msg_analog_channels_raw_get_ch16(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  30);
}

/**
 * @brief Get field rssi from analog_channels_raw message
 *
 * @return Receive signal strength indicator, 0: 0%, 255: 100%
 */
static inline uint64_t mavlink_msg_analog_channels_raw_get_usec(const mavlink_message_t* msg)
{
	//return _MAV_RETURN_uint16_t(msg,  32);
	return _MAV_RETURN_uint64_t(msg,  32);
}

/**
 * @brief Decode a analog_channels_raw message into a struct
 *
 * @param msg The message to decode
 * @param analog_channels_raw C-struct to decode the message contents into
 */
static inline void mavlink_msg_analog_channels_raw_decode(const mavlink_message_t* msg, mavlink_analog_channels_raw_t* analog_channels_raw)
{
#if MAVLINK_NEED_BYTE_SWAP
	analog_channels_raw->ch01 = mavlink_msg_analog_channels_raw_get_ch01(msg);
	analog_channels_raw->ch02 = mavlink_msg_analog_channels_raw_get_ch02(msg);
	analog_channels_raw->ch03 = mavlink_msg_analog_channels_raw_get_ch03(msg);
	analog_channels_raw->ch04 = mavlink_msg_analog_channels_raw_get_ch04(msg);
	analog_channels_raw->ch05 = mavlink_msg_analog_channels_raw_get_ch05(msg);
	analog_channels_raw->ch06 = mavlink_msg_analog_channels_raw_get_ch06(msg);
	analog_channels_raw->ch07 = mavlink_msg_analog_channels_raw_get_ch07(msg);
	analog_channels_raw->ch08 = mavlink_msg_analog_channels_raw_get_ch08(msg);
	analog_channels_raw->ch09 = mavlink_msg_analog_channels_raw_get_ch09(msg);
	analog_channels_raw->ch10 = mavlink_msg_analog_channels_raw_get_ch10(msg);
	analog_channels_raw->ch11 = mavlink_msg_analog_channels_raw_get_ch11(msg);
	analog_channels_raw->ch12 = mavlink_msg_analog_channels_raw_get_ch12(msg);
	analog_channels_raw->ch13 = mavlink_msg_analog_channels_raw_get_ch13(msg);
	analog_channels_raw->ch14 = mavlink_msg_analog_channels_raw_get_ch14(msg);
	analog_channels_raw->ch15 = mavlink_msg_analog_channels_raw_get_ch15(msg);
	analog_channels_raw->ch16 = mavlink_msg_analog_channels_raw_get_ch16(msg);
	analog_channels_raw->usec = mavlink_msg_analog_channels_raw_get_usec(msg);
#else
	memcpy(analog_channels_raw, _MAV_PAYLOAD(msg), 40);
#endif
}
#else
	#error SLV_GCS_MAVLINK_NEWMESSAGE DISABLED.
#endif
