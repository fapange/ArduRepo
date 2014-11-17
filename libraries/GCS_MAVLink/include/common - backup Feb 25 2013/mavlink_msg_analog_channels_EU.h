//------------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------------//

// SLV_GCS_MAVLINK_NEWMESSAGE
#include "../../ArduPlane/mydefines.h"

#if SLV_GCS_MAVLINK_NEWMESSAGE == ENABLED
// MESSAGE ANALOG_CHANNELS_EU PACKING

#define MAVLINK_MSG_ID_ANALOG_CHANNELS_EU 112

typedef struct __mavlink_analog_channels_EU_t 
{
	float ch01; ///< ANALOG channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
	float ch02; ///< ANALOG channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
	float ch03; ///< ANALOG channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
	float ch04; ///< ANALOG channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
	float ch05; ///< ANALOG channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
	float ch06; ///< ANALOG channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
	float ch07; ///< ANALOG channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
	float ch08; ///< ANALOG channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
	float ch09; ///< ANALOG channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
	float ch10; ///< ANALOG channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
	float ch11; ///< ANALOG channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
	float ch12; ///< ANALOG channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
	float ch13; ///< ANALOG channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
	float ch14; ///< ANALOG channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
	float ch15; ///< ANALOG channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
	float ch16; ///< ANALOG channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
	uint64_t usec;
	//uint8_t rssi; ///< Receive signal strength indicator, 0: 0%, 255: 100%

} mavlink_analog_channels_EU_t;

#define MAVLINK_MSG_ID_ANALOG_CHANNELS_EU_LEN 72
#define MAVLINK_MSG_ID_112_LEN MAVLINK_MSG_ID_ANALOG_CHANNELS_EU_LEN

#define MAVLINK_MESSAGE_INFO_ANALOG_CHANNELS_EU { \
	"ANALOG_CHANNELS_EU", \
	17, \
	{  { "ch01", NULL, MAVLINK_TYPE_FLOAT, 0,  0, offsetof(mavlink_rc_channels_EU_t, ch01) }, \
       { "ch02", NULL, MAVLINK_TYPE_FLOAT, 0,  4, offsetof(mavlink_rc_channels_EU_t, ch02) }, \
       { "ch03", NULL, MAVLINK_TYPE_FLOAT, 0,  8, offsetof(mavlink_rc_channels_EU_t, ch03) }, \
       { "ch04", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_rc_channels_EU_t, ch04) }, \
       { "ch05", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_rc_channels_EU_t, ch05) }, \
       { "ch06", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_rc_channels_EU_t, ch06) }, \
       { "ch07", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_rc_channels_EU_t, ch07) }, \
       { "ch08", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_rc_channels_EU_t, ch08) }, \
       { "ch09", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_rc_channels_EU_t, ch09) }, \
       { "ch10", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_rc_channels_EU_t, ch10) }, \
       { "ch11", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_rc_channels_EU_t, ch11) }, \
       { "ch12", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_rc_channels_EU_t, ch12) }, \
       { "ch13", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_rc_channels_EU_t, ch13) }, \
       { "ch14", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_rc_channels_EU_t, ch14) }, \
       { "ch15", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_rc_channels_EU_t, ch15) }, \
       { "ch16", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_rc_channels_EU_t, ch16) }, \
       { "usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 64, offsetof(mavlink_rc_channels_EU_t, usec) }, \
    } \
}




/**
 * @brief Pack a analog_channels_EU message
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
static inline uint16_t mavlink_msg_analog_channels_EU_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, 
	float ch01, float ch02, float ch03, float ch04, 
	float ch05, float ch06, float ch07, float ch08, 
	float ch09, float ch10, float ch11, float ch12, 
	float ch13, float ch14, float ch15, float ch16, 
	uint64_t usec) //uint8_t rssi)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ANALOG_CHANNELS_EU_LEN];
	_mav_put_float(buf,  0, ch01);
	_mav_put_float(buf,  4, ch02);
	_mav_put_float(buf,  8, ch03);
	_mav_put_float(buf, 12, ch04);
	_mav_put_float(buf, 16, ch05);
	_mav_put_float(buf, 20, ch06);
	_mav_put_float(buf, 24, ch07);
	_mav_put_float(buf, 28, ch08);
	_mav_put_float(buf, 32, ch09);
	_mav_put_float(buf, 36, ch10);
	_mav_put_float(buf, 40, ch11);
	_mav_put_float(buf, 44, ch12);
	_mav_put_float(buf, 48, ch13);
	_mav_put_float(buf, 52, ch14);
	_mav_put_float(buf, 56, ch15);
	_mav_put_float(buf, 60, ch16);
	_mav_put_uint64_t(buf, 64, usec);

        memcpy(_MAV_PAYLOAD(msg), buf, MAVLINK_MSG_ID_ANALOG_CHANNELS_EU_LEN);
#else
	mavlink_rc_channels_EU_t packet;
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

        memcpy(_MAV_PAYLOAD(msg), &packet, MAVLINK_MSG_ID_ANALOG_CHANNELS_EU_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ANALOG_CHANNELS_EU;
	return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ANALOG_CHANNELS_EU_LEN);
}

/**
 * @brief Pack a analog_channels_EU message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 400 for IMU)
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
static inline uint16_t mavlink_msg_analog_channels_EU_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, 
	float ch01, float ch02, float ch03, float ch04, 
	float ch05, float ch06, float ch07, float ch08, 
	float ch09, float ch10, float ch11, float ch12, 
	float ch13, float ch14, float ch15, float ch16, 
	uint64_t usec) //uint8_t rssi)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ANALOG_CHANNELS_EU_LEN];
	_mav_put_float(buf,  0, ch01);
	_mav_put_float(buf,  4, ch02);
	_mav_put_float(buf,  8, ch03);
	_mav_put_float(buf, 12, ch04);
	_mav_put_float(buf, 16, ch05);
	_mav_put_float(buf, 20, ch06);
	_mav_put_float(buf, 24, ch07);
	_mav_put_float(buf, 28, ch08);
	_mav_put_float(buf, 32, ch09);
	_mav_put_float(buf, 36, ch10);
	_mav_put_float(buf, 40, ch11);
	_mav_put_float(buf, 44, ch12);
	_mav_put_float(buf, 48, ch13);
	_mav_put_float(buf, 52, ch14);
	_mav_put_float(buf, 56, ch15);
	_mav_put_float(buf, 60, ch16);
	_mav_put_uint64_t(buf, 64, usec);

        memcpy(_MAV_PAYLOAD(msg), buf, MAVLINK_MSG_ID_ANALOG_CHANNELS_EU_LEN);
#else
	mavlink_rc_channels_EU_t packet;
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

        memcpy(_MAV_PAYLOAD(msg), &packet, MAVLINK_MSG_ID_ANALOG_CHANNELS_EU_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ANALOG_CHANNELS_EU;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ANALOG_CHANNELS_EU_LEN);
}

/**
 * @brief Encode a analog_channels_EU struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 400 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param analog_channels_EU C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_analog_channels_EU_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_analog_channels_EU_t* analog_channels_EU)
{
	return mavlink_msg_analog_channels_EU_pack(system_id, component_id, msg, 
		analog_channels_EU->ch01, 
		analog_channels_EU->ch02, 
		analog_channels_EU->ch03, 
		analog_channels_EU->ch04, 
		analog_channels_EU->ch05, 
		analog_channels_EU->ch06, 
		analog_channels_EU->ch07, 
		analog_channels_EU->ch08, 
		analog_channels_EU->ch09, 
		analog_channels_EU->ch10, 
		analog_channels_EU->ch11, 
		analog_channels_EU->ch12, 
		analog_channels_EU->ch13, 
		analog_channels_EU->ch14, 
		analog_channels_EU->ch15, 
		analog_channels_EU->ch16, 
		analog_channels_EU->usec);
}

/**
 * @brief Send a analog_channels_EU message
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

static inline void mavlink_msg_analog_channels_EU_send(mavlink_channel_t chan, 
	float ch01, float ch02, float ch03, float ch04, 
	float ch05, float ch06, float ch07, float ch08, 
	float ch09, float ch10, float ch11, float ch12, 
	float ch13, float ch14, float ch15, float ch16, 
	uint64_t usec) //uint8_t rssi)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ANALOG_CHANNELS_EU_LEN];
	_mav_put_float(buf,  0, ch01);
	_mav_put_float(buf,  4, ch02);
	_mav_put_float(buf,  8, ch03);
	_mav_put_float(buf, 12, ch04);
	_mav_put_float(buf, 16, ch05);
	_mav_put_float(buf, 20, ch06);
	_mav_put_float(buf, 24, ch07);
	_mav_put_float(buf, 28, ch08);
	_mav_put_float(buf, 32, ch09);
	_mav_put_float(buf, 36, ch10);
	_mav_put_float(buf, 40, ch11);
	_mav_put_float(buf, 44, ch12);
	_mav_put_float(buf, 48, ch13);
	_mav_put_float(buf, 52, ch14);
	_mav_put_float(buf, 56, ch15);
	_mav_put_float(buf, 60, ch16);
	_mav_put_uint64_t(buf, 64, usec);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ANALOG_CHANNELS_EU, buf, MAVLINK_MSG_ID_ANALOG_CHANNELS_EU_LEN);
#else
	mavlink_rc_channels_EU_t packet;
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

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ANALOG_CHANNELS_EU, (const char *)&packet, MAVLINK_MSG_ID_ANALOG_CHANNELS_EU_LEN);
#endif
}

#endif
// MESSAGE ANALOG_CHANNELS_EU UNPACKING

/**
 * @brief Get field ch01 from analog_channels_EU message
 *
 * @return ANALOG channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
static inline float mavlink_msg_analog_channels_EU_get_ch01(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field ch02 from analog_channels_EU message
 *
 * @return ANALOG channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
static inline float mavlink_msg_analog_channels_EU_get_ch02(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field chan3 from analog_channels_EU message
 *
 * @return ANALOG channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
static inline float mavlink_msg_analog_channels_EU_get_ch03(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field chan4 from analog_channels_EU message
 *
 * @return ANALOG channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
static inline float mavlink_msg_analog_channels_EU_get_ch04(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field chan5 from analog_channels_EU message
 *
 * @return ANALOG channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
static inline float mavlink_msg_analog_channels_EU_get_ch05(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field chan6 from analog_channels_EU message
 *
 * @return ANALOG channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
static inline float mavlink_msg_analog_channels_EU_get_ch06(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field chan7 from analog_channels_EU message
 *
 * @return ANALOG channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
static inline float mavlink_msg_analog_channels_EU_get_ch07(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field chan8 from analog_channels_EU message
 *
 * @return ANALOG channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
static inline float mavlink_msg_analog_channels_EU_get_ch08(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field chan9 from analog_channels_EU message
 *
 * @return ANALOG channel 9 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
static inline float mavlink_msg_analog_channels_EU_get_ch09(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field chan10 from analog_channels_EU message
 *
 * @return ANALOG channel 10 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
static inline float mavlink_msg_analog_channels_EU_get_ch10(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field chan11 from analog_channels_EU message
 *
 * @return ANALOG channel 11 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
static inline float mavlink_msg_analog_channels_EU_get_ch11(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field chan12 from analog_channels_EU message
 *
 * @return ANALOG channel 12 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
static inline float mavlink_msg_analog_channels_EU_get_ch12(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field chan13 from analog_channels_EU message
 *
 * @return ANALOG channel 13 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
static inline float mavlink_msg_analog_channels_EU_get_ch13(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field chan14 from analog_channels_EU message
 *
 * @return ANALOG channel 14 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
static inline float mavlink_msg_analog_channels_EU_get_ch14(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field chan15 from analog_channels_EU message
 *
 * @return ANALOG channel 15 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
static inline float mavlink_msg_analog_channels_EU_get_ch15(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field chan16 from analog_channels_EU message
 *
 * @return ANALOG channel 16 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
static inline float mavlink_msg_analog_channels_EU_get_ch16(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  60);
}

/**
 * @brief Get field rssi from analog_channels_EU message
 *
 * @return Receive signal strength indicator, 0: 0%, 255: 100%
 */
static inline uint64_t mavlink_msg_analog_channels_EU_get_usec(const mavlink_message_t* msg)
{
	//return _MAV_RETURN_uint16_t(msg,  64);
	return _MAV_RETURN_uint64_t(msg,  64);
}

/**
 * @brief Decode a analog_channels_EU message into a struct
 *
 * @param msg The message to decode
 * @param analog_channels_EU C-struct to decode the message contents into
 */
static inline void mavlink_msg_analog_channels_EU_decode(const mavlink_message_t* msg, mavlink_analog_channels_EU_t* analog_channels_EU)
{
#if MAVLINK_NEED_BYTE_SWAP
	analog_channels_EU->ch01 = mavlink_msg_analog_channels_EU_get_ch01(msg);
	analog_channels_EU->ch02 = mavlink_msg_analog_channels_EU_get_ch02(msg);
	analog_channels_EU->ch03 = mavlink_msg_analog_channels_EU_get_ch03(msg);
	analog_channels_EU->ch04 = mavlink_msg_analog_channels_EU_get_ch04(msg);
	analog_channels_EU->ch05 = mavlink_msg_analog_channels_EU_get_ch05(msg);
	analog_channels_EU->ch06 = mavlink_msg_analog_channels_EU_get_ch06(msg);
	analog_channels_EU->ch07 = mavlink_msg_analog_channels_EU_get_ch07(msg);
	analog_channels_EU->ch08 = mavlink_msg_analog_channels_EU_get_ch08(msg);
	analog_channels_EU->ch09 = mavlink_msg_analog_channels_EU_get_ch09(msg);
	analog_channels_EU->ch10 = mavlink_msg_analog_channels_EU_get_ch10(msg);
	analog_channels_EU->ch11 = mavlink_msg_analog_channels_EU_get_ch11(msg);
	analog_channels_EU->ch12 = mavlink_msg_analog_channels_EU_get_ch12(msg);
	analog_channels_EU->ch13 = mavlink_msg_analog_channels_EU_get_ch13(msg);
	analog_channels_EU->ch14 = mavlink_msg_analog_channels_EU_get_ch14(msg);
	analog_channels_EU->ch15 = mavlink_msg_analog_channels_EU_get_ch15(msg);
	analog_channels_EU->ch16 = mavlink_msg_analog_channels_EU_get_ch16(msg);
	analog_channels_EU->usec = mavlink_msg_analog_channels_EU_get_usec(msg);
#else
	memcpy(analog_channels_EU, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_ANALOG_CHANNELS_EU_LEN);
#endif
}
#else
	#error SLV_GCS_MAVLINK_NEWMESSAGE DISABLED.
#endif
