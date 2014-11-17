// MESSAGE STATE_DATA PACKING
// Generated from parse_APM_XML.m on 19-Aug-2013 09:01:20

#define MAVLINK_MSG_ID_STATE_DATA 115 

typedef struct __mavlink_state_data_t 
{ 
  uint8_t num;  ///< Vehicle number, e.g. 1 for R1, 2 for R2, 3 for R3
  uint8_t atloiter;  ///< if 0, vehicle is not currently loitering, if 1, vehicle is loitering
  uint8_t mode;  ///< mode: MANUAL=0, CIRCLE=1, STABILIZE=2, FLY_BY_WIRE_A=5, FLY_BY_WIRE_B=6, FLY_BY_WIRE_C=7, AUTO=10, RTL=11, LOITER=12, GUIDED=15, INITIALISING=16
  uint64_t usec;  ///< time
} mavlink_state_data_t;

#define MAVLINK_MSG_ID_STATE_DATA_LEN 11
#define MAVLINK_MSG_ID_115_LEN 11

#define MAVLINK_MESSAGE_INFO_STATE_DATA \
{ \
  "STATE_DATA", \
  4, \
  { \
    { "num", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_state_data_t, num) }, \
    { "atloiter", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_state_data_t, atloiter) }, \
    { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_state_data_t, mode) }, \
    { "usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 3, offsetof(mavlink_state_data_t, usec) }, \
  } \
}

/**
 * @brief Pack a state_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param num Vehicle number, e.g. 1 for R1, 2 for R2, 3 for R3
 * @param atloiter if 0, vehicle is not currently loitering, if 1, vehicle is loitering
 * @param mode mode: MANUAL=0, CIRCLE=1, STABILIZE=2, FLY_BY_WIRE_A=5, FLY_BY_WIRE_B=6, FLY_BY_WIRE_C=7, AUTO=10, RTL=11, LOITER=12, GUIDED=15, INITIALISING=16
 * @param usec time
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_state_data_pack(
	 uint8_t system_id
	,uint8_t component_id
	,mavlink_message_t* msg
	,uint8_t num
	,uint8_t atloiter
	,uint8_t mode
	,uint64_t usec
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_STATE_DATA_LEN];
	_mav_put_uint8_t(buf, 0, num);
	_mav_put_uint8_t(buf, 1, atloiter);
	_mav_put_uint8_t(buf, 2, mode);
	_mav_put_uint64_t(buf, 3, usec);

	memcpy(_MAV_PAYLOAD(msg), buf, MAVLINK_MSG_ID_STATE_DATA_LEN);
#else
	mavlink_state_data_t packet;
	packet.num = num;
	packet.atloiter = atloiter;
	packet.mode = mode;
	packet.usec = usec;

	memcpy(_MAV_PAYLOAD(msg), &packet, MAVLINK_MSG_ID_STATE_DATA_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_STATE_DATA;
	return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_STATE_DATA_LEN);
}

/**
 * @brief Pack a state_data message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 *
 * @param num Vehicle number, e.g. 1 for R1, 2 for R2, 3 for R3
 * @param atloiter if 0, vehicle is not currently loitering, if 1, vehicle is loitering
 * @param mode mode: MANUAL=0, CIRCLE=1, STABILIZE=2, FLY_BY_WIRE_A=5, FLY_BY_WIRE_B=6, FLY_BY_WIRE_C=7, AUTO=10, RTL=11, LOITER=12, GUIDED=15, INITIALISING=16
 * @param usec time
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_state_data_pack_chan(
	 uint8_t system_id
	,uint8_t component_id
	,uint8_t chan
	,mavlink_message_t* msg
	,uint8_t num
	,uint8_t atloiter
	,uint8_t mode
	,uint64_t usec
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_STATE_DATA_LEN];
	_mav_put_uint8_t(buf, 0, num);
	_mav_put_uint8_t(buf, 1, atloiter);
	_mav_put_uint8_t(buf, 2, mode);
	_mav_put_uint64_t(buf, 3, usec);

	memcpy(_MAV_PAYLOAD(msg), buf, MAVLINK_MSG_ID_STATE_DATA_LEN);
#else
	mavlink_state_data_t packet;
	packet.num = num;
	packet.atloiter = atloiter;
	packet.mode = mode;
	packet.usec = usec;

	memcpy(_MAV_PAYLOAD(msg), &packet, MAVLINK_MSG_ID_STATE_DATA_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_STATE_DATA;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_STATE_DATA_LEN);
}

/**
 * @brief Encode a state_data struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param vfr_hud C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_state_data_encode(
	 uint8_t system_id
	,uint8_t component_id
	,mavlink_message_t* msg
	,const mavlink_state_data_t* state_data
	)
{
	return mavlink_msg_state_data_pack(
	 system_id
	,component_id
	,msg
	,state_data->num
	,state_data->atloiter
	,state_data->mode
	,state_data->usec
	);
}

/**
 * @brief Send a state_data message
 * @param chan The MAVLink channel to send this message
 *
 * @param num Vehicle number, e.g. 1 for R1, 2 for R2, 3 for R3
 * @param atloiter if 0, vehicle is not currently loitering, if 1, vehicle is loitering
 * @param mode mode: MANUAL=0, CIRCLE=1, STABILIZE=2, FLY_BY_WIRE_A=5, FLY_BY_WIRE_B=6, FLY_BY_WIRE_C=7, AUTO=10, RTL=11, LOITER=12, GUIDED=15, INITIALISING=16
 * @param usec time
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_state_data_send(
	 mavlink_channel_t chan
	,uint8_t num
	,uint8_t atloiter
	,uint8_t mode
	,uint64_t usec
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_STATE_DATA_LEN];
	_mav_put_uint8_t(buf, 0, num);
	_mav_put_uint8_t(buf, 1, atloiter);
	_mav_put_uint8_t(buf, 2, mode);
	_mav_put_uint64_t(buf, 3, usec);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATE_DATA, buf, MAVLINK_MSG_ID_STATE_DATA_LEN);
#else
	mavlink_state_data_t packet;
	packet.num = num;
	packet.atloiter = atloiter;
	packet.mode = mode;
	packet.usec = usec;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATE_DATA, (const char*)&packet, MAVLINK_MSG_ID_STATE_DATA_LEN);
#endif
}

#endif // MAVLINK_USE_CONVENIENCE_FUNCTIONS

// MESSAGE STATE_DATA UNPACKING

/**
 * @brief Get field num from state_data message
 *
 * @return Vehicle number, e.g. 1 for R1, 2 for R2, 3 for R3
 */
static inline uint8_t mavlink_msg_state_data_get_num(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,0);
}

/**
 * @brief Get field atloiter from state_data message
 *
 * @return if 0, vehicle is not currently loitering, if 1, vehicle is loitering
 */
static inline uint8_t mavlink_msg_state_data_get_atloiter(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,1);
}

/**
 * @brief Get field mode from state_data message
 *
 * @return mode: MANUAL=0, CIRCLE=1, STABILIZE=2, FLY_BY_WIRE_A=5, FLY_BY_WIRE_B=6, FLY_BY_WIRE_C=7, AUTO=10, RTL=11, LOITER=12, GUIDED=15, INITIALISING=16
 */
static inline uint8_t mavlink_msg_state_data_get_mode(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,2);
}

/**
 * @brief Get field usec from state_data message
 *
 * @return time
 */
static inline uint64_t mavlink_msg_state_data_get_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,3);
}

/**
 * @brief Decode a state_data message into a struct
 *
 * @param msg The message to decode
 * @param state_data C-struct to decode the message contents into
 */
static inline void mavlink_msg_state_data_decode(const mavlink_message_t* msg, mavlink_state_data_t* state_data)
{
#if MAVLINK_NEED_BYTE_SWAP
	state_data->num = mavlink_msg_state_data_get_num(msg);
	state_data->atloiter = mavlink_msg_state_data_get_atloiter(msg);
	state_data->mode = mavlink_msg_state_data_get_mode(msg);
	state_data->usec = mavlink_msg_state_data_get_usec(msg);
#else
	memcpy(state_data, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_STATE_DATA_LEN);
#endif
}
