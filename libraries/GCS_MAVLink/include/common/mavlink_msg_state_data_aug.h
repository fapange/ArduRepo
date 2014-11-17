// MESSAGE STATE_DATA_AUG PACKING
// Generated from parse_APM_XML.m on 24-Feb-2014 13:50:53

#define MAVLINK_MSG_ID_STATE_DATA_AUG 117 

typedef struct __mavlink_state_data_aug_t 
{ 
  uint8_t num;  ///< Vehicle number, e.g. 1 for R1, 2 for R2, 3 for R3
  uint8_t atloiter;  ///< if 0, vehicle is not currently loitering, if 1, vehicle is loitering
  uint8_t mode;  ///< mode: MANUAL=0, CIRCLE=1, STABILIZE=2, FLY_BY_WIRE_A=5, FLY_BY_WIRE_B=6, FLY_BY_WIRE_C=7, AUTO=10, RTL=11, LOITER=12, GUIDED=15, INITIALISING=16
  uint8_t RC_state;  ///< state: OFF=0, ON=1, FAILSAFE=255
  uint64_t usec;  ///< time
} mavlink_state_data_aug_t;

#define MAVLINK_MSG_ID_STATE_DATA_AUG_LEN 12
#define MAVLINK_MSG_ID_117_LEN 12

#define MAVLINK_MESSAGE_INFO_STATE_DATA_AUG \
{ \
  "STATE_DATA_AUG", \
  5, \
  { \
    { "num", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_state_data_aug_t, num) }, \
    { "atloiter", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_state_data_aug_t, atloiter) }, \
    { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_state_data_aug_t, mode) }, \
    { "RC_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_state_data_aug_t, RC_state) }, \
    { "usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 4, offsetof(mavlink_state_data_aug_t, usec) }, \
  } \
}

/**
 * @brief Pack a state_data_aug message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param num Vehicle number, e.g. 1 for R1, 2 for R2, 3 for R3
 * @param atloiter if 0, vehicle is not currently loitering, if 1, vehicle is loitering
 * @param mode mode: MANUAL=0, CIRCLE=1, STABILIZE=2, FLY_BY_WIRE_A=5, FLY_BY_WIRE_B=6, FLY_BY_WIRE_C=7, AUTO=10, RTL=11, LOITER=12, GUIDED=15, INITIALISING=16
 * @param RC_state state: OFF=0, ON=1, FAILSAFE=255
 * @param usec time
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_state_data_aug_pack(
	 uint8_t system_id
	,uint8_t component_id
	,mavlink_message_t* msg
	,uint8_t num
	,uint8_t atloiter
	,uint8_t mode
	,uint8_t RC_state
	,uint64_t usec
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_STATE_DATA_AUG_LEN];
	_mav_put_uint8_t(buf, 0, num);
	_mav_put_uint8_t(buf, 1, atloiter);
	_mav_put_uint8_t(buf, 2, mode);
	_mav_put_uint8_t(buf, 3, RC_state);
	_mav_put_uint64_t(buf, 4, usec);

	memcpy(_MAV_PAYLOAD(msg), buf, MAVLINK_MSG_ID_STATE_DATA_AUG_LEN);
#else
	mavlink_state_data_aug_t packet;
	packet.num = num;
	packet.atloiter = atloiter;
	packet.mode = mode;
	packet.RC_state = RC_state;
	packet.usec = usec;

	memcpy(_MAV_PAYLOAD(msg), &packet, MAVLINK_MSG_ID_STATE_DATA_AUG_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_STATE_DATA_AUG;
	return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_STATE_DATA_AUG_LEN);
}

/**
 * @brief Pack a state_data_aug message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 *
 * @param num Vehicle number, e.g. 1 for R1, 2 for R2, 3 for R3
 * @param atloiter if 0, vehicle is not currently loitering, if 1, vehicle is loitering
 * @param mode mode: MANUAL=0, CIRCLE=1, STABILIZE=2, FLY_BY_WIRE_A=5, FLY_BY_WIRE_B=6, FLY_BY_WIRE_C=7, AUTO=10, RTL=11, LOITER=12, GUIDED=15, INITIALISING=16
 * @param RC_state state: OFF=0, ON=1, FAILSAFE=255
 * @param usec time
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_state_data_aug_pack_chan(
	 uint8_t system_id
	,uint8_t component_id
	,uint8_t chan
	,mavlink_message_t* msg
	,uint8_t num
	,uint8_t atloiter
	,uint8_t mode
	,uint8_t RC_state
	,uint64_t usec
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_STATE_DATA_AUG_LEN];
	_mav_put_uint8_t(buf, 0, num);
	_mav_put_uint8_t(buf, 1, atloiter);
	_mav_put_uint8_t(buf, 2, mode);
	_mav_put_uint8_t(buf, 3, RC_state);
	_mav_put_uint64_t(buf, 4, usec);

	memcpy(_MAV_PAYLOAD(msg), buf, MAVLINK_MSG_ID_STATE_DATA_AUG_LEN);
#else
	mavlink_state_data_aug_t packet;
	packet.num = num;
	packet.atloiter = atloiter;
	packet.mode = mode;
	packet.RC_state = RC_state;
	packet.usec = usec;

	memcpy(_MAV_PAYLOAD(msg), &packet, MAVLINK_MSG_ID_STATE_DATA_AUG_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_STATE_DATA_AUG;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_STATE_DATA_AUG_LEN);
}

/**
 * @brief Encode a state_data_aug struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param state_data_aug C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_state_data_aug_encode(
	 uint8_t system_id
	,uint8_t component_id
	,mavlink_message_t* msg
	,const mavlink_state_data_aug_t* state_data_aug
	)
{
	return mavlink_msg_state_data_aug_pack(
	 system_id
	,component_id
	,msg
	,state_data_aug->num
	,state_data_aug->atloiter
	,state_data_aug->mode
	,state_data_aug->RC_state
	,state_data_aug->usec
	);
}

/**
 * @brief Send a state_data_aug message
 * @param chan The MAVLink channel to send this message
 *
 * @param num Vehicle number, e.g. 1 for R1, 2 for R2, 3 for R3
 * @param atloiter if 0, vehicle is not currently loitering, if 1, vehicle is loitering
 * @param mode mode: MANUAL=0, CIRCLE=1, STABILIZE=2, FLY_BY_WIRE_A=5, FLY_BY_WIRE_B=6, FLY_BY_WIRE_C=7, AUTO=10, RTL=11, LOITER=12, GUIDED=15, INITIALISING=16
 * @param RC_state state: OFF=0, ON=1, FAILSAFE=255
 * @param usec time
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_state_data_aug_send(
	 mavlink_channel_t chan
	,uint8_t num
	,uint8_t atloiter
	,uint8_t mode
	,uint8_t RC_state
	,uint64_t usec
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_STATE_DATA_AUG_LEN];
	_mav_put_uint8_t(buf, 0, num);
	_mav_put_uint8_t(buf, 1, atloiter);
	_mav_put_uint8_t(buf, 2, mode);
	_mav_put_uint8_t(buf, 3, RC_state);
	_mav_put_uint64_t(buf, 4, usec);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATE_DATA_AUG, buf, MAVLINK_MSG_ID_STATE_DATA_AUG_LEN);
#else
	mavlink_state_data_aug_t packet;
	packet.num = num;
	packet.atloiter = atloiter;
	packet.mode = mode;
	packet.RC_state = RC_state;
	packet.usec = usec;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATE_DATA_AUG, (const char*)&packet, MAVLINK_MSG_ID_STATE_DATA_AUG_LEN);
#endif
}

static inline void mavlink_msg_wID_state_data_aug_send(
	 mavlink_channel_t chan
	,uint8_t sID
	,uint8_t cID
	,uint8_t num
	,uint8_t atloiter
	,uint8_t mode
	,uint8_t RC_state
	,uint64_t usec
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_STATE_DATA_AUG_LEN];
	_mav_put_uint8_t(buf, 0, num);
	_mav_put_uint8_t(buf, 1, atloiter);
	_mav_put_uint8_t(buf, 2, mode);
	_mav_put_uint8_t(buf, 3, RC_state);
	_mav_put_uint64_t(buf, 4, usec);

	_mav_wID_finalize_message_chan_send(chan, sID, cID, MAVLINK_MSG_ID_STATE_DATA_AUG, buf, MAVLINK_MSG_ID_STATE_DATA_AUG_LEN);
#else
	mavlink_state_data_aug_t packet;
	packet.num = num;
	packet.atloiter = atloiter;
	packet.mode = mode;
	packet.RC_state = RC_state;
	packet.usec = usec;

	_mav_wID_finalize_message_chan_send(chan, sID, cID, MAVLINK_MSG_ID_STATE_DATA_AUG, (const char*)&packet, MAVLINK_MSG_ID_STATE_DATA_AUG_LEN);
#endif
}

#endif // MAVLINK_USE_CONVENIENCE_FUNCTIONS

// MESSAGE STATE_DATA_AUG UNPACKING

/**
 * @brief Get field num from state_data_aug message
 *
 * @return Vehicle number, e.g. 1 for R1, 2 for R2, 3 for R3
 */
static inline uint8_t mavlink_msg_state_data_aug_get_num(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,0);
}

/**
 * @brief Get field atloiter from state_data_aug message
 *
 * @return if 0, vehicle is not currently loitering, if 1, vehicle is loitering
 */
static inline uint8_t mavlink_msg_state_data_aug_get_atloiter(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,1);
}

/**
 * @brief Get field mode from state_data_aug message
 *
 * @return mode: MANUAL=0, CIRCLE=1, STABILIZE=2, FLY_BY_WIRE_A=5, FLY_BY_WIRE_B=6, FLY_BY_WIRE_C=7, AUTO=10, RTL=11, LOITER=12, GUIDED=15, INITIALISING=16
 */
static inline uint8_t mavlink_msg_state_data_aug_get_mode(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,2);
}

/**
 * @brief Get field RC_state from state_data_aug message
 *
 * @return state: OFF=0, ON=1, FAILSAFE=255
 */
static inline uint8_t mavlink_msg_state_data_aug_get_RC_state(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,3);
}

/**
 * @brief Get field usec from state_data_aug message
 *
 * @return time
 */
static inline uint64_t mavlink_msg_state_data_aug_get_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,4);
}

/**
 * @brief Decode a state_data_aug message into a struct
 *
 * @param msg The message to decode
 * @param state_data_aug C-struct to decode the message contents into
 */
static inline void mavlink_msg_state_data_aug_decode(const mavlink_message_t* msg, mavlink_state_data_aug_t* state_data_aug)
{
#if MAVLINK_NEED_BYTE_SWAP
	state_data_aug->num = mavlink_msg_state_data_aug_get_num(msg);
	state_data_aug->atloiter = mavlink_msg_state_data_aug_get_atloiter(msg);
	state_data_aug->mode = mavlink_msg_state_data_aug_get_mode(msg);
	state_data_aug->RC_state = mavlink_msg_state_data_aug_get_RC_state(msg);
	state_data_aug->usec = mavlink_msg_state_data_aug_get_usec(msg);
#else
	memcpy(state_data_aug, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_STATE_DATA_AUG_LEN);
#endif
}
