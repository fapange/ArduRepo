// MESSAGE SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST PACKING
// Generated from parse_APM_XML.m on 19-Aug-2013 09:01:21

#define MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST 61 

typedef struct __mavlink_set_quad_swarm_roll_pitch_yaw_thrust_t 
{ 
  uint8_t group;  ///< ID of the quadrotor group (0 - 255, up to 256 groups supported)
  uint8_t mode;  ///< ID of the flight mode (0 - 255, up to 256 modes supported)
  int16_t roll[4];  ///< Desired roll angle in radians +-PI (+-32767)
  int16_t pitch[4];  ///< Desired pitch angle in radians +-PI (+-32767)
  int16_t yaw[4];  ///< Desired yaw angle in radians, scaled to int16 +-PI (+-32767)
  uint16_t thrust[4];  ///< Collective thrust, scaled to uint16 (0..65535)
} mavlink_set_quad_swarm_roll_pitch_yaw_thrust_t;

#define MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST_LEN 34
#define MAVLINK_MSG_ID_61_LEN 34

#define MAVLINK_MESSAGE_INFO_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST \
{ \
  "SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST", \
  6, \
  { \
    { "group", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_set_quad_swarm_roll_pitch_yaw_thrust_t, group) }, \
    { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_set_quad_swarm_roll_pitch_yaw_thrust_t, mode) }, \
    { "roll", NULL, MAVLINK_TYPE_INT16_T, 4, 2, offsetof(mavlink_set_quad_swarm_roll_pitch_yaw_thrust_t, roll) }, \
    { "pitch", NULL, MAVLINK_TYPE_INT16_T, 4, 10, offsetof(mavlink_set_quad_swarm_roll_pitch_yaw_thrust_t, pitch) }, \
    { "yaw", NULL, MAVLINK_TYPE_INT16_T, 4, 18, offsetof(mavlink_set_quad_swarm_roll_pitch_yaw_thrust_t, yaw) }, \
    { "thrust", NULL, MAVLINK_TYPE_UINT16_T, 4, 26, offsetof(mavlink_set_quad_swarm_roll_pitch_yaw_thrust_t, thrust) }, \
  } \
}

/**
 * @brief Pack a set_quad_swarm_roll_pitch_yaw_thrust message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param group ID of the quadrotor group (0 - 255, up to 256 groups supported)
 * @param mode ID of the flight mode (0 - 255, up to 256 modes supported)
 * @param roll Desired roll angle in radians +-PI (+-32767)
 * @param pitch Desired pitch angle in radians +-PI (+-32767)
 * @param yaw Desired yaw angle in radians, scaled to int16 +-PI (+-32767)
 * @param thrust Collective thrust, scaled to uint16 (0..65535)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_pack(
	 uint8_t system_id
	,uint8_t component_id
	,mavlink_message_t* msg
	,uint8_t group
	,uint8_t mode
	,const int16_t* roll
	,const int16_t* pitch
	,const int16_t* yaw
	,const uint16_t* thrust
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST_LEN];
	_mav_put_uint8_t(buf, 0, group);
	_mav_put_uint8_t(buf, 1, mode);
	_mav_put_int16_t_array(buf, 2, roll, 4);
	_mav_put_int16_t_array(buf, 10, pitch, 4);
	_mav_put_int16_t_array(buf, 18, yaw, 4);
	_mav_put_uint16_t_array(buf, 26, thrust, 4);

	memcpy(_MAV_PAYLOAD(msg), buf, MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST_LEN);
#else
	mavlink_set_quad_swarm_roll_pitch_yaw_thrust_t packet;
	packet.group = group;
	packet.mode = mode;
	mav_array_memcpy(packet.roll, roll, sizeof(int16_t)*4);
	mav_array_memcpy(packet.pitch, pitch, sizeof(int16_t)*4);
	mav_array_memcpy(packet.yaw, yaw, sizeof(int16_t)*4);
	mav_array_memcpy(packet.thrust, thrust, sizeof(uint16_t)*4);

	memcpy(_MAV_PAYLOAD(msg), &packet, MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST;
	return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST_LEN);
}

/**
 * @brief Pack a set_quad_swarm_roll_pitch_yaw_thrust message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 *
 * @param group ID of the quadrotor group (0 - 255, up to 256 groups supported)
 * @param mode ID of the flight mode (0 - 255, up to 256 modes supported)
 * @param roll Desired roll angle in radians +-PI (+-32767)
 * @param pitch Desired pitch angle in radians +-PI (+-32767)
 * @param yaw Desired yaw angle in radians, scaled to int16 +-PI (+-32767)
 * @param thrust Collective thrust, scaled to uint16 (0..65535)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_pack_chan(
	 uint8_t system_id
	,uint8_t component_id
	,uint8_t chan
	,mavlink_message_t* msg
	,uint8_t group
	,uint8_t mode
	,const int16_t* roll
	,const int16_t* pitch
	,const int16_t* yaw
	,const uint16_t* thrust
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST_LEN];
	_mav_put_uint8_t(buf, 0, group);
	_mav_put_uint8_t(buf, 1, mode);
	_mav_put_int16_t_array(buf, 2, roll, 4);
	_mav_put_int16_t_array(buf, 10, pitch, 4);
	_mav_put_int16_t_array(buf, 18, yaw, 4);
	_mav_put_uint16_t_array(buf, 26, thrust, 4);

	memcpy(_MAV_PAYLOAD(msg), buf, MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST_LEN);
#else
	mavlink_set_quad_swarm_roll_pitch_yaw_thrust_t packet;
	packet.group = group;
	packet.mode = mode;
	mav_array_memcpy(packet.roll, roll, sizeof(int16_t)*4);
	mav_array_memcpy(packet.pitch, pitch, sizeof(int16_t)*4);
	mav_array_memcpy(packet.yaw, yaw, sizeof(int16_t)*4);
	mav_array_memcpy(packet.thrust, thrust, sizeof(uint16_t)*4);

	memcpy(_MAV_PAYLOAD(msg), &packet, MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST_LEN);
}

/**
 * @brief Encode a set_quad_swarm_roll_pitch_yaw_thrust struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param vfr_hud C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_encode(
	 uint8_t system_id
	,uint8_t component_id
	,mavlink_message_t* msg
	,const mavlink_set_quad_swarm_roll_pitch_yaw_thrust_t* set_quad_swarm_roll_pitch_yaw_thrust
	)
{
	return mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_pack(
	 system_id
	,component_id
	,msg
	,set_quad_swarm_roll_pitch_yaw_thrust->group
	,set_quad_swarm_roll_pitch_yaw_thrust->mode
	,set_quad_swarm_roll_pitch_yaw_thrust->roll
	,set_quad_swarm_roll_pitch_yaw_thrust->pitch
	,set_quad_swarm_roll_pitch_yaw_thrust->yaw
	,set_quad_swarm_roll_pitch_yaw_thrust->thrust
	);
}

/**
 * @brief Send a set_quad_swarm_roll_pitch_yaw_thrust message
 * @param chan The MAVLink channel to send this message
 *
 * @param group ID of the quadrotor group (0 - 255, up to 256 groups supported)
 * @param mode ID of the flight mode (0 - 255, up to 256 modes supported)
 * @param roll Desired roll angle in radians +-PI (+-32767)
 * @param pitch Desired pitch angle in radians +-PI (+-32767)
 * @param yaw Desired yaw angle in radians, scaled to int16 +-PI (+-32767)
 * @param thrust Collective thrust, scaled to uint16 (0..65535)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_send(
	 mavlink_channel_t chan
	,uint8_t group
	,uint8_t mode
	,const int16_t* roll
	,const int16_t* pitch
	,const int16_t* yaw
	,const uint16_t* thrust
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST_LEN];
	_mav_put_uint8_t(buf, 0, group);
	_mav_put_uint8_t(buf, 1, mode);
	_mav_put_int16_t_array(buf, 2, roll, 4);
	_mav_put_int16_t_array(buf, 10, pitch, 4);
	_mav_put_int16_t_array(buf, 18, yaw, 4);
	_mav_put_uint16_t_array(buf, 26, thrust, 4);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST, buf, MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST_LEN);
#else
	mavlink_set_quad_swarm_roll_pitch_yaw_thrust_t packet;
	packet.group = group;
	packet.mode = mode;
	mav_array_memcpy(packet.roll, roll, sizeof(int16_t)*4);
	mav_array_memcpy(packet.pitch, pitch, sizeof(int16_t)*4);
	mav_array_memcpy(packet.yaw, yaw, sizeof(int16_t)*4);
	mav_array_memcpy(packet.thrust, thrust, sizeof(uint16_t)*4);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST, (const char*)&packet, MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST_LEN);
#endif
}

#endif // MAVLINK_USE_CONVENIENCE_FUNCTIONS

// MESSAGE SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST UNPACKING

/**
 * @brief Get field group from set_quad_swarm_roll_pitch_yaw_thrust message
 *
 * @return ID of the quadrotor group (0 - 255, up to 256 groups supported)
 */
static inline uint8_t mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_get_group(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,0);
}

/**
 * @brief Get field mode from set_quad_swarm_roll_pitch_yaw_thrust message
 *
 * @return ID of the flight mode (0 - 255, up to 256 modes supported)
 */
static inline uint8_t mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_get_mode(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,1);
}

/**
 * @brief Get field roll from set_quad_swarm_roll_pitch_yaw_thrust message
 *
 * @return Desired roll angle in radians +-PI (+-32767)
 */
static inline int16_t mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_get_roll(const mavlink_message_t* msg, int16_t* roll)
{
	return _MAV_RETURN_int16_t_array(msg,roll,4,2);
}

/**
 * @brief Get field pitch from set_quad_swarm_roll_pitch_yaw_thrust message
 *
 * @return Desired pitch angle in radians +-PI (+-32767)
 */
static inline int16_t mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_get_pitch(const mavlink_message_t* msg, int16_t* pitch)
{
	return _MAV_RETURN_int16_t_array(msg,pitch,4,10);
}

/**
 * @brief Get field yaw from set_quad_swarm_roll_pitch_yaw_thrust message
 *
 * @return Desired yaw angle in radians, scaled to int16 +-PI (+-32767)
 */
static inline int16_t mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_get_yaw(const mavlink_message_t* msg, int16_t* yaw)
{
	return _MAV_RETURN_int16_t_array(msg,yaw,4,18);
}

/**
 * @brief Get field thrust from set_quad_swarm_roll_pitch_yaw_thrust message
 *
 * @return Collective thrust, scaled to uint16 (0..65535)
 */
static inline uint16_t mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_get_thrust(const mavlink_message_t* msg, uint16_t* thrust)
{
	return _MAV_RETURN_uint16_t_array(msg,thrust,4,26);
}

/**
 * @brief Decode a set_quad_swarm_roll_pitch_yaw_thrust message into a struct
 *
 * @param msg The message to decode
 * @param set_quad_swarm_roll_pitch_yaw_thrust C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_decode(const mavlink_message_t* msg, mavlink_set_quad_swarm_roll_pitch_yaw_thrust_t* set_quad_swarm_roll_pitch_yaw_thrust)
{
#if MAVLINK_NEED_BYTE_SWAP
	set_quad_swarm_roll_pitch_yaw_thrust->group = mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_get_group(msg);
	set_quad_swarm_roll_pitch_yaw_thrust->mode = mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_get_mode(msg);
	mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_get_roll(msg, set_quad_swarm_roll_pitch_yaw_thrust->roll);
	mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_get_pitch(msg, set_quad_swarm_roll_pitch_yaw_thrust->pitch);
	mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_get_yaw(msg, set_quad_swarm_roll_pitch_yaw_thrust->yaw);
	mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_get_thrust(msg, set_quad_swarm_roll_pitch_yaw_thrust->thrust);
#else
	memcpy(set_quad_swarm_roll_pitch_yaw_thrust, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST_LEN);
#endif
}