// MESSAGE COMMAND_LONG PACKING
// Generated from parse_APM_XML.m on 06-Jun-2013 10:07:53

#define MAVLINK_MSG_ID_COMMAND_LONG 76 

typedef struct __mavlink_command_long_t 
{ 
  uint8_t target_system;  ///< System which should execute the command
  uint8_t target_component;  ///< Component which should execute the command, 0 for all components
  uint16_t command;  ///< Command ID, as defined by MAV_CMD enum.
  uint8_t confirmation;  ///< 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
  float param1;  ///< Parameter 1, as defined by MAV_CMD enum.
  float param2;  ///< Parameter 2, as defined by MAV_CMD enum.
  float param3;  ///< Parameter 3, as defined by MAV_CMD enum.
  float param4;  ///< Parameter 4, as defined by MAV_CMD enum.
  float param5;  ///< Parameter 5, as defined by MAV_CMD enum.
  float param6;  ///< Parameter 6, as defined by MAV_CMD enum.
  float param7;  ///< Parameter 7, as defined by MAV_CMD enum.
} mavlink_command_long_t;

#define MAVLINK_MSG_ID_COMMAND_LONG_LEN 33
#define MAVLINK_MSG_ID_76_LEN 33

#define MAVLINK_MESSAGE_INFO_COMMAND_LONG \
{ \
  "COMMAND_LONG", \
  11, \
  { \
    { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_command_long_t, target_system) }, \
    { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_command_long_t, target_component) }, \
    { "command", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_command_long_t, command) }, \
    { "confirmation", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_command_long_t, confirmation) }, \
    { "param1", NULL, MAVLINK_TYPE_FLOAT, 0, 5, offsetof(mavlink_command_long_t, param1) }, \
    { "param2", NULL, MAVLINK_TYPE_FLOAT, 0, 9, offsetof(mavlink_command_long_t, param2) }, \
    { "param3", NULL, MAVLINK_TYPE_FLOAT, 0, 13, offsetof(mavlink_command_long_t, param3) }, \
    { "param4", NULL, MAVLINK_TYPE_FLOAT, 0, 17, offsetof(mavlink_command_long_t, param4) }, \
    { "param5", NULL, MAVLINK_TYPE_FLOAT, 0, 21, offsetof(mavlink_command_long_t, param5) }, \
    { "param6", NULL, MAVLINK_TYPE_FLOAT, 0, 25, offsetof(mavlink_command_long_t, param6) }, \
    { "param7", NULL, MAVLINK_TYPE_FLOAT, 0, 29, offsetof(mavlink_command_long_t, param7) }, \
  } \
}

/**
 * @brief Pack a command_long message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System which should execute the command
 * @param target_component Component which should execute the command, 0 for all components
 * @param command Command ID, as defined by MAV_CMD enum.
 * @param confirmation 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
 * @param param1 Parameter 1, as defined by MAV_CMD enum.
 * @param param2 Parameter 2, as defined by MAV_CMD enum.
 * @param param3 Parameter 3, as defined by MAV_CMD enum.
 * @param param4 Parameter 4, as defined by MAV_CMD enum.
 * @param param5 Parameter 5, as defined by MAV_CMD enum.
 * @param param6 Parameter 6, as defined by MAV_CMD enum.
 * @param param7 Parameter 7, as defined by MAV_CMD enum.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_command_long_pack(
	 uint8_t system_id
	,uint8_t component_id
	,mavlink_message_t* msg
	,uint8_t target_system
	,uint8_t target_component
	,uint16_t command
	,uint8_t confirmation
	,float param1
	,float param2
	,float param3
	,float param4
	,float param5
	,float param6
	,float param7
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_uint16_t(buf, 2, command);
	_mav_put_uint8_t(buf, 4, confirmation);
	_mav_put_float(buf, 5, param1);
	_mav_put_float(buf, 9, param2);
	_mav_put_float(buf, 13, param3);
	_mav_put_float(buf, 17, param4);
	_mav_put_float(buf, 21, param5);
	_mav_put_float(buf, 25, param6);
	_mav_put_float(buf, 29, param7);

	memcpy(_MAV_PAYLOAD(msg), buf, MAVLINK_MSG_ID_COMMAND_LONG_LEN);
#else
	mavlink_command_long_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.command = command;
	packet.confirmation = confirmation;
	packet.param1 = param1;
	packet.param2 = param2;
	packet.param3 = param3;
	packet.param4 = param4;
	packet.param5 = param5;
	packet.param6 = param6;
	packet.param7 = param7;

	memcpy(_MAV_PAYLOAD(msg), &packet, MAVLINK_MSG_ID_COMMAND_LONG_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_COMMAND_LONG;
	return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_COMMAND_LONG_LEN);
}

/**
 * @brief Pack a command_long message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System which should execute the command
 * @param target_component Component which should execute the command, 0 for all components
 * @param command Command ID, as defined by MAV_CMD enum.
 * @param confirmation 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
 * @param param1 Parameter 1, as defined by MAV_CMD enum.
 * @param param2 Parameter 2, as defined by MAV_CMD enum.
 * @param param3 Parameter 3, as defined by MAV_CMD enum.
 * @param param4 Parameter 4, as defined by MAV_CMD enum.
 * @param param5 Parameter 5, as defined by MAV_CMD enum.
 * @param param6 Parameter 6, as defined by MAV_CMD enum.
 * @param param7 Parameter 7, as defined by MAV_CMD enum.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_command_long_pack_chan(
	 uint8_t system_id
	,uint8_t component_id
	,uint8_t chan
	,mavlink_message_t* msg
	,uint8_t target_system
	,uint8_t target_component
	,uint16_t command
	,uint8_t confirmation
	,float param1
	,float param2
	,float param3
	,float param4
	,float param5
	,float param6
	,float param7
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_uint16_t(buf, 2, command);
	_mav_put_uint8_t(buf, 4, confirmation);
	_mav_put_float(buf, 5, param1);
	_mav_put_float(buf, 9, param2);
	_mav_put_float(buf, 13, param3);
	_mav_put_float(buf, 17, param4);
	_mav_put_float(buf, 21, param5);
	_mav_put_float(buf, 25, param6);
	_mav_put_float(buf, 29, param7);

	memcpy(_MAV_PAYLOAD(msg), buf, MAVLINK_MSG_ID_COMMAND_LONG_LEN);
#else
	mavlink_command_long_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.command = command;
	packet.confirmation = confirmation;
	packet.param1 = param1;
	packet.param2 = param2;
	packet.param3 = param3;
	packet.param4 = param4;
	packet.param5 = param5;
	packet.param6 = param6;
	packet.param7 = param7;

	memcpy(_MAV_PAYLOAD(msg), &packet, MAVLINK_MSG_ID_COMMAND_LONG_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_COMMAND_LONG;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_COMMAND_LONG_LEN);
}

/**
 * @brief Encode a command_long struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param vfr_hud C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_command_long_encode(
	 uint8_t system_id
	,uint8_t component_id
	,mavlink_message_t* msg
	,const mavlink_command_long_t* command_long
	)
{
	return mavlink_msg_command_long_pack(
	 system_id
	,component_id
	,msg
	,command_long->target_system
	,command_long->target_component
	,command_long->command
	,command_long->confirmation
	,command_long->param1
	,command_long->param2
	,command_long->param3
	,command_long->param4
	,command_long->param5
	,command_long->param6
	,command_long->param7
	);
}

/**
 * @brief Send a command_long message
 * @param chan The MAVLink channel to send this message
 *
 * @param target_system System which should execute the command
 * @param target_component Component which should execute the command, 0 for all components
 * @param command Command ID, as defined by MAV_CMD enum.
 * @param confirmation 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
 * @param param1 Parameter 1, as defined by MAV_CMD enum.
 * @param param2 Parameter 2, as defined by MAV_CMD enum.
 * @param param3 Parameter 3, as defined by MAV_CMD enum.
 * @param param4 Parameter 4, as defined by MAV_CMD enum.
 * @param param5 Parameter 5, as defined by MAV_CMD enum.
 * @param param6 Parameter 6, as defined by MAV_CMD enum.
 * @param param7 Parameter 7, as defined by MAV_CMD enum.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_command_long_send(
	 mavlink_channel_t chan
	,uint8_t target_system
	,uint8_t target_component
	,uint16_t command
	,uint8_t confirmation
	,float param1
	,float param2
	,float param3
	,float param4
	,float param5
	,float param6
	,float param7
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_uint16_t(buf, 2, command);
	_mav_put_uint8_t(buf, 4, confirmation);
	_mav_put_float(buf, 5, param1);
	_mav_put_float(buf, 9, param2);
	_mav_put_float(buf, 13, param3);
	_mav_put_float(buf, 17, param4);
	_mav_put_float(buf, 21, param5);
	_mav_put_float(buf, 25, param6);
	_mav_put_float(buf, 29, param7);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMMAND_LONG, buf, MAVLINK_MSG_ID_COMMAND_LONG_LEN);
#else
	mavlink_command_long_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.command = command;
	packet.confirmation = confirmation;
	packet.param1 = param1;
	packet.param2 = param2;
	packet.param3 = param3;
	packet.param4 = param4;
	packet.param5 = param5;
	packet.param6 = param6;
	packet.param7 = param7;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMMAND_LONG, (const char*)&packet, MAVLINK_MSG_ID_COMMAND_LONG_LEN);
#endif
}

#endif // MAVLINK_USE_CONVENIENCE_FUNCTIONS

// MESSAGE COMMAND_LONG UNPACKING

/**
 * @brief Get field target_system from command_long message
 *
 * @return System which should execute the command
 */
static inline uint8_t mavlink_msg_command_long_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,0);
}

/**
 * @brief Get field target_component from command_long message
 *
 * @return Component which should execute the command, 0 for all components
 */
static inline uint8_t mavlink_msg_command_long_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,1);
}

/**
 * @brief Get field command from command_long message
 *
 * @return Command ID, as defined by MAV_CMD enum.
 */
static inline uint16_t mavlink_msg_command_long_get_command(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,2);
}

/**
 * @brief Get field confirmation from command_long message
 *
 * @return 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
 */
static inline uint8_t mavlink_msg_command_long_get_confirmation(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,4);
}

/**
 * @brief Get field param1 from command_long message
 *
 * @return Parameter 1, as defined by MAV_CMD enum.
 */
static inline float mavlink_msg_command_long_get_param1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,5);
}

/**
 * @brief Get field param2 from command_long message
 *
 * @return Parameter 2, as defined by MAV_CMD enum.
 */
static inline float mavlink_msg_command_long_get_param2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,9);
}

/**
 * @brief Get field param3 from command_long message
 *
 * @return Parameter 3, as defined by MAV_CMD enum.
 */
static inline float mavlink_msg_command_long_get_param3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,13);
}

/**
 * @brief Get field param4 from command_long message
 *
 * @return Parameter 4, as defined by MAV_CMD enum.
 */
static inline float mavlink_msg_command_long_get_param4(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,17);
}

/**
 * @brief Get field param5 from command_long message
 *
 * @return Parameter 5, as defined by MAV_CMD enum.
 */
static inline float mavlink_msg_command_long_get_param5(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,21);
}

/**
 * @brief Get field param6 from command_long message
 *
 * @return Parameter 6, as defined by MAV_CMD enum.
 */
static inline float mavlink_msg_command_long_get_param6(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,25);
}

/**
 * @brief Get field param7 from command_long message
 *
 * @return Parameter 7, as defined by MAV_CMD enum.
 */
static inline float mavlink_msg_command_long_get_param7(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,29);
}

/**
 * @brief Decode a command_long message into a struct
 *
 * @param msg The message to decode
 * @param command_long C-struct to decode the message contents into
 */
static inline void mavlink_msg_command_long_decode(const mavlink_message_t* msg, mavlink_command_long_t* command_long)
{
#if MAVLINK_NEED_BYTE_SWAP
	command_long->target_system = mavlink_msg_command_long_get_target_system(msg);
	command_long->target_component = mavlink_msg_command_long_get_target_component(msg);
	command_long->command = mavlink_msg_command_long_get_command(msg);
	command_long->confirmation = mavlink_msg_command_long_get_confirmation(msg);
	command_long->param1 = mavlink_msg_command_long_get_param1(msg);
	command_long->param2 = mavlink_msg_command_long_get_param2(msg);
	command_long->param3 = mavlink_msg_command_long_get_param3(msg);
	command_long->param4 = mavlink_msg_command_long_get_param4(msg);
	command_long->param5 = mavlink_msg_command_long_get_param5(msg);
	command_long->param6 = mavlink_msg_command_long_get_param6(msg);
	command_long->param7 = mavlink_msg_command_long_get_param7(msg);
#else
	memcpy(command_long, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_COMMAND_LONG_LEN);
#endif
}
