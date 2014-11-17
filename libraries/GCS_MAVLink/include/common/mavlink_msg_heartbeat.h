// MESSAGE HEARTBEAT PACKING
// Generated from parse_APM_XML.m on 24-Feb-2014 13:50:53

#define MAVLINK_MSG_ID_HEARTBEAT 0 

typedef struct __mavlink_heartbeat_t 
{ 
  uint8_t type;  ///< Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
  uint8_t autopilot;  ///< Type of the Autopilot: 0: Generic, 1: PIXHAWK, 2: SLUGS, 3: Ardupilot (up to 15 types), defined in MAV_AUTOPILOT_TYPE ENUM
  uint8_t mavlink_version;  ///< MAVLink version
  uint8_t vehicleID;  ///< Vehicle ID: 1: R1, 2: R2, 3: R3 (up to xx types), defined in "mydefines.h"
  uint8_t numWpts;  ///< Number of Waypoints in Flight Plan
} mavlink_heartbeat_t;

#define MAVLINK_MSG_ID_HEARTBEAT_LEN 5
#define MAVLINK_MSG_ID_0_LEN 5

#define MAVLINK_MESSAGE_INFO_HEARTBEAT \
{ \
  "HEARTBEAT", \
  5, \
  { \
    { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_heartbeat_t, type) }, \
    { "autopilot", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_heartbeat_t, autopilot) }, \
    { "mavlink_version", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_heartbeat_t, mavlink_version) }, \
    { "vehicleID", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_heartbeat_t, vehicleID) }, \
    { "numWpts", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_heartbeat_t, numWpts) }, \
  } \
}

/**
 * @brief Pack a heartbeat message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param type Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
 * @param autopilot Type of the Autopilot: 0: Generic, 1: PIXHAWK, 2: SLUGS, 3: Ardupilot (up to 15 types), defined in MAV_AUTOPILOT_TYPE ENUM
 * @param mavlink_version MAVLink version
 * @param vehicleID Vehicle ID: 1: R1, 2: R2, 3: R3 (up to xx types), defined in "mydefines.h"
 * @param numWpts Number of Waypoints in Flight Plan
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_heartbeat_pack(
	 uint8_t system_id
	,uint8_t component_id
	,mavlink_message_t* msg
	,uint8_t type
	,uint8_t autopilot
	,uint8_t mavlink_version
	,uint8_t vehicleID
	,uint8_t numWpts
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_HEARTBEAT_LEN];
	_mav_put_uint8_t(buf, 0, type);
	_mav_put_uint8_t(buf, 1, autopilot);
	_mav_put_uint8_t(buf, 2, mavlink_version);
	_mav_put_uint8_t(buf, 3, vehicleID);
	_mav_put_uint8_t(buf, 4, numWpts);

	memcpy(_MAV_PAYLOAD(msg), buf, MAVLINK_MSG_ID_HEARTBEAT_LEN);
#else
	mavlink_heartbeat_t packet;
	packet.type = type;
	packet.autopilot = autopilot;
	packet.mavlink_version = mavlink_version;
	packet.vehicleID = vehicleID;
	packet.numWpts = numWpts;

	memcpy(_MAV_PAYLOAD(msg), &packet, MAVLINK_MSG_ID_HEARTBEAT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_HEARTBEAT;
	return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_HEARTBEAT_LEN);
}

/**
 * @brief Pack a heartbeat message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 *
 * @param type Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
 * @param autopilot Type of the Autopilot: 0: Generic, 1: PIXHAWK, 2: SLUGS, 3: Ardupilot (up to 15 types), defined in MAV_AUTOPILOT_TYPE ENUM
 * @param mavlink_version MAVLink version
 * @param vehicleID Vehicle ID: 1: R1, 2: R2, 3: R3 (up to xx types), defined in "mydefines.h"
 * @param numWpts Number of Waypoints in Flight Plan
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_heartbeat_pack_chan(
	 uint8_t system_id
	,uint8_t component_id
	,uint8_t chan
	,mavlink_message_t* msg
	,uint8_t type
	,uint8_t autopilot
	,uint8_t mavlink_version
	,uint8_t vehicleID
	,uint8_t numWpts
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_HEARTBEAT_LEN];
	_mav_put_uint8_t(buf, 0, type);
	_mav_put_uint8_t(buf, 1, autopilot);
	_mav_put_uint8_t(buf, 2, mavlink_version);
	_mav_put_uint8_t(buf, 3, vehicleID);
	_mav_put_uint8_t(buf, 4, numWpts);

	memcpy(_MAV_PAYLOAD(msg), buf, MAVLINK_MSG_ID_HEARTBEAT_LEN);
#else
	mavlink_heartbeat_t packet;
	packet.type = type;
	packet.autopilot = autopilot;
	packet.mavlink_version = mavlink_version;
	packet.vehicleID = vehicleID;
	packet.numWpts = numWpts;

	memcpy(_MAV_PAYLOAD(msg), &packet, MAVLINK_MSG_ID_HEARTBEAT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_HEARTBEAT;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_HEARTBEAT_LEN);
}

/**
 * @brief Encode a heartbeat struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param heartbeat C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_heartbeat_encode(
	 uint8_t system_id
	,uint8_t component_id
	,mavlink_message_t* msg
	,const mavlink_heartbeat_t* heartbeat
	)
{
	return mavlink_msg_heartbeat_pack(
	 system_id
	,component_id
	,msg
	,heartbeat->type
	,heartbeat->autopilot
	,heartbeat->mavlink_version
	,heartbeat->vehicleID
	,heartbeat->numWpts
	);
}

/**
 * @brief Send a heartbeat message
 * @param chan The MAVLink channel to send this message
 *
 * @param type Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
 * @param autopilot Type of the Autopilot: 0: Generic, 1: PIXHAWK, 2: SLUGS, 3: Ardupilot (up to 15 types), defined in MAV_AUTOPILOT_TYPE ENUM
 * @param mavlink_version MAVLink version
 * @param vehicleID Vehicle ID: 1: R1, 2: R2, 3: R3 (up to xx types), defined in "mydefines.h"
 * @param numWpts Number of Waypoints in Flight Plan
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_heartbeat_send(
	 mavlink_channel_t chan
	,uint8_t type
	,uint8_t autopilot
	,uint8_t mavlink_version
	,uint8_t vehicleID
	,uint8_t numWpts
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_HEARTBEAT_LEN];
	_mav_put_uint8_t(buf, 0, type);
	_mav_put_uint8_t(buf, 1, autopilot);
	_mav_put_uint8_t(buf, 2, mavlink_version);
	_mav_put_uint8_t(buf, 3, vehicleID);
	_mav_put_uint8_t(buf, 4, numWpts);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HEARTBEAT, buf, MAVLINK_MSG_ID_HEARTBEAT_LEN);
#else
	mavlink_heartbeat_t packet;
	packet.type = type;
	packet.autopilot = autopilot;
	packet.mavlink_version = mavlink_version;
	packet.vehicleID = vehicleID;
	packet.numWpts = numWpts;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HEARTBEAT, (const char*)&packet, MAVLINK_MSG_ID_HEARTBEAT_LEN);
#endif
}

static inline void mavlink_msg_wID_heartbeat_send(
	 mavlink_channel_t chan
	,uint8_t sID
	,uint8_t cID
	,uint8_t type
	,uint8_t autopilot
	,uint8_t mavlink_version
	,uint8_t vehicleID
	,uint8_t numWpts
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_HEARTBEAT_LEN];
	_mav_put_uint8_t(buf, 0, type);
	_mav_put_uint8_t(buf, 1, autopilot);
	_mav_put_uint8_t(buf, 2, mavlink_version);
	_mav_put_uint8_t(buf, 3, vehicleID);
	_mav_put_uint8_t(buf, 4, numWpts);

	_mav_wID_finalize_message_chan_send(chan, sID, cID, MAVLINK_MSG_ID_HEARTBEAT, buf, MAVLINK_MSG_ID_HEARTBEAT_LEN);
#else
	mavlink_heartbeat_t packet;
	packet.type = type;
	packet.autopilot = autopilot;
	packet.mavlink_version = mavlink_version;
	packet.vehicleID = vehicleID;
	packet.numWpts = numWpts;

	_mav_wID_finalize_message_chan_send(chan, sID, cID, MAVLINK_MSG_ID_HEARTBEAT, (const char*)&packet, MAVLINK_MSG_ID_HEARTBEAT_LEN);
#endif
}

#endif // MAVLINK_USE_CONVENIENCE_FUNCTIONS

// MESSAGE HEARTBEAT UNPACKING

/**
 * @brief Get field type from heartbeat message
 *
 * @return Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
 */
static inline uint8_t mavlink_msg_heartbeat_get_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,0);
}

/**
 * @brief Get field autopilot from heartbeat message
 *
 * @return Type of the Autopilot: 0: Generic, 1: PIXHAWK, 2: SLUGS, 3: Ardupilot (up to 15 types), defined in MAV_AUTOPILOT_TYPE ENUM
 */
static inline uint8_t mavlink_msg_heartbeat_get_autopilot(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,1);
}

/**
 * @brief Get field mavlink_version from heartbeat message
 *
 * @return MAVLink version
 */
static inline uint8_t mavlink_msg_heartbeat_get_mavlink_version(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,2);
}

/**
 * @brief Get field vehicleID from heartbeat message
 *
 * @return Vehicle ID: 1: R1, 2: R2, 3: R3 (up to xx types), defined in "mydefines.h"
 */
static inline uint8_t mavlink_msg_heartbeat_get_vehicleID(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,3);
}

/**
 * @brief Get field numWpts from heartbeat message
 *
 * @return Number of Waypoints in Flight Plan
 */
static inline uint8_t mavlink_msg_heartbeat_get_numWpts(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,4);
}

/**
 * @brief Decode a heartbeat message into a struct
 *
 * @param msg The message to decode
 * @param heartbeat C-struct to decode the message contents into
 */
static inline void mavlink_msg_heartbeat_decode(const mavlink_message_t* msg, mavlink_heartbeat_t* heartbeat)
{
#if MAVLINK_NEED_BYTE_SWAP
	heartbeat->type = mavlink_msg_heartbeat_get_type(msg);
	heartbeat->autopilot = mavlink_msg_heartbeat_get_autopilot(msg);
	heartbeat->mavlink_version = mavlink_msg_heartbeat_get_mavlink_version(msg);
	heartbeat->vehicleID = mavlink_msg_heartbeat_get_vehicleID(msg);
	heartbeat->numWpts = mavlink_msg_heartbeat_get_numWpts(msg);
#else
	memcpy(heartbeat, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_HEARTBEAT_LEN);
#endif
}
