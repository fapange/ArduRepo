// MESSAGE OBJECT_DETECTION_EVENT PACKING
// Generated from parse_APM_XML.m on 24-Feb-2014 13:50:53

#define MAVLINK_MSG_ID_OBJECT_DETECTION_EVENT 140 

typedef struct __mavlink_object_detection_event_t 
{ 
  uint32_t time;  ///< Timestamp in milliseconds since system boot
  uint16_t object_id;  ///< Object ID
  uint8_t type;  ///< Object type: 0: image, 1: letter, 2: ground vehicle, 3: air vehicle, 4: surface vehicle, 5: sub-surface vehicle, 6: human, 7: animal
  char name[20];  ///< Name of the object as defined by the detector
  uint8_t quality;  ///< Detection quality / confidence. 0: bad, 255: maximum confidence
  float bearing;  ///< Angle of the object with respect to the body frame in NED coordinates in radians. 0: front
  float distance;  ///< Ground distance in meters
} mavlink_object_detection_event_t;

#define MAVLINK_MSG_ID_OBJECT_DETECTION_EVENT_LEN 36
#define MAVLINK_MSG_ID_140_LEN 36

#define MAVLINK_MESSAGE_INFO_OBJECT_DETECTION_EVENT \
{ \
  "OBJECT_DETECTION_EVENT", \
  7, \
  { \
    { "time", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_object_detection_event_t, time) }, \
    { "object_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_object_detection_event_t, object_id) }, \
    { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_object_detection_event_t, type) }, \
    { "name", NULL, MAVLINK_TYPE_CHAR, 20, 7, offsetof(mavlink_object_detection_event_t, name) }, \
    { "quality", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_object_detection_event_t, quality) }, \
    { "bearing", NULL, MAVLINK_TYPE_FLOAT, 0, 9, offsetof(mavlink_object_detection_event_t, bearing) }, \
    { "distance", NULL, MAVLINK_TYPE_FLOAT, 0, 13, offsetof(mavlink_object_detection_event_t, distance) }, \
  } \
}

/**
 * @brief Pack a object_detection_event message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time Timestamp in milliseconds since system boot
 * @param object_id Object ID
 * @param type Object type: 0: image, 1: letter, 2: ground vehicle, 3: air vehicle, 4: surface vehicle, 5: sub-surface vehicle, 6: human, 7: animal
 * @param name Name of the object as defined by the detector
 * @param quality Detection quality / confidence. 0: bad, 255: maximum confidence
 * @param bearing Angle of the object with respect to the body frame in NED coordinates in radians. 0: front
 * @param distance Ground distance in meters
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_object_detection_event_pack(
	 uint8_t system_id
	,uint8_t component_id
	,mavlink_message_t* msg
	,uint32_t time
	,uint16_t object_id
	,uint8_t type
	,const char* name
	,uint8_t quality
	,float bearing
	,float distance
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_OBJECT_DETECTION_EVENT_LEN];
	_mav_put_uint32_t(buf, 0, time);
	_mav_put_uint16_t(buf, 4, object_id);
	_mav_put_uint8_t(buf, 6, type);
	_mav_put_char_array(buf, 7, name, 20);
	_mav_put_uint8_t(buf, 8, quality);
	_mav_put_float(buf, 9, bearing);
	_mav_put_float(buf, 13, distance);

	memcpy(_MAV_PAYLOAD(msg), buf, MAVLINK_MSG_ID_OBJECT_DETECTION_EVENT_LEN);
#else
	mavlink_object_detection_event_t packet;
	packet.time = time;
	packet.object_id = object_id;
	packet.type = type;
	mav_array_memcpy(packet.name, name, sizeof(char)*20);
	packet.quality = quality;
	packet.bearing = bearing;
	packet.distance = distance;

	memcpy(_MAV_PAYLOAD(msg), &packet, MAVLINK_MSG_ID_OBJECT_DETECTION_EVENT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_OBJECT_DETECTION_EVENT;
	return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_OBJECT_DETECTION_EVENT_LEN);
}

/**
 * @brief Pack a object_detection_event message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 *
 * @param time Timestamp in milliseconds since system boot
 * @param object_id Object ID
 * @param type Object type: 0: image, 1: letter, 2: ground vehicle, 3: air vehicle, 4: surface vehicle, 5: sub-surface vehicle, 6: human, 7: animal
 * @param name Name of the object as defined by the detector
 * @param quality Detection quality / confidence. 0: bad, 255: maximum confidence
 * @param bearing Angle of the object with respect to the body frame in NED coordinates in radians. 0: front
 * @param distance Ground distance in meters
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_object_detection_event_pack_chan(
	 uint8_t system_id
	,uint8_t component_id
	,uint8_t chan
	,mavlink_message_t* msg
	,uint32_t time
	,uint16_t object_id
	,uint8_t type
	,const char* name
	,uint8_t quality
	,float bearing
	,float distance
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_OBJECT_DETECTION_EVENT_LEN];
	_mav_put_uint32_t(buf, 0, time);
	_mav_put_uint16_t(buf, 4, object_id);
	_mav_put_uint8_t(buf, 6, type);
	_mav_put_char_array(buf, 7, name, 20);
	_mav_put_uint8_t(buf, 8, quality);
	_mav_put_float(buf, 9, bearing);
	_mav_put_float(buf, 13, distance);

	memcpy(_MAV_PAYLOAD(msg), buf, MAVLINK_MSG_ID_OBJECT_DETECTION_EVENT_LEN);
#else
	mavlink_object_detection_event_t packet;
	packet.time = time;
	packet.object_id = object_id;
	packet.type = type;
	mav_array_memcpy(packet.name, name, sizeof(char)*20);
	packet.quality = quality;
	packet.bearing = bearing;
	packet.distance = distance;

	memcpy(_MAV_PAYLOAD(msg), &packet, MAVLINK_MSG_ID_OBJECT_DETECTION_EVENT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_OBJECT_DETECTION_EVENT;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_OBJECT_DETECTION_EVENT_LEN);
}

/**
 * @brief Encode a object_detection_event struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param object_detection_event C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_object_detection_event_encode(
	 uint8_t system_id
	,uint8_t component_id
	,mavlink_message_t* msg
	,const mavlink_object_detection_event_t* object_detection_event
	)
{
	return mavlink_msg_object_detection_event_pack(
	 system_id
	,component_id
	,msg
	,object_detection_event->time
	,object_detection_event->object_id
	,object_detection_event->type
	,object_detection_event->name
	,object_detection_event->quality
	,object_detection_event->bearing
	,object_detection_event->distance
	);
}

/**
 * @brief Send a object_detection_event message
 * @param chan The MAVLink channel to send this message
 *
 * @param time Timestamp in milliseconds since system boot
 * @param object_id Object ID
 * @param type Object type: 0: image, 1: letter, 2: ground vehicle, 3: air vehicle, 4: surface vehicle, 5: sub-surface vehicle, 6: human, 7: animal
 * @param name Name of the object as defined by the detector
 * @param quality Detection quality / confidence. 0: bad, 255: maximum confidence
 * @param bearing Angle of the object with respect to the body frame in NED coordinates in radians. 0: front
 * @param distance Ground distance in meters
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_object_detection_event_send(
	 mavlink_channel_t chan
	,uint32_t time
	,uint16_t object_id
	,uint8_t type
	,const char* name
	,uint8_t quality
	,float bearing
	,float distance
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_OBJECT_DETECTION_EVENT_LEN];
	_mav_put_uint32_t(buf, 0, time);
	_mav_put_uint16_t(buf, 4, object_id);
	_mav_put_uint8_t(buf, 6, type);
	_mav_put_char_array(buf, 7, name, 20);
	_mav_put_uint8_t(buf, 8, quality);
	_mav_put_float(buf, 9, bearing);
	_mav_put_float(buf, 13, distance);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OBJECT_DETECTION_EVENT, buf, MAVLINK_MSG_ID_OBJECT_DETECTION_EVENT_LEN);
#else
	mavlink_object_detection_event_t packet;
	packet.time = time;
	packet.object_id = object_id;
	packet.type = type;
	mav_array_memcpy(packet.name, name, sizeof(char)*20);
	packet.quality = quality;
	packet.bearing = bearing;
	packet.distance = distance;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OBJECT_DETECTION_EVENT, (const char*)&packet, MAVLINK_MSG_ID_OBJECT_DETECTION_EVENT_LEN);
#endif
}

static inline void mavlink_msg_wID_object_detection_event_send(
	 mavlink_channel_t chan
	,uint8_t sID
	,uint8_t cID
	,uint32_t time
	,uint16_t object_id
	,uint8_t type
	,const char* name
	,uint8_t quality
	,float bearing
	,float distance
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_OBJECT_DETECTION_EVENT_LEN];
	_mav_put_uint32_t(buf, 0, time);
	_mav_put_uint16_t(buf, 4, object_id);
	_mav_put_uint8_t(buf, 6, type);
	_mav_put_char_array(buf, 7, name, 20);
	_mav_put_uint8_t(buf, 8, quality);
	_mav_put_float(buf, 9, bearing);
	_mav_put_float(buf, 13, distance);

	_mav_wID_finalize_message_chan_send(chan, sID, cID, MAVLINK_MSG_ID_OBJECT_DETECTION_EVENT, buf, MAVLINK_MSG_ID_OBJECT_DETECTION_EVENT_LEN);
#else
	mavlink_object_detection_event_t packet;
	packet.time = time;
	packet.object_id = object_id;
	packet.type = type;
	mav_array_memcpy(packet.name, name, sizeof(char)*20);
	packet.quality = quality;
	packet.bearing = bearing;
	packet.distance = distance;

	_mav_wID_finalize_message_chan_send(chan, sID, cID, MAVLINK_MSG_ID_OBJECT_DETECTION_EVENT, (const char*)&packet, MAVLINK_MSG_ID_OBJECT_DETECTION_EVENT_LEN);
#endif
}

#endif // MAVLINK_USE_CONVENIENCE_FUNCTIONS

// MESSAGE OBJECT_DETECTION_EVENT UNPACKING

/**
 * @brief Get field time from object_detection_event message
 *
 * @return Timestamp in milliseconds since system boot
 */
static inline uint32_t mavlink_msg_object_detection_event_get_time(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,0);
}

/**
 * @brief Get field object_id from object_detection_event message
 *
 * @return Object ID
 */
static inline uint16_t mavlink_msg_object_detection_event_get_object_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,4);
}

/**
 * @brief Get field type from object_detection_event message
 *
 * @return Object type: 0: image, 1: letter, 2: ground vehicle, 3: air vehicle, 4: surface vehicle, 5: sub-surface vehicle, 6: human, 7: animal
 */
static inline uint8_t mavlink_msg_object_detection_event_get_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,6);
}

/**
 * @brief Get field name from object_detection_event message
 *
 * @return Name of the object as defined by the detector
 */
static inline char mavlink_msg_object_detection_event_get_name(const mavlink_message_t* msg, char* name)
{
	return _MAV_RETURN_char_array(msg,name,20,7);
}

/**
 * @brief Get field quality from object_detection_event message
 *
 * @return Detection quality / confidence. 0: bad, 255: maximum confidence
 */
static inline uint8_t mavlink_msg_object_detection_event_get_quality(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,8);
}

/**
 * @brief Get field bearing from object_detection_event message
 *
 * @return Angle of the object with respect to the body frame in NED coordinates in radians. 0: front
 */
static inline float mavlink_msg_object_detection_event_get_bearing(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,9);
}

/**
 * @brief Get field distance from object_detection_event message
 *
 * @return Ground distance in meters
 */
static inline float mavlink_msg_object_detection_event_get_distance(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,13);
}

/**
 * @brief Decode a object_detection_event message into a struct
 *
 * @param msg The message to decode
 * @param object_detection_event C-struct to decode the message contents into
 */
static inline void mavlink_msg_object_detection_event_decode(const mavlink_message_t* msg, mavlink_object_detection_event_t* object_detection_event)
{
#if MAVLINK_NEED_BYTE_SWAP
	object_detection_event->time = mavlink_msg_object_detection_event_get_time(msg);
	object_detection_event->object_id = mavlink_msg_object_detection_event_get_object_id(msg);
	object_detection_event->type = mavlink_msg_object_detection_event_get_type(msg);
	mavlink_msg_object_detection_event_get_name(msg, object_detection_event->name);
	object_detection_event->quality = mavlink_msg_object_detection_event_get_quality(msg);
	object_detection_event->bearing = mavlink_msg_object_detection_event_get_bearing(msg);
	object_detection_event->distance = mavlink_msg_object_detection_event_get_distance(msg);
#else
	memcpy(object_detection_event, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_OBJECT_DETECTION_EVENT_LEN);
#endif
}
