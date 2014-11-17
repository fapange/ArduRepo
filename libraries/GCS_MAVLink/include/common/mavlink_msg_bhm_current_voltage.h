// MESSAGE BHM_CURRENT_VOLTAGE PACKING
// Generated from parse_APM_XML.m on 24-Feb-2014 13:50:53

#define MAVLINK_MSG_ID_BHM_CURRENT_VOLTAGE 121 

typedef struct __mavlink_bhm_current_voltage_t 
{ 
  float fwd_mtr_cur;  ///< Fwd Motor Current (A)
  float aft_mtr_cur;  ///< Aft Motor Current (A)
  float RPM;  ///< Revolutions. Per Minute
  float llf20v;  ///< Lower Left Fwd Mtr 20-Volt Battery Voltage (V)
  float ula20v;  ///< Upper Left Aft Mtr 20-Volt Battery Voltage (V)
  float lrf40v;  ///< Lower Right Fwd Mtr 40-Volt Battery Voltage (V)
  float ura40v;  ///< Upper Right Aft Mtr 40-Volt Battery Voltage (V)
  float llf20c;  ///< Lower Left Fwd Mtr 20-Volt Battery Current (A)
  float ula20c;  ///< Upper Left Aft Mtr 20-Volt Battery Current (A)
  float lrf40c;  ///< Lower Right Fwd Mtr 40-Volt Battery Current (A)
  float ura40c;  ///< Upper Right Aft Mtr 40-Volt Battery Current (A)
  float alert;  ///< Alert Flag (boolean)
} mavlink_bhm_current_voltage_t;

#define MAVLINK_MSG_ID_BHM_CURRENT_VOLTAGE_LEN 48
#define MAVLINK_MSG_ID_121_LEN 48

#define MAVLINK_MESSAGE_INFO_BHM_CURRENT_VOLTAGE \
{ \
  "BHM_CURRENT_VOLTAGE", \
  12, \
  { \
    { "fwd_mtr_cur", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_bhm_current_voltage_t, fwd_mtr_cur) }, \
    { "aft_mtr_cur", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_bhm_current_voltage_t, aft_mtr_cur) }, \
    { "RPM", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_bhm_current_voltage_t, RPM) }, \
    { "llf20v", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_bhm_current_voltage_t, llf20v) }, \
    { "ula20v", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_bhm_current_voltage_t, ula20v) }, \
    { "lrf40v", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_bhm_current_voltage_t, lrf40v) }, \
    { "ura40v", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_bhm_current_voltage_t, ura40v) }, \
    { "llf20c", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_bhm_current_voltage_t, llf20c) }, \
    { "ula20c", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_bhm_current_voltage_t, ula20c) }, \
    { "lrf40c", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_bhm_current_voltage_t, lrf40c) }, \
    { "ura40c", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_bhm_current_voltage_t, ura40c) }, \
    { "alert", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_bhm_current_voltage_t, alert) }, \
  } \
}

/**
 * @brief Pack a bhm_current_voltage message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param fwd_mtr_cur Fwd Motor Current (A)
 * @param aft_mtr_cur Aft Motor Current (A)
 * @param RPM Revolutions. Per Minute
 * @param llf20v Lower Left Fwd Mtr 20-Volt Battery Voltage (V)
 * @param ula20v Upper Left Aft Mtr 20-Volt Battery Voltage (V)
 * @param lrf40v Lower Right Fwd Mtr 40-Volt Battery Voltage (V)
 * @param ura40v Upper Right Aft Mtr 40-Volt Battery Voltage (V)
 * @param llf20c Lower Left Fwd Mtr 20-Volt Battery Current (A)
 * @param ula20c Upper Left Aft Mtr 20-Volt Battery Current (A)
 * @param lrf40c Lower Right Fwd Mtr 40-Volt Battery Current (A)
 * @param ura40c Upper Right Aft Mtr 40-Volt Battery Current (A)
 * @param alert Alert Flag (boolean)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_bhm_current_voltage_pack(
	 uint8_t system_id
	,uint8_t component_id
	,mavlink_message_t* msg
	,float fwd_mtr_cur
	,float aft_mtr_cur
	,float RPM
	,float llf20v
	,float ula20v
	,float lrf40v
	,float ura40v
	,float llf20c
	,float ula20c
	,float lrf40c
	,float ura40c
	,float alert
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_BHM_CURRENT_VOLTAGE_LEN];
	_mav_put_float(buf, 0, fwd_mtr_cur);
	_mav_put_float(buf, 4, aft_mtr_cur);
	_mav_put_float(buf, 8, RPM);
	_mav_put_float(buf, 12, llf20v);
	_mav_put_float(buf, 16, ula20v);
	_mav_put_float(buf, 20, lrf40v);
	_mav_put_float(buf, 24, ura40v);
	_mav_put_float(buf, 28, llf20c);
	_mav_put_float(buf, 32, ula20c);
	_mav_put_float(buf, 36, lrf40c);
	_mav_put_float(buf, 40, ura40c);
	_mav_put_float(buf, 44, alert);

	memcpy(_MAV_PAYLOAD(msg), buf, MAVLINK_MSG_ID_BHM_CURRENT_VOLTAGE_LEN);
#else
	mavlink_bhm_current_voltage_t packet;
	packet.fwd_mtr_cur = fwd_mtr_cur;
	packet.aft_mtr_cur = aft_mtr_cur;
	packet.RPM = RPM;
	packet.llf20v = llf20v;
	packet.ula20v = ula20v;
	packet.lrf40v = lrf40v;
	packet.ura40v = ura40v;
	packet.llf20c = llf20c;
	packet.ula20c = ula20c;
	packet.lrf40c = lrf40c;
	packet.ura40c = ura40c;
	packet.alert = alert;

	memcpy(_MAV_PAYLOAD(msg), &packet, MAVLINK_MSG_ID_BHM_CURRENT_VOLTAGE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_BHM_CURRENT_VOLTAGE;
	return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_BHM_CURRENT_VOLTAGE_LEN);
}

/**
 * @brief Pack a bhm_current_voltage message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 *
 * @param fwd_mtr_cur Fwd Motor Current (A)
 * @param aft_mtr_cur Aft Motor Current (A)
 * @param RPM Revolutions. Per Minute
 * @param llf20v Lower Left Fwd Mtr 20-Volt Battery Voltage (V)
 * @param ula20v Upper Left Aft Mtr 20-Volt Battery Voltage (V)
 * @param lrf40v Lower Right Fwd Mtr 40-Volt Battery Voltage (V)
 * @param ura40v Upper Right Aft Mtr 40-Volt Battery Voltage (V)
 * @param llf20c Lower Left Fwd Mtr 20-Volt Battery Current (A)
 * @param ula20c Upper Left Aft Mtr 20-Volt Battery Current (A)
 * @param lrf40c Lower Right Fwd Mtr 40-Volt Battery Current (A)
 * @param ura40c Upper Right Aft Mtr 40-Volt Battery Current (A)
 * @param alert Alert Flag (boolean)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_bhm_current_voltage_pack_chan(
	 uint8_t system_id
	,uint8_t component_id
	,uint8_t chan
	,mavlink_message_t* msg
	,float fwd_mtr_cur
	,float aft_mtr_cur
	,float RPM
	,float llf20v
	,float ula20v
	,float lrf40v
	,float ura40v
	,float llf20c
	,float ula20c
	,float lrf40c
	,float ura40c
	,float alert
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_BHM_CURRENT_VOLTAGE_LEN];
	_mav_put_float(buf, 0, fwd_mtr_cur);
	_mav_put_float(buf, 4, aft_mtr_cur);
	_mav_put_float(buf, 8, RPM);
	_mav_put_float(buf, 12, llf20v);
	_mav_put_float(buf, 16, ula20v);
	_mav_put_float(buf, 20, lrf40v);
	_mav_put_float(buf, 24, ura40v);
	_mav_put_float(buf, 28, llf20c);
	_mav_put_float(buf, 32, ula20c);
	_mav_put_float(buf, 36, lrf40c);
	_mav_put_float(buf, 40, ura40c);
	_mav_put_float(buf, 44, alert);

	memcpy(_MAV_PAYLOAD(msg), buf, MAVLINK_MSG_ID_BHM_CURRENT_VOLTAGE_LEN);
#else
	mavlink_bhm_current_voltage_t packet;
	packet.fwd_mtr_cur = fwd_mtr_cur;
	packet.aft_mtr_cur = aft_mtr_cur;
	packet.RPM = RPM;
	packet.llf20v = llf20v;
	packet.ula20v = ula20v;
	packet.lrf40v = lrf40v;
	packet.ura40v = ura40v;
	packet.llf20c = llf20c;
	packet.ula20c = ula20c;
	packet.lrf40c = lrf40c;
	packet.ura40c = ura40c;
	packet.alert = alert;

	memcpy(_MAV_PAYLOAD(msg), &packet, MAVLINK_MSG_ID_BHM_CURRENT_VOLTAGE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_BHM_CURRENT_VOLTAGE;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_BHM_CURRENT_VOLTAGE_LEN);
}

/**
 * @brief Encode a bhm_current_voltage struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param bhm_current_voltage C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_bhm_current_voltage_encode(
	 uint8_t system_id
	,uint8_t component_id
	,mavlink_message_t* msg
	,const mavlink_bhm_current_voltage_t* bhm_current_voltage
	)
{
	return mavlink_msg_bhm_current_voltage_pack(
	 system_id
	,component_id
	,msg
	,bhm_current_voltage->fwd_mtr_cur
	,bhm_current_voltage->aft_mtr_cur
	,bhm_current_voltage->RPM
	,bhm_current_voltage->llf20v
	,bhm_current_voltage->ula20v
	,bhm_current_voltage->lrf40v
	,bhm_current_voltage->ura40v
	,bhm_current_voltage->llf20c
	,bhm_current_voltage->ula20c
	,bhm_current_voltage->lrf40c
	,bhm_current_voltage->ura40c
	,bhm_current_voltage->alert
	);
}

/**
 * @brief Send a bhm_current_voltage message
 * @param chan The MAVLink channel to send this message
 *
 * @param fwd_mtr_cur Fwd Motor Current (A)
 * @param aft_mtr_cur Aft Motor Current (A)
 * @param RPM Revolutions. Per Minute
 * @param llf20v Lower Left Fwd Mtr 20-Volt Battery Voltage (V)
 * @param ula20v Upper Left Aft Mtr 20-Volt Battery Voltage (V)
 * @param lrf40v Lower Right Fwd Mtr 40-Volt Battery Voltage (V)
 * @param ura40v Upper Right Aft Mtr 40-Volt Battery Voltage (V)
 * @param llf20c Lower Left Fwd Mtr 20-Volt Battery Current (A)
 * @param ula20c Upper Left Aft Mtr 20-Volt Battery Current (A)
 * @param lrf40c Lower Right Fwd Mtr 40-Volt Battery Current (A)
 * @param ura40c Upper Right Aft Mtr 40-Volt Battery Current (A)
 * @param alert Alert Flag (boolean)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_bhm_current_voltage_send(
	 mavlink_channel_t chan
	,float fwd_mtr_cur
	,float aft_mtr_cur
	,float RPM
	,float llf20v
	,float ula20v
	,float lrf40v
	,float ura40v
	,float llf20c
	,float ula20c
	,float lrf40c
	,float ura40c
	,float alert
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_BHM_CURRENT_VOLTAGE_LEN];
	_mav_put_float(buf, 0, fwd_mtr_cur);
	_mav_put_float(buf, 4, aft_mtr_cur);
	_mav_put_float(buf, 8, RPM);
	_mav_put_float(buf, 12, llf20v);
	_mav_put_float(buf, 16, ula20v);
	_mav_put_float(buf, 20, lrf40v);
	_mav_put_float(buf, 24, ura40v);
	_mav_put_float(buf, 28, llf20c);
	_mav_put_float(buf, 32, ula20c);
	_mav_put_float(buf, 36, lrf40c);
	_mav_put_float(buf, 40, ura40c);
	_mav_put_float(buf, 44, alert);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BHM_CURRENT_VOLTAGE, buf, MAVLINK_MSG_ID_BHM_CURRENT_VOLTAGE_LEN);
#else
	mavlink_bhm_current_voltage_t packet;
	packet.fwd_mtr_cur = fwd_mtr_cur;
	packet.aft_mtr_cur = aft_mtr_cur;
	packet.RPM = RPM;
	packet.llf20v = llf20v;
	packet.ula20v = ula20v;
	packet.lrf40v = lrf40v;
	packet.ura40v = ura40v;
	packet.llf20c = llf20c;
	packet.ula20c = ula20c;
	packet.lrf40c = lrf40c;
	packet.ura40c = ura40c;
	packet.alert = alert;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BHM_CURRENT_VOLTAGE, (const char*)&packet, MAVLINK_MSG_ID_BHM_CURRENT_VOLTAGE_LEN);
#endif
}

static inline void mavlink_msg_wID_bhm_current_voltage_send(
	 mavlink_channel_t chan
	,uint8_t sID
	,uint8_t cID
	,float fwd_mtr_cur
	,float aft_mtr_cur
	,float RPM
	,float llf20v
	,float ula20v
	,float lrf40v
	,float ura40v
	,float llf20c
	,float ula20c
	,float lrf40c
	,float ura40c
	,float alert
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_BHM_CURRENT_VOLTAGE_LEN];
	_mav_put_float(buf, 0, fwd_mtr_cur);
	_mav_put_float(buf, 4, aft_mtr_cur);
	_mav_put_float(buf, 8, RPM);
	_mav_put_float(buf, 12, llf20v);
	_mav_put_float(buf, 16, ula20v);
	_mav_put_float(buf, 20, lrf40v);
	_mav_put_float(buf, 24, ura40v);
	_mav_put_float(buf, 28, llf20c);
	_mav_put_float(buf, 32, ula20c);
	_mav_put_float(buf, 36, lrf40c);
	_mav_put_float(buf, 40, ura40c);
	_mav_put_float(buf, 44, alert);

	_mav_wID_finalize_message_chan_send(chan, sID, cID, MAVLINK_MSG_ID_BHM_CURRENT_VOLTAGE, buf, MAVLINK_MSG_ID_BHM_CURRENT_VOLTAGE_LEN);
#else
	mavlink_bhm_current_voltage_t packet;
	packet.fwd_mtr_cur = fwd_mtr_cur;
	packet.aft_mtr_cur = aft_mtr_cur;
	packet.RPM = RPM;
	packet.llf20v = llf20v;
	packet.ula20v = ula20v;
	packet.lrf40v = lrf40v;
	packet.ura40v = ura40v;
	packet.llf20c = llf20c;
	packet.ula20c = ula20c;
	packet.lrf40c = lrf40c;
	packet.ura40c = ura40c;
	packet.alert = alert;

	_mav_wID_finalize_message_chan_send(chan, sID, cID, MAVLINK_MSG_ID_BHM_CURRENT_VOLTAGE, (const char*)&packet, MAVLINK_MSG_ID_BHM_CURRENT_VOLTAGE_LEN);
#endif
}

#endif // MAVLINK_USE_CONVENIENCE_FUNCTIONS

// MESSAGE BHM_CURRENT_VOLTAGE UNPACKING

/**
 * @brief Get field fwd_mtr_cur from bhm_current_voltage message
 *
 * @return Fwd Motor Current (A)
 */
static inline float mavlink_msg_bhm_current_voltage_get_fwd_mtr_cur(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,0);
}

/**
 * @brief Get field aft_mtr_cur from bhm_current_voltage message
 *
 * @return Aft Motor Current (A)
 */
static inline float mavlink_msg_bhm_current_voltage_get_aft_mtr_cur(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,4);
}

/**
 * @brief Get field RPM from bhm_current_voltage message
 *
 * @return Revolutions. Per Minute
 */
static inline float mavlink_msg_bhm_current_voltage_get_RPM(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,8);
}

/**
 * @brief Get field llf20v from bhm_current_voltage message
 *
 * @return Lower Left Fwd Mtr 20-Volt Battery Voltage (V)
 */
static inline float mavlink_msg_bhm_current_voltage_get_llf20v(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,12);
}

/**
 * @brief Get field ula20v from bhm_current_voltage message
 *
 * @return Upper Left Aft Mtr 20-Volt Battery Voltage (V)
 */
static inline float mavlink_msg_bhm_current_voltage_get_ula20v(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,16);
}

/**
 * @brief Get field lrf40v from bhm_current_voltage message
 *
 * @return Lower Right Fwd Mtr 40-Volt Battery Voltage (V)
 */
static inline float mavlink_msg_bhm_current_voltage_get_lrf40v(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,20);
}

/**
 * @brief Get field ura40v from bhm_current_voltage message
 *
 * @return Upper Right Aft Mtr 40-Volt Battery Voltage (V)
 */
static inline float mavlink_msg_bhm_current_voltage_get_ura40v(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,24);
}

/**
 * @brief Get field llf20c from bhm_current_voltage message
 *
 * @return Lower Left Fwd Mtr 20-Volt Battery Current (A)
 */
static inline float mavlink_msg_bhm_current_voltage_get_llf20c(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,28);
}

/**
 * @brief Get field ula20c from bhm_current_voltage message
 *
 * @return Upper Left Aft Mtr 20-Volt Battery Current (A)
 */
static inline float mavlink_msg_bhm_current_voltage_get_ula20c(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,32);
}

/**
 * @brief Get field lrf40c from bhm_current_voltage message
 *
 * @return Lower Right Fwd Mtr 40-Volt Battery Current (A)
 */
static inline float mavlink_msg_bhm_current_voltage_get_lrf40c(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,36);
}

/**
 * @brief Get field ura40c from bhm_current_voltage message
 *
 * @return Upper Right Aft Mtr 40-Volt Battery Current (A)
 */
static inline float mavlink_msg_bhm_current_voltage_get_ura40c(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,40);
}

/**
 * @brief Get field alert from bhm_current_voltage message
 *
 * @return Alert Flag (boolean)
 */
static inline float mavlink_msg_bhm_current_voltage_get_alert(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,44);
}

/**
 * @brief Decode a bhm_current_voltage message into a struct
 *
 * @param msg The message to decode
 * @param bhm_current_voltage C-struct to decode the message contents into
 */
static inline void mavlink_msg_bhm_current_voltage_decode(const mavlink_message_t* msg, mavlink_bhm_current_voltage_t* bhm_current_voltage)
{
#if MAVLINK_NEED_BYTE_SWAP
	bhm_current_voltage->fwd_mtr_cur = mavlink_msg_bhm_current_voltage_get_fwd_mtr_cur(msg);
	bhm_current_voltage->aft_mtr_cur = mavlink_msg_bhm_current_voltage_get_aft_mtr_cur(msg);
	bhm_current_voltage->RPM = mavlink_msg_bhm_current_voltage_get_RPM(msg);
	bhm_current_voltage->llf20v = mavlink_msg_bhm_current_voltage_get_llf20v(msg);
	bhm_current_voltage->ula20v = mavlink_msg_bhm_current_voltage_get_ula20v(msg);
	bhm_current_voltage->lrf40v = mavlink_msg_bhm_current_voltage_get_lrf40v(msg);
	bhm_current_voltage->ura40v = mavlink_msg_bhm_current_voltage_get_ura40v(msg);
	bhm_current_voltage->llf20c = mavlink_msg_bhm_current_voltage_get_llf20c(msg);
	bhm_current_voltage->ula20c = mavlink_msg_bhm_current_voltage_get_ula20c(msg);
	bhm_current_voltage->lrf40c = mavlink_msg_bhm_current_voltage_get_lrf40c(msg);
	bhm_current_voltage->ura40c = mavlink_msg_bhm_current_voltage_get_ura40c(msg);
	bhm_current_voltage->alert = mavlink_msg_bhm_current_voltage_get_alert(msg);
#else
	memcpy(bhm_current_voltage, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_BHM_CURRENT_VOLTAGE_LEN);
#endif
}
