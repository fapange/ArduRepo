// MESSAGE BHM_SOC_EOD_RUL PACKING
// Generated from parse_APM_XML.m on 24-Feb-2014 13:50:53

#define MAVLINK_MSG_ID_BHM_SOC_EOD_RUL 120 

typedef struct __mavlink_bhm_soc_eod_rul_t 
{ 
  float ura_eod;  ///< Upper Right Aft Motor EOD (seconds)
  float ura_rul;  ///< Upper Right Aft Motor RUL (seconds
  float ura_soc;  ///< Upper Right Aft Motor SOC (percent)
  float lrf_eod;  ///< Lower Right Fwd Motor EOD (seconds)
  float lrf_rul;  ///< Lower Right Fwd Motor RUL (seconds
  float lrf_soc;  ///< Lower Right Fwd Motor SOC (percent)
  float ula_eod;  ///< Upper Left Aft Motor EOD (seconds)
  float ula_rul;  ///< Upper Left Aft Motor RUL (seconds
  float ula_soc;  ///< Upper Left Aft Motor SOC (percent)
  float llf_eod;  ///< Lower Left Fwd Motor EOD (seconds)
  float llf_rul;  ///< Lower Left Fwd Motor RUL (seconds
  float llf_soc;  ///< Lower Left Fwd Motor SOC (percent)
} mavlink_bhm_soc_eod_rul_t;

#define MAVLINK_MSG_ID_BHM_SOC_EOD_RUL_LEN 48
#define MAVLINK_MSG_ID_120_LEN 48

#define MAVLINK_MESSAGE_INFO_BHM_SOC_EOD_RUL \
{ \
  "BHM_SOC_EOD_RUL", \
  12, \
  { \
    { "ura_eod", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_bhm_soc_eod_rul_t, ura_eod) }, \
    { "ura_rul", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_bhm_soc_eod_rul_t, ura_rul) }, \
    { "ura_soc", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_bhm_soc_eod_rul_t, ura_soc) }, \
    { "lrf_eod", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_bhm_soc_eod_rul_t, lrf_eod) }, \
    { "lrf_rul", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_bhm_soc_eod_rul_t, lrf_rul) }, \
    { "lrf_soc", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_bhm_soc_eod_rul_t, lrf_soc) }, \
    { "ula_eod", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_bhm_soc_eod_rul_t, ula_eod) }, \
    { "ula_rul", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_bhm_soc_eod_rul_t, ula_rul) }, \
    { "ula_soc", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_bhm_soc_eod_rul_t, ula_soc) }, \
    { "llf_eod", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_bhm_soc_eod_rul_t, llf_eod) }, \
    { "llf_rul", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_bhm_soc_eod_rul_t, llf_rul) }, \
    { "llf_soc", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_bhm_soc_eod_rul_t, llf_soc) }, \
  } \
}

/**
 * @brief Pack a bhm_soc_eod_rul message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param ura_eod Upper Right Aft Motor EOD (seconds)
 * @param ura_rul Upper Right Aft Motor RUL (seconds
 * @param ura_soc Upper Right Aft Motor SOC (percent)
 * @param lrf_eod Lower Right Fwd Motor EOD (seconds)
 * @param lrf_rul Lower Right Fwd Motor RUL (seconds
 * @param lrf_soc Lower Right Fwd Motor SOC (percent)
 * @param ula_eod Upper Left Aft Motor EOD (seconds)
 * @param ula_rul Upper Left Aft Motor RUL (seconds
 * @param ula_soc Upper Left Aft Motor SOC (percent)
 * @param llf_eod Lower Left Fwd Motor EOD (seconds)
 * @param llf_rul Lower Left Fwd Motor RUL (seconds
 * @param llf_soc Lower Left Fwd Motor SOC (percent)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_bhm_soc_eod_rul_pack(
	 uint8_t system_id
	,uint8_t component_id
	,mavlink_message_t* msg
	,float ura_eod
	,float ura_rul
	,float ura_soc
	,float lrf_eod
	,float lrf_rul
	,float lrf_soc
	,float ula_eod
	,float ula_rul
	,float ula_soc
	,float llf_eod
	,float llf_rul
	,float llf_soc
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_BHM_SOC_EOD_RUL_LEN];
	_mav_put_float(buf, 0, ura_eod);
	_mav_put_float(buf, 4, ura_rul);
	_mav_put_float(buf, 8, ura_soc);
	_mav_put_float(buf, 12, lrf_eod);
	_mav_put_float(buf, 16, lrf_rul);
	_mav_put_float(buf, 20, lrf_soc);
	_mav_put_float(buf, 24, ula_eod);
	_mav_put_float(buf, 28, ula_rul);
	_mav_put_float(buf, 32, ula_soc);
	_mav_put_float(buf, 36, llf_eod);
	_mav_put_float(buf, 40, llf_rul);
	_mav_put_float(buf, 44, llf_soc);

	memcpy(_MAV_PAYLOAD(msg), buf, MAVLINK_MSG_ID_BHM_SOC_EOD_RUL_LEN);
#else
	mavlink_bhm_soc_eod_rul_t packet;
	packet.ura_eod = ura_eod;
	packet.ura_rul = ura_rul;
	packet.ura_soc = ura_soc;
	packet.lrf_eod = lrf_eod;
	packet.lrf_rul = lrf_rul;
	packet.lrf_soc = lrf_soc;
	packet.ula_eod = ula_eod;
	packet.ula_rul = ula_rul;
	packet.ula_soc = ula_soc;
	packet.llf_eod = llf_eod;
	packet.llf_rul = llf_rul;
	packet.llf_soc = llf_soc;

	memcpy(_MAV_PAYLOAD(msg), &packet, MAVLINK_MSG_ID_BHM_SOC_EOD_RUL_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_BHM_SOC_EOD_RUL;
	return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_BHM_SOC_EOD_RUL_LEN);
}

/**
 * @brief Pack a bhm_soc_eod_rul message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 *
 * @param ura_eod Upper Right Aft Motor EOD (seconds)
 * @param ura_rul Upper Right Aft Motor RUL (seconds
 * @param ura_soc Upper Right Aft Motor SOC (percent)
 * @param lrf_eod Lower Right Fwd Motor EOD (seconds)
 * @param lrf_rul Lower Right Fwd Motor RUL (seconds
 * @param lrf_soc Lower Right Fwd Motor SOC (percent)
 * @param ula_eod Upper Left Aft Motor EOD (seconds)
 * @param ula_rul Upper Left Aft Motor RUL (seconds
 * @param ula_soc Upper Left Aft Motor SOC (percent)
 * @param llf_eod Lower Left Fwd Motor EOD (seconds)
 * @param llf_rul Lower Left Fwd Motor RUL (seconds
 * @param llf_soc Lower Left Fwd Motor SOC (percent)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_bhm_soc_eod_rul_pack_chan(
	 uint8_t system_id
	,uint8_t component_id
	,uint8_t chan
	,mavlink_message_t* msg
	,float ura_eod
	,float ura_rul
	,float ura_soc
	,float lrf_eod
	,float lrf_rul
	,float lrf_soc
	,float ula_eod
	,float ula_rul
	,float ula_soc
	,float llf_eod
	,float llf_rul
	,float llf_soc
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_BHM_SOC_EOD_RUL_LEN];
	_mav_put_float(buf, 0, ura_eod);
	_mav_put_float(buf, 4, ura_rul);
	_mav_put_float(buf, 8, ura_soc);
	_mav_put_float(buf, 12, lrf_eod);
	_mav_put_float(buf, 16, lrf_rul);
	_mav_put_float(buf, 20, lrf_soc);
	_mav_put_float(buf, 24, ula_eod);
	_mav_put_float(buf, 28, ula_rul);
	_mav_put_float(buf, 32, ula_soc);
	_mav_put_float(buf, 36, llf_eod);
	_mav_put_float(buf, 40, llf_rul);
	_mav_put_float(buf, 44, llf_soc);

	memcpy(_MAV_PAYLOAD(msg), buf, MAVLINK_MSG_ID_BHM_SOC_EOD_RUL_LEN);
#else
	mavlink_bhm_soc_eod_rul_t packet;
	packet.ura_eod = ura_eod;
	packet.ura_rul = ura_rul;
	packet.ura_soc = ura_soc;
	packet.lrf_eod = lrf_eod;
	packet.lrf_rul = lrf_rul;
	packet.lrf_soc = lrf_soc;
	packet.ula_eod = ula_eod;
	packet.ula_rul = ula_rul;
	packet.ula_soc = ula_soc;
	packet.llf_eod = llf_eod;
	packet.llf_rul = llf_rul;
	packet.llf_soc = llf_soc;

	memcpy(_MAV_PAYLOAD(msg), &packet, MAVLINK_MSG_ID_BHM_SOC_EOD_RUL_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_BHM_SOC_EOD_RUL;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_BHM_SOC_EOD_RUL_LEN);
}

/**
 * @brief Encode a bhm_soc_eod_rul struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param bhm_soc_eod_rul C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_bhm_soc_eod_rul_encode(
	 uint8_t system_id
	,uint8_t component_id
	,mavlink_message_t* msg
	,const mavlink_bhm_soc_eod_rul_t* bhm_soc_eod_rul
	)
{
	return mavlink_msg_bhm_soc_eod_rul_pack(
	 system_id
	,component_id
	,msg
	,bhm_soc_eod_rul->ura_eod
	,bhm_soc_eod_rul->ura_rul
	,bhm_soc_eod_rul->ura_soc
	,bhm_soc_eod_rul->lrf_eod
	,bhm_soc_eod_rul->lrf_rul
	,bhm_soc_eod_rul->lrf_soc
	,bhm_soc_eod_rul->ula_eod
	,bhm_soc_eod_rul->ula_rul
	,bhm_soc_eod_rul->ula_soc
	,bhm_soc_eod_rul->llf_eod
	,bhm_soc_eod_rul->llf_rul
	,bhm_soc_eod_rul->llf_soc
	);
}

/**
 * @brief Send a bhm_soc_eod_rul message
 * @param chan The MAVLink channel to send this message
 *
 * @param ura_eod Upper Right Aft Motor EOD (seconds)
 * @param ura_rul Upper Right Aft Motor RUL (seconds
 * @param ura_soc Upper Right Aft Motor SOC (percent)
 * @param lrf_eod Lower Right Fwd Motor EOD (seconds)
 * @param lrf_rul Lower Right Fwd Motor RUL (seconds
 * @param lrf_soc Lower Right Fwd Motor SOC (percent)
 * @param ula_eod Upper Left Aft Motor EOD (seconds)
 * @param ula_rul Upper Left Aft Motor RUL (seconds
 * @param ula_soc Upper Left Aft Motor SOC (percent)
 * @param llf_eod Lower Left Fwd Motor EOD (seconds)
 * @param llf_rul Lower Left Fwd Motor RUL (seconds
 * @param llf_soc Lower Left Fwd Motor SOC (percent)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_bhm_soc_eod_rul_send(
	 mavlink_channel_t chan
	,float ura_eod
	,float ura_rul
	,float ura_soc
	,float lrf_eod
	,float lrf_rul
	,float lrf_soc
	,float ula_eod
	,float ula_rul
	,float ula_soc
	,float llf_eod
	,float llf_rul
	,float llf_soc
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_BHM_SOC_EOD_RUL_LEN];
	_mav_put_float(buf, 0, ura_eod);
	_mav_put_float(buf, 4, ura_rul);
	_mav_put_float(buf, 8, ura_soc);
	_mav_put_float(buf, 12, lrf_eod);
	_mav_put_float(buf, 16, lrf_rul);
	_mav_put_float(buf, 20, lrf_soc);
	_mav_put_float(buf, 24, ula_eod);
	_mav_put_float(buf, 28, ula_rul);
	_mav_put_float(buf, 32, ula_soc);
	_mav_put_float(buf, 36, llf_eod);
	_mav_put_float(buf, 40, llf_rul);
	_mav_put_float(buf, 44, llf_soc);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BHM_SOC_EOD_RUL, buf, MAVLINK_MSG_ID_BHM_SOC_EOD_RUL_LEN);
#else
	mavlink_bhm_soc_eod_rul_t packet;
	packet.ura_eod = ura_eod;
	packet.ura_rul = ura_rul;
	packet.ura_soc = ura_soc;
	packet.lrf_eod = lrf_eod;
	packet.lrf_rul = lrf_rul;
	packet.lrf_soc = lrf_soc;
	packet.ula_eod = ula_eod;
	packet.ula_rul = ula_rul;
	packet.ula_soc = ula_soc;
	packet.llf_eod = llf_eod;
	packet.llf_rul = llf_rul;
	packet.llf_soc = llf_soc;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BHM_SOC_EOD_RUL, (const char*)&packet, MAVLINK_MSG_ID_BHM_SOC_EOD_RUL_LEN);
#endif
}

static inline void mavlink_msg_wID_bhm_soc_eod_rul_send(
	 mavlink_channel_t chan
	,uint8_t sID
	,uint8_t cID
	,float ura_eod
	,float ura_rul
	,float ura_soc
	,float lrf_eod
	,float lrf_rul
	,float lrf_soc
	,float ula_eod
	,float ula_rul
	,float ula_soc
	,float llf_eod
	,float llf_rul
	,float llf_soc
	)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_BHM_SOC_EOD_RUL_LEN];
	_mav_put_float(buf, 0, ura_eod);
	_mav_put_float(buf, 4, ura_rul);
	_mav_put_float(buf, 8, ura_soc);
	_mav_put_float(buf, 12, lrf_eod);
	_mav_put_float(buf, 16, lrf_rul);
	_mav_put_float(buf, 20, lrf_soc);
	_mav_put_float(buf, 24, ula_eod);
	_mav_put_float(buf, 28, ula_rul);
	_mav_put_float(buf, 32, ula_soc);
	_mav_put_float(buf, 36, llf_eod);
	_mav_put_float(buf, 40, llf_rul);
	_mav_put_float(buf, 44, llf_soc);

	_mav_wID_finalize_message_chan_send(chan, sID, cID, MAVLINK_MSG_ID_BHM_SOC_EOD_RUL, buf, MAVLINK_MSG_ID_BHM_SOC_EOD_RUL_LEN);
#else
	mavlink_bhm_soc_eod_rul_t packet;
	packet.ura_eod = ura_eod;
	packet.ura_rul = ura_rul;
	packet.ura_soc = ura_soc;
	packet.lrf_eod = lrf_eod;
	packet.lrf_rul = lrf_rul;
	packet.lrf_soc = lrf_soc;
	packet.ula_eod = ula_eod;
	packet.ula_rul = ula_rul;
	packet.ula_soc = ula_soc;
	packet.llf_eod = llf_eod;
	packet.llf_rul = llf_rul;
	packet.llf_soc = llf_soc;

	_mav_wID_finalize_message_chan_send(chan, sID, cID, MAVLINK_MSG_ID_BHM_SOC_EOD_RUL, (const char*)&packet, MAVLINK_MSG_ID_BHM_SOC_EOD_RUL_LEN);
#endif
}

#endif // MAVLINK_USE_CONVENIENCE_FUNCTIONS

// MESSAGE BHM_SOC_EOD_RUL UNPACKING

/**
 * @brief Get field ura_eod from bhm_soc_eod_rul message
 *
 * @return Upper Right Aft Motor EOD (seconds)
 */
static inline float mavlink_msg_bhm_soc_eod_rul_get_ura_eod(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,0);
}

/**
 * @brief Get field ura_rul from bhm_soc_eod_rul message
 *
 * @return Upper Right Aft Motor RUL (seconds
 */
static inline float mavlink_msg_bhm_soc_eod_rul_get_ura_rul(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,4);
}

/**
 * @brief Get field ura_soc from bhm_soc_eod_rul message
 *
 * @return Upper Right Aft Motor SOC (percent)
 */
static inline float mavlink_msg_bhm_soc_eod_rul_get_ura_soc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,8);
}

/**
 * @brief Get field lrf_eod from bhm_soc_eod_rul message
 *
 * @return Lower Right Fwd Motor EOD (seconds)
 */
static inline float mavlink_msg_bhm_soc_eod_rul_get_lrf_eod(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,12);
}

/**
 * @brief Get field lrf_rul from bhm_soc_eod_rul message
 *
 * @return Lower Right Fwd Motor RUL (seconds
 */
static inline float mavlink_msg_bhm_soc_eod_rul_get_lrf_rul(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,16);
}

/**
 * @brief Get field lrf_soc from bhm_soc_eod_rul message
 *
 * @return Lower Right Fwd Motor SOC (percent)
 */
static inline float mavlink_msg_bhm_soc_eod_rul_get_lrf_soc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,20);
}

/**
 * @brief Get field ula_eod from bhm_soc_eod_rul message
 *
 * @return Upper Left Aft Motor EOD (seconds)
 */
static inline float mavlink_msg_bhm_soc_eod_rul_get_ula_eod(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,24);
}

/**
 * @brief Get field ula_rul from bhm_soc_eod_rul message
 *
 * @return Upper Left Aft Motor RUL (seconds
 */
static inline float mavlink_msg_bhm_soc_eod_rul_get_ula_rul(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,28);
}

/**
 * @brief Get field ula_soc from bhm_soc_eod_rul message
 *
 * @return Upper Left Aft Motor SOC (percent)
 */
static inline float mavlink_msg_bhm_soc_eod_rul_get_ula_soc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,32);
}

/**
 * @brief Get field llf_eod from bhm_soc_eod_rul message
 *
 * @return Lower Left Fwd Motor EOD (seconds)
 */
static inline float mavlink_msg_bhm_soc_eod_rul_get_llf_eod(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,36);
}

/**
 * @brief Get field llf_rul from bhm_soc_eod_rul message
 *
 * @return Lower Left Fwd Motor RUL (seconds
 */
static inline float mavlink_msg_bhm_soc_eod_rul_get_llf_rul(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,40);
}

/**
 * @brief Get field llf_soc from bhm_soc_eod_rul message
 *
 * @return Lower Left Fwd Motor SOC (percent)
 */
static inline float mavlink_msg_bhm_soc_eod_rul_get_llf_soc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,44);
}

/**
 * @brief Decode a bhm_soc_eod_rul message into a struct
 *
 * @param msg The message to decode
 * @param bhm_soc_eod_rul C-struct to decode the message contents into
 */
static inline void mavlink_msg_bhm_soc_eod_rul_decode(const mavlink_message_t* msg, mavlink_bhm_soc_eod_rul_t* bhm_soc_eod_rul)
{
#if MAVLINK_NEED_BYTE_SWAP
	bhm_soc_eod_rul->ura_eod = mavlink_msg_bhm_soc_eod_rul_get_ura_eod(msg);
	bhm_soc_eod_rul->ura_rul = mavlink_msg_bhm_soc_eod_rul_get_ura_rul(msg);
	bhm_soc_eod_rul->ura_soc = mavlink_msg_bhm_soc_eod_rul_get_ura_soc(msg);
	bhm_soc_eod_rul->lrf_eod = mavlink_msg_bhm_soc_eod_rul_get_lrf_eod(msg);
	bhm_soc_eod_rul->lrf_rul = mavlink_msg_bhm_soc_eod_rul_get_lrf_rul(msg);
	bhm_soc_eod_rul->lrf_soc = mavlink_msg_bhm_soc_eod_rul_get_lrf_soc(msg);
	bhm_soc_eod_rul->ula_eod = mavlink_msg_bhm_soc_eod_rul_get_ula_eod(msg);
	bhm_soc_eod_rul->ula_rul = mavlink_msg_bhm_soc_eod_rul_get_ula_rul(msg);
	bhm_soc_eod_rul->ula_soc = mavlink_msg_bhm_soc_eod_rul_get_ula_soc(msg);
	bhm_soc_eod_rul->llf_eod = mavlink_msg_bhm_soc_eod_rul_get_llf_eod(msg);
	bhm_soc_eod_rul->llf_rul = mavlink_msg_bhm_soc_eod_rul_get_llf_rul(msg);
	bhm_soc_eod_rul->llf_soc = mavlink_msg_bhm_soc_eod_rul_get_llf_soc(msg);
#else
	memcpy(bhm_soc_eod_rul, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_BHM_SOC_EOD_RUL_LEN);
#endif
}
