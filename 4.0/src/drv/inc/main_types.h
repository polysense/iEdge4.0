/**
 * @file  main_types.h  
 * @brief MA common types
 * @author Runby F.
 * @date 2022-2-9
 * @copyright Polysense
 */

#ifndef __MAIN_TYPES_H__
#define __MAIN_TYPES_H__

#include "pos_lib.h"

#define MA_EEPROM_OTA_MAGIC 0x88af9123 /**< Magic signature for EEPROM OTA configuraiton */
#define MA_EEPROM_ONENET_MAGIC 0x3789ab34 /**< Magic signature for EEPROM ONENET configuraiton */

#define MA_EEPROM_MAGIC  0x92747613 /**< Magic signature for EEPROM configuraiton */
#define MA_SENSOR_SLOT_NUM 8 /**< Sensor slot number */
#define MA_SENSOR_POLL_NUM 8 /**< Sensor polling pool number */
#define MA_SENSOR_CYCLE_SECONDS_MAX (86400*2) /**< Sensor max duty cycle in unit of second */

#define MA_REPORT_TYPE_IS_LORA(rtype)   (((rtype)&0xff00) == (MA_REPORT_LORA_US902&0xff00)) /**< LORA report check */
#define MA_REPORT_TYPE_IS_PLSS(rtype)   (((rtype)&0xff00) == (MA_REPORT_PLSS_NBIOT&0xff00)) /**< PLSS report check */
#define MA_REPORT_TYPE_IS_MQTT(rtype)   (((rtype)&0xff00) == (MA_REPORT_MQTT_NBIOT&0xff00)) /**< MQTT report check */
#define MA_REPORT_TYPE_IS_NATIVE(rtype)   (((rtype)&0xff00) == (MA_REPORT_NATIVE_NBIOT&0xff00)) /**< NATIVE report check */
#define MA_REPORT_TYPE_IS_NBIOT(rtype)  (((rtype)&0xff)== (MA_REPORT_MQTT_NBIOT&0xff)) /**< NBIOT report check */
#define MA_REPORT_TYPE_IS_WIFI(rtype)  (((rtype)&0xff)== (MA_REPORT_MQTT_WIFI&0xff)) /**< WIFI report check */
#define MA_REPORT_TYPE_IS_LTE(rtype)  (((rtype)&0xff)== (MA_REPORT_MQTT_LTE&0xff)) /**< LTE report check */

#define MA_SENSOR_TYPE_IS_VALID(stype)  ((stype) < MA_REPORT_TYPE_START) /**< Sensor type valid check (only type number check. Not checking driver firmware in flash)*/


/**
 * @brief Calculate from two chars into a U16
 */
#define MA_CH_TO_U16(c0, c1) (((c1) << 8) + (c0))

/**
 * @brief Calculate from four chars into a U16
 */
#define MA_CH_TO_U32(c0, c1, c2, c3) (((c3) << 24) + ((c2) << 16) + ((c1) << 8) + (c0))

/**
 * @brief Calculate from four IP address bytes in network order into a U32 in host order
 */
#define MA_IP_TO_U32(ip0, ip1, ip2, ip3) (((ip0) << 24) + ((ip1) << 16) + ((ip2) << 8) + (ip3))

/**
 * @brief Calculate from IP address array in network order into a U32 in host order
 */
#define MA_IPA_TO_U32(ipa)  MA_IP_TO_U32(ipa[0], ipa[1], ipa[2], ipa[3])

/**
 * @brief EEPROM configuration field/offset definition
 */
#define MA_CFG_OFS_MAGIC                0 /**< Magic signature @ref ma_eeprom_cfg_t::magic */
#define MA_CFG_OFS_REPORT_TYPE    (MA_CFG_OFS_MAGIC+4) /**< Field offset for @ref ma_eeprom_cfg_t::report_type */
#define MA_CFG_OFS_CTRL    (MA_CFG_OFS_REPORT_TYPE+2) /**< Field offset for @ref ma_eeprom_cfg_t::ctrl */
#define MA_CFG_OFS_EUI    (MA_CFG_OFS_CTRL+2) /**< Field offset for @ref ma_eeprom_cfg_t::eui */
#define MA_CFG_OFS_LORA_EUI    (MA_CFG_OFS_EUI+8) /**< Field offset for @ref ma_lora_cfg_t::lora_dev_eui */
#define MA_CFG_OFS_LORA_APP    (MA_CFG_OFS_LORA_EUI+8) /**< Field offset for @ref ma_lora_cfg_t::lora_app_eui */
#define MA_CFG_OFS_LORA_KEY    (MA_CFG_OFS_LORA_APP+8) /**< Field offset for @ref ma_lora_cfg_t::lora_app_key */
#define MA_CFG_OFS_LORA_ASKEY   (MA_CFG_OFS_LORA_KEY+16) /**< Field offset for @ref ma_lora_cfg_t::lora_app_skey */
#define MA_CFG_OFS_LORA_NSKEY   (MA_CFG_OFS_LORA_ASKEY+16) /**< Field offset for @ref ma_lora_cfg_t::lora_nwk_skey */
#define MA_CFG_OFS_LORA_ADDR   (MA_CFG_OFS_LORA_NSKEY+16) /**< Field offset for @ref ma_lora_cfg_t::lora_dev_addr */
#define MA_CFG_OFS_LORA_NONCE   (MA_CFG_OFS_LORA_ADDR+4) /**< Field offset for @ref ma_lora_cfg_t::lora_dev_nonce */
#define MA_CFG_OFS_LORA_MODEL   (MA_CFG_OFS_LORA_NONCE+2) /**< Field offset for @ref ma_lora_cfg_t::lora_model */
#define MA_CFG_OFS_LORA_ACKLMT   (MA_CFG_OFS_LORA_MODEL+1) /**< Field offset for @ref ma_lora_cfg_t::lora_ack_lmt */
#define MA_CFG_OFS_LORA_ACKDLY  (MA_CFG_OFS_LORA_ACKLMT+1) /**< Field offset for @ref ma_lora_cfg_t::lora_ack_delay */
#define MA_CFG_OFS_LORA_NBTRANS  (MA_CFG_OFS_LORA_ACKDLY+1) /**< Field offset for @ref ma_lora_cfg_t::lora_nb_trans */
#define MA_CFG_OFS_LORA_CTRL  (MA_CFG_OFS_LORA_NBTRANS+1) /**< Field offset for @ref ma_lora_cfg_t::lora_ctrl */
#define MA_CFG_OFS_LORA_PWR  (MA_CFG_OFS_LORA_CTRL+1) /**< Field offset for @ref ma_lora_cfg_t::lora_tx_pwr */
#define MA_CFG_OFS_LORA_GRP  (MA_CFG_OFS_LORA_PWR+1) /**< Field offset for @ref ma_lora_cfg_t::lora_chan_grp */
#define MA_CFG_OFS_LORA_TX  (MA_CFG_OFS_LORA_GRP+1) /**< Field offset for @ref ma_lora_cfg_t::lora_fport_tx */
#define MA_CFG_OFS_LORA_RX  (MA_CFG_OFS_LORA_TX+1) /**< Field offset for @ref ma_lora_cfg_t::lora_fport_rx */
#define MA_CFG_OFS_LORA_FLAGS  (MA_CFG_OFS_LORA_RX+1)/**< Field offset for @ref ma_lora_cfg_t::lora_flags */
#define MA_CFG_OFS_RESET_MOD  (MA_CFG_OFS_LORA_FLAGS+1) /**< Field offset for @ref ma_eeprom_cfg_t::module_reset_err */
#define MA_CFG_OFS_RESET_SYS  (MA_CFG_OFS_RESET_MOD+2) /**< Field offset for @ref ma_eeprom_cfg_t::sys_reset_err */
#define MA_CFG_OFS_MBUS  (MA_CFG_OFS_RESET_SYS+2) /**< Field offset for ma_eeprom_cfg_t:rsvd */
#define MA_CFG_OFS_TIME_CTRL  (MA_CFG_OFS_MBUS+2) /**< Field offset for ma_eeprom_cfg_t:time_ctrl */
#define MA_CFG_OFS_RSVD  (MA_CFG_OFS_TIME_CTRL+5) /**< Field offset for ma_eeprom_cfg_t:rsvd */
#define MA_CFG_OFS_MCALIB  (MA_CFG_OFS_RSVD+0x10) /**< Field offset for ma_eeprom_cfg_t:module_calib */
#define MA_CFG_OFS_MDR  (MA_CFG_OFS_MCALIB+0x1) /**< Field offset for ma_eeprom_cfg_t:module_dr */
#define MA_CFG_OFS_MIO  (MA_CFG_OFS_MDR+1) /**< Field offset for ma_eeprom_cfg_t:module_io */
#define MA_CFG_OFS_CONSOLE  (MA_CFG_OFS_MIO+1) /**< Field offset for @ref ma_eeprom_cfg_t::console */
#define MA_CFG_OFS_DS18B20_NUM  (MA_CFG_OFS_CONSOLE+1) /**< Field offset for @ref ma_eeprom_cfg_t::ds18b20_num */
#define MA_CFG_OFS_SLOT  (MA_CFG_OFS_DS18B20_NUM+1) /**< Field offset for @ref ma_eeprom_cfg_t::slot */
#define MA_CFG_OFS_SLOT_CYCLE 0 /**< Slot offset for @ref ma_sensor_slot_t::cycle */
#define MA_CFG_OFS_SLOT_STYPE (MA_CFG_OFS_SLOT_CYCLE+4) /**< Slot offset for @ref ma_sensor_slot_t::sensor_type */
#define MA_CFG_OFS_SLOT_PWR (MA_CFG_OFS_SLOT_STYPE+2) /**< Slot offset for @ref ma_sensor_slot_t::power_ms */
#define MA_CFG_OFS_SLOT_PERIODIC (MA_CFG_OFS_SLOT_PWR+2) /**< Slot offset for @ref ma_sensor_slot_t::periodic */
#define MA_CFG_OFS_SLOT_IO (MA_CFG_OFS_SLOT_PERIODIC+2)/**< Slot offset for @ref ma_sensor_slot_t::io */
#define MA_CFG_OFS_SLOT_RSVD32 (MA_CFG_OFS_SLOT_IO+2) /**< Slot offset for @ref ma_sensor_slot_t::rsvd32 */
#define MA_CFG_OFS_SLOT_ARG (MA_CFG_OFS_SLOT_RSVD32+4) /**< Slot offset for @ref ma_sensor_slot_t::arg */
#define MA_CFG_OFS_SLOT_THLD (MA_CFG_OFS_SLOT_ARG+16) /**< Slot offset for @ref ma_sensor_slot_t::thld */
#define MA_CFG_OFS_DS18B20  (MA_CFG_OFS_SLOT+0x30*8) /**< Field offset for @ref ma_eeprom_cfg_t::ds18b20 */
#define MA_CFG_OFS_DS18B20_POS  (MA_CFG_OFS_DS18B20+0x8*16) /**< Field offset for @ref ma_eeprom_cfg_t::ds18b20_pos */
#define MA_CFG_OFS_TOPIC_PUB  (MA_CFG_OFS_DS18B20_POS+16) /**< Field offset for @ref ma_network_cfg_t::topic_pub */
#define MA_CFG_OFS_TOPIC_SUB  (MA_CFG_OFS_TOPIC_PUB+64) /**< Field offset for @ref ma_network_cfg_t::topic_sub */
#define MA_CFG_OFS_MUSR  (MA_CFG_OFS_TOPIC_SUB+64) /**< Field offset for @ref ma_network_cfg_t::musr */
#define MA_CFG_OFS_MPWD  (MA_CFG_OFS_MUSR+16) /**< Field offset for @ref ma_network_cfg_t::mpwd */
#define MA_CFG_OFS_MCID  (MA_CFG_OFS_MPWD+16) /**< Field offset for @ref ma_network_cfg_t::mcid */
#define MA_CFG_OFS_DNS  (MA_CFG_OFS_MCID+32) /**< Field offset for @ref ma_network_cfg_t::dns */
#define MA_CFG_OFS_QOS_PUB  (MA_CFG_OFS_DNS+4) /**< Field offset for @ref ma_network_cfg_t::qos_pub */
#define MA_CFG_OFS_QOS_SUB  (MA_CFG_OFS_QOS_PUB+1) /**< Field offset for @ref ma_network_cfg_t::qos_sub */
#define MA_CFG_OFS_NRSVD  (MA_CFG_OFS_QOS_SUB+1) /**< Field offset for @ref ma_network_cfg_t::rsvd */
#define MA_CFG_OFS_DNS_RETRY  (MA_CFG_OFS_NRSVD+6) /**< Field offset for @ref ma_network_cfg_t::dns_retry */
#define MA_CFG_OFS_DNS_DELAY  (MA_CFG_OFS_DNS_RETRY+1) /**< Field offset for @ref ma_network_cfg_t::dns_delay */
#define MA_CFG_OFS_RSVR_PORT  (MA_CFG_OFS_DNS_DELAY+1) /**< Field offset for @ref ma_network_cfg_t::rsvr_port */
#define MA_CFG_OFS_RSVR_URL  (MA_CFG_OFS_RSVR_PORT+2) /**< Field offset for @ref ma_network_cfg_t::rsvr_url */
#define MA_CFG_OFS_BAND  (MA_CFG_OFS_RSVR_URL+32) /**< Field offset for @ref ma_network_cfg_t::band */
#define MA_CFG_OFS_APN  (MA_CFG_OFS_BAND+32) /**< Field offset for @ref ma_network_cfg_t::apn */
#define MA_CFG_OFS_SSID  (MA_CFG_OFS_APN+32) /**< Field offset for @ref ma_network_cfg_t::ssid */
#define MA_CFG_OFS_WRSVD  (MA_CFG_OFS_SSID+32) /**< Field offset for @ref ma_network_cfg_t::wrsvd */
#define MA_CFG_OFS_WPWD  (MA_CFG_OFS_WRSVD+16) /**< Field offset for @ref ma_network_cfg_t::wpwd */

/**
 * @brief EEPROM PIN configuration field/offset definition
 */
#define MA_PCFG_OFS_MAGIC        0                      /**< Magic signature @ref pos_board_pin_t::magic */
#define MA_PCFG_OFS_VERSION     (MA_PCFG_OFS_MAGIC+4)   /**< Field offset for @ref pos_board_pin_t::version */
#define MA_PCFG_OFS_LED         (MA_PCFG_OFS_VERSION+4) /**< Field offset for @ref pos_board_pin_t::led */
#define MA_PCFG_OFS_PULL        (MA_PCFG_OFS_LED+4)     /**< Field offset for @ref pos_board_pin_t::pull */
#define MA_PCFG_OFS_SETH        (MA_PCFG_OFS_PULL+4)    /**< Field offset for @ref pos_board_pin_t::seth */
#define MA_PCFG_OFS_SETL        (MA_PCFG_OFS_SETH+8)    /**< Field offset for @ref pos_board_pin_t::setl */
#define MA_PCFG_OFS_PWR         (MA_PCFG_OFS_SETL+8)    /**< Field offset for @ref pos_board_pin_t::pwr */
#define MA_PCFG_OFS_POLARITY    (MA_PCFG_OFS_PWR+32)    /**< Field offset for @ref pos_board_pin_t::polarity */
#define MA_PCFG_OFS_WIRE1       (MA_PCFG_OFS_POLARITY+32) /**< Field offset for @ref pos_board_pin_t::wire1 */
#define MA_PCFG_OFS_AIN         (MA_PCFG_OFS_WIRE1+8)   /**< Field offset for @ref pos_board_pin_t::ain */
#define MA_PCFG_OFS_DIO         (MA_PCFG_OFS_AIN+8)     /**< Field offset for @ref pos_board_pin_t::dio */
#define MA_PCFG_OFS_PWM         (MA_PCFG_OFS_DIO+8)     /**< Field offset for @ref pos_board_pin_t::pwm */
#define MA_PCFG_OFS_PWM_MODE    (MA_PCFG_OFS_PWM+4)     /**< Field offset for @ref pos_board_pin_t::pwm_mode */
#define MA_PCFG_OFS_CTRL        (MA_PCFG_OFS_PWM_MODE+4) /**< Field offset for @ref pos_board_pin_t::ctrl */
#define MA_PCFG_OFS_RSVD        (MA_PCFG_OFS_CTRL+32) /**< Field offset for @ref pos_board_pin_t::rsvd */
#define MA_PCFG_OFS_IO          (MA_PCFG_OFS_RSVD+96) /**< Field offset for @ref pos_board_pin_t::io */


/**
 * @brief EEPROM ONENET configuration field/offset definition
 */
#define MA_ONECFG_OFS_MAGIC        0                        /**< Magic signature @ref ma_eeprom_onenet_cfg_t::magic */
#define MA_ONECFG_OFS_VERSION     (MA_ONECFG_OFS_MAGIC+4)   /**< Field offset for @ref ma_eeprom_onenet_cfg_t::version */
#define MA_ONECFG_OFS_RSVD        (MA_ONECFG_OFS_VERSION+4) /**< Field offset for @ref ma_eeprom_onenet_cfg_t::rsvd */
#define MA_ONECFG_OFS_PID         (MA_ONECFG_OFS_RSVD+0xd8) /**< Field offset for @ref ma_eeprom_onenet_cfg_t::pid */
#define MA_ONECFG_OFS_TOKEN       (MA_ONECFG_OFS_PID+0x20)  /**< Field offset for @ref ma_eeprom_onenet_cfg_t::token */
#define MA_ONECFG_OFS_DEVK        (MA_ONECFG_OFS_TOKEN+192) /**< Field offset for @ref ma_eeprom_onenet_cfg_t::devk */

/**
 * @brief EEPROM configuration CTRL bitmap definition (@ref ma_eeprom_cfg_t::ctrl) 
 */
#define MA_CFG_CTRL_CONSOLE_REPORT  (1L<<0)      /**< Console report print control:\n
  0: Normal mode\n
  1: Print report data in console */
#define MA_CFG_CTRL_VERBOSE_LOG     (1L<<1)      /**< Console detailed log control:\n
  0: Normal mode\n
  1: Print sys running log in console */
#define MA_CFG_CTRL_DRV_LOG         (1L<<2)      /**< Console driver log control:\n
  0: Normal mode\n
  1: Print driver log in console */
#define MA_CFG_CTRL_DBG             (1L<<3)      /**< Console AT debug control:\n
  0: Normal mode\n
  1: Print AT raw commands in console */
#define MA_CFG_CTRL_MANUFACTURE_SLEEP (1L<<6)    /**< Manufacture sleeping mode:\n
    0: Regular working\n
    1: Always sleep until next reset*/
#define MA_CFG_CTRL_BEEP_SILENT     (1L<<7)        /**< Beep control mode:\n
  0: Regular beep\n
  1: Always silent*/
#define MA_CFG_CTRL_POWER_MODE     (1L<<8)        /**< Power control mode:\n
    0: Regular all sensor power on/off\n
    1: Single sensor power on/off */
#define MA_CFG_CTRL_LED_MODE     (1L<<9)        /**< LED mode control:\n
  0: Regular LED on/off\n
  1: LED silent mode */
#define MA_CFG_CTRL_ONENET_MODE     (1L<<10)     /**< OneNET(Uniot) JSON mode control:\n
  0: Regular IVIEW/Common management or LWM2M IPSO (both OneNet and CUIOT)\n
  1: OneJSON or CU-JSON management */
#define MA_CFG_CTRL_DTU_MODE       (1L<<11)     /**< DTU mode control:\n
  0: Regular MOTE device\n
  1: DTU device */
#define MA_CFG_CTRL_DNS_STRICT      (1L<<12)     /**< Strict DNS TTL control:\n
    0: At least use 86400s DNS TTL\n
    1: Use the DNS TTL strictly */
#define MA_CFG_CTRL_LED_LONG        (1L<<13)     /**< LED power time control:\n
  0: Shutdown LED after collecting done\n
  1: Shutdown LED before sleeping */
#define MA_CFG_CTRL_POWER_SAVING    (1L<<14)     /**< External module power control:\n
  0: Power down when idle\n
  1: Never power down (using module itself's software power-saving mechanism) */
#define MA_CFG_CTRL_LORA_JSON_REPORT (1L<<15)    /**< Use JSON report for lora/native\n
  0: PLSS report format for LORA/Native \n
  1: Regular JSON report for LORA/Native */

#define MA_CFG_LORA_CTRL_CLS_C (1L<<7)    /**< Support LoRa Class C\n
  0: Class C disabled \n
  1: Class C enabled */

/**
 * Is CUIOT
 */
#define MA_CFG_IS_CUIOT(cfg) ((cfg)->net.rsvr_url[0] == 'd' && (cfg)->net.rsvr_url[1] == 'm' && (cfg)->net.rsvr_url[2] == 'p' &&  (cfg)->net.rsvr_url[3] == '-' )

/**
 * @brief Report type check for none-sensor type (LORA/NB/....)
 */
#define MA_REPORT_TYPE_IS_VALID(type) ((type) >= MA_REPORT_LORA_US902)

/**
 * @brief Report type check for pseudo modules (NULL/CONSOLE/....)
 */
#define MA_REPORT_TYPE_IS_PSEUDO(type) (((type) & 0xfffc) == MA_REPORT_NATIVE_NULL)

/**
 * @brief P2P flags for first payload
 */
#define MA_LORA_P2P_HDR 0xfe

/**
 * @brief P2P min packet length (FE LEN8 DA_EUI64 SA_EUI64 XX)
 */
#define MA_LORA_P2P_MIN_LEN 19

/**
 * @brief Is P2P paylaod 
 */
#define MA_LORA_IS_P2P(buf, len) \
  ((buf)[0] == MA_LORA_P2P_HDR && \
   (len) >= MA_LORA_P2P_MIN_LEN && \
   (buf)[1] == (len) - 2 )


/**
* @brief Report type definition
*/
typedef enum {
  MA_REPORT_TYPE_START = 0x4000,
  MA_REPORT_LORA_US902 = MA_REPORT_TYPE_START,    /**< 0x4000: LORA US902 */  
  MA_REPORT_LORA_CN470 = 0x4001,    /**< 0x4001: LORA CN470 */
  MA_REPORT_MQTT_NBIOT = 0x4180,    /**< 0x4180: MQTT/NBIOT(BC28) */
  MA_REPORT_MQTT_WIFI = 0x4181,     /**< 0x4181: MQTT/WIFI */  
  MA_REPORT_MQTT_LTE = 0x4182,      /**< 0x4182: MQTT/4G/LTE */  
  MA_REPORT_MQTT_CAT1 = 0x4183,     /**< 0x4183: MQTT/CAT1 */    
  MA_REPORT_MQTT_N306 = 0x4184,     /**< 0x4184: MQTT/NBIOT(N306) */  
  MA_REPORT_MQTT_BG95 = 0x4185,     /**< 0x4185: MQTT/NBIOT(BG95) */    
  MA_REPORT_MQTT_MN316 = 0x4186,    /**< 0x4186: MQTT/NBIOT(MN316) */      
  MA_REPORT_NATIVE_NBIOT = 0x4280,  /**< 0x4280: NATIVE MQTT/NBIOT(BC28) (NB module will process MQTT/DNS by itself) */
  MA_REPORT_NATIVE_WIFI = 0x4281,   /**< 0x4281: NATIVE MQTT/WIFI (WIFI module will process MQTT/DNS by itself) */  
  MA_REPORT_NATIVE_LTE = 0x4282,    /**< 0x4282: NATIVE MQTT/4G/LTE (LTE module will process MQTT/DNS by itself) */  
  MA_REPORT_NATIVE_CAT1 = 0x4283,   /**< 0x4282: NATIVE MQTT/CAT1 (CAT1 module will process MQTT/DNS by itself) */    
  MA_REPORT_NATIVE_N306 = 0x4284,   /**< 0x4284: NATIVE MQTT/NBIOT(N306) (NB module will process MQTT/DNS by itself) */    
  MA_REPORT_NATIVE_BG95 = 0x4285,   /**< 0x4285: NATIVE MQTT/NBIOT(BG95) (Module will process MQTT/DNS by itself) */      
  MA_REPORT_NATIVE_MN316 = 0x4286,  /**< 0x4286: NATIVE MQTT/NBIOT(MN316) (Module will process MQTT/DNS by itself) */          
  MA_REPORT_NATIVE_NULL = 0x42fc,   /**< 0x42fe: NATIVE pseudo (No external module) */
  MA_REPORT_NATIVE_CONSOLE = 0x42ff,   /**< 0x42ff: NATIVE console */      
  MA_REPORT_PLSS_NBIOT = 0x4380,    /**< 0x4380: PLSS UDP/NBIOT(BC28) (Polysense WXS8800 compatible UDP report) */
  MA_REPORT_PLSS_WIFI = 0x4381,     /**< 0x4381: PLSS UDP/WIFI (Polysense WXS8800 compatible UDP report) */  
  MA_REPORT_PLSS_LTE = 0x4382,      /**< 0x4382: PLSS UDP/4G/LTE (Polysense WXS8800 compatible UDP report) */  
  MA_REPORT_PLSS_CAT1 = 0x4383,     /**< 0x4383: PLSS UDP/CAT1 (Polysense WXS8800 compatible UDP report) */
  MA_REPORT_PLSS_N306 = 0x4384,     /**< 0x4384: PLSS UDP/NBIOT(N306) (Polysense WXS8800 compatible UDP report) */  
  MA_REPORT_PLSS_BG95 = 0x4385,     /**< 0x4385: PLSS UDP/NBIOT(BG95) (Polysense WXS8800 compatible UDP report) */    
  MA_REPORT_PLSS_MN316 = 0x4386,    /**< 0x4386: PLSS UDP/NBIOT(MN316) (Polysense WXS8800 compatible UDP report) */    

} ma_report_type_t;

/**
* @brief MA FSM definition
*/
typedef enum {
  MA_FSM_INIT_START = 0,             /**< Starting state */  
  MA_FSM_INIT_CFG = 0x1000,          /**< Init configuration */  
  MA_FSM_INIT_DRV = 0x2000,          /**< Init drivers */    
  MA_FSM_INIT_CLI = 0x3000,          /**< Init CLI */      
  MA_FSM_INIT_SENSOR = 0x4000,       /**< Init sensors */      
  MA_FSM_INIT_DEMO = 0x5000,         /**< Init demonstration/debug */ 
  MA_FSM_INIT_EXT = 0x6000,          /**< Init external module */ 
  MA_FSM_INIT_DATA = 0x7000,         /**< Init reporting data buffer */
  MA_FSM_COLLECT = 0x10000,          /**< Sensors collecting start (before duty check) */
  MA_FSM_COLLECT_EVENT1 = 0x10100,   /**< Poling event processing before sleeping */  
  MA_FSM_COLLECT_EVENT2 = 0x10200,   /**< Poling event processing after sleeping */
  MA_FSM_COLLECT_EVENT_ACT = 0x10400,   /**< Poling event processing OTA EVENT_ACT command */  
  MA_FSM_COLLECT_PREPARE = 0x10800,  /**< Duty check and prepare for collecting */
  MA_FSM_COLLECT_POWER_ON = 0x11000, /**< Power on all aged sensors */
  MA_FSM_COLLECT_DUTY = 0x12000,     /**< Duty start and perform sensor collecting */  
  MA_FSM_COLLECT_POWER_OFF = 0x13000,/**< Power off all aged sensors */  
  MA_FSM_REPORT = 0x20000,           /**< Report data valid check */  
  MA_FSM_REPORT_PREPARE = 0x21000,   /**< Prepare data format for reporting and check whether external module is ready */    
  MA_FSM_REPORT_SEND = 0x22000,      /**< Send data */      
  MA_FSM_REPORT_DONE = 0x23000,      /**< Data reporting done */        
  MA_FSM_SLEEP = 0x30000,            /**< Prepare to sleep */ 
  MA_FSM_SLEEP_DUTY = 0x31000,       /**< Calculte duty sleep time */ 
  MA_FSM_SLEEP_DEEP = 0x32000,       /**< Perform deep sleep */ 
  MA_FSM_STOP = 0x80000,             /**< FSM stop state (used for driver maintenance or debugging) */ 
} ma_fsm_t;

/**
* @brief LORA module control structure
*/
typedef struct {
  pos_u8_t       lora_dev_eui[8];   /**< ECFG#0x10 LORA MOTE EUI */
  pos_u8_t       lora_app_eui[8];   /**< ECFG#0x18 APP EUI */
  pos_u8_t       lora_app_key[16];  /**< ECFG#0x20 APP KEY */
  pos_u8_t       lora_app_skey[16]; /**< ECFG#0x30 APP SKEY */
  pos_u8_t       lora_nwk_skey[16]; /**< ECFG#0x40 NWK SKEY */

  pos_u32_t      lora_dev_addr;     /**< ECFG#0x50 DEV Address */
  pos_u16_t      lora_dev_nonce;    /**< ECFG#0x54 DEV NONCE when OTA_FIXED_NONCE()==true */
  pos_u8_t       lora_model;        /**< ECFG#0x56 Model Field \n
  Bit[1]: Fixed 16-Bit Counter Mode\n
  Others: Reserved\n
  */
  pos_u8_t       lora_ack_lmt;      /**< ECFG#0x57 ACK Limit */
  
  pos_u8_t       lora_ack_delay;    /**< ECFG#0x58 ACK Delay */
  pos_u8_t       lora_nb_trans;     /**< ECFG#0x59 NB Trans */
  pos_u8_t       lora_ctrl;         /**< ECFG#0x5a Join/Ctrl \n
  <pre>
  Bit[0]: Listen Before Talk
      0 - Always Send immediately
      1 - Do NOT Send RX RSSI is high (payload is dropped directly when RX RSSI is high)
      
  Bit[1]: ADR Control 
      0 - ADR OFF (DR is automatically controlled by module)
      1 - ADR ON (DR is controlled by NS side)
      
  Bit[2]: Fixed Dev Nonce
      0 - Random dev nonce during each JOIN
      1 - Fixed Dev Nonce (use the above lora_dev_nonce)
      
  Bit[4]: ABP (None OTA) Activation
      0 - OTA Activation (normal JOIN)
      1 - ABP Activation

  Bit[5]: Context(session) saving
      0 - JOIN on each system reset
      1 - Only JOIN once unless CFG is changed

  Bit[6]: Class-B Support (Only B or C can be enabled. Do NOT enable both)
      0 - Disable Class-B
      1 - Enable Class-B

  Bit[7]: Class-C Support (Only B or C can be enabled. Do NOT enable both)
      0 - Disable Class-C
      1 - Enable Class-C

  Others: Reserved
  </pre>  
  */
  pos_u8_t       lora_tx_pwr;       /**< ECFG#0x5b TX Power ID */
  
  pos_u8_t       lora_chan_grp;     /**< ECFG#0x5c Channel Group (8 Frequencies for each group)\n
  <pre>
  Bit[0..3]: From Frequent Group ID
  Bit[4..7]: To Frequent Group ID
  </pre>
  */
  pos_u8_t       lora_fport_tx;     /**< ECFG#0x5d TX FPort */  
  pos_u8_t       lora_fport_rx;     /**< ECFG#0x5e RX FPort */  
  pos_u8_t       lora_flags;        /**< ECFG#0x5f Ctrl Flags \n
  <pre>
   Bit[0]: TX message type
        0 - Unconfirm TX
        1 - Confirm TX
        
   Bit[1~7]: Reserved
  </pre>
  */  
} ma_lora_cfg_t;

/**
* @brief Sensor type definition
*/
typedef enum {
  MA_SENSOR_LINK = 0, /**< #0 Duty report with duty counter */

  MA_SENSOR_VBAT = 1, /**< #1 Voltage of battery\n
  * bit[0] of rsvd32 field is used to control RF signal report: 0-Not Report, 1-Report\n
  * bit[1] of rsvd32 field is used to control BAT voltage mute: 0-Report Voltage, 1-Mute/Not Report\n
  * bit[2] of rsvd32 field is used to control BAT percentage report: 0-Not Report, 1-Report\n
  * bit[8..15] of rsvd32 field is used to control BAT cell num: 0/1-1 Cell, 2~3-2~3 Cells\n
  * arg.u16[0] is used to control the adc voltage calculate ratio: final_voltage = adc_voltage * ratio / 1000 + offset\n
  * arg.i16[1] is used to control the adc voltage calculate offset (it can be negative)\n
  * arg.u16[2] is used to control the 0% voltage (default 3300)
  * arg.u16[3] is used to control the 100% voltage (default 3600)
  * <pre>
  * When thld.u16[0] >= thld.u16[1]\n
  *     If Report Data>=thld.u16[0] OR Data<thld.u16[1],  report; or else, not report\n
  * When thld.u16[0] < thld.u16[1]\n
  *     If Report Data>=thld.u16[0] AND Data<thld.u16[1], report; or else, not report
  * </pre>
  * thld.u16[2] is used to give up the remaining slot collectings when u16[2] > 0 && vbat < u16[2]\n
  * @note All sensors when thold are all zero will report on each duty. If it's expected to have data report on each duty, please leave all thold as zero
  */
  
  MA_SENSOR_DS18B20 = 2, /**< #2 DS18B20 temperature sensor (unit: 0.1 celsius)\n
  * io field is used to specify the single wire port (pin)
  * <pre>
  * When thld.i16[0] >= thld.i16[1]\n
  *     If Report Data>=thld.i16[0] OR Data<thld.i16[1],  report; or else, not report\n
  * When thld.i16[0] < thld.i16[1]\n
  *     If Report Data>=thld.i16[0] AND Data<thld.i16[1], report; or else, not report
  * </pre>
  */

  MA_SENSOR_FASTCNT = 3, /**< #3 IRQ based interrupt counter\n
  * io filed is used to specify the IRQ input pin (DIN)\n
  * bit[0] of rsvd32 field is used to control IRQ counter read/clear: 0-Read/Keep, 1-Read/Clear\n
  * bit[1] of rsvd32 field is used to control IO state report in highest bit: 0-Report, 1-Not Report\n
  * bit[2] of rsvd32 field is used to control PLSS report type: 0-8Bit Counter(PL-56 or Leakage report), 1-32Bit Counter\n
  * bit[3] of rsvd32 field is used to control IRQ Event/polling: 0-Polling, 1-Not Polling\n
  * bit[4] of rsvd32 field is used to control Single/Dual IRQ polarity: 0-Dual RISING+FALLING, 1-Single Polarity\n
  * bit[5] of rsvd32 field is used to control Single IRQ polarity: 0-IRQ Falling, 1-IRQ Rising\n
  * bit[6] of rsvd32 field is used to control EVENT/ACT: 0-Support, 1-Not Support EVENT/ACT\n
  * bit[7] of rsvd32 field is used to control duty trigger: 0-Not Trigger, 1-Trigger all duty slots report\n  
  * bif[8..15] of rsvd32 field is used to specify the action power id used when EVENT/ACT is supported\n
  * arg.u16[0] is used to control the mute time in unit of ms from each data collecting (irq will be ignored in this time, 65001~65535 for 1~535 minutes)
  */

  MA_SENSOR_PT100 = 4, /**< #4 PT100 temperature sensor  (unit: 0.1 celsius)\n
  * io field is used to specify the AIN input pin\n
  * arg.i16[0] is used to control data offset (postive: increase final data; negative: decrease final data)
  */

  MA_SENSOR_PH = 5, /**< #5 PH sensor\n
  * bit[0..7] of io field is used to specify the UART port ID\n
  * bit[0..7] of rsvd32 field is used to specify the PH sensor MODBUS address\n
  * bit[8..31] of rsvd32 field is used to specify the UART baudrate
  */

  MA_SENSOR_MTLCD = 6, /**< #6 MT-LCD/RFID module */

  MA_SENSOR_CO2 = 7, /**< #7 CO2 sensor\n
  * bit[0..7] of io field is used to specify the UART port ID\n
  * bit[0..7] of rsvd32 field is used to trigger CO2 calibration by setting value to 1
  */

  MA_SENSOR_O2 = 8, /**< #8 OSM/O2 sensor\n
  * bit[0..7] of io field is used to specify the UART port ID\n
  * bit[0..7] of rsvd32 field is used to specify the sensor MODBUS address
  */

  MA_SENSOR_CO2NS = 9, /**< #9 NS/CO2 sensor\n
  * bit[0..7] of io field is used to specify the UART port ID\n
  * bit[0..7] of rsvd32 field is used to trigger CO2 OTA calibration by setting value to 1\n
  * bit[8..15] of rsvd32 field is used to trigger CO2 sampling duration seconds for AVG calc\n
  * bit[16..23] of rsvd32 field is used to trigger CO2 sleeping seconds for each round of sampling\n
  * bit[24..28] of rsvd32 field is used to control moving ratio in unit of 5%. 0x14 is 100%.\n
  * bit[29]   of rsvd32 field is used to choose DUTY cycle as auto-calib feed or else 30mins\n
  * bit[30]   of rsvd32 field is used to turn OFF the auto-calib function\n
  * bit[31]   of rsvd32 field is used to turn on CO2 button based calibration (reset required)\n
  * arg.i16[0] is used to control data offset (postive: increase final data; negative: decrease final data)\n
  * arg.u16[1] is used to control data ratio (final = (data+offset)*ratio/100)\n
  * thw0 is used to enable long cycle when co2ppm < thw0\n
  * thw1 is used to enable long cycle when delta(co2ppm) < thw0\n  
  * thw2/3 is used to enable the long cycle when current time is not within this range\n
  * thw2 is used to define the working time range from. For example, 0x081e for 8:30\n
  * thw3 is used to define the working time range to. For example, 0x1200 for 18:00\n
  * thw4 is used to define the long cycle in unit of s\n  
  * When thw2/thw3 are both zero, always disable the long cycle mechanism no matter how thw0/1 are defined\n
  */

  MA_SENSOR_EPDLCD = 10, /**< #10 EPD-LCD module\n
    * bit[0] of rsvd32 field is used to control language: 0-ENGLISH, 1-CHINESE\n
    * bit[1] of rsvd32 field is used to control temp unit: 0-C, 1-F\n
    * bit[2] of rsvd32 field is used to control vbat type: 0-3.6v fixed, 1-3~4.1V (obsolete from v1.35 by using VBAT/PER)\n
    * bit[4..5] of rsvd32 field is used to overwrite face: 0-regular face logic, 1-force bad, 2-force normal, 3-force good\n
    * arg.i8[0]..arg.i8[1] is used to control humidity/good low..high in unit of 1%\n
    * arg.i8[2]..arg.i8[3] is used to control humidity/normal low..high in unit of 1%\n
    * arg.i8[4]..arg.i8[5] is used to control co2/good low..high in unit of 100PPM\n
    * arg.i8[6]..arg.i8[7] is used to control co2/normal low..high in unit of 100PPM\n
    * arg.i8[8]..arg.i8[9] is used to control temperature/good low..high in unit of 1 celsius degree\n
    * arg.i8[10]..arg.i8[11] is used to control temperature/normal low..high in unit of 1 celsius degree\n
    * bit[0..1] of thw0 field is used to control log/type: 0-CO2 displaying, 1-logo A, 2/3 - logo PLSS\n
    * bit[2] of thw0 field is used to control good face: 0-face/hpa displaying, 1-CO2(bit[0..1]==0)/TEMP(bit[0..1]>0) history bar\n
    * bit[3] of thw0 field is used to control LED sync: 0-no LED sync, 1-LED sync with good/bad face\n
    * bit[4..7] of thw0 field is used to control RS485 temp: \n
    *    0-Regular CO2/TEMP displaying\n
    *    1-Three 485 temp/hum\n
    *    2-Two 485 temp/hum\n
    * bit[8] of thw0 field is used to control title: 0-report time, 1-EUI\n
    * bit[9..10] of thw0 field is used to control face area (when bit[2]==0 and bit[4..7]==0): 0-face, 1-HPA, 2/3-rsvd\n
    * bit[0..1] of thw1 field is used to control EPD revisions: 0-uc8276cBB, 1-ssd1683, 2-uc8276cAA\n
    * bit[8] of thw1 field is used to control displaying dir: 0-normal X asis, 1-reversed X asis\n
    * bit[9] of thw1 field is used to control displaying dir: 0-normal Y asis, 1-reversed Y asis\n
    * thw2 field is used to control low voltage to stop refreshing, 0-3100 (3.1V will be used by default)\n
    * thw3 field is used to control high voltage to resume refreshing, 0-3200 (3.2V will be used by default)\n
    */

  MA_SENSOR_SHT30 = 11, /**< #11 SHT30/HP203/... I2C sensors \n
  * bit[0..7] of io field is used to specify the I2C ID\n
  * bit[0] of rsvd32 field is used to control temp report\n
  * bit[1] of rsvd32 field is used to control humidity report\n
  * bit[2] of rsvd32 field is used to control hpa report\n
  * bit[3] of rsvd32 field is used to control voc report\n
  * bit[4] of rsvd32 field is used to control light report\n
  * bit[5] of rsvd32 field is used to control ADXL35x mems report (mg)\n
  * bit[6] of rsvd32 field is used to control ADXL35x mems report (angle)\n  
  * bit[16..31] of rsvd32 field is used to control I2C speed in unit of 100Hz (default zero will use 100KHz)\n
  * arg.u16[1] is used to control read retry number (default zero will use 1024 retry)\n
  * For ADXL35x:\n
  *   arg.u16[0] is used to activate the search mode in unit of s. Searching at most given seconds until thld crossed\n
  *   thw0/thw1 X >= thw1 or X <= thw0 will be treated as thld crossed\n
  *   thw2/thw3 Y >= thw3 or Y <= thw2 will be treated as thld crossed\n
  *   thw4/thw5 Z >= thw5 or Z <= thw4 will be treated as thld crossed\n
  * For SHT30:\n
  *   arg.u8[0] is used to set the report accumaltion number (32 max). When report failed, data will be accumulated\n
  *   thw0 will be used as temperature calibration in unit of 0.1 celsius\n
  *   thw1 will be used as humidity calibration in unit of 0.01%\n  
  */

  MA_SENSOR_O2ZE03 = 12, /**< #12 ZE03/MO2 sensor\n
  * bit[0..7] of io field is used to specify the UART port ID
  */

  MA_SENSOR_PM = 13, /**< #13 ZE03 PM2.5/10/1.0 sensor\n
  * bit[0..7] of io field is used to specify the UART port ID
  */

  MA_SENSOR_UART = 14, /**< #14 UART sensor\n
  * bit[0..7] of io field is used to specify the UART port ID\n
  * rsvd32 filed is isued to specify the UART baudrate (0 will be treated as 9600)
  */

  MA_SENSOR_ULTRA = 15, /**< #15 ULTRA-SONIC sensor\n
  * bit[0..7] of io field is used to specify the UART port ID\n
  * bit[0..7] of rsvd32 field is used to specify the sensor MODBUS address\n
  * bit[8..31] of rsvd32 field is used to specify the UART baudrate
  */  

  MA_SENSOR_SOUND = 16, /**< #16 Sound sensor (MODBUS)\n
  * bit[0..7] of io field is used to specify the UART port ID\n
  * bit[0..7] of rsvd32 field is used to specify the sensor MODBUS address\n
  * bit[8..31] of rsvd32 field is used to specify the UART baudrate\n
  * thw3 is used to report theshold control: 0-report on each cycle, other-report history data when value >= this numer
  * thw7 is used to enable and define the accumulating cycles for min/max/avg report (only work when thw3 is 0)
  */

  MA_SENSOR_SOIL = 17, /**< #17 Soil sensor\n
  * bit[0..7] of io field is used to specify the UART port ID\n
  * bit[0..7] of rsvd32 field is used to specify the sensor MODBUS address\n
  * bit[8..31] of rsvd32 field is used to specify the UART baudrate
  * arg.u8[0] is used to control various sub-datas (Not support this field 
  currently). This field is a bitmap definition. See below:\n
  *   bit[0]: Report soil humidity\n
  *   bit[1]: Report Soil temperature\n  
  *   bit[2]: Report soil solinity\n
  *   bit[3]: Report soil ec
  */

  MA_SENSOR_NPK = 18, /**< #18 NPK sensor\n
  * bit[0..7] of io field is used to specify the UART port ID\n
  * bit[0..7] of rsvd32 field is used to specify the sensor MODBUS address\n
  * bit[8..31] of rsvd32 field is used to specify the UART baudrate
  * arg.u8[0] is used to control various sub-datas. This field is a bitmap definition. See below:\n
  *   bit[0]: Report N index\n
  *   bit[1]: Report P index\n  
  *   bit[2]: Report K index\n
  */

  MA_SENSOR_ZE03 = 19, /**< #19 ZE03 (or ZC13) sensor\n
  * bit[0..7] of io field is used to specify the UART port ID\n
  * bit[0..7] of rsvd32 field is used to specify the sensor special address (0 for ZE03, 1 for ZC101-CH4, 0xff for ZC13/SC16)\n
  * bit[8..31] of rsvd32 field is used to specify the UART baudrate
  */

  MA_SENSOR_ULTRA_MBUS = 20, /**< #20 RS485/MODBUS Ultrasonic sensor\n
  * bit[0..7] of io field is used to specify the UART port ID\n
  * bit[0..7] of rsvd32 field is used to specify the sensor MODBUS address\n
  * bit[8..31] of rsvd32 field is used to specify the UART baudrate\n
  * arg.u8[0] is used to control number of sensors attached
  */

  MA_SENSOR_TEMP_485 = 21, /**< #21 RS485/MODBUS Temperature/Humidity sensor\n
  * bit[0..7] of io field is used to specify the UART port ID\n
  * bit[0..7] of rsvd32 field is used to specify the sensor MODBUS address\n
  * bit[8..31] of rsvd32 field is used to specify the UART baudrate\n
  * arg.u8[0] is used to control number of sensors attached (continuous MODBUS address)\n
  */

  MA_SENSOR_LASER_DISTANCE = 22, /**< #22 Laser Distance sensor\n
  * bit[0..7] of io field is used to specify the UART port ID\n
  * bit[8..31] of rsvd32 field is used to specify the UART baudrate
  */

  MA_SENSOR_ZPHS_MULTI = 23, /**< #23 ZPHS Multi sensor\n
  * bit[0..7] of io field is used to specify the UART port ID\n
  * bit[8..31] of rsvd32 field is used to specify the UART baudrate
  * arg.u16[0] is used to control various sub-datas. This field is a bitmap definition. See below:\n
  *   bit[0]: Report CO2 data\n
  *   bit[1]: Report CH2O/VOC (by bit[15]) data\n  
  *   bit[2]: Report Humidity data\n
  *   bit[3]: Report Temperature data\n
  *   bit[4]: Report PM2.5 or PM2.5/PM10/PM1.0 data\n  
  *   bit[14]: Report using polling mode for (0-PM2.5 only sensor, 1-PM2.5/PM10/PM1.0 sensor)\n
  *   bit[15]: Report CH2O/VOC, 0-VOC, 1-CH2O\n
  * arg.u16[1] is used to control CO2 calibration data. 0 for Not-calib, 400 (or others) for calib
  */

  MA_SENSOR_DGM10_MULTI = 24, /**< #24 DGM10 Multi sensor\n
  * bit[0..7] of io field is used to specify the UART port ID\n
  * bit[8..31] of rsvd32 field is used to specify the UART baudrate
  * arg.u16[0] is used to control various sub-datas. This field is a bitmap definition. See below:\n
  *   bit[0]: Report #0 Temperature\n
  *   bit[1]: Report #1 Humidity\n  
  *   bit[2]: Report #2 Gas#0\n
  *   bit[3]: Report #3 Gas#1\n
  * arg.u16[1] is used to control calibration:\n
  *   bit[0]: Gas#0 calib, 0-Start ZERO recalib, 1-Load ZERO default\n
  *   bit[1]: Gas#0 zero, 0-Keep current, 1-Write current as ZERO\n  
  *   bit[2]: Gas#1 calib, 0-Start ZERO recalib, 1-Load ZERO default\n
  *   bit[3]: Gas#2 setup zero, 0-Keep current, 1-Write current as ZERO\n  
  *   bit[4]: Gas#0 calib control, 0-Do nothing, 1-Set\n
  *   bit[5]: Gas#0 zero control, 0-Do nothing, 1-Set\n 
  *   bit[6]: Gas#1 calib control, 0-Do nothing, 1-Set\n
  *   bit[7]: Gas#1 zero control, 0-Do nothing, 1-Set
  */

  MA_SENSOR_WIND_SPEED = 25, /**< #25 Wind Speed sensor\n
  * bit[0..7] of io field is used to specify the UART port ID\n
  * bit[8..31] of rsvd32 field is used to specify the UART baudrate
  */

  MA_SENSOR_WIND_DIR = 26, /**< #26 Wind Dir sensor\n
  * bit[0..7] of io field is used to specify the UART port ID\n
  * bit[8..31] of rsvd32 field is used to specify the UART baudrate
  */

  MA_SENSOR_FLOW = 27, /**< #27 Velocity flow\n
  * bit[0..7] of io field is used to specify the UART port ID\n
  * bit[8..31] of rsvd32 field is used to specify the UART baudrate
  */

  MA_SENSOR_DO = 28, /**< #28 DO (O2 in unit of mg/L)\n
  * bit[0..7] of io field is used to specify the UART port ID\n
  * bit[0..7] of rsvd32 field is used to specify the sensor MODBUS address\n
  * bit[8..31] of rsvd32 field is used to specify the UART baudrate
  */

  MA_SENSOR_LIGHT = 29, /**< #29 Light (lux)\n
  * bit[0..7] of io field is used to specify the UART port ID\n
  * bit[8..31] of rsvd32 field is used to specify the UART baudrate
  */

  MA_SENSOR_ZCE04 = 30, /**< #30 ZCE04B multi  sensors\n
  * bit[0..7] of io field is used to specify the UART port ID\n
  * bit[8..31] of rsvd32 field is used to specify the UART baudrate
  */

  MA_SENSOR_MBUS_END = 31, /**< #31 Last sensor to be defined for MODBUS series */
  
  MA_SENSOR_UV = 32, /**< #32 UV Power sensor (Analog)\n
  * bit[0..7] of io field is used to specify the AI port ID\n
  * arg.u32[0] is used to control the result multi-ratio for 0.0001 based. For exmaple: 10000 for 1.0. When this value is 0, 20151 (2.0151) will be used.
  */  

  MA_SENSOR_MV = 33, /**< #33 Voltage/current sensor (Analog)\n
  * bit[0..7] of io field is used to specify the AI port ID\n
  * bit[0] of rsvd32 field is used to control report type, 0-MV voltage, 1-Current\n
  * bit[7] of rsvd32 field is used to control MV report, 0-not report, 1-report orginal mv\n
  * bit[8] of rsvd32 field is used to control ADC filter, 0-AVG filter, 1-MAX filter\n
  * arg.u32[0] is used to control the result multi-ratio for 0.0001 based. For exmaple: 10000 for 1.0. When this value is 0, 10000 (1.0) will be used.\n  
  * arg.u32[1] is used to control the ADC duration for MAX filter, 0-200ms, others-specified ms\n
  * thw3/thw4 is used to report high/low theshold control: 0/0-report on each cycle, other-report history data when value >= thw[3] || value <= thw[4] \n
  */

  MA_SENSOR_WATER = 34, /**< #32 Water sensor (Analog, MPa)\n
  * bit[0..7] of io field is used to specify the AI port ID\n
  * arg.u16[0] is used to control the result multi-ratio. For exmaple: 1280 for 1.6MPa/1250mv. When this value is 0, 1280 will be used\n
  * arg.u16[1] is used to control the dead zone. For exmaple: 100 for 100mv deadzone. When this value is 0, 100 will be used.  
  */

  MA_SENSOR_WIND_SPEED_ANALOG = 35, /**< #35 Wind Speed analog sensor\n
  * bit[0..7] of io field is used to specify the AI port ID\n
  * arg.u16[0] is used to control the result multi-ratio. For exmaple: 15000 for 30000mm/s under 2000mv. When this value is 0, 15000 will be used\n
  * arg.u16[1] is used to control the dead zone. For exmaple: 100 for 100mv deadzone. 
  */

  MA_SENSOR_WIND_DIR_ANALOG = 36, /**< #36 Wind Dir analog sensor\n
  * bit[0..7] of io field is used to specify the AI port ID\n
  * arg.u16[0] is used to control the result multi-ratio. For exmaple: 75 for 2V/NorthWest. When this value is 0, 15 will be used\n
  * arg.u16[1] is used to control the dead zone. For exmaple: 100 for 100mv deadzone. 
  */

  MA_SENSOR_DISP_ANALOG = 37, /**< #36 Displacement analog sensor\n
  * bit[0..7] of io field is used to specify the AI port ID\n
  * arg.u16[0] is used to control the result multi-ratio. For exmaple: 21140 for 52.85mm/0V. When this value is 0, 21140 will be used\n
  * arg.u16[1] is used to control the dead zone. For exmaple: 100 for 100mv deadzone. 
  */

  MA_SENSOR_ANALOG_END = 47, /**< #47 Last sensor to be defined for Analog series */

  MA_SENSOR_IO_GEN = 48, /**< #48 Generic IO control and counter report\n
  This is a sub type of 3. For more detailed, refer to @ref MA_SENSOR_FASTCNT 
  */

  MA_SENSOR_IO_TRIGGER = 49, /**< #49 Generic IO trigger an duty report\n
  * This is a sub type of 3. For more detailed, refer to @ref MA_SENSOR_FASTCNT 
  * arg.u16[0] is used to control the mute time after each duty triggerring (irq will be disabled in this time, 65001~65535 for 1~535 minutes. default 60s.)
  */

  MA_SENSOR_IO_PULL = 56, /**< #56 Generic IO control and pulling report\n
  * io filed is used to specify the IRQ input pin (DIN)\n
  * bit[0..14] of rsvd32 field is used to specify the wait/polling time (when zero, not wait/polling)\n
  * bit[15] of rsvd32 field is used to specify the the above time unit: 0-s, 1-ms\n
  * bit[16..30] of rsvd32 field is used to specify the the sleeping ms during wait/polling\n
  * bit[31] of rsvd32 field is used to specify the the waiting polarity: 0-waiting until high, 1-waiting until low
  */

  MA_SENSOR_IO_END = 63, /**< #63 Last sensor to be defined for Analog series */  
  
  MA_SENSOR_CNTR12 = 64, /**< #64 Counter 1-2 sensor (human detection with two IRQs)\n
  * io filed is used to specify the #1 IRQ input pin (DIN)\n
  * bit[0..15] of rsvd32 field is used to specify the #2 IRQ input pin (DIN)\n
  * bit[16] of rsvd32 field is used to control Single/Dual IRQ polarity: 0-Dual RISING+FALLING, 1-Single Polarity\n
  * bit[17] of rsvd32 field is used to control Single IRQ polarity: 0-IRQ Falling, 1-IRQ Rising\n
  * arg.u16[0] is used to specify the IRQ jitter delay (unit: ms)\n
  * arg.u16[1] is used to specify the state clear time (unit: ms)\n
  * bit[0] of thw0 field is used to report accumulating counter: 0-not report, 1-report\n
  */

  MA_SENSOR_GPS = 65, /**< #65 GPS sensor\n
  * bit[0..7] of io field is used to specify the UART port ID\n
  * bit[8..31] of rsvd32 field is used to specify the UART baudrate
  */  

  MA_SENSOR_SOIL2 = 66, /**< #66 Soil sensor#2 (Hum/Temp/EC/N/P/K/PH)\n
  * bit[0..7] of io field is used to specify the UART port ID\n
  * bit[0..7] of rsvd32 field is used to specify the sensor MODBUS address\n
  * bit[8..31] of rsvd32 field is used to specify the UART baudrate
  * arg.u8[0] is used to control various sub-datas. This field is a bitmap definition. See below:\n
  *   bit[0]: Report soil humidity\n
  *   bit[1]: Report Soil temperature\n  
  *   bit[2]: Report soil ec
  *   bit[3]: Report N index\n
  *   bit[4]: Report P index\n  
  *   bit[5]: Report K index\n  
  *   bit[6]: Report PH index\n    
  */

  MA_SENSOR_PH_ORP = 67, /**< #67 PH/ORP sensor\n
  * bit[0..7] of io field is used to specify the UART port ID\n
  * bit[0..7] of rsvd32 field is used to specify the PH sensor MODBUS address\n
  * bit[8..31] of rsvd32 field is used to specify the UART baudrate
  */

  MA_SENSOR_TB600 = 68, /**< #68 TB600 multi sensor\n
  * bit[0..7] of io field is used to specify the UART port ID (0~3) or I2C ID (0x20~0x23)
  * bit[8..31] of rsvd32 field is used to specify the UART or I2C baudrate (default UART=9600 / I2C=8KHz)
  * arg.u8[0] is used to control various sub-datas. This field is a bitmap definition. See below:\n
  *   bit[0]: Report main sensor data\n
  *   bit[1]: Report temperature\n  
  *   bit[2]: Report humidty  
  */

  MA_SENSOR_XDIO = 69, /**< #69 Multi DIO IRQ interrupt bmp sensor\n
  * io filed is used to specify the first IRQ input pin (using pin rsvd[n])\n
  * bit[0] of rsvd32 field is used to control which pin channel (0-rsvd/pin, 1=dio/pin)\n
  * arg.u8[0] is used to control how many PINs will be used from the given first input pin (when zero, 10 will be used)\n
  * arg.u8[1] is used to control the settling down duration in unit of 0.1s (when zero, 2.5s will be used)\n
  * arg.u8[2] is used to control the additional settling down cycles (total settling time = u8[1] * (u8[2]+1) )\n
  */

  MA_SENSOR_CO2SS = 70, /**< #70 Senseair(SS)/CO2 sensor\n
  * bit[0..7] of io field is used to specify the UART port ID\n
  * bit[0..7] of rsvd32 field is used to trigger CO2 calibration by setting value to 1\n
  * bit[8..15] of rsvd32 field is used to trigger CO2 sampling duration seconds for AVG calc\n
  * bit[16..23] of rsvd32 field is used to trigger CO2 sleeping seconds for each round of sampling\n
  * bit[24..28] of rsvd32 field is used to control moving ratio in unit of 5%. 0x14 is 100%.\n
  * bit[29]   of rsvd32 field is used to choose DUTY cycle as auto-calib feed or else 30mins\n
  * bit[30]   of rsvd32 field is used to turn OFF the auto-calib function\n
  * bit[31]   of rsvd32 field is used to turn on CO2 button based calibration (reset required)\n
  * arg.i16[0] is used to control data offset (postive: increase final data; negative: decrease final data)\n
  * arg.u16[1] is used to control data ratio (final = (data+offset)*ratio/100)\n
  * thw0 is used to enable long cycle when co2ppm < thw0\n
  * thw1 is used to enable long cycle when delta(co2ppm) < thw0\n  
  * thw2/3 is used to enable the long cycle when current time is not within this range\n
  * thw2 is used to define the working time range from. For example, 0x081e for 8:30\n
  * thw3 is used to define the working time range to. For example, 0x1200 for 18:00\n
  * thw4 is used to define the long cycle in unit of s\n  
  * When thw2/thw3 are both zero, always disable the long cycle mechanism no matter how thw0/1 are defined\n
  */

  MA_SENSOR_IM948 = 71, /**< #71 IM948 Multi axis (angles) sensor\n
  * io filed is used to specify the UART port ID\n
  * bit[0..7] of rsvd32 field is used to trigger IM948 calibration\n
  */

  MA_SENSOR_RSCOIL = 72, /**< #72 RSxxxx COIL sensor (RS485 current)\n
  * io filed is used to specify the UART port ID\n
  */

  MA_SENSOR_FS801 = 73, /**< #73 NS/CO2 sensor\n
  * bit[0..7] of io field is used to specify the UART port ID\n
  * bit[0..7] of rsvd32 field is used to trigger CO2 OTA calibration by setting value to 1\n
  * bit[8..15] of rsvd32 field is used to trigger CO2 sampling duration seconds for AVG calc\n
  * bit[16..23] of rsvd32 field is used to trigger CO2 sleeping seconds for each round of sampling\n
  * bit[24..28] of rsvd32 field is used to control moving ratio in unit of 5%. 0x14 is 100%.\n
  * arg.i16[0] is used to control data offset (postive: increase final data; negative: decrease final data)\n
  * arg.u16[1] is used to control data ratio (final = (data+offset)*ratio/100)\n
  * thw0 is used to enable long cycle when co2ppm < thw0\n
  * thw1 is used to enable long cycle when delta(co2ppm) < thw0\n  
  * thw2/3 is used to enable the long cycle when current time is not within this range\n
  * thw2 is used to define the working time range from. For example, 0x081e for 8:30\n
  * thw3 is used to define the working time range to. For example, 0x1200 for 18:00\n
  * thw4 is used to define the long cycle in unit of s\n  
  * When thw2/thw3 are both zero, always disable the long cycle mechanism no matter how thw0/1 are defined\n
  */

  MA_SENSOR_FSXCS = 74, /**< #74 RS-FSXCS-Nxx multi sensor\n
  * bit[0..7] of io field is used to specify the UART port ID\n
  * bit[0..7] of rsvd32 field is used to specify the PH sensor MODBUS address\n
  * bit[8..31] of rsvd32 field is used to specify the UART baudrate
  * arg.u16[0] is used to control various sub-datas. This field is a bitmap definition. See below:\n
  *   bit[0]: Report wind speed\n
  *   bit[1]: Report wind dir\n  
  *   bit[2]: Report humidity\n
  *   bit[3]: Report temperature\n
  *   bit[4]: Report sound/noise\n
  *   bit[5]: Report PM2.5/PM10\n
  *   bit[6]: Report atmosphere pressure\n
  *   bit[7]: Report light\n
  *   bit[8]: Report rainfall\n
  *   bit[9]: Report global horizontal irradiance\n 
  */

  MA_SENSOR_JC = 75, /**< #75 JC meter sensor\n
  * bit[0..7] of io field is NOT used by this sensor (always set zero here)\n
  * bit[0..7] of rsvd32 field is used to specify the report style: 0-raw JC payload, 1-PLSS payload\n
  */

  MA_SENSOR_CO2Z19 = 76, /**< #76 Z19B/CO2 sensor\n
  * bit[0..7] of io field is used to specify the UART port ID\n
  * bit[0..7] of rsvd32 field is used to trigger CO2 calibration by setting value to 1\n
  * bit[8..15] of rsvd32 field is used to trigger CO2 sampling duration seconds for AVG calc\n
  * bit[16..23] of rsvd32 field is used to trigger CO2 sleeping seconds for each round of sampling\n
  * bit[24..28] of rsvd32 field is used to control moving ratio in unit of 5%. 0x14 is 100%.\n
  * arg.i16[0] is used to control data offset (postive: increase final data; negative: decrease final data)\n
  * arg.u16[1] is used to control data ratio (final = (data+offset)*ratio/100)\n
  * thw0 is used to enable long cycle when co2ppm < thw0\n
  * thw1 is used to enable long cycle when delta(co2ppm) < thw0\n  
  * thw2/3 is used to enable the long cycle when current time is not within this range\n
  * thw2 is used to define the working time range from. For example, 0x081e for 8:30\n
  * thw3 is used to define the working time range to. For example, 0x1200 for 18:00\n
  * thw4 is used to define the long cycle in unit of s\n  
  * When thw2/thw3 are both zero, always disable the long cycle mechanism no matter how thw0/1 are defined\n
  */

  MA_SENSOR_MIX = 77, /**< #77 Mixed multi sensors (O2/CH4/H2S/CO/...)\n
  * bit[0..7] of io field used to select 1~3 for Group1~Group3 (0 is also Group1)\n
  * bit[0..7] of rsvd32 field is used to specify the report bitmap. See below:\n
  *  bit[0]: Report O2 (through per grp uart0)\n
  *  bit[1]: Report CH4 (through per grp uart1)\n
  *  bit[2]: Report CO... TB600 I2C single gas\n
  *  bit[3]: Report H2S... TB200 UART single gas (through grp1 uart2 or grp2 uart3)\n
  *  bit[4]: Report CO2 (MH-xxx series)\n
  *  bit[5]: Report DGM10 CO+H2S... (two in one) gas\n
  * thw0 is used to enable O2 waiting (wait at most 10mins, until O2 >= thw0 in unit 0.1%)\n
  */

  MA_SENSOR_MTEPD = 78, /**< #78 MT-EPD/RFID module\n
  * rsvd32 field is used to specify the RFID value\n
  * bit[0..1] of thw0 field is used to control display type: 0-MTG 3 groups, 1-MTG 5 groups\n  
  * bit[0..1] of thw1 field is used to control EPD revisions: 0-uc8276cBB, 1-ssd1683, 2-uc8276cAA\n
  * bit[8] of thw1 field is used to control displaying dir: 0-normal X asis, 1-reversed X asis\n
  * bit[9] of thw1 field is used to control displaying dir: 0-normal Y asis, 1-reversed Y asis\n
  * bit[10] of thw1 field is used to control HR/AW displaying: 0-HR, 1-AW\n
  * thw2 field is used to control low voltage to stop refreshing, 0-3100 (3.1V will be used by default)\n
  * thw3 field is used to control high voltage to resume refreshing, 0-3200 (3.2V will be used by default)\n
  */

  MA_SENSOR_IM1281 = 79, /**< #79 RS485/MODBUS IM1281 E-meter sensor\n
  * bit[0..7] of io field is used to specify the UART port ID\n
  * bit[0..7] of rsvd32 field is used to specify the sensor MODBUS address\n
  * bit[8..31] of rsvd32 field is used to specify the UART baudrate
  * arg.u8[0] field is used to specify the report bitmap (when 0, uses 0x9f). See below:\n
  *  bit[0]: Report VAC voltage \n
  *  bit[1]: Report AAC ampire\n
  *  bit[2]: Report W\n
  *  bit[3]: Report KWH\n
  *  bit[4]: Report PF\n
  *  bit[5]: Report CO2 (0.0001kg Carbon dioxide)\n
  *  bit[6]: Report Temperature\n
  *  bit[7]: Report HZ\n
  */

  MA_SENSOR_IPOS = 80, /**< #80 Indoor Position (ES8266/ESP32 WIFI SCAN) sensor\n
  * bit[0..7] of io field is used to specify the UART port ID\n
  * bit[0..15] of rsvd32 field is used to specify the SHAKING detecting PIN (0 for WIRE1#0 PIN)\n
  * arg.u8[0] field is used to specify the minimal report gap in unit of seconds. Default Zero will use 10s.\n
  * arg.u8[1] field is used to specify the MAX SSID number reported in each fragment. Default Zero will use 12.\n
  * arg.u8[2] field is used to specify the MAX SSID collected in each round. Default Zero will use 60.\n
  * thw0 field is used to specify the UART baudrate in unit of 100Hz (default will use 115200)\n
  */

  MA_SENSOR_SGA457 = 81, /**< #81 SGA-400/500/700 Modbus sensor\n
  * bit[0..7] of io field is used to specify the UART port ID\n
  * bit[0..7] of rsvd32 field is used to specify the sensor MODBUS address\n
  * bit[8..31] of rsvd32 field is used to specify the UART baudrate  
  * arg.u16[0] field is used to trigger the calibration (0xffff: ReAdjust 0; 0xfffe: calib by 0; others: calib)\n
  */

  MA_SENSOR_MTS4 = 82, /**< #82 MTS4 Ultra-Low Temp I2C sensors \n
  * bit[0..7] of io field is used to specify the I2C ID\n
  * bit[16..31] of rsvd32 field is used to control I2C speed in unit of 100Hz (default zero will use 100KHz)\n
  */

  MA_SENSOR_FS217 = 83, /**< #83 FS217 PM sensors \n
  * bit[0..7] of io field is used to specify the UART port ID\n
  * bit[0] of rsvd32 field is used to control dataset (0-atmosphere dataset, 1=standard dataset)\n
  */

  MA_SENSOR_SWR = 84, /**< #84 SWR Spray Water \n
  * bit[0..7] of io field is used to specify the wheel detector IO (a reference to DIO#N)\n
  * thw[0] is used to control the area width in unit of mm (default zero will use 5000 mm)\n
  * thw[1] is used to control the water speed in unit of ml/m3 (default will use 2000ml/m3)\n
  * thw[2] is used to control the valve/big flow speed in unit of ml/min (default zero will use 10000 ml/min)\n
  * thw[3] is used to control the valve/small flow speed in unit of ml/min (default zero will use 5000 ml/min)\n
  * thw[4].bit[0..7] is used to control the small valve number (default zero will use 2)\n
  * thw[4].bit[8..15] is used to control the big valve number (default zero will use 4)\n
  * thw[5] is used to control the dead zone speed in unit of mm/min (default zero will use 1000mm/min)\n
  * thw[6].bit[0..7] is used to control the pump on delay in unit of 1s\n
  * thw[6].bit[8..15] is used to control the pump off delay in unit of 1s\n
  * arg[0] field is used to control the manual spaying speed (uint ml/min): (0-100000ml/min, 0xffffffff-0ml/min)\n
  */

  MA_SENSOR_IPS = 85, /**< #85 IPS Inteligent Power Strip \n
  * bit[0..7] of io field is used to specify the UART port ID\n
  * bit[0] of rsvd32 field is used to control default power output state (1-on, 0-off)\n
  * bit[4..7] of rsvd32 field is used to specify the sensor base address(0~3)\n
  * bit[8..31] of rsvd32 field is used to specify the UART baudrate\n
  * thw[0] is used to control the watt detecting thold in unit of 0.1w (default zero will use 10.0w (100)\n
  * thw[1] is used to control the effective button pressing time in unit of 1ms (default zero will use 50ms)\n
  * thw[2] is used to control the operating delay for each power control switching (default zero will use 500ms)\n
  * arg.u8[0] field is used to specify the report data (1-report, 0-not report. when all zero, 0xf will be used):
  *  bit[0]: report IAC
  *  bit[1]: report VAC
  *  bit[2]: report Watt
  *  bit[3]: report KWH
  *  bit[4]: report Hz  
  */

  MA_SENSOR_PUF = 86, /**< #86 PUF sensors (security processing)\n
  * bit[0..7] of io field is used to specify the I2C ID\n
  * bit[16..31] of rsvd32 field is used to control I2C speed in unit of 100Hz (default zero will use 100KHz)\n  
  */

  MA_SENSOR_SOIL3 = 87, /**< #87 Soil sensor#3 (Hum/Temp/EC/N/P/K/PH)\n
  * bit[0..7] of io field is used to specify the UART port ID\n
  * bit[0..7] of rsvd32 field is used to specify the sensor MODBUS address\n
  * bit[8..31] of rsvd32 field is used to specify the UART baudrate
  * arg.u8[0] is used to control various sub-datas. This field is a bitmap definition. See below:\n
  *   bit[0]: Report Soil temperature\n
  *   bit[1]: Report soil humidity\n
  *   bit[2]: Report soil ec\n
  *   bit[3]: Report N index\n
  *   bit[4]: Report P index\n
  *   bit[5]: Report K index\n
  *   bit[6]: Report PH index\n
  */

  MA_SENSOR_MFM = 88, /**< #88 MFM gas flow sensor\n
  * bit[0..7] of io field is used to specify the UART port ID\n
  * bit[0..7] of rsvd32 field is used to specify the sensor MODBUS address\n
  * bit[8..31] of rsvd32 field is used to specify the UART baudrate
  */

  MA_SENSOR_RFID = 89, /**< #89 Ultra highspeed RFID sensor\n
  * bit[0..7] of io field is used to specify the UART port ID\n
  * bit[8..31] of rsvd32 field is used to specify the UART baudrate\n
  * arg.u8[0] is used to control RFID collectin time in unit of seconds. When zero, 1s will be used.\n
  */

  MA_SENSOR_COIL = 90, /**< #90 RS232/485 based coil sensor\n
  * bit[0..7] of io field is used to specify the UART port ID\n
  * bit[0..7] of rsvd32 field is used to specify the sensor MODBUS address\n
  * bit[8..31] of rsvd32 field is used to specify the UART baudrate\n
  * arg.u16[0] is used to the set primary current transformer value. When zero, nothing will be set.\n
  * thw[0].bit[0..7] is used to define the report bitmap. When zero, 15 will be used. The definitions is:\n
  *   bit[0]: Report #1 coil (#A)\n
  *   bit[1]: Report #2 coil (#B)\n
  *   bit[2]: Report #3 coil (#C)\n
  *   bit[3]: Report #4 coil (#D)\n
  */

  MA_SENSOR_OIW = 91, /**< #91 RS232/485 based OIW (oil in water) sensor\n
  * bit[0..7] of io field is used to specify the UART port ID\n
  * bit[0..7] of rsvd32 field is used to specify the sensor MODBUS address\n
  * bit[8..31] of rsvd32 field is used to specify the UART baudrate\n
  * arg.u8[0] is used to the control the reporting data. When zero, 7 will be used. This is a bitmap definiton:\n
  *   bit[0]: Report temperature\n
  *   bit[1]: Report RFU (relative fluorescence units)\n
  *   bit[2]: Report OIW (oil in water)\n
  */

  MA_SENSOR_BMP390 = 92, /**< #92 BMP390 I2C HPA+TEMPERATURE sensors \n
  * bit[0..7] of io field is used to specify the I2C ID\n
  * bit[16..31] of rsvd32 field is used to control I2C speed in unit of 100Hz (default zero will use 100KHz)\n
  * bit[0..7] of rsvd32 is used to control report bmp. When zero, 0x2 will be used (only HPA). See below:\n
  *   bit[0]: Report temperature\n
  *   bit[1]: Report HPA\n  
  * arg.u16[0] is used to the control the ODR+OSR. When zero, 0x0415 (ODR=4, OSR_T=2, OSR_P=5) will be sued. See below:\n
  *   bit[0..2]: OSR_P\n
  *   bit[3..5]: OSR_T\n
  *   bit[6..7]: Not used. Keep it as zero\n
  *   Bit[8..15]: ODR\n
  */

  MA_SENSOR_SM9569 = 93, /**< #93 RS232/485 based GHI radiation sensor (SM9569)\n
  * bit[0..7] of io field is used to specify the UART port ID\n
  * bit[0..7] of rsvd32 field is used to specify the sensor MODBUS address\n
  * bit[8..31] of rsvd32 field is used to specify the UART baudrate\n
  */

  MA_SENSOR_PACK = 4097, /**< #4097 Report data packer (group multiple report into one big packet and send it later)\n
  * bit[0] of rsvd32 field is used to control whether to report timestamp or not (0-report, 1=not report)\n
  * bit[1] of rsvd32 field is used to control whether to report data threshold flag (0-not report, 1=report)\n
  * arg.u16[0] is used to control how many cycles of report will be packed into one big report\n
  * thw[0]~thw[7] is used to define each data threshold\n
  */
  
} ma_sensor_type_t;

/**
* @brief EEPROM sensor slot configuration structure
*/
typedef struct {
  /**
  * @brief ECFG#0x080+0x30*SLOT Duty cycle in unit of second
  */
  pos_u32_t  cycle; /**< 
  * 0: Shutdown this sensor (never collect it in any duty)\n
  * 1~0xfffffffe: Second based duty interval\n
  * 0xffffffff: Collecting on each duty\n
  */

  /**
  * @brief ECFG#0x084+0x30*SLOT Sensor type
  */  
  pos_u16_t  sensor_type; /**< Refer to @ref ma_sensor_type_t */

  /**
  * @brief ECFG#0x086+0x30*SLOT Sensor power on time
  */  
  pos_u16_t    power_ms; /**<  
  * 0~0xfffd: Power on time in unit of ms. After duty collecting done, the sensor will be power off immediately\n  
  * 0xfff0: 2 minutes\n
  * 0xfff1: 4 minutes\n
  * 0xfff2: 8 minutes\n
  * 0xfff3: 16 minutes\n
  * 0xfffe: Do NOT perform any power ON/OFF operatoin for this sensor\n
  * 0xffff: Always keep power on and never perform power off for this sensor\n
  */

  /**
  * @brief ECFG#0x088+0x30*SLOT Periodic cycle
  * @note Peridoic cycle is a long report time control. When sensor data threshold is activated (not zero) and sensor data is under this\n
  * threshold control, data will be disabled to report for at most continuous periodic times.\n For example, vbat is chosen for report only\n
  * when vbat < 4.0v; when vbat is >4.0v, no vbat data will be reported. In this case, when vbat's periodic is 4, the vbat data will be reported\n
  * once for every 4 duties.
  */  
  pos_u16_t   periodic; /**< Periodic reporting cycle when data is muted by theshold check\n
  * 0: Turn OFF periodic cycle. If data is always muted, it will never be reported unless threshold is meet (or threshold is zero)\n
  * 1: Report on each duty\n
  * 2~65535: Report once on every 2~65535 duties.\n
  */

  /**
  * @brief ECFG#0x08A+0x30*SLOT Hardware IO port (pin)
  */    
  pos_u16_t io; /**< HW instance ID for multiple senssors. Refer to each sensor definition: @ref ma_sensor_type_t */

  /**
  * @brief ECFG#0x08C+0x30*SLOT Sensor reserved
  */      
  pos_u32_t rsvd32; /**< Leave it zero unless it's defined in each sensor: @ref ma_sensor_type_t */
  
  /**
  * @brief ECFG#0x090+0x30*SLOT Sensor arguments\n
  * Refer to @ref ma_sensor_type_t
  */
  union {
    pos_u8_t u8[16];  
    pos_i8_t i8[16];
    pos_u16_t u16[8];
    pos_i16_t i16[8];
    pos_u32_t u32[4];
    pos_i32_t i32[4];
  } arg;

  /**
  * @brief ECFG#0x0A0+0x30*SLOT Threshold definition\n
  * Refer to @ref ma_sensor_type_t
  */
  union {
    pos_u8_t u8[16];  
    pos_i8_t i8[16];
    pos_u16_t u16[8];
    pos_i16_t i16[8];
    pos_u32_t u32[4];
    pos_i32_t i32[4];
  } thld;

} ma_sensor_slot_t;

/**
* @brief EEPROM network configuratoin structure
*/
typedef struct {
  char       topic_pub[64]; /**< ECFG#0x290 MQTT Publish Topic */
  char       topic_sub[64]; /**< ECFG#0x2d0 MQTT Subscribe Topic */
  char       musr[16];      /**< ECFG#0x310 MQTT User */
  char       mpwd[16];      /**< ECFG#0x320 MQTT Password */
  char       mcid[32];      /**< ECFG#0x330 MQTT Client ID */

  pos_u8_t       dns[4];    /**< ECFG#0x350 DNS IP Address */
  pos_u8_t       qos_pub;   /**< ECFG#0x354 MQTT Publish QOS */
  pos_u8_t       qos_sub;   /**< ECFG#0x355 MQTT Subscribe QOS */
  pos_u8_t       rsvd[6];   /**< ECFG#0x356 Reserved */
  pos_u8_t       dns_retry; /**< ECFG#0x35c DNS Retry Times */
  pos_u8_t       dns_delay; /**< ECFG#0x35d DNS Retry Delay seconds */
  pos_u16_t     rsvr_port;  /**< ECFG#0x35e Remote Server Port */
  
  char       rsvr_url[32];  /**< ECFG#0x360 Remote Server URL/IP */

  char       band[32];      /**< ECFG#0x380 NB/4G_LTE FREQ Band */
  char       apn[32];       /**< ECFG#0x3a0 NB/4G_LTE APN */

  char       ssid[32];      /**< ECFG#0x3c0 WIFI SSID */
  char       wrsvd[16];     /**< ECFG#0x3d0 WIFI Reserved */
  char       wpwd[16];      /**< ECFG#0x3e0 WIFI Password */

} ma_network_cfg_t;

/**
* @brief EEPROM configuration structure
* @note This structure is stored in EEPROM by Little-Endina. Software could use this structure (global pointer) directly. 
*/
typedef struct {
  /**
  * @brief ECFG#0x000 Magic value, refer to @ref MA_EEPROM_MAGIC
  */
  pos_u32_t magic; /**< It will be treated as valid content only when this field is specified pattern */
  
  /**
  * @brief ECFG#0x004 Report type (reboot to take effect after changing)
  */  
  pos_u16_t  report_type; /**< Refer to @ref ma_report_type_t */
  
  /**
  * @brief ECFG#0x006 Global control
  */  
  pos_u16_t  ctrl; /**< Bitmap control fields. Refer to @ref MA_CFG_CTRL_DBG */  
  
  /**
  * @brief ECFG#0x008 EUI
  */    
  pos_u8_t eui[8];  /**< 8 bytes of EUI definition */
  
  /**
  * @brief ECFG#0x010 LORA controls structure (reboot to take effect after changing)
  */    
  ma_lora_cfg_t lora; /**< Refer to @ref ma_lora_cfg_t */

  /**
  * @brief ECFG#0x060 ACK failure times before module reset
  */    
  pos_u16_t module_reset_err; /**< External module reset control:\n
  0: Disable this module reset mechanism\n
  1~65535: ACK failure times, after continuous of ACK failure, external module will reset */

  /**
  * @brief ECFG#0x062 ACK failure times before system reset
  */    
  pos_u16_t sys_reset_err; /**< System reset control:\n
  0: Disable this system reset mechanism\n
  1~65535: ACK failure times, after continuous of ACK failure, system will reset*/

  /**
  * @brief ECFG#0x064 Reserved
  */        
  pos_u8_t mbus_addr;  /**< Modbus address */

  /**
  * @brief ECFG#0x065 Reserved
  */        
  pos_u8_t mbus_ctrl;  /**< Modbus ctrl bitmap  */

  /**
  * @brief ECFG#0x066 timectrl
  */        
  pos_u8_t time_ctrl[0x5];  /**< Time control:\n
  #0 = 0: Regular cycle periodical scheduling\n
  #0 = 1: Hour based time scheduling\n
      #1: BCD Minutes (0x00~0x59)\n
      #2: BCD Seconds (0x00~0x59)\n
      #3..#4: Reserved (keep zero)\n
  #0 = 2: Day based time scheduling\n
      #1: Reserved (keep zero)
      #2: BCD Hours (0x00~0x23)\n
      #3: BCD Minutes (0x00~0x59)\n
      #4: BCD Seconds (0x00~0x59)\n
  #0 = 3: Week based time scheduling\n
      #1: 0~6 for Sunday .. Saturnday
      #2: BCD Hours (0x00~0x23)\n
      #3: BCD Minutes (0x00~0x59)\n
      #4: BCD Seconds (0x00~0x59)\n
  #0 = 4: Month based time scheduling\n
      #1: BCD Days (1~31)\n
      #2: BCD Hours (0x00~0x23)\n
      #3: BCD Minutes (0x00~0x59)\n
      #4: BCD Seconds (0x00~0x59)\n
  #0 = Others: Regular cycle priodical scheduling\n
  
  */

  /**
  * @brief ECFG#0x06b Reserved
  */        
  pos_u8_t rsvd[0x10];  /**< Keep it for all zero */

  /**
  * @brief ECFG#0x07b Module calib (0~127-postive, 128~255-negative, unit 100 PPM)
  */      
  pos_u8_t module_calib;  /**< Keep it for zero to choose default. Or else use IO+0x80 to choose specified IO */

  /**
  * @brief ECFG#0x07c Module datarate (0-module default dararate, 0x80+XX-use datarate=XX)
  */      
  pos_u8_t module_dr;  /**< Keep it for zero to choose default. Or else use DR+0x80 to choose specified DR */

  /**
  * @brief ECFG#0x07d Module IO field
  */      
  pos_u8_t module_io;  /**< Keep it for zero to choose default. Or else use IO+0x80 to choose specified IO */

  /**
  * @brief ECFG#0x07e Console control field
  */      
  pos_u8_t console;  /**< Keep it for zero */

  /**
  * @brief ECFG#0x07f DS18B20 rememberred ROM number
  */      
  pos_u8_t ds18b20_num;  /**< A valud of none-zero will drive DS18B20 only for provided ROM ID. Or else, scan DS18B20 dynamicly */

  /**
  * @brief ECFG#0x080 Sensor slot control
  */        
  ma_sensor_slot_t slot[MA_SENSOR_SLOT_NUM]; /**< Refer to @ref ma_sensor_slot_t */

  /**
  * @brief ECFG#0x200 DS18B20 rememberred ROM ID
  * @note At most 16 DS18B20 sensors is supported in each slot
  */
  struct {
    pos_u8_t rom_id[8]; /**< ECFG#0x200+8*ID DS18B20 ROM ID */
  } ds18b20[16];

  /**
  * @brief ECFG#0x280 DS18B20 rememberred ROM reporting position
  * @note If DS18B20 rememberred ROM is diffifult to change, this field can also remap the reporting sequence. For example:\n
  *   rom_id[] = {1,0,3,2, 0,...0}\n
  * The final reporting DS18B20 sequence would be:\n
  *   ROM#1, ROM#0, ROM#3, ROM#2  
  */  
  pos_u8_t ds18b20_pos[16]; /**< Reporting ID of 0..15 (#0 is this first, #15 is the last in report buffer) */

 /**
 * @brief ECFG#0x290 MQTT/WIFI network configuration (reboot to take effect after changing)
 */
 ma_network_cfg_t net;   /**< Refer to @ref ma_network_cfg_t */
} ma_eeprom_cfg_t; /* 1024 bytes of EEPROM configuration strcuture */


/**
* @brief EEPROM OTA configuration structure
* @note This structure is stored in EEPROM OTA area by Little-Endina. Software could use this structure (global pointer) directly. 
*/
typedef struct {
  /**
  * @brief OTACFG#0x000 Magic value, refer to @ref MA_EEPROM_OTA_MAGIC
  */
  pos_u32_t magic; /**< It will be treated as valid content only when this field is specified pattern */
  
  /**
  * @brief OTACFG#0x004 OTA flag
  */  
  pos_u16_t  flag; /**< Refer to @ref MA_DTU_OTA_FLAG_USE_PLSS_SITE */
  
  /**
  * @brief OTACFG#0x005 Step control
  */  
  pos_u8_t   step; /**< Current operating OTA image ID */  

  /**
  * @brief OTACFG#0x006 Image number
  */  
  pos_u8_t   num; /**< Total OTA image number defined as below */    

  /**
  * @brief ECFG#0x064 Reserved
  */
  pos_u8_t rsvd[0xf8];  /**< Keep it for all zero */

  /**
  * @brief ECFG#0x100 OTA Image Version list
  */
  pos_u32_t img_ver32[64]; /**< Valid number is controled the the above field - num */
} ma_eeprom_ota_cfg_t; /* 512 bytes of EEPROM OTA configuration strcuture */

/**
* @brief EEPROM ONENET configuration structure
* @note This structure is stored in EEPROM ONENET area by Little-Endina. Software could use this structure (global pointer) directly. 
*/
typedef struct {
  /**
  * @brief ONECFG#0x000 Magic value, refer to @ref MA_EEPROM_ONENET_MAGIC
  */
  pos_u32_t magic; /**< It will be treated as valid content only when this field is specified pattern */
  
  /**
  * @brief ONECFG#0x004 OTA flag
  */  
  pos_u32_t  version; /**< Has to be zero */

  /**
  * @brief ONECFG#0x008 Reserved
  */
  pos_u8_t rsvd[0xd8];  /**< Keep it for all zero */
  
  /**
  * @brief ONECFG#0x0e0 Product ID
  */  
  pos_u8_t   pid[32]; /**< Product ID string from OneNet */  

  /**
  * @brief ONECFG#0x100 Token
  */  
  pos_u8_t   token[192]; /**< Token string from OneNet */  

  /**
  * @brief ONECFG#0x1e0 Device Key (CUIOT only)
  */
  pos_u8_t devk[32];  /**< Device key string from CUIOT */
  
} ma_eeprom_onenet_cfg_t; /* 512 bytes of EEPROM onenet configuration strcuture */


#endif
