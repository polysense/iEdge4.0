/**
 * @file  drv_api.h  
 * @brief Driver development common API
 * @author Runby F.
 * @date 2022-3-9
 * @copyright Polysense
 */

#ifndef __DRV_API_H__
#define __DRV_API_H__

#include <stdint.h>

#include "main_types.h"
#include "main_board.h"
#include "ma_sensor.h"
#include "ma_ext.h"
#include "ma_ota.h"
#include "../../../../common/include/plss_mote.h"
/**
* @brief Sensor driver version's first word
*/
#define DRV_VERSION_SENSOR 0x0000

/**
* @brief LORA driver version's first word
*/
#define DRV_VERSION_NET_LORA    (MA_REPORT_LORA_US902 & 0xff00)

/**
* @brief Sensor driver version full (32b)
*/
#define DRV_SENSOR_VERSION(type, ver16) ((DRV_VERSION_SENSOR<<16) + ((type)<<16) + ver16)

/**
* @brief LORA driver version full (32b)
*/
#define DRV_NET_LORA_VERSION(ver16) ((DRV_VERSION_NET_LORA<<16) +  ver16)

/**
* @brief MQTT/NET driver version full (32b)
*/
#define DRV_NET_VERSION(report_type, ver16) (((report_type)<<16) +  ver16)

/**
* @brief Full version check for sensor driver detection
*/
#define DRV_VERSION_IS_SENSOR(ver32)    ((((ver32)>>16)&0xf000) == DRV_VERSION_SENSOR)

/**
* @brief Full version check for LORA driver detection
*/
#define DRV_VERSION_IS_LORA(ver32)    ((((ver32)>>16)&0xff00) == DRV_VERSION_NET_LORA)

/**
* @brief Full version to driver type (first word)
*/
#define DRV_VERSION_TO_TYPE(ver32)    (((ver32)>>16)&0xffff)

/**
* @brief Full version check with given type (first word)
*/
#define DRV_VERSION_IS_TYPE(ver32, type16)    (DRV_VERSION_TO_TYPE(ver32) == type16)

/**
* @brief Generate a PLSS TRANSLATOR for PLSS/JSON put
*/
#define DRV_MA_PLSS_TRANS(plss_pl_ind, plss_pl_len, plss_v_d, json_v_d, json_float_d) \
  ((plss_pl_ind)+((plss_pl_len)<<8) + ((plss_v_d)<<16) + ((json_v_d)<<20) + ((json_float_d)<<24))

/**
 * @brief Set next duty sleeping time (unit:ms) for time correction or other purpose\n
 * arg1 Provide a U32 data in unit of ms
 */
#define DRV_CALL_FUNC_ID_SET_DUTY_MS 0

/**
 * @brief Init data paylaod as empty so as to give us this round of report\n
 * arg Not required
 */
#define DRV_CALL_FUNC_ID_SET_DATA_CLEAR 1

/**
 * @brief Clear ACK TIMEOUT flag 
 * arg Not required
 */
#define DRV_CALL_FUNC_ID_SET_ACK_CLEAR 2

/**
 * @brief Get PLSS Data buffer pointer
 * arg Provide a pos_u8_t ** for returning the PLSS data buffer
 */
#define DRV_CALL_FUNC_ID_GET_PLSS_DATA_BUF 3

/**
 * @brief Get PLSS Data Length
 * arg Provide a pos_u32_t * for returning the PLSS data length
 */
#define DRV_CALL_FUNC_ID_GET_PLSS_DATA_LEN 4

/**
 * @brief Get Date Time structure
 * arg Provide a drv_data_time_t * for returning the date and time
 */
#define DRV_CALL_FUNC_ID_GET_DATE_TIME 5

/**
 * @brief Set CLI/DN raw payload callback 
 * arg Provide a drv_call_cb_param_t * for callback parameters
 */
#define DRV_CALL_FUNC_ID_SET_CB  6

/**
 * @brief Get CLI/DN raw payload callback 
 * arg Provide a drv_call_cb_param_t* for returning the current CLI cb parameters
 */
#define DRV_CALL_FUNC_ID_GET_CB  7

/**
 * @brief Reschedule next duty after given ms
 * arg Provide a UINT32 of ms for the next duty time
 */
#define DRV_CALL_FUNC_ID_SET_NEXT_DUTY 8

/**
 * @brief Set history extended data
 * arg Provide a UINT32 of bit[0..15]=value16, bit[16..30]=ext_ofs, bit[31]=0 for U8 and 1 for U16
 */
#define DRV_CALL_FUNC_ID_SET_HISTORY_EXT 9

/**
 * @brief Setup history extended data U16
 */
#define DRV_SET_HISTORY_EXT_U16(drv, blk, id, v) do {\
  (drv)->call(DRV_CALL_FUNC_ID_SET_HISTORY_EXT, (0x80000000+(((blk)*8+(id))<<16) + (v))); \
} while(0)

/**
 * @brief Drv extended history data ready check
 */
#define DRV_HISOTRY_EXT_IS_READY(drv) (((drv)->history->rsvd[1] & 2) != 0)

/**
 * @brief Drv extended history data set ready flag
 */
#define DRV_HISOTRY_EXT_SET_READY(drv) do { (drv)->history->rsvd[1] |= 2; } while(0)

/**
 * @brief Drv extended history data clear ready flag
 */
#define DRV_HISOTRY_EXT_CLR_READY(drv) do { (drv)->history->rsvd[1] &= ~2; } while(0)

/**
 * @brief Driver callback source indication enum type
 */
typedef enum {
  DRV_CB_SRC_CLI = 0, /** CLI input */
  DRV_CB_SRC_DN_MSG,  /** Downstream control message */
} drv_cb_src_t;

/**
 * @brief Driver console/cli/dn msg callback prototype
 * @param[in] src  Callback source
 * @param[in] buf  Console raw input buffer
 * @param[in] len  Console raw input buffer length
 * @param[in] cookie  Cookie regiserred during callback set 
 * @return 0: The content in buffer has been processed\n
         Others: Continue regular main/cli processing for the buffer content
 */
typedef pos_status_t (*drv_call_cb_t)(drv_cb_src_t src, pos_u8_t *buf, pos_size_t len, void *cookie);

/**
* @brief Call paremters for CLI_CB_SET Structure
*/
typedef struct {
  /**
   * Call back ptr
   */
  drv_call_cb_t    cb;

  /**
   * Cookie ptr
   */  
  void            *cookie;
} drv_call_cb_param_t;

/**
* @brief EEPROM Driving Structure
*/
typedef struct {
  /** 
  * Save configuration to EEPROM area
   * @param[in] ecfg  Configuration to be saved
   */ 
  void (*save)(ma_eeprom_cfg_t *ecfg);

  /** 
   * Update EEPROM by given offset and data/length
   * @param[in] ofs  EEPROM offset
   * @param[in] buf  Data pointer
   * @param[in] len  Data length
   * @note buf can be NULL, in this case the whole EEPROM area will be programmed to all zero.
   */ 
  pos_status_t (*update)(pos_u32_t ofs, void * buf, pos_size_t len);
} drv_eeprom_api_t;

/**
* @brief Threshold Processing Structure
*/
typedef struct  {
  /**
   * U16 threshold process / periodic updating
   */
  void (*u16_process)(ma_sensor_ctrl_t *s, pos_u16_t v);
  
  /**
   * I16 threshold process / periodic updating
   */
  void (*i16_process)(ma_sensor_ctrl_t *s, pos_i16_t v);

  /**
   * Data zero filter by threshold (thw0/thw1)
   * @param[in] value Original value to be filterred
   * @return any: Filtered final result
   */
  pos_u32_t (*zero_filter)(ma_sensor_ctrl_t *s, pos_u32_t v);

  /**
   * Periodic age control
   */
  void (*periodic_age)(ma_sensor_ctrl_t *s);
  
} drv_thld_api_t;

/**
* @brief Typical Time Data Structure
*/
typedef struct  {
  pos_i16_t year; /**< year */

  pos_i8_t  month; /**< month */

  pos_i8_t  day; /**< day */

  pos_i8_t  hour; /**< hour 24 */

  pos_i8_t  minute; /**< minute */

  pos_i8_t  second; /**< second */
  
  pos_i8_t  tz_quarter; /**< timezone (unit of 15 minutes) */

  pos_u32_t tick; /**< sys tick */

} drv_data_time_t;

/**
* @brief Numeric Data Processing Structure
*/
typedef struct  {
  /**
   * Record sensor data by given pattern/data
   * @param[in] pattern Record format, refer to @ref g_ma_data_pattern_u
   * @param[in] field Data field name
   * @param[in] v1 Data1
   * @param[in] v2 Data2
   * @param[in] v3 Data3
   * @param[in] v4 Data4
   * @return     0: Successful\n
                !=0: Failed, refer to @ref pos_status_t
   */
  pos_status_t (*put)(const char *pattern, const char *field, pos_u32_t v1, pos_u32_t v2, pos_u32_t v3, pos_u32_t v4);

  /**
   * Record signed integer into floating format
   * @param[in] v  Origianl integer value
   * @param[in] divide Dividing by
   * @return     0: Successful\n
                !=0: Failed, refer to @ref pos_status_t
   */
  pos_status_t (*put_float)(pos_i32_t v, pos_u32_t divide);

  /**
   * Record unsigned integer into floating format
   * @param[in] v  Origianl integer value
   * @param[in] divide Dividing by
   * @return     0: Successful\n
                !=0: Failed, refer to @ref pos_status_t
   */
  pos_status_t (*put_ufloat)(pos_u32_t v, pos_u32_t divide);

  /**
   * Stop recording data array (put finish charactoer in json array format)
   * @return     0: Successful\n
                !=0: Failed, refer to @ref pos_status_t
   */
  pos_status_t (*put_array_stop)(void);

  /**
   * Record sensor data with original content
   * @param[in] data Original data content 
   * @param[in] len Content length
   * @return     0: Successful\n
                !=0: Failed, refer to @ref pos_status_t
   */
  pos_status_t (*put_raw)(const void *data, pos_size_t len);


  /**
   * Init data buffer for each duty cycle
   */
  void (*init)(void);  

  /**
   * Record U8 data into PLSS/UDP data buffer
   * @param[in] pl_type  Polysense WXS8800 payload indicator
   * @param[in] v Data value
   * @return     0: Successful\n
                !=0: Failed, refer to @ref pos_status_t
   */
  pos_status_t (*plss_put_u8)(pos_u8_t pl_type, pos_u8_t v);

  /**
   * Record U16 data into PLSS/UDP data buffer
   * @param[in] pl_type  Polysense WXS8800 payload indicator
   * @param[in] v Data value
   * @return     0: Successful\n
                !=0: Failed, refer to @ref pos_status_t
   */
  pos_status_t (*plss_put_u16)(pos_u8_t pl_type, pos_u16_t v);

  /**
   * Record U32 data into PLSS/UDP data buffer
   * @param[in] pl_type  Polysense WXS8800 payload indicator
   * @param[in] v Data value
   * @return     0: Successful\n
                !=0: Failed, refer to @ref pos_status_t
   */
  pos_status_t (*plss_put_u32)(pos_u8_t pl_type, pos_u32_t v); 

  /**
   * Record any type of data into PLSS/UDP data buffer
   * @param[in] data_len Data value length
   * @param[in] p_data Pointer for number of bytes data
   * @return     0: Successful\n
                !=0: Failed, refer to @ref pos_status_t
   */
  pos_status_t (*plss_put_raw)(pos_u32_t data_len, void *p_data);  

  /**
   * Adjust time
   * @param[in] t Data Time structure to be adjusted
   * @param[in] seconds Adjusted seconds
   */
  void (*plss_time_adj)(drv_data_time_t *t, pos_i32_t seconds);  


  /**
   * Record any type of data for both PLSS/UDP and JSON data buffer
   * @param[in] v Data to be reported
   * @param[in] plss_trans Translator definitions
   * @param[in] json_str JSON field name
   * @return     0: Successful\n
                !=0: Failed, refer to @ref pos_status_t
   */
  pos_status_t (*plss_json_put)(pos_u32_t v, pos_u32_t plss_trans, const char *json_str);

  /**
   * MODBUS register read
   * @param[in] io IO operating pointer
   * @param[out] buf Return buffer 
   * @param[in] addr MODBUS device address
   * @param[in] func MODBUS function code (ie: 3 for REG READ)
   * @param[in] reg MODBUS register address
   * @param[in] reg_cnt MODBUS register counter 
   * @return 0: Nothing return of CRC16 is wrong\n
           !=0: Valid buffer returned (CRC16 is also included and correct)
   */
  pos_size_t (*modbus_read)(pos_io_handle_t *io, void *buf, pos_u8_t addr, pos_u8_t func, pos_u16_t reg, pos_u16_t reg_cnt);

  /**
   * MODBUS register read and record/put to data buffer
   * @param[in] io IO operating pointer
   * @param[in] addr MODBUS device address
   * @param[in] func MODBUS function code (ie: 3 for REG READ)
   * @param[in] reg MODBUS register address
   * @param[in] reg_cnt MODBUS register counter 
   * @param[in] plss_trans Translator definitions
   * @param[in] json_str JSON field name
   * @return 0: Nothing return of CRC16 is wrong\n
           !=0: Valid buffer returned (CRC16 is also included and correct)
   */
  pos_status_t (*modbus_put)(pos_io_handle_t *io, pos_u8_t addr, pos_u8_t func, pos_u16_t reg, pos_u16_t reg_cnt, pos_u32_t plss_trans, const char *json_str);


  /**
   * Record any type of data for both PLSS/UDP and JSON data buffer
   * @param[in] v Data to be reported
   * @param[in] plss_trans Translator definitions
   * @param[in] json_str JSON field name
   * @return     0: Successful\n
                !=0: Failed, refer to @ref pos_status_t
   */
  pos_status_t (*plss_json_puti)(pos_i32_t v, pos_u32_t plss_trans, const char *json_str);    
} drv_data_api_t;

/**
* @brief LOG Processing Structure
*/
typedef struct  {
  /**
   * LOG print with argument
   */
  void (*data)(const char *str, pos_u32_t arg);
  
  /**
   * LOG print with data content 
   */
  void (*buf)(const char *str, void *data, pos_size_t len);
} drv_log_api_t;

/**
* @brief Event Processing Structure
*/
typedef struct  {
  /** 
   * Event registration
   * @param[in] cb Event callback function
   * @param[in] arg Argument for callback
   * @return     0: Successful\n
                !=0: Failed, refer to @ref pos_status_t
   */ 
  pos_status_t (*event_register)(pos_func_t cb, void *arg);
  
  /** 
   * Event unregistration
   * @param[in] cb Event callback function
   * @param[in] arg Argument for callback
   * @return     0: Successful\n
                !=0: Failed, refer to @ref pos_status_t
   */ 
  pos_status_t (*event_unregister)(pos_func_t cb, void *arg);
} drv_process_api_t;

/**
* @brief Typical History Extended Data Structure
*/
typedef struct  {
  pos_u16_t u_size; /**< Number of byes size for u */
  pos_u16_t rsvd; /**< rsvd */
  union {
    pos_u8_t u8[0]; /**< uint8 data array */
    pos_u8_t i8[0]; /**< int8 data array */
    pos_u16_t u16[0]; /**< uint16 data array */
    pos_u16_t i16[0]; /**< int16 data array */
    pos_u32_t u32[0]; /**< uint32 data array */
    pos_u32_t i32[0]; /**< int32 data array */
  } u; /**< union for different types */
} drv_data_history_ext_t;

/**
* @brief Typical History Data Structure
*/
typedef struct  {
  pos_u32_t mv_num; /**< Number of voltage */

  pos_u16_t mv[8]; /**< Voltage in unit mv */
  /* Typically:
   * #0 - VBAT
   * #1 - BAT Percentage
   * #2 - cntr12.0
   * #3 - cntr12.1
   * #4 - Distance
   * #6(hi)/#5(lo) - HPA
   * #7 - CO2 PPM
  */
  pos_u32_t temp_num; /**< Number of temperature */

  pos_i16_t temp[16]; /**< Temperature (unit: 0.1 celsius) */  

  pos_u8_t  signal; /**< RF Signal: 0% ~ 100% */

  pos_u8_t  rsvd[3]; /**< rsvd[0] for MA_SENSOR_ULTRA_MBUS periodic, rsvd[1] for co2 refresh flag, rsvd[2] Reserved */

  drv_data_time_t time; /**< Report time */

  drv_data_history_ext_t *ext; /**< Exteded history data, only available when rsvd[1].bit1 is 1 */
} drv_data_history_t;

/**
* @brief Driver Common API Structure
*/
typedef struct {
  pos_u32_t ma_version;           /**< MA version */
  
  const pos_lib_t *os;            /**< POS library */
  
  const ma_eeprom_cfg_t *cfg;     /**< Global EEPROM configration */

  const drv_eeprom_api_t *eeprom; /**< EEPROM processing */

  const drv_thld_api_t  *thld;    /**< Threshold processing */

  const drv_data_api_t *data;     /**< Data recording processing */ 
  
  const drv_log_api_t *log;       /**< LOG processing */ 

  ma_ext_ctrl_t *ext;             /**< External module library */

  ma_sensor_ctrl_t *s;            /**< Sensor control pointer */

  ma_board_t *board;              /**< Board/PIN managment library */ 
  
  pos_u32_t run_loop;             /**< Main loop counter */
  
  pos_u32_t duty_loop;            /**< Duty counter */  
  
  ma_fsm_t fsm;                   /**< Main FSM */
  
  pos_u32_t ctrl;                 /**< Ctrl (stop) flags */
  
  ma_ota_ctrl_t ota;              /**< OTA global control */

  const drv_process_api_t *proc;  /**< Event processing */
  
  drv_data_history_t *history;    /**< History data */

  /** 
   * MA generic API calling (not support now)
   * @param[in] drv_func_id Fucntion code\n
   0 - Set special DUTY sleep time (ms)\n
   others - Not supported
   * @param[in] arg         Calling parameter
   * @return     0: Successful\n
                !=0: Failed, refer to @ref pos_status_t
   */ 
  pos_status_t (*call)(pos_u32_t drv_func_id, pos_u32_t arg);
} drv_api_t;

/**
* @brief Driver export (used for driver development to declare its functions)
*/
extern ma_drv_export_t g_export;

/**
* @brief Global driver API pointer
* @note Only MA compiling will define this variable. Sensor and module drivers will use macro g_drv=((drv_api_t*)(((pos_u32_t*)0x0001002c)[0]))
*/
#ifdef MA_APP
extern drv_api_t *g_drv;
#else
#define g_drv ((drv_api_t*)(((pos_u32_t*)0x0001002c)[0]))
#endif

#endif
