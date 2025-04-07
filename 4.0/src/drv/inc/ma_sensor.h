/**
 * @file  ma_sensor.h  
 * @brief Sensor/external modules control definition
 * @author Runby F.
 * @date 2022-2-10
 * @copyright Polysense
 */

#ifndef __MA_SENSOR_H__
#define __MA_SENSOR_H__

/**
 * Sensor control flag for duty timeout
 */
#define MA_SENSOR_CTRL_TIMEOUT  1

/**
 * Sensor control flag for data report ready
 */
#define MA_SENSOR_CTRL_REPORT 2

/**
 * Sensor control flag for too long data that exceeding the remaining buffer
 */
#define MA_SENSOR_CTRL_TOO_LONG 4

/**
 * Sensor control flag for Normal working status
 */
#define MA_SENSOR_CTRL_STAT_NORMAL 8

/**
 * Sensor control flag for Error working status
 */
#define MA_SENSOR_CTRL_STAT_ERROR 0x10

/**
 * Sensor control flag for Abandoning all remaining collectings
 */
#define MA_SENSOR_CTRL_ABANDON 0x20

/**
 * Sensor control flag for driver init flag
 */
#define MA_SENSOR_CTRL_DRV_INIT 0x80

/**
 * Duty time when no sensor is activated
 */
#define MA_SENSOR_LINK_TIMEOUT_WHEN_EMPTY  10000

/**
 * @brief Sensor driver library
 */
typedef struct {
  /** 
  * Sensor driver load/unload control
  * @param[in]  load   Load control:\n
    0 - Unload driver\n
    1 - Load drvier
  * @return     0: Successful\n
               !=0: Failed, refer to @ref pos_status_t
  * @note If the sensor does NOT require any init operation, a NULL pointer can be provided here
  */ 
   pos_status_t (*init)(pos_u32_t load);  

  /** 
  * Sensor power management
  * @param[in]  on   Power control:\n
    0 - Power off\n
    1 - Power on
  * @return     0: Successful\n
               !=0: Failed, refer to @ref pos_status_t
  * @note If the sensor does not require any power control, a NULL pointer can be provided here
  */ 
  pos_status_t (*power_set)(pos_u32_t on);  

  /** 
  * Sensor collecting
  * @return     0: Successful\n
               !=0: Failed, refer to @ref pos_status_t
  */ 
  pos_status_t (*collect)(void); 

  /** 
  * Sensor callback for IRQ event (executed in IRQ interrupt context)
  * @return     0: Successful\n
                !=0: Failed, refer to @ref pos_status_t  
  * @note This function pointer will finally be mapped to physical address so as to take effect in IRQ context
  */ 
  pos_status_t (*irq_cb)(void *arg);   

  /** 
  * Sensor callback for regular pulling (executed in normal process context)
  * @return     0: Successful\n
                !=0: Failed, refer to @ref pos_status_t  
  * @note This function pointer will finally be mapped to physical address so as to take effect in process context
  */ 
  pos_status_t (*poll_cb)(void *arg);   

  /** 
  * Reserved
  */ 
  pos_u32_t rsvd[0];
} ma_sensor_drv_t;

/**
* @brief Sensor/external module driver export
*/
typedef struct {
  pos_u32_t version; /**< Sensor/module type/version. Use following macros to generate: \n
  @ref DRV_SENSOR_VERSION - Sensors  \n
  @ref DRV_NET_VERSION - External modules \n
  @ref DRV_NET_LORA_VERSION - LoRa modules */ 
  const char *name; /**< Sensor/module name (Do not use too long name, as it will give trouble during driver file management. <=8 is recommended.) */ 
  union {
    ma_sensor_drv_t   sensor; /**< Sensor driver */ 
    pos_net_t   net;    /**< External module driver */ 
  } u; /**< Union definitions, depending on version */
} ma_drv_export_t;

/**
 * @brief Sensor control structure
 */
typedef struct {
  pos_u32_t timeout;  /**< Next duty cycle in unit of ms */
  pos_u16_t periodic_age;  /**< Current periodic aging time */
  pos_u8_t ctrl; /**< Control fields:\n
  <pre>
  Bit0 - Already aged and able to collect in this cycle\n
  Bit1 - Already collected and have valid data for reporting\n  
  Bit2 - Data too long for reporting buffer\n
  Bit3 - Status/Normal indicator\n
  Bit4 - Status/Error indicator\n
  Bit5 - Abandon all remaining slots collecting\n
  </pre>
  */
  pos_u8_t rsvd; /**< Reserved */
  union {
    pos_u8_t  u8;
    pos_i8_t  i8;    
    pos_u16_t  u16;
    pos_i16_t  i16;  
    pos_u32_t  u32;
    pos_i32_t  i32;
  } data; /**< Recent data record */
  ma_sensor_slot_t *slot;  /**< Origianl senslor slot configuration of EEPROM */
  pos_u32_t drv_arg; /**< Sensor argument (used by sensor itself) */  
  void *drv_buf; /**< Sensor private buffer (used by sensor itself) */
  void *hdr; /**< Origianl sensor file header in virtual address */
  ma_drv_export_t drv; /**< Mapped sensor driver in physical address */
} ma_sensor_ctrl_t;

extern pos_u32_t g_poll_num; /**< Global pulling number */
extern pos_u32_t g_duty_sleep_ms; /**< Additional control of duty sleep */

#endif
