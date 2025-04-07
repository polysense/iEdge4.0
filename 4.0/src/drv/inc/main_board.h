/**
 * @file  main_board.h  
 * @brief MA PCB and pin managment definitions
 * @author Runby F.
 * @date 2022-2-18
 * @copyright Polysense
 */

#ifndef __MAIN_BOARD_H__
#define __MAIN_BOARD_H__

#if !defined(BOARD_B40_LCD) && !defined(BOARD_B40)
/**
* @brief 4.0 main board definition
*/
#define BOARD_B40
#endif

/**
* @brief ADC channel valid check
*/
#define MB_ADC_CHANNEL_IS_VALID(adc)      ((adc) <= 25)

/**
* @brief Battery voltage pin name
*/
#define MB_VBAT_PIN         POS_PB13

/**
* @brief Battery voltage sampling ADC channel
*/
#define MB_VBAT_ADC_CHANNEL 20

/**
* @brief External module UART baudrate
*/
#define MB_EXT_UART_BAUD    9600

/**
* @brief External module UART control (turn ON IRQ)
*/
#define MB_EXT_UART_CTRL    1

/*
 * MCU PIN definitions
 */
#define MB_PIN_LED_R(os)    os->pin->led[0]       /**< LED RED */

#define MB_PIN_LED_G(os)    os->pin->led[1]       /**< LED GREEN */

#define MB_PIN_LED_B(os)    os->pin->led[2]       /**< LED BLUE */

#define MB_PIN_LED_Y(os)    os->pin->led[3]       /**< LED YELLOW */

#define MB_PIN_LED_SET(os,led,on)  os->gpio->polarity_set(led,on) /**< LED Control */

#define MB_PIN_LED_SET_R(os,on)  os->gpio->polarity_set(MB_PIN_LED_R(os),on) /**< LED RED Control */

#define MB_PIN_LED_SET_G(os,on)  os->gpio->polarity_set(MB_PIN_LED_G(os),on) /**< LED GREEN Control */

#define MB_PIN_LED_SET_B(os,on)  os->gpio->polarity_set(MB_PIN_LED_B(os),on) /**< LED BLUE Control */

#define MB_PIN_LED_SET_Y(os,on)  os->gpio->polarity_set(MB_PIN_LED_Y(os),on) /**< LED YELLOW Control */

/**
* @brief External analog input PIN names corresponding ADC channel #0..#25
*/
#define MB_EXT_AIN_ADC_POOL POS_PA0, POS_PA1, POS_PA2, POS_PA3, \
                            POS_PA4, POS_PA5, POS_PA6, POS_PA7, \
                            POS_PB0, POS_PB1, POS_PC0, POS_PC1, \
                            POS_PC2, POS_PC3, POS_PC4, POS_PC5, \
                            POS_PB2, POS_PB10,POS_PB11,POS_PB12,\
                            POS_PB13,POS_PB14,POS_PB15,POS_PE15,\
                            POS_PE14,POS_PE13


/**
* @brief Single wire pin names
*/
#ifdef BOARD_B40_LCD
#define MB_EXT_WIRE1_POOL   POS_PB5
#endif
#ifdef BOARD_B40
#define MB_EXT_WIRE1_POOL   MB_PIN_WIRE1
#endif

/**
* @brief LED on GPIO state
*/
#define MB_LED_ON          0

/**
* @brief LED off GPIO state
*/
#define MB_LED_OFF         1

/**
* @brief GPIO state for sensor power on
*/
#define MB_SENSOR_PWR_ON   0

/**
* @brief GPIO state for sensor power off
*/
#define MB_SENSOR_PWR_OFF   1

/**
* @brief GPIO state for sensor power on (used when reversed polarity)
*/
#define MB_SENSOR_PWR_ON_R  1

/**
* @brief GPIO state for sensor power off (used when reversed polarity)
*/
#define MB_SENSOR_PWR_OFF_R 0

/**
* @brief FLASH/EEPROM sector size
*/
#define MB_FLASH_SECTOR_SIZE  512

/**
* @brief FLASH MA firmware address
*/
#define MB_FLASH_MAIN_ADDR  0x10000

/**
* @brief FLASH POS firmware address
*/
#define MB_FLASH_POS_ADDR  0

/**
* @brief EEPROM address
*/
#ifdef BOARD_B40_LCD
#define MA_EEPROM_ADDR    0x0001f000 
#endif
#ifdef BOARD_B40
#define MA_EEPROM_ADDR    0x0002f000 
#endif

/**
* Backup EEPROM area
*/
#define MA_EEPROM_BKUP_ADDR (MA_EEPROM_ADDR+0x400)

/**
* OTA CFG area
*/
#define MA_EEPROM_OTA_ADDR (MA_EEPROM_ADDR+0x800) 

/**
* ONENET CFG area
*/
#define MA_EEPROM_ONENET_ADDR (MA_EEPROM_ADDR+0xa00) 

/**
* Backup PIN CFG area
*/
#define MA_EEPROM_PIN_ADDR (MA_EEPROM_ADDR+0xc00) 

/**
 * @brief Main board pin types
 */
typedef enum {
  MB_PT_AIN = 0,        /**< Analong input */
  MB_PT_AIN_ADC = 1,    /**< ADC channel */ 
  MB_PT_DIO = 2,        /**< Digital IO */
  MB_PT_WIRE1 = 3,      /**< Single wire (DS18B20) */
  MB_PT_CTRL = 4,       /**< Special control pins */  
} mb_pin_type_t;

/**
 * @brief Main board power control types
 */
typedef enum {
  MB_PWR_ID_DEFAULT = 0,      /**< Driver default control */
  MB_PWR_ID_VCCN_PWR3 = 1,    /**< Reversed polarity control for 3.3V */
  MB_PWR_ID_VCCN_PT100 = 2,   /**< Reversed polarity control for PT100 */    
  MB_PWR_ID_VCCN_WIRE1 = 3,   /**< Reversed polarity control for single wire power */      
  MB_PWR_ID_VCCN_MODULE = 4,  /**< Reversed polarity control for external module */  
  MB_PWR_ID_VCCN_AIN = 5,     /**< Reversed polarity control for ADC sampling */    
  MB_PWR_ID_VCC_PWRH = 6,     /**< PWRH+PWRH_EN power control */
  MB_PWR_ID_VCC_OD3 = 7,     /**< 3.3V OD power control */
  MB_PWR_ID_VCC_OD5 = 8,     /**< 5.0V OD power control */
  MB_PWR_ID_VCC_LED = 9,     /**< LED/LCD displaying power control */ 
  MB_PWR_ID_VCC_VBAT = 10,     /**< VBAT power control */   
  MB_PWR_ID_VCC_CTRL1 = 11, /**< User defined sensor/module power control */
  MB_PWR_ID_VCC_CTRL2 = 12, /**< User defined sensor/module power control */
  MB_PWR_ID_VCC_CTRL3 = 13, /**< User defined sensor/module power control */  
  MB_PWR_ID_VCC_CTRL4 = 14, /**< User defined sensor/module power control */  
  MB_PWR_ID_VCC_CTRL5 = 15, /**< User defined sensor/module power control */    
  MB_PWR_ID_VCCN_PWRH_EN = 16, /**< PWRH high voltage enable control */
  MB_PWR_ID_PWRH_ONLY = 61, /**< PWRH only enable control */  
  MB_PWR_ID_PWRH_ONLY_PWR3 = 62, /**< PWRH only and 3.3V enable control */    
  MB_PWR_ID_PWRH_PWR3 = 63, /**< PWRH + PWRH_EN and 3.3V enable control */
} mb_pwr_id_t;

/**
 * @brief Main board analog input types
 */
typedef enum {
  MB_AIN_ID_DEFAULT = 0,      /**< Sensor default */
  MB_AIN_ID_1 = 1,            /**< AIN1 */
  MB_AIN_ID_2 = 2,            /**< AIN2/VBAT4 */
  MB_AIN_ID_3 = 3,            /**< AIN3/VBAT3 */
  MB_AIN_ID_4 = 4,            /**< AIN4/VBAT2 */  
  MB_AIN_ID_VBAT = 5,         /**< Battery voltage */
  MB_AIN_ID_PT100 = 6,        /**< PT100 */
} mb_ain_id_t;

/**
 * @brief Main board digital IO
 */
typedef enum {
  MB_DIO_ID_DEFAULT = 0,      /**< Sensor default */
  MB_DIO_ID_1 = 1,            /**< DIO/AIN1 */
  MB_DIO_ID_2 = 2,            /**< DIO/AIN2 */
  MB_DIO_ID_3 = 3,            /**< DIO/AIN3 */
  MB_DIO_ID_4 = 4,            /**< DIO/AIN4 */  
  MB_DIO_ID_BTN = 5,          /**< User key */
} mb_dio_id_t;

/**
 * @brief Main board special controls
 */
typedef enum {
  MB_CTRL_ID_DEFAULT = 0,     /**< Sensor default */
  MB_CTRL_ID_RFID_RST = 1,    /**< RFID_RST */
  MB_CTRL_ID_LED_RST = 2,     /**< LED_RST for LED/LCD displaying */  
  MB_CTRL_ID_LED_DC = 3,      /**< LED_DC for LED/LCD displaying */    
  MB_CTRL_ID_UDF1 = 4,      /**< User defined control pin */    
  MB_CTRL_ID_UDF2 = 5,      /**< User defined control pin */    
  MB_CTRL_ID_UDF3 = 6,      /**< User defined control pin */    
  MB_CTRL_ID_UDF4 = 7,      /**< User defined control pin */      

  MB_CTRL_ID_MODULE_WAKEUP = 8,      /**< Module wakeup pin */
  MB_CTRL_ID_MODULE_PWRKEY = 9,      /**< Module pwrkey pin */
  
} mb_ctrl_id_t;

/**
 * @brief Main board pin librarty
 */
typedef struct {
  /**
   * @brief Pin mapping
   * @param[in] type Pin types. Refer to @ref mb_pin_type_t
   * @param[in] id IO ID(0 Based)
   * @return GPIO pin name (refer to @ref pos_gpio_pin_t ), or ADC channel ID (when type = @ref MB_PT_AIN_ADC)
   * @note If id/type is invalid, always return 0xffffffff
   */
  pos_u32_t (*pin_map)(mb_pin_type_t type, pos_u32_t id);

  /**
   * @brief Power control
   * @param[in] pwr_id Power control type
   * @param[in] on Power on/off (0:off, others: on)
   */
  void (*pwr_set)(mb_pwr_id_t pwr_id, pos_u32_t on);

  /**
   * @brief Buzzer control
   * @param[in] Frequency (0: Buzzer power off, others: Beep at given frequency)
   */
  void (*beep_set)(pos_u32_t hz);

  /**
   * @brief Return POS console IO ID
   */
  pos_io_t (*console_io_get)(void);

} ma_board_t;

#endif
