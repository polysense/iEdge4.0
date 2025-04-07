/**
 * @file  pos_gpio.h  
 * @brief GPIO Definitions
 * @author Runby F.
 * @date 2022-2-10
 * @copyright Polysense
 */

#ifndef __POS_GPIO_H__
#define __POS_GPIO_H__

#include "pos_types.h"

#define POS_GPIO_MODE_DIGITAL        0x0000 ///< Digital mode

#define POS_GPIO_MODE_ANALOG         0x8000 ///< Analog mode

#define POS_GPIO_MODE_OUTPUT          0x0000 ///< Output

#define POS_GPIO_MODE_INPUT             0x4000 ///< Input

#define POS_GPIO_MODE_DRV_HIGH      0x0000 ///< High IO drive current
#define POS_GPIO_MODE_DRV_LOW       0x2000 ///< Low IO drive current

#define POS_GPIO_MODE_PU_DISABLE   0x0000 ///< Pullup disable

#define POS_GPIO_MODE_PU_ENABLE    0x1000 ///< Pullup enable

#define POS_GPIO_MODE_PD_DISABLE   0x0000 ///< Pulldown disable

#define POS_GPIO_MODE_PD_ENABLE    0x0800 ///< Pulldown enable

#define POS_GPIO_MODE_OD_DISABLE  0x0000 ///< Open-drain disable

#define POS_GPIO_MODE_OD_ENABLE    0x0400 ///< Open-drain enable

#define POS_GPIO_MODE_OUT_HIGH     0x0200 ///< Current OUTPUT high state (only for mode GET)

#define POS_GPIO_MODE_IRQ_HIGH     0x10000 ///< Level high IRQ trigger

#define POS_GPIO_MODE_IRQ_LOW      0x20000 ///< Level low IRQ trigger

#define POS_GPIO_MODE_IRQ_RISING   0x40000 ///< Rising trigger IRQ

#define POS_GPIO_MODE_IRQ_FALLING  0x80000 ///< Falling trigger IRQ

#define POS_GPIO_MODE_IRQ_DISABLE  0x00000 ///< No IRQ trigger

#define POS_GPIO_MODE_AF0                0x0000 ///< Standard GPIO mode (alternative mode 0)

#define POS_GPIO_MODE_AF1               0x0001 ///< Alternative mode 1

#define POS_GPIO_MODE_AF2               0x0002 ///< Alternative mode 2

#define POS_GPIO_MODE_AF3               0x0003 ///< Alternative mode 3

#define POS_GPIO_MODE_AF4               0x0004 ///< Alternative mode 4

#define POS_GPIO_MODE_AF5               0x0005 ///< Alternative mode 5

#define POS_GPIO_MODE_AF6               0x0006 ///< Alternative mode 6

#define POS_GPIO_MODE_AF7               0x0007 ///< Alternative mode 7

#define POS_GPIO_MODE_ANALOG_INPUT  (POS_GPIO_MODE_ANALOG+POS_GPIO_MODE_INPUT) ///< Analog input 

#define POS_GPIO_MODE_ANALOG_INPUT_PU  (POS_GPIO_MODE_ANALOG+POS_GPIO_MODE_INPUT+POS_GPIO_MODE_PU_ENABLE) ///< Analog input with pullup enable

#define POS_GPIO_MODE_ANALOG_INPUT_PD  (POS_GPIO_MODE_ANALOG+POS_GPIO_MODE_INPUT+POS_GPIO_MODE_PD_ENABLE) ///< Analog input with pulldown enable

#define POS_GPIO_MODE_OUTPUT_OD  (POS_GPIO_MODE_OUTPUT+POS_GPIO_MODE_OD_ENABLE) ///< Output open-drain

#define POS_GPIO_MODE_INPUT_PU  (POS_GPIO_MODE_INPUT+POS_GPIO_MODE_PU_ENABLE) ///< Input with pullup enable

#define POS_GPIO_MODE_INPUT_PD  (POS_GPIO_MODE_INPUT+POS_GPIO_MODE_PD_ENABLE) ///< Input with pulldown enable

#define POS_GPIO_MODE_INPUT_AF0  (POS_GPIO_MODE_INPUT+POS_GPIO_MODE_AF0) ///< Standard input (alternative mode 0)

#define POS_GPIO_MODE_INPUT_AF1  (POS_GPIO_MODE_INPUT+POS_GPIO_MODE_AF1) ///< Input/alternative mode 1

#define POS_GPIO_MODE_INPUT_AF2  (POS_GPIO_MODE_INPUT+POS_GPIO_MODE_AF2) ///< Input/alternative mode 2

#define POS_GPIO_MODE_INPUT_AF3  (POS_GPIO_MODE_INPUT+POS_GPIO_MODE_AF3) ///< Input/alternative mode 3

#define POS_GPIO_MODE_INPUT_AF4  (POS_GPIO_MODE_INPUT+POS_GPIO_MODE_AF4) ///< Input/alternative mode 4

#define POS_GPIO_MODE_INPUT_AF5  (POS_GPIO_MODE_INPUT+POS_GPIO_MODE_AF5) ///< Input/alternative mode 5

#define POS_GPIO_MODE_INPUT_AF6  (POS_GPIO_MODE_INPUT+POS_GPIO_MODE_AF6) ///< Input/alternative mode 6

#define POS_GPIO_MODE_INPUT_AF7  (POS_GPIO_MODE_INPUT+POS_GPIO_MODE_AF7) ///< Input/alternative mode 7

#define POS_GPIO_MODE_OUTPUT_AF1  (POS_GPIO_MODE_OUTPUT+POS_GPIO_MODE_AF1) ///< Output/alternative mode 1

#define POS_GPIO_MODE_OUTPUT_AF2  (POS_GPIO_MODE_OUTPUT+POS_GPIO_MODE_AF2) ///< Output/alternative mode 2

#define POS_GPIO_MODE_OUTPUT_AF3  (POS_GPIO_MODE_OUTPUT+POS_GPIO_MODE_AF3) ///< Output/alternative mode 3

#define POS_GPIO_MODE_OUTPUT_AF4  (POS_GPIO_MODE_OUTPUT+POS_GPIO_MODE_AF4) ///< Output/alternative mode 4

#define POS_GPIO_MODE_OUTPUT_AF5  (POS_GPIO_MODE_OUTPUT+POS_GPIO_MODE_AF5) ///< Output/alternative mode 5

#define POS_GPIO_MODE_OUTPUT_AF6  (POS_GPIO_MODE_OUTPUT+POS_GPIO_MODE_AF6) ///< Output/alternative mode 6

#define POS_GPIO_MODE_OUTPUT_AF7  (POS_GPIO_MODE_OUTPUT+POS_GPIO_MODE_AF7) ///< Output/alternative mode 7

/**
 * @brief Rising trigger IRQ mode
 * @note IRQ mode has to be set through API irq_set(). Do NOT set IRQ in mode_set().
 */
#define POS_GPIO_MODE_INPUT_PD_IRQ_RISING  (POS_GPIO_MODE_INPUT_PD+POS_GPIO_MODE_IRQ_RISING) 

/**
 * @brief Falling trigger IRQ mode
 * @note IRQ mode has to be set through API irq_set(). Do NOT set IRQ in mode_set().
 */
#define POS_GPIO_MODE_INPUT_PU_IRQ_FALLING  (POS_GPIO_MODE_INPUT_PU+POS_GPIO_MODE_IRQ_FALLING)

/**
 * @brief Rising or Falling trigger IRQ mode
 * @note IRQ mode has to be set through API irq_set(). Do NOT set IRQ in mode_set().
 */
#define POS_GPIO_MODE_INPUT_IRQ_RISING_FALLING  (POS_GPIO_MODE_INPUT+POS_GPIO_MODE_IRQ_RISING+POS_GPIO_MODE_IRQ_FALLING)

/**
* @brief NULL pin or unable to control pins
*/
#define POS_NULL_PIN         POS_PH31

/**
 * @brief GPIO pin name
 */
typedef enum {
   POS_PA0 =0  , POS_PA1 =1  , POS_PA2 =2  , POS_PA3 =3  , POS_PA4 =4  , POS_PA5 =5  , POS_PA6 =6  , POS_PA7 =7  ,
   POS_PA8 =8  , POS_PA9 =9  , POS_PA10=10 , POS_PA11=11 , POS_PA12=12 , POS_PA13=13 , POS_PA14=14 , POS_PA15=15 ,
   POS_PA16=16 , POS_PA17=17 , POS_PA18=18 , POS_PA19=19 , POS_PA20=20 , POS_PA21=21 , POS_PA22=22 , POS_PA23=23 ,
   POS_PA24=24 , POS_PA25=25 , POS_PA26=26 , POS_PA27=27 , POS_PA28=28 , POS_PA29=29 , POS_PA30=30 , POS_PA31=31 ,
   POS_PB0 =32 , POS_PB1 =33 , POS_PB2 =34 , POS_PB3 =35 , POS_PB4 =36 , POS_PB5 =37 , POS_PB6 =38 , POS_PB7 =39 ,
   POS_PB8 =40 , POS_PB9 =41 , POS_PB10=42 , POS_PB11=43 , POS_PB12=44 , POS_PB13=45 , POS_PB14=46 , POS_PB15=47 ,
   POS_PB16=48 , POS_PB17=49 , POS_PB18=50 , POS_PB19=51 , POS_PB20=52 , POS_PB21=53 , POS_PB22=54 , POS_PB23=55 ,
   POS_PB24=56 , POS_PB25=57 , POS_PB26=58 , POS_PB27=59 , POS_PB28=60 , POS_PB29=61 , POS_PB30=62 , POS_PB31=63 ,
   POS_PC0 =64 , POS_PC1 =65 , POS_PC2 =66 , POS_PC3 =67 , POS_PC4 =68 , POS_PC5 =69 , POS_PC6 =70 , POS_PC7 =71 ,
   POS_PC8 =72 , POS_PC9 =73 , POS_PC10=74 , POS_PC11=75 , POS_PC12=76 , POS_PC13=77 , POS_PC14=78 , POS_PC15=79 ,
   POS_PC16=80 , POS_PC17=81 , POS_PC18=82 , POS_PC19=83 , POS_PC20=84 , POS_PC21=85 , POS_PC22=86 , POS_PC23=87 ,
   POS_PC24=88 , POS_PC25=89 , POS_PC26=90 , POS_PC27=91 , POS_PC28=92 , POS_PC29=93 , POS_PC30=94 , POS_PC31=95 ,
   POS_PD0 =96 , POS_PD1 =97 , POS_PD2 =98 , POS_PD3 =99 , POS_PD4 =100, POS_PD5 =101, POS_PD6 =102, POS_PD7 =103,
   POS_PD8 =104, POS_PD9 =105, POS_PD10=106, POS_PD11=107, POS_PD12=108, POS_PD13=109, POS_PD14=110, POS_PD15=111,
   POS_PD16=112, POS_PD17=113, POS_PD18=114, POS_PD19=115, POS_PD20=116, POS_PD21=117, POS_PD22=118, POS_PD23=119,
   POS_PD24=120, POS_PD25=121, POS_PD26=122, POS_PD27=123, POS_PD28=124, POS_PD29=125, POS_PD30=126, POS_PD31=127,
   POS_PE0 =128, POS_PE1 =129, POS_PE2 =130, POS_PE3 =131, POS_PE4 =132, POS_PE5 =133, POS_PE6 =134, POS_PE7 =135,
   POS_PE8 =136, POS_PE9 =137, POS_PE10=138, POS_PE11=139, POS_PE12=140, POS_PE13=141, POS_PE14=142, POS_PE15=143,
   POS_PE16=144, POS_PE17=145, POS_PE18=146, POS_PE19=147, POS_PE20=148, POS_PE21=149, POS_PE22=150, POS_PE23=151,
   POS_PE24=152, POS_PE25=153, POS_PE26=154, POS_PE27=155, POS_PE28=156, POS_PE29=157, POS_PE30=158, POS_PE31=159,
   POS_PF0 =160, POS_PF1 =161, POS_PF2 =162, POS_PF3 =163, POS_PF4 =164, POS_PF5 =165, POS_PF6 =166, POS_PF7 =167,
   POS_PF8 =168, POS_PF9 =169, POS_PF10=170, POS_PF11=171, POS_PF12=172, POS_PF13=173, POS_PF14=174, POS_PF15=175,
   POS_PF16=176, POS_PF17=177, POS_PF18=178, POS_PF19=179, POS_PF20=180, POS_PF21=181, POS_PF22=182, POS_PF23=183,
   POS_PF24=184, POS_PF25=185, POS_PF26=186, POS_PF27=187, POS_PF28=188, POS_PF29=189, POS_PF30=190, POS_PF31=191,
   POS_PG0 =192, POS_PG1 =193, POS_PG2 =194, POS_PG3 =195, POS_PG4 =196, POS_PG5 =197, POS_PG6 =198, POS_PG7 =199,
   POS_PG8 =200, POS_PG9 =201, POS_PG10=202, POS_PG11=203, POS_PG12=204, POS_PG13=205, POS_PG14=206, POS_PG15=207,
   POS_PG16=208, POS_PG17=209, POS_PG18=210, POS_PG19=211, POS_PG20=212, POS_PG21=213, POS_PG22=214, POS_PG23=215,
   POS_PG24=216, POS_PG25=217, POS_PG26=218, POS_PG27=219, POS_PG28=220, POS_PG29=221, POS_PG30=222, POS_PG31=223,
   POS_PH0 =224, POS_PH1 =225, POS_PH2 =226, POS_PH3 =227, POS_PH4 =228, POS_PH5 =229, POS_PH6 =230, POS_PH7 =231,
   POS_PH8 =232, POS_PH9 =233, POS_PH10=234, POS_PH11=235, POS_PH12=236, POS_PH13=237, POS_PH14=238, POS_PH15=239,
   POS_PH16=240, POS_PH17=241, POS_PH18=242, POS_PH19=243, POS_PH20=244, POS_PH21=245, POS_PH22=246, POS_PH23=247,
   POS_PH24=248, POS_PH25=249, POS_PH26=250, POS_PH27=251, POS_PH28=252, POS_PH29=253, POS_PH30=254, POS_PH31=255,
} pos_gpio_pin_t;

/**
 * @brief GPIO processing structure
 */
typedef struct {
  /** 
 * Set GPIO mode
 * @param[in]  p  GPIO pin name, refer to @ref pos_gpio_pin_t
 * @param[in]  mode  GPIO mode, refer to @ref POS_GPIO_MODE_DIGITAL
 * @return     0: Successful\n
              !=0: Failed, refer to @ref pos_status_t
 */ 
  pos_status_t (*mode_set)(pos_gpio_pin_t p, pos_u32_t mode);

  /** 
  * Get GPIO mode
  * @param[in]  p  GPIO pin name, refer to @ref pos_gpio_pin_t
  * @param[out]  mode_ptr  Returning GPIO mode, refer to @ref POS_GPIO_MODE_DIGITAL
  * @return     0: Successful\n
               !=0: Failed, refer to @ref pos_status_t
  */ 
  pos_status_t (*mode_get)(pos_gpio_pin_t p, pos_u32_t *mode_ptr) ;

  /** 
  * Set GPIO output high/low
  * @param[in]  p  GPIO pin name, refer to @ref pos_gpio_pin_t
  * @param[in]  v  0: output high, !=0: output low
  */ 
  void (*set)(pos_gpio_pin_t p, pos_u32_t v);

  /** 
  * Get GPIO input state
  * @param[in]  p  GPIO pin name, refer to @ref pos_gpio_pin_t
  * @return     0: input high, !=0: input low
  */ 
  pos_u32_t (*get)(pos_gpio_pin_t p);

  /** 
  * Set GPIO IRQ mode
  * @param[in]  p  GPIO pin name, refer to @ref pos_gpio_pin_t
  * @param[in]  mode  GPIO IRQ mode, refer to @ref POS_GPIO_MODE_INPUT_PD_IRQ_RISING
  * @param[in]  irq_cb IRQ callback function
  * @param[in]  arg IRQ callback parameter
  */ 
  void (*irq_set)(pos_gpio_pin_t p, pos_u32_t mode, pos_func_t irq_cb, void *arg);

  /** 
  * Clear GPIO IRQ state
  * @param[in]  p  GPIO pin name, refer to @ref pos_gpio_pin_t
  * @note Interrupt will only be trigger once, until this CLR operation is called
  */ 
  void (*irq_clr)(pos_gpio_pin_t p);

  /** 
  * Set GPIO output status according to polarity
  * @param[in]  p  GPIO pin name, refer to @ref pos_gpio_pin_t
  * @param[in]  v  0: output active state, !=0: output deactive state
  */ 
  void (*polarity_set)(pos_gpio_pin_t p, pos_u32_t v);

} pos_lib_gpio_t;

/**
 * @brief Pin configuration structure
 */ 
typedef struct {
  /**
   * @brief Magic signature for PIN configuration bloack
   */
  pos_u32_t magic;

  /**
   * @brief Hardware version
   */
  pos_u32_t version;
  
  /**
   * @brief LED pins for (Red/Green/Blue/Yellow)
   */
  pos_u8_t  led[4];

  /**
   * @brief PULLUP/DN pins
   */      
  pos_u8_t  pull[4];

  /**
   * @brief Output high pins
   */    
  pos_u8_t  seth[8];

  /**
   * @brief Output low pins
   */      
  pos_u8_t  setl[8];

  /**
   * @brief Power control pins
   */
  pos_u8_t  pwr[32];

  /**
   * @brief Polarity control bitmap 
   * @note A bit of 0 for active low and 1 for active high. polarity[0].bit[0] for PA0, polarity[0].bit[31] for PA31, ..., polarity[7].bit[31] for PH31
   */  
  pos_u32_t polarity[8];

  /**
   * @brief DS18B20 single wire pins
   */      
  pos_u8_t  wire1[8];

  /**
   * @brief Analog input pins
   */      
  pos_u8_t  ain[8];

  /**
   * @brief Digital IO pins
   */      
  pos_u8_t  dio[8];

  /**
   * @brief PWM pins
   */      
  pos_u8_t  pwm[4];

  /**
   * @brief PWM mode for each PWM pin
   * @note pwm_mode[0] for pwm[0], pwm_mode[3] for pwm[3]. ID=Bit[5..4], AF=Bit[3..0]. 
   */        
  pos_u8_t  pwm_mode[4];

  /**
   * @brief Speical control pins
   */        
  pos_u8_t  ctrl[32];

  /**
   * @brief Reserved field
   */        
  pos_u8_t  rsvd[96];

  /**
   * @brief IO pins
   */          
  pos_u8_t  io[64*4];
} pos_board_pin_t;

#endif
