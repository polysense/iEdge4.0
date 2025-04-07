/**
 * @file  drv_epd.h  
 * @brief MT LCD驱动函数及类型定义
 * @author Runby F.
 * @date 2022-4-1
 * @copyright Polysense
 */

#ifndef __DRV_EPD_H__
#define __DRV_EPD_H__

/**
* @brief LCD模块SPI接口
*/
#define EPD_LCD_SPI    POS_IO_SPI1

/**
* @brief LCD模块SPI工作速率
*/
#define EPD_LCD_SPI_SPEED 500000

/**
* @brief LCD模块SPI工作模式
*/
#define EPD_LCD_SPI_MODE  (POS_IO_CTRL_SPI_MASTER+POS_IO_CTRL_SPI_POLARITY_LOW+POS_IO_CTRL_SPI_PHASE_1)

/**
* @brief LCD模块阵列宽度
*/
#define EPD_LCD_WIDTH                     400

/**
* @brief LCD模块阵列高度
*/
#define EPD_LCD_HEIGHT                    300


/**
 * @brief 帧缓存大小
 */
#define EPD_FBUF_SIZE   (EPD_LCD_WIDTH*EPD_LCD_HEIGHT/8)   

/**
 * @brief 12x14点阵汉字宽度
 */
#define HZ_DOT_SIZE   12

/**
 * @brief 16x19点阵汉字宽度
 */
#define HZ16_DOT_SIZE   16

/**
 * @brief 当前电池电压
 */
#define VBAT_GET(p) (p)->drv.history->mv[0]
#define VBAT_PER_GET(p) (p)->drv.history->mv[1]

/**
 * @brief Current Temperature
 */
#define TEMP_GET(p) (p)->drv.history->temp[0]

/**
 * @brief Current Humidity
 */
#define HUMIDITY_GET(p) (p)->drv.history->temp[8]

/**
 * @brief Current HPA
 */
#define HPA_GET(p) ((p)->drv.history->mv[6]*65536 + (p)->drv.history->mv[5])

/**
 * @brief Current CO2
 */
#define CO2_GET(p) (p)->drv.history->mv[7]

/**
 * @brief Current Signal
 */
#define SIGNAL_GET(p) (p)->drv.history->signal

/**
* @brief LCD模块SPI工作模式
*/
#define EPD_RFID_SPI_MODE  (POS_IO_CTRL_SPI_MASTER+POS_IO_CTRL_SPI_POLARITY_LOW+POS_IO_CTRL_SPI_PHASE_1)

/**
 * @brief 首次亮屏标志
 */
#define DRV_EPD_FLAG_FIRST_ON  0x80

/**
 * @brief 当前亮屏标志
 */
#define DRV_EPD_FLAG_ON  0x40

/**
 * @brief NO BATTERY flag
 */
#define DRV_EPD_FLAG_NO_BATTERY  0x20

/**
 * @brief Y reverse flag
 */
#define DRV_EPD_FLAG_Y_REVERSE 0x10

/**
 * @brief X reverse flag
 */
#define DRV_EPD_FLAG_X_REVERSE 0x08

/**
 * @brief WARNING flag
 */
#define DRV_EPD_FLAG_WARNING  0x01

/**
 * @brief Default LUT table FLAG
 */
#define DRV_EPD_LUT_OTP_DEFAULT 1

/**
 * @brief MAX CO2 history data num
 */
#define DRV_EPD_CO2_HISTORY_MAX 12

/**
 * @brief MAX TEMP history data num
 */
#define DRV_EPD_TEMP_HISTORY_MAX 48

/**
 * @brief EPD HW revisions for UC8276CBB
 */
#define DRV_EPD_HW_REV_UC8276 0

/**
 * @brief EPD HW revisions for SSD1683
 */
#define DRV_EPD_HW_REV_SSD1683 1

/**
 * @brief EPD HW revisions for UC8276CAA
 */
#define DRV_EPD_HW_REV_UC8276CAA 2

/**
 * @brief EPD value content structure
 */
typedef struct {
  pos_i16_t     temp[16];     /**< Temperature */
  pos_u32_t     hpa;      /**< HPA */
  pos_u32_t     report_ticks; /**< Report ticks */
  pos_i16_t     hum;      /**< Humidity */
  pos_u16_t     co2;      /**< CO2 PPM */
  pos_u16_t     vbat;     /** < VBAT */  
  pos_u8_t      charge;   /**< Charge status */
  pos_u8_t      vbat_per; /**< VBAT percentage */  
  pos_u8_t      signal_per; /**< Signal percentage */
  pos_u8_t      disp_mode; /**< Display mode */
  pos_u8_t      flags;    /**< Value change status */
  pos_u8_t      rsvd;     /**< reserved */
} epd_value_t;

/**
 * @brief IRQ控制结构
 */
typedef struct {
  drv_api_t     drv;      /**< 驱动克隆，指向固定SLOT的驱动指针 */
  epd_value_t value;      /**< Value structure */
  pos_u8_t      pin[2];   /**< charge pin */
  pos_u8_t      pin_busy; /**< busy pin */
  pos_u8_t      flags;    /**< control flags */
  pos_u8_t      *fbuf;    /**< 帧缓冲区 */
  pos_io_handle_t *io;    /**< SPI驱动IO */
  pos_u32_t irq_ticks;    /**< 中断时刻 */
  pos_u8_t      pin_btn;  /**< Push btn */
  pos_u8_t      co2_history_num; /**< CO2 history data num */
  pos_u8_t      hw_rev; /**< HW revisions */
  pos_u8_t      rsvd;  /**< reserved (set to 1 for a force refreshing even when data is not changed) */
  pos_i16_t     temp_good[2]; /**< Good temp from/to */
  pos_i16_t     temp_normal[2]; /**< normal temp from/to */  
  pos_i16_t     hum_good[2]; /**< Good humidity from/to */
  pos_i16_t     hum_normal[2]; /**< normal humidity from/to */  
  pos_u16_t     co2_good[2]; /**< Good co2 from/to */
  pos_u16_t     co2_normal[2]; /**< normal co2 from/to */  
  pos_u16_t     co2_history[DRV_EPD_CO2_HISTORY_MAX]; /**< max co2 history */
  pos_i16_t     temp_history[DRV_EPD_TEMP_HISTORY_MAX]; /**< max temp history */
  pos_u8_t      temp_history_num; /**< Temp history number */
  pos_u8_t      rsvd2[3]; /**< Reserved */
} epd_param_t;


void epd_drv_init(epd_param_t *p);

pos_status_t epd_drv_io_init(epd_param_t *p);

pos_status_t epd_drv_io_deinit(epd_param_t *p);

void epd_drv_display(epd_param_t *p, pos_u8_t *fbuf);

void epd_drv_on(epd_param_t *p);

void epd_drv_off(epd_param_t *p);

void epd_drv_write_stream(epd_param_t *p, pos_u8_t *buf, pos_size_t len, pos_u32_t is_data );

void epd_refresh(epd_param_t *p, pos_u8_t disp_mode);

void sensor_epd_on(epd_param_t *p);

void sensor_epd_off(epd_param_t *p);


#endif
