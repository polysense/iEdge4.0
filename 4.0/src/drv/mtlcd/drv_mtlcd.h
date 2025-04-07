/**
 * @file  drv_mtlcd.h  
 * @brief MT LCD驱动函数及类型定义
 * @author Runby F.
 * @date 2022-4-1
 * @copyright Polysense
 */

#ifndef __DRV_MTLCD_H__
#define __DRV_MTLCD_H__

/**
* @brief LCD模块SPI接口
*/
#define MB_LCD_SPI    POS_IO_SPI0

/**
* @brief LCD模块SPI工作速率
*/
#define MB_LCD_SPI_SPEED 1000000

/**
* @brief LCD模块SPI工作模式
*/
#define MB_LCD_SPI_MODE  (POS_IO_CTRL_SPI_MASTER+POS_IO_CTRL_SPI_POLARITY_LOW+POS_IO_CTRL_SPI_PHASE_1)

/**
* @brief LCD模块阵列宽度
*/
#define MB_LCD_WIDTH                     128

/**
* @brief LCD模块阵列高度
*/
#define MB_LCD_HEIGHT                    64

/**
 * @brief 字符最大x列位置
 */
#define LCD_CH_X_MAX    ((MB_LCD_WIDTH/6)-1) /*21: 0..20 columns */

/**
 * @brief 字符最大y行位置
 */
#define LCD_CH_Y_MAX    ((MB_LCD_HEIGHT/8)-1)   /* 8: 0..7 rows */

/**
 * @brief 帧缓存大小
 */
#define LCD_FBUF_SIZE   (MB_LCD_WIDTH*MB_LCD_HEIGHT/8)   

/**
 * @brief 12x14点阵汉字宽度
 */
#define HZ_DOT_SIZE   12

/**
 * @brief 16x19点阵汉字宽度
 */
#define HZ16_DOT_SIZE   16

/**
 * @brief 保存的当前RFID
 */
#define RFID_GET_U32(p) (p)->drv.s->slot->rsvd32

/**
 * @brief 当前电池电压
 */
#define VBAT_GET(p) (p)->drv.history->mv[0]

/**
* @brief RFID模块SPI接口
*/
#define MB_RFID_SPI    POS_IO_SPI1

/**
* @brief LCD模块SPI工作速率
*/
#define MB_RFID_SPI_SPEED 1000000

/**
* @brief LCD模块SPI工作模式
*/
#define MB_RFID_SPI_MODE  (POS_IO_CTRL_SPI_MASTER+POS_IO_CTRL_SPI_POLARITY_LOW+POS_IO_CTRL_SPI_PHASE_1)

/**
 * @brief 首次亮屏标志
 */
#define DRV_MTLCD_FLAG_FIRST_ON  0x80

/**
 * @brief 当前亮屏标志
 */
#define DRV_MTLCD_FLAG_ON  0x40

/**
 * @brief DC供电标志
 */
#define DRV_MTLCD_FLAG_DC  0x20

/**
 * @brief DISABLE ALARM FLAG
 */
#define DRV_MTLCD_FLAG_NO_ALARM  0x10

/**
 * @brief IRQ控制结构
 */
typedef struct {
  drv_api_t     drv;      /**< 驱动克隆，指向固定SLOT的驱动指针 */
  pos_u32_t     rfid;     /**< 当前RFID */
  pos_u8_t      pin;      /**< 按钮管脚 */
  pos_u8_t      flags;    /**< 内部状态 */
  pos_u8_t      ch_x;     /**< X字符列位置 */
  pos_u8_t      ch_y;     /**< Y字符行位置 */
  pos_u8_t      *fbuf;    /**< 帧缓冲区 */
  pos_io_handle_t *io;    /**< SPI驱动IO */
  pos_io_handle_t *rfid_io;  /**< RFID驱动IO */
  pos_u32_t     lcd_time; /**< 亮屏按钮毫秒时长 */
  pos_u32_t     rfid_time;  /**< 扫码按钮毫秒时长 */
  pos_u32_t     erase_time; /**< 清除按钮毫秒时长 */
  pos_u32_t     keep_time;  /**< 亮屏保持时长 */
  pos_u32_t     irq_ticks;    /**< 中断时刻 */
} mtlcd_param_t;

/**
 * @brief 汉字类型
 */
typedef enum {
  HZ_ZHI, /**< 制 */
  HZ_QU,  /**< 曲 */
  HZ_CHE, /**< 车 */
  HZ_JIAN, /**< 间 */
  HZ_BAN,  /**< 班 */
  HZ_ZU,    /**< 组 */
  HZ_JIAO,/**< 窖 */  
  HZ_CHI, /**< 池 */
  HZ_XIAO,  /**< 酵, 未避免和窖字歧义, 此处故意读XIAO */
  HZ_DUI,  /**< 堆 */
  HZ_LIANG,/**< 粮 */
  HZ_JIU,/**< 酒 */
  HZ_ZHI16, /**< 制 16点阵 */
  HZ_QU16, /**< 曲 16点阵 */
  HZ_JIU16,/**< 酒 16点阵 */
  HZ_QING16,/**< 请 16点阵 */
  HZ_SHUA16,/**< 刷 16点阵 */
  HZ_KA16,/**< 卡 16点阵 */
  HZ_CHENG16,/**< 成 16点阵 */
  HZ_GONG16,/**< 功 16点阵 */
  HZ_WEI16,/**< 未 16点阵 */
  HZ_SAO16,/**< 扫 16点阵 */
  HZ_MA16,/**< 码 16点阵 */
  
  HZ_MAX, /**< 最大值不得>=这个数字 */
} HZ_t;

void mtlcd_drv_init(mtlcd_param_t *p);

pos_status_t mtlcd_drv_io_init(mtlcd_param_t *p);

pos_status_t mtlcd_drv_io_deinit(mtlcd_param_t *p);

void mtlcd_drv_set_page_column(mtlcd_param_t *p, pos_u8_t page, pos_u8_t column);

void mtlcd_drv_on(mtlcd_param_t *p);

void mtlcd_drv_off(mtlcd_param_t *p);

void mtlcd_drv_write_stream(mtlcd_param_t *p, pos_u8_t *buf, pos_size_t len, pos_u32_t is_data );

void mtlcd_refresh(mtlcd_param_t *p, pos_u8_t disp_mode);

void mtlcd_time_collect(mtlcd_param_t *p);

pos_u32_t mtlcd_btn_time_get(mtlcd_param_t *p, pos_u32_t max_time);

void mtlcd_alarm_set(mtlcd_param_t *p, pos_u32_t mode);

pos_status_t drv_rfid_read(pos_u8_t *buf);

pos_status_t drv_rfid_init(void);

pos_status_t drv_rfid_done(void);

#endif
