/**
 * @file  mtepd.h  
 * @brief MT EPD驱动函数及类型定义
 * @author Runby F.
 * @date 2022-4-1
 * @copyright Polysense
 */

#ifndef __DRV_MTEPD_H__
#define __DRV_MTEPD_H__

/**
* @brief EPD模块SPI接口
*/
#define MB_EPD_SPI    POS_IO_SPI1

/**
* @brief EPD模块SPI工作速率
*/
#define MB_EPD_SPI_SPEED 500000

/**
* @brief EPD模块SPI工作模式
*/
#define MB_EPD_SPI_MODE  (POS_IO_CTRL_SPI_MASTER+POS_IO_CTRL_SPI_POLARITY_LOW+POS_IO_CTRL_SPI_PHASE_1)

/**
* @brief EPD模块阵列宽度
*/
#define MB_EPD_WIDTH                     400

/**
* @brief EPD模块阵列高度
*/
#define MB_EPD_HEIGHT                    300


/**
 * @brief 帧缓存大小
 */
#define MTEPD_FBUF_SIZE   (MB_EPD_WIDTH*MB_EPD_HEIGHT/8)   


/**
 * @brief 保存的当前RFID
 */
#define RFID_GET_U32(p) (p)->drv.s->slot->rsvd32

/**
 * @brief 当前电池电压
 */
#define VBAT_GET(p) (p)->drv.history->mv[0]
#define VBAT_PER_GET(p) (p)->drv.history->mv[1]

/**
* @brief RFID模块SPI接口
*/
#define MB_RFID_SPI    POS_IO_SPI0

/**
* @brief RFID模块SPI工作速率
*/
#define MB_RFID_SPI_SPEED 1000000

/**
* @brief LCD模块SPI工作模式
*/
#define MB_RFID_SPI_MODE  (POS_IO_CTRL_SPI_MASTER+POS_IO_CTRL_SPI_POLARITY_LOW+POS_IO_CTRL_SPI_PHASE_1)

/**
 * @brief REFRESH标志 (triggered by each cycle)
 */
#define DRV_MTEPD_FLAG_REFRESH  0x80

/**
 * @brief NO BATTERY flag
 */
#define DRV_MTEPD_FLAG_NO_BATTERY  0x20

/**
 * @brief DC供电标志 (not support currently)
 */
#define DRV_MTEPD_FLAG_DC  0x10

/**
 * @brief DISABLE ALARM FLAG
 */
#define DRV_MTEPD_FLAG_NO_ALARM  0x08

/**
 * @brief Y reverse flag
 */
#define DRV_MTEPD_FLAG_Y_REVERSE 0x04

/**
 * @brief X reverse flag
 */
#define DRV_MTEPD_FLAG_X_REVERSE 0x02

/**
 * @brief Currently under ALARM
 */
#define DRV_MTEPD_FLAG_IN_ALARM 0x01

/**
 * @brief EPD HW revisions for UC8276CBB
 */
#define DRV_MTEPD_HW_REV_UC8276CBB 0

/**
 * @brief EPD HW revisions for SSD1683
 */
#define DRV_MTEPD_HW_REV_SSD1683 1

/**
 * @brief EPD HW revisions for UC8276CAA
 */
#define DRV_MTEPD_HW_REV_UC8276CAA 2


/**
 * @brief IRQ控制结构
 */
typedef struct {
  drv_api_t     drv;      /**< 驱动克隆，指向固定SLOT的驱动指针 */
  pos_u32_t     rfid;     /**< 当前RFID */
  pos_u8_t      pin_btn;   /**< 按钮管脚 */
  pos_u8_t      pin_busy;   /** busy pin */
  pos_u8_t      flags;    /**< 内部状态 */
  pos_u8_t      hw_rev;   /**< hw module */;
  pos_u8_t      *fbuf;    /**< 帧缓冲区 */
  pos_io_handle_t *io;    /**< SPI驱动IO */
  pos_io_handle_t *rfid_io;  /**< RFID驱动IO */
  pos_u32_t     epd_time; /**< 亮屏按钮毫秒时长 (MTEPD not use this) */
  pos_u32_t     rfid_time;  /**< 扫码按钮毫秒时长 */
  pos_u32_t     erase_time; /**< 清除按钮毫秒时长 */
  pos_u32_t     keep_time;  /**< 亮屏保持时长 */  
  pos_u32_t     irq_ticks;    /**< 中断时刻 */
} mtepd_param_t;

/**
 * @brief 汉字类型
 */
typedef enum {
  MHZ_NULL, 

  MHZ_ZHI, /**< 制 */
  MHZ_QU,  /**< 曲 */
  MHZ_CHE, /**< 车 */
  MHZ_JIAN, /**< 间 */
  MHZ_BAN,  /**< 班 */
  MHZ_ZU,    /**< 组 */
  MHZ_XIAO,  /**< 酵, 未避免和窖字歧义, 此处故意读XIAO */
  MHZ_DUI,  /**< 堆 */

  MHZ_JIAO,/**< 窖 */ 
  MHZ_CHI, /**< 池 */
  MHZ_LIANG,/**< 粮 */
  MHZ_JIU,/**< 酒 */
  MHZ_QING,/**< 请 */
  MHZ_SHUA,/**< 刷 */
  MHZ_KA,/**< 卡 */
  MHZ_CHENG,/**< 成 */
  
  MHZ_GONG,/**< 功 */
  MHZ_WEI,/**< 未 */
  MHZ_SAO,/**< 扫 */
  MHZ_MA,/**< 码 */
  MHZ_SHANG,/**< 上 */
  MHZ_ZHONG,/**< 中 */
  MHZ_XIA,/**< 下 */
  MHZ_ZHU,/**< 注 */
  
  MHZ_MAX, /**< 最大值不得>=这个数字 */
} MHZ_t;

void mtepd_drv_init(mtepd_param_t *p);

pos_status_t mtepd_drv_io_init(mtepd_param_t *p);

pos_status_t mtepd_drv_io_deinit(mtepd_param_t *p);

void mtepd_drv_on(mtepd_param_t *p);

void mtepd_drv_off(mtepd_param_t *p);

void mtepd_drv_write_stream(mtepd_param_t *p, pos_u8_t *buf, pos_size_t len, pos_u32_t is_data );

void mtepd_drv_display(mtepd_param_t *p, pos_u8_t *fbuf);

void mtepd_refresh(mtepd_param_t *p, pos_u8_t disp_mode);

void mtepd_time_collect(mtepd_param_t *p);

pos_u32_t mtepd_btn_time_get(mtepd_param_t *p, pos_u32_t max_time);

void mtepd_alarm_set(mtepd_param_t *p, pos_u32_t mode);

pos_status_t drv_rfid_mtepd_read(pos_u8_t *buf);

pos_status_t drv_rfid_mtepd_init(void);

pos_status_t drv_rfid_mtepd_done(void);

#endif
