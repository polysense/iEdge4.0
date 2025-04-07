/**
 * @file  mtepd.h  
 * @brief MT EPD�������������Ͷ���
 * @author Runby F.
 * @date 2022-4-1
 * @copyright Polysense
 */

#ifndef __DRV_MTEPD_H__
#define __DRV_MTEPD_H__

/**
* @brief EPDģ��SPI�ӿ�
*/
#define MB_EPD_SPI    POS_IO_SPI1

/**
* @brief EPDģ��SPI��������
*/
#define MB_EPD_SPI_SPEED 500000

/**
* @brief EPDģ��SPI����ģʽ
*/
#define MB_EPD_SPI_MODE  (POS_IO_CTRL_SPI_MASTER+POS_IO_CTRL_SPI_POLARITY_LOW+POS_IO_CTRL_SPI_PHASE_1)

/**
* @brief EPDģ�����п��
*/
#define MB_EPD_WIDTH                     400

/**
* @brief EPDģ�����и߶�
*/
#define MB_EPD_HEIGHT                    300


/**
 * @brief ֡�����С
 */
#define MTEPD_FBUF_SIZE   (MB_EPD_WIDTH*MB_EPD_HEIGHT/8)   


/**
 * @brief ����ĵ�ǰRFID
 */
#define RFID_GET_U32(p) (p)->drv.s->slot->rsvd32

/**
 * @brief ��ǰ��ص�ѹ
 */
#define VBAT_GET(p) (p)->drv.history->mv[0]
#define VBAT_PER_GET(p) (p)->drv.history->mv[1]

/**
* @brief RFIDģ��SPI�ӿ�
*/
#define MB_RFID_SPI    POS_IO_SPI0

/**
* @brief RFIDģ��SPI��������
*/
#define MB_RFID_SPI_SPEED 1000000

/**
* @brief LCDģ��SPI����ģʽ
*/
#define MB_RFID_SPI_MODE  (POS_IO_CTRL_SPI_MASTER+POS_IO_CTRL_SPI_POLARITY_LOW+POS_IO_CTRL_SPI_PHASE_1)

/**
 * @brief REFRESH��־ (triggered by each cycle)
 */
#define DRV_MTEPD_FLAG_REFRESH  0x80

/**
 * @brief NO BATTERY flag
 */
#define DRV_MTEPD_FLAG_NO_BATTERY  0x20

/**
 * @brief DC�����־ (not support currently)
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
 * @brief IRQ���ƽṹ
 */
typedef struct {
  drv_api_t     drv;      /**< ������¡��ָ��̶�SLOT������ָ�� */
  pos_u32_t     rfid;     /**< ��ǰRFID */
  pos_u8_t      pin_btn;   /**< ��ť�ܽ� */
  pos_u8_t      pin_busy;   /** busy pin */
  pos_u8_t      flags;    /**< �ڲ�״̬ */
  pos_u8_t      hw_rev;   /**< hw module */;
  pos_u8_t      *fbuf;    /**< ֡������ */
  pos_io_handle_t *io;    /**< SPI����IO */
  pos_io_handle_t *rfid_io;  /**< RFID����IO */
  pos_u32_t     epd_time; /**< ������ť����ʱ�� (MTEPD not use this) */
  pos_u32_t     rfid_time;  /**< ɨ�밴ť����ʱ�� */
  pos_u32_t     erase_time; /**< �����ť����ʱ�� */
  pos_u32_t     keep_time;  /**< ��������ʱ�� */  
  pos_u32_t     irq_ticks;    /**< �ж�ʱ�� */
} mtepd_param_t;

/**
 * @brief ��������
 */
typedef enum {
  MHZ_NULL, 

  MHZ_ZHI, /**< �� */
  MHZ_QU,  /**< �� */
  MHZ_CHE, /**< �� */
  MHZ_JIAN, /**< �� */
  MHZ_BAN,  /**< �� */
  MHZ_ZU,    /**< �� */
  MHZ_XIAO,  /**< ��, δ����ͽ�������, �˴������XIAO */
  MHZ_DUI,  /**< �� */

  MHZ_JIAO,/**< �� */ 
  MHZ_CHI, /**< �� */
  MHZ_LIANG,/**< �� */
  MHZ_JIU,/**< �� */
  MHZ_QING,/**< �� */
  MHZ_SHUA,/**< ˢ */
  MHZ_KA,/**< �� */
  MHZ_CHENG,/**< �� */
  
  MHZ_GONG,/**< �� */
  MHZ_WEI,/**< δ */
  MHZ_SAO,/**< ɨ */
  MHZ_MA,/**< �� */
  MHZ_SHANG,/**< �� */
  MHZ_ZHONG,/**< �� */
  MHZ_XIA,/**< �� */
  MHZ_ZHU,/**< ע */
  
  MHZ_MAX, /**< ���ֵ����>=������� */
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
