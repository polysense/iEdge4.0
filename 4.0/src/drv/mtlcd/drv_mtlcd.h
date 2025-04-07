/**
 * @file  drv_mtlcd.h  
 * @brief MT LCD�������������Ͷ���
 * @author Runby F.
 * @date 2022-4-1
 * @copyright Polysense
 */

#ifndef __DRV_MTLCD_H__
#define __DRV_MTLCD_H__

/**
* @brief LCDģ��SPI�ӿ�
*/
#define MB_LCD_SPI    POS_IO_SPI0

/**
* @brief LCDģ��SPI��������
*/
#define MB_LCD_SPI_SPEED 1000000

/**
* @brief LCDģ��SPI����ģʽ
*/
#define MB_LCD_SPI_MODE  (POS_IO_CTRL_SPI_MASTER+POS_IO_CTRL_SPI_POLARITY_LOW+POS_IO_CTRL_SPI_PHASE_1)

/**
* @brief LCDģ�����п��
*/
#define MB_LCD_WIDTH                     128

/**
* @brief LCDģ�����и߶�
*/
#define MB_LCD_HEIGHT                    64

/**
 * @brief �ַ����x��λ��
 */
#define LCD_CH_X_MAX    ((MB_LCD_WIDTH/6)-1) /*21: 0..20 columns */

/**
 * @brief �ַ����y��λ��
 */
#define LCD_CH_Y_MAX    ((MB_LCD_HEIGHT/8)-1)   /* 8: 0..7 rows */

/**
 * @brief ֡�����С
 */
#define LCD_FBUF_SIZE   (MB_LCD_WIDTH*MB_LCD_HEIGHT/8)   

/**
 * @brief 12x14�����ֿ��
 */
#define HZ_DOT_SIZE   12

/**
 * @brief 16x19�����ֿ��
 */
#define HZ16_DOT_SIZE   16

/**
 * @brief ����ĵ�ǰRFID
 */
#define RFID_GET_U32(p) (p)->drv.s->slot->rsvd32

/**
 * @brief ��ǰ��ص�ѹ
 */
#define VBAT_GET(p) (p)->drv.history->mv[0]

/**
* @brief RFIDģ��SPI�ӿ�
*/
#define MB_RFID_SPI    POS_IO_SPI1

/**
* @brief LCDģ��SPI��������
*/
#define MB_RFID_SPI_SPEED 1000000

/**
* @brief LCDģ��SPI����ģʽ
*/
#define MB_RFID_SPI_MODE  (POS_IO_CTRL_SPI_MASTER+POS_IO_CTRL_SPI_POLARITY_LOW+POS_IO_CTRL_SPI_PHASE_1)

/**
 * @brief �״�������־
 */
#define DRV_MTLCD_FLAG_FIRST_ON  0x80

/**
 * @brief ��ǰ������־
 */
#define DRV_MTLCD_FLAG_ON  0x40

/**
 * @brief DC�����־
 */
#define DRV_MTLCD_FLAG_DC  0x20

/**
 * @brief DISABLE ALARM FLAG
 */
#define DRV_MTLCD_FLAG_NO_ALARM  0x10

/**
 * @brief IRQ���ƽṹ
 */
typedef struct {
  drv_api_t     drv;      /**< ������¡��ָ��̶�SLOT������ָ�� */
  pos_u32_t     rfid;     /**< ��ǰRFID */
  pos_u8_t      pin;      /**< ��ť�ܽ� */
  pos_u8_t      flags;    /**< �ڲ�״̬ */
  pos_u8_t      ch_x;     /**< X�ַ���λ�� */
  pos_u8_t      ch_y;     /**< Y�ַ���λ�� */
  pos_u8_t      *fbuf;    /**< ֡������ */
  pos_io_handle_t *io;    /**< SPI����IO */
  pos_io_handle_t *rfid_io;  /**< RFID����IO */
  pos_u32_t     lcd_time; /**< ������ť����ʱ�� */
  pos_u32_t     rfid_time;  /**< ɨ�밴ť����ʱ�� */
  pos_u32_t     erase_time; /**< �����ť����ʱ�� */
  pos_u32_t     keep_time;  /**< ��������ʱ�� */
  pos_u32_t     irq_ticks;    /**< �ж�ʱ�� */
} mtlcd_param_t;

/**
 * @brief ��������
 */
typedef enum {
  HZ_ZHI, /**< �� */
  HZ_QU,  /**< �� */
  HZ_CHE, /**< �� */
  HZ_JIAN, /**< �� */
  HZ_BAN,  /**< �� */
  HZ_ZU,    /**< �� */
  HZ_JIAO,/**< �� */  
  HZ_CHI, /**< �� */
  HZ_XIAO,  /**< ��, δ����ͽ�������, �˴������XIAO */
  HZ_DUI,  /**< �� */
  HZ_LIANG,/**< �� */
  HZ_JIU,/**< �� */
  HZ_ZHI16, /**< �� 16���� */
  HZ_QU16, /**< �� 16���� */
  HZ_JIU16,/**< �� 16���� */
  HZ_QING16,/**< �� 16���� */
  HZ_SHUA16,/**< ˢ 16���� */
  HZ_KA16,/**< �� 16���� */
  HZ_CHENG16,/**< �� 16���� */
  HZ_GONG16,/**< �� 16���� */
  HZ_WEI16,/**< δ 16���� */
  HZ_SAO16,/**< ɨ 16���� */
  HZ_MA16,/**< �� 16���� */
  
  HZ_MAX, /**< ���ֵ����>=������� */
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
