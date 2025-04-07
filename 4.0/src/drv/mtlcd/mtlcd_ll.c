/**
 * @file  mtlcd_ll.c
 * @brief MT LCDҺ�����ײ�����
 * @author Runby F.
 * @date 2022-4-1
 * @copyright Polysense
 */

#include "drv_api.h"
#include "drv_mtlcd.h"

/**
 * @brief LCDд����
 */
#define write_i(p,v)  mtlcd_drv_write(p, v, 0)

/**
 * @brief LCDд����
 */
#define write_d(p,v)  mtlcd_drv_write(p, v, 1)

/**
 * @brief д���ݻ�����������LCDģ��
 */
void mtlcd_drv_write_stream(mtlcd_param_t *p, pos_u8_t *buf, pos_size_t len, pos_u32_t is_data ) {
  p->drv.os->gpio->set(p->drv.board->pin_map(MB_PT_CTRL, MB_CTRL_ID_LED_DC), is_data);
  
  p->drv.os->util->udelay(1); /* very short delay */
  
  p->io->write(p->io, buf, len, POS_IO_WAIT_SPI_NORMAL(0));
}

/**
 * @brief д��һ���ݻ������LCDģ��
 */
void mtlcd_drv_write(mtlcd_param_t *p, pos_u8_t value, pos_u8_t is_data ) {
  mtlcd_drv_write_stream(p, &value, 1, is_data);
}

/**
 * @brief ����LCD��Ļ
 */
void mtlcd_drv_set_display(mtlcd_param_t *p, pos_u8_t on) {
  write_i(p, on ? 0xaf : 0xae);
}

/**
 * @brief ����LCD��
 */
void mtlcd_drv_set_column(mtlcd_param_t *p, pos_u8_t column) {
  write_i(p, 0x00|(column&0xf)); /* set  lower column start address */
  write_i(p, 0x10|(column>>4)); /* set  higher column start address */
}

/**
 * @brief ����LCDҳ��
 */
void mtlcd_drv_set_page(mtlcd_param_t *p, pos_u8_t page) {
  write_i(p, 0xB0|(page&0x7));
}

/**
 * @brief ����LCDҳ����
 */
void mtlcd_drv_set_page_column(mtlcd_param_t *p, pos_u8_t page, pos_u8_t column) {
  mtlcd_drv_set_page(p, page);
  mtlcd_drv_set_column(p, column);
}

/**
 * @brief ����LCD��ʼ��
 */
void mtlcd_drv_set_start_line(mtlcd_param_t *p, pos_u8_t y) {
  write_i(p, 0x40|(y&0x3f));
}

/**
 * @brief ����LCD�Աȶ�
 */
void mtlcd_drv_set_contrast(mtlcd_param_t *p, pos_u8_t contrast) {
  write_i(p, 0x81); /* set  contrast control */
  write_i(p, contrast); 
}

/**
 * @brief ����LCD����ģʽ
 */
void mtlcd_drv_set_scroll(mtlcd_param_t *p, pos_u8_t on) {
  write_i(p, 0x2E|(on&0x1));
}

/**
 * @brief ����LCD�з�ת
 */
void mtlcd_drv_set_segment_remap(mtlcd_param_t *p, pos_u8_t reverse) { /* 1: column from 127, 0: column from 0 */
  write_i(p, 0xa0|(reverse&1)); /* set  segment remap  */
}

/**
 * @brief ����LCD�з�ת
 */
void mtlcd_drv_set_com_scan(mtlcd_param_t *p, pos_u8_t reverse) { /* 1: row scan from 127, 0: row from 0 */
  write_i(p, reverse ? 0xc8 : 0xc0); /* set row scan  */
}

/**
 * @brief ����LCDλ������ʾ����
 */
void mtlcd_drv_set_normal_display(mtlcd_param_t *p, pos_u8_t inverse) {
  write_i(p, 0xa6|(inverse&1)); /* 1: inverser (zero for point, 1 for none) */
}

/**
 * @brief ����LCD��ʾ��ʼ��ƫ��
 */
void mtlcd_drv_set_display_offset(mtlcd_param_t *p, pos_u8_t rows) {
  write_i(p, 0xd3); /* set  display offset  */
  write_i(p, rows);
}

/**
 * @brief ����LCD���ñ���
 */
void mtlcd_drv_set_mux_ratio(mtlcd_param_t *p, pos_u8_t mux) {
  write_i(p, 0xa8); /* set  multiplex ratio */
  write_i(p, mux); /* 1/64 */
}

/**
 * @brief ����LCD��ƵƵ��
 */
void mtlcd_drv_set_display_clock(mtlcd_param_t *p, pos_u8_t d) {
	write_i(p, 0xD5);			// Set Display Clock Divide Ratio / Oscillator Frequency
	write_i(p, d);			//   Default => 0x70
						//     D[3:0] => Display Clock Divider
						//     D[7:4] => Oscillator Frequency
}

/**
 * @brief ����LCDԤ�������
 */
void mtlcd_drv_set_precharge(mtlcd_param_t *p, pos_u8_t d)
{
	write_i(p, 0xD9);			// Set Pre-Charge Period
	write_i(p, d);			//   Default => 0x22 (2 Display Clocks [Phase 2] / 2 Display Clocks [Phase 1])
						//     D[3:0] => Phase 1 Period in 1~15 Display Clocks
						//     D[7:4] => Phase 2 Period in 1~15 Display Clocks
}

/**
 * @brief LCDģ���ʼ��
 */
void mtlcd_drv_init(mtlcd_param_t *p) {     
  mtlcd_drv_set_display(p, 0); /* set  display off */

  mtlcd_drv_set_column(p, 0);

  mtlcd_drv_set_start_line(p, 0); /* set  display start line */

  mtlcd_drv_set_scroll(p, 0);

  mtlcd_drv_set_contrast(p, 0xbf); 

#if 0 /* unknown field, comment it out */
  write_i(p, 0x82);    
  write_i(p, 0x80);
#endif

  mtlcd_drv_set_segment_remap(p, 1);

  mtlcd_drv_set_normal_display(p, 0);

  /* set  multiplex ratio */
  mtlcd_drv_set_mux_ratio(p, 0x3f); /* 1/64 */

#if 0 /* unknown field, comment it out */
  write_i(0xad); /* master configuration */
  write_i(0x8e); /* external vcc supply */
#endif

  mtlcd_drv_set_com_scan(p, 1); /* set com scan direction */

  mtlcd_drv_set_display_offset(p, 0x40);

  mtlcd_drv_set_display_clock(p, 0xf0); /* set  display clock divide/oscillator frequency */

#if 0 /* unknown field, comment it out */
  write_i(p, 0xD8);    /*set area color mode off */
  write_i(p, 0x05);
#endif

  mtlcd_drv_set_precharge(p, 0xF1);
  
#if 0 /* useless for SPI */
  write_i(p, 0xda); /* set  com pin configuartion */
  write_i(p, 0x12); 
#endif

#if 0 /* unkown field comment it out */
  write_i(p, 0x91);
  write_i(p, 0x3F);
  write_i(p, 0x3F);
  write_i(p, 0x3F);
  write_i(p, 0x3F); 
#endif

  mtlcd_drv_set_display(p, 1); /* set  display on */
}

/**
 * @brief LCDģ��ײ�IO������ʼ��
 */
pos_status_t mtlcd_drv_io_init(mtlcd_param_t *p) {     
  const pos_lib_gpio_t *gpio = p->drv.os->gpio;
  ma_board_t *board = p->drv.board;
  
  /* GPIO PIN ��ʼ�� */
  gpio->mode_set(board->pin_map(MB_PT_CTRL, MB_CTRL_ID_LED_DC), POS_GPIO_MODE_OUTPUT);
  gpio->mode_set(board->pin_map(MB_PT_CTRL, MB_CTRL_ID_LED_RST), POS_GPIO_MODE_OUTPUT);  

  if( !p->io ) {
    p->io = p->drv.os->io->init(MB_LCD_SPI, MB_LCD_SPI_SPEED, MB_LCD_SPI_MODE);
    if( !p->io )
      return POS_STATUS_E_INIT;
  }
  
  return POS_STATUS_OK;
}

/**
 * @brief LCDģ��ײ�IO�����ͷ�
 */
pos_status_t mtlcd_drv_io_deinit(mtlcd_param_t *p) {     
  const pos_lib_gpio_t *gpio = p->drv.os->gpio;
  ma_board_t *board = p->drv.board;

  /* GPIO PIN ��ʼ�� */
  gpio->mode_set(board->pin_map(MB_PT_CTRL, MB_CTRL_ID_LED_DC), POS_GPIO_MODE_ANALOG+POS_GPIO_MODE_INPUT);
  gpio->mode_set(board->pin_map(MB_PT_CTRL, MB_CTRL_ID_LED_RST), POS_GPIO_MODE_ANALOG+POS_GPIO_MODE_INPUT);  

  if( p->io ) {
    p->io->release(p->io);
    p->io = POS_NULL;
  }
  
  return POS_STATUS_OK;
}

/**
 * @brief ������Ļ
 */
void mtlcd_drv_on(mtlcd_param_t *p) {
  const pos_lib_gpio_t *gpio = p->drv.os->gpio;
  pos_gpio_pin_t rst;
  ma_board_t *board = p->drv.board;

  /* ��IO */
  mtlcd_drv_io_init(p);
  
  rst = board->pin_map(MB_PT_CTRL, MB_CTRL_ID_LED_RST);
  
  /* ��ʼ��LCDģ�鹩�� */
  board->pwr_set(MB_PWR_ID_VCC_LED, 1);
  gpio->set(rst, 0);
  p->drv.os->tick->sleep(5);
  gpio->set(rst, 1);
  p->drv.os->tick->sleep(5);

  /* ��ʼ����Ļ������ */
  mtlcd_drv_init(p);  
}

/**
 * @brief �ر���Ļ
 */
void mtlcd_drv_off(mtlcd_param_t *p) {
  const pos_lib_gpio_t *gpio = p->drv.os->gpio;
  ma_board_t *board = p->drv.board;

  /* �ر���Ļ */
  mtlcd_drv_set_display(p, 0);
  p->drv.os->tick->sleep(100);

  /* LCDģ��ϵ� */
  gpio->set(board->pin_map(MB_PT_CTRL, MB_CTRL_ID_LED_RST), 0);  
  board->pwr_set(MB_PWR_ID_VCC_LED, 0);

  /* �ر�IO */
  mtlcd_drv_io_deinit(p);
}


