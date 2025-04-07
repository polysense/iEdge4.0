/**
 * @file  epd_ll.c
 * @brief MT LCD液晶面板底层驱动
 * @author Runby F.
 * @date 2022-4-1
 * @copyright Polysense
 */

#include "drv_api.h"
#include "drv_mtepd.h"

/**
 * @brief LCD写命令
 */
#define write_i(p,v)  mtepd_drv_write(p, v, 0)

/**
 * @brief LCD写数据
 */
#define write_d(p,v)  mtepd_drv_write(p, v, 1)


/**
 * @brief 写数据或者命令流到LCD模块
 */
void mtepd_drv_write_stream(mtepd_param_t *p, pos_u8_t *buf, pos_size_t len, pos_u32_t is_data ) {
  p->drv.os->gpio->set(p->drv.board->pin_map(MB_PT_CTRL, MB_CTRL_ID_LED_DC), is_data);
  
  p->drv.os->util->udelay(1); /* very short delay */
  
  p->io->write(p->io, buf, len, POS_IO_WAIT_SPI_NORMAL(0));
}

/**
 * @brief 写单一数据或者命令到LCD模块
 */
void mtepd_drv_write(mtepd_param_t *p, pos_u8_t value, pos_u8_t is_data ) {
  mtepd_drv_write_stream(p, &value, 1, is_data);
}

/**
 * @brief Deep Sleep
 */
void mtepd_drv_sleep(mtepd_param_t *p) {
  write_i(p,0x07);
  write_d(p,0xa5);
}

/**
 * @brief Wait busy for at most 6000ms
 */
pos_u32_t mtepd_drv_wait_busy(mtepd_param_t *p, pos_u32_t gran) {
  pos_u32_t s, v, ready_flag;
  s = p->drv.os->tick->get();
  v = 0;
  if( !gran )
    gran = 300;
  ready_flag = 1;
  while( v < 6000 ) {
    if( p->drv.os->gpio->get(p->pin_busy) == ready_flag )
      break;
    p->drv.os->tick->sleep(gran);
    v = p->drv.os->tick->elaps(s);      
  }
  return v;
}

/**
 * @brief Refresh
 */
void mtepd_drv_refresh(mtepd_param_t *p) {
  write_i(p,0x17);
  write_d(p,0xa5);

  /* refresh to normal by default */
  p->drv.s->ctrl |= MA_SENSOR_CTRL_STAT_NORMAL;
  p->drv.s->ctrl &= ~MA_SENSOR_CTRL_STAT_ERROR;

  /* wait until busy is not zero */
  {
    pos_u32_t v; 
    v = mtepd_drv_wait_busy(p, 0);
    if( p->drv.cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      p->drv.log->data("refresh time", v);
    }
    /* set error stat */
    if( v >= 6000 )
      p->drv.s->ctrl |= MA_SENSOR_CTRL_STAT_ERROR;    
  }
}


/**
 * @brief UC8276CAA LCD模块初始化
 */ 
void mtepd_drv_init_uc8276caa(mtepd_param_t *p) {
  write_i(p,0x00); 
  write_d(p,0x3f); 
#ifdef UC8276CAA_OLD
  write_d(p,0xca);
#else
  write_d(p,0x4d);
#endif

  write_i(p,0x01);
  write_d(p,0x03);
  write_d(p,0x10); 
  write_d(p,0x3f); 
  write_d(p,0x3f); 
  write_d(p,0x03);

  write_i(p,0x06);
  write_d(p,0xe7); 
  write_d(p,0xe7); 
  write_d(p,0x3d); 
  
  write_i(p,0x60); // TCON
  write_d(p,0x22); 

  write_i(p,0x82); // vcom
  write_d(p,0x00); 

  write_i(p,0x30); // 50HZ
  write_d(p,0x09);

#ifdef UC8276CAA_OLD  
  write_i(p,0xe3); // 
  write_d(p,0x88); 

  write_i(p,0x61);
  write_d(p,0x01);
  write_d(p,0x90);
  write_d(p,0x01);
  write_d(p,0x2c);

  write_i(p,0x50); // VCOM AND DATA INTERVAL
  write_d(p,0xb7);
#else
  write_i(p,0x50); // VCOM AND DATA INTERVAL
  write_d(p,0xD7);

  write_i(p,0xe3); 
  write_d(p,0x88); 

  write_i(p,0x61);
  write_d(p,0x01);
  write_d(p,0x90);
  write_d(p,0x01);
  write_d(p,0x2c);
  
  write_i(p,0x10); // Transfer new data
  { 
    pos_u32_t i;
    for( i = 0; i < MB_EPD_WIDTH * MB_EPD_HEIGHT / 8; i++ )
      write_d(p, 0xff);
  }
  if( p->drv.cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    p->drv.log->data("m0805", 0); /* indicating V0805 revision */
  }
#endif
}

void mtepd_drv_init_uc8276cbb(mtepd_param_t *p) {
  write_i(p,0x00); 
  write_d(p,0x1f); //USE Default LUT
  write_d(p,0x8a);

  write_i(p,0x01); 
  write_d(p,0x03); 
  write_d(p,0x10); 
  write_d(p,0x3f); 
  write_d(p,0x3f); 
  write_d(p,0x03);

  write_i(p,0x06);
  write_d(p,0xe7); 
  write_d(p,0xe7); 
  write_d(p,0x3d); 
  
  write_i(p,0x60); // TCON
  write_d(p,0x22); 

  write_i(p,0x82); // vcom
  write_d(p,0x00); 

  write_i(p,0x30); // 50HZ
  write_d(p,0x09); 

  write_i(p,0x50); // VCOM AND DATA INTERVAL
  write_d(p,0x17); 

  write_i(p,0xe3); // 
  write_d(p,0x88); 

  write_i(p,0x61); 
  write_d(p,0x01); 
  write_d(p,0x90); 
  write_d(p,0x01); 
  write_d(p,0x2c); 
}

/**
 * @brief Common EPD/LCD模块初始化
 */ 
void mtepd_drv_init(mtepd_param_t *p) {
  if( p->hw_rev == DRV_MTEPD_HW_REV_UC8276CBB )
    mtepd_drv_init_uc8276cbb(p);
  else
    mtepd_drv_init_uc8276caa(p);
}

/**
 * @brief reverse u8
 */
pos_u8_t mtepd_u8_reverse(pos_u8_t c) {
  static const pos_u8_t r[16] = {
    0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe,
    0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf,};
  return (r[c&0xf]<<4) + r[(c>>4)&0xf];
}

/**
 * @brief SSD1683 LCD模块显示刷新
 */
void mtepd_drv_display_fbuf_dir(mtepd_param_t *p, pos_u8_t *fbuf, pos_u8_t x_reverse, pos_u8_t y_reverse) {
  pos_u32_t i, pos, o;
  for( i = 0; i < MB_EPD_HEIGHT; i++ ) {
    if( y_reverse )
      pos = MB_EPD_HEIGHT - 1 - i;
    else
      pos = i;
    pos *= MB_EPD_WIDTH/8;
    for( o = 0; o < MB_EPD_WIDTH/8; o++ ) {
      if( x_reverse ) 
        write_d(p, mtepd_u8_reverse(~fbuf[pos+MB_EPD_WIDTH/8-1-o]));
      else
        write_d(p, ~fbuf[pos+o]);
    }
  }
}

/**
 * @brief UC8276CAA LCD模块显示刷新
 */
void mtepd_drv_display_uc8276caa(mtepd_param_t *p, pos_u8_t *fbuf) {
  pos_u32_t i;

#ifdef UC8276CAA_OLD
  write_i(p,0x10); // Transfer new data
  for( i = 0; i < MB_EPD_WIDTH * MB_EPD_HEIGHT / 8; i++ )
    write_d(p, 0xff);

  write_i(p,0x13); // Transfer new data
  mtepd_drv_display_fbuf_dir(p, fbuf, p->flags & DRV_MTEPD_FLAG_X_REVERSE, p->flags & DRV_MTEPD_FLAG_Y_REVERSE);
  /* UC8276CAA has to LUT update */
  {
    static const pos_u8_t lut_r20[7] = {0x01, 0x0f, 0x0f, 0x0f, 0x01, 0x01, 0x01,};
    static const pos_u8_t lut_r21[7] = {0x01, 0x4f, 0x8f, 0x0f, 0x01, 0x01, 0x01,};
    static const pos_u8_t lut_r22[7] = {0x01, 0x0f, 0x8f, 0x0f, 0x01, 0x01, 0x01,};
    static const pos_u8_t lut_r23[7] = {0x01, 0x4f, 0x8f, 0x4f, 0x01, 0x01, 0x01,};
    static const pos_u8_t lut_r24[7] = {0x01, 0x0f, 0x8f, 0x4f, 0x01, 0x01, 0x01,};

    write_i(p,0x20);
    for( i = 0; i < 42; i++ )
      write_d(p, i < 7 ? lut_r20[i] : 0);
    write_i(p,0x21);
    for( i = 0; i < 42; i++ )
      write_d(p, i < 7 ? lut_r21[i] : 0);
    write_i(p,0x24);
    for( i = 0; i < 42; i++ )
      write_d(p, i < 7 ? lut_r24[i] : 0);
    write_i(p,0x22);
    for( i = 0; i < 42; i++ )
      write_d(p, i < 7 ? lut_r22[i] : 0);
    write_i(p,0x23);
    for( i = 0; i < 42; i++ )
      write_d(p, i < 7 ? lut_r23[i] : 0);
  }
#else
  write_i(p,0x13); // Transfer new data
  mtepd_drv_display_fbuf_dir(p, fbuf, p->flags & DRV_MTEPD_FLAG_X_REVERSE, p->flags & DRV_MTEPD_FLAG_Y_REVERSE);
  {
    static const pos_u8_t lut_r20[7] = {0x01, 0x14, 0x14, 0x01, 0x01, 0x01, 0x01,};
    static const pos_u8_t lut_r21[7] = {0x01, 0x54, 0x94, 0x01, 0x01, 0x01, 0x01,};
    static const pos_u8_t lut_r22[7] = {0x01, 0x54, 0x94, 0x01, 0x01, 0x01, 0x01,};
    static const pos_u8_t lut_r23[7] = {0x01, 0x94, 0x54, 0x01, 0x01, 0x01, 0x01,};
    static const pos_u8_t lut_r24[7] = {0x01, 0x94, 0x54, 0x01, 0x01, 0x01, 0x01,};

    write_i(p,0x20);
    for( i = 0; i < 42; i++ )
      write_d(p, i < 7 ? lut_r20[i] : 0);
    write_i(p,0x21);
    for( i = 0; i < 42; i++ )
      write_d(p, i < 7 ? lut_r21[i] : 0);
    write_i(p,0x24);
    for( i = 0; i < 42; i++ )
      write_d(p, i < 7 ? lut_r24[i] : 0);
    write_i(p,0x22);
    for( i = 0; i < 42; i++ )
      write_d(p, i < 7 ? lut_r22[i] : 0);
    write_i(p,0x23);
    for( i = 0; i < 42; i++ )
      write_d(p, i < 7 ? lut_r23[i] : 0);
  }
#endif
}

/**
 * @brief UC8276CBB LCD模块显示刷新
 */
void mtepd_drv_display_uc8276cbb(mtepd_param_t *p, pos_u8_t *fbuf) {
#if 0  
  write_i(p,0x50);
  write_d(p,0xb7);
#endif

  write_i(p,0x10); // Transfer new data
  mtepd_drv_display_fbuf_dir(p, fbuf, p->flags & DRV_MTEPD_FLAG_X_REVERSE, p->flags & DRV_MTEPD_FLAG_Y_REVERSE);

  write_i(p,0x13); // Transfer new data
  mtepd_drv_display_fbuf_dir(p, fbuf, p->flags & DRV_MTEPD_FLAG_X_REVERSE, p->flags & DRV_MTEPD_FLAG_Y_REVERSE);

#if 0
  write_i(p,0x50);
  write_d(p,0xb7);
  {
    static const pos_u8_t lut_r20[7] = {0x01, 0x0f, 0x0f, 0x0f, 0x01, 0x01, 0x01,};
    static const pos_u8_t lut_r21[7] = {0x01, 0x4f, 0x8f, 0x0f, 0x01, 0x01, 0x01,};
    static const pos_u8_t lut_r22[7] = {0x01, 0x0f, 0x8f, 0x0f, 0x01, 0x01, 0x01,};
    static const pos_u8_t lut_r23[7] = {0x01, 0x4f, 0x8f, 0x4f, 0x01, 0x01, 0x01,};
    static const pos_u8_t lut_r24[7] = {0x01, 0x0f, 0x8f, 0x4f, 0x01, 0x01, 0x01,};    
    write_i(p,0x20); 
    for( i = 0; i < 56; i++ )
      write_d(p, i < 7 ? lut_r20[i] : 0);
    write_i(p,0x21); 
    for( i = 0; i < 56; i++ )
      write_d(p, i < 7 ? lut_r21[i] : 0);
    write_i(p,0x24); 
    for( i = 0; i < 56; i++ )
      write_d(p, i < 7 ? lut_r24[i] : 0);
    write_i(p,0x22); 
    for( i = 0; i < 56; i++ )
      write_d(p, i < 7 ? lut_r22[i] : 0);
    write_i(p,0x23); 
    for( i = 0; i < 56; i++ )
      write_d(p, i < 7 ? lut_r23[i] : 0);
  }
#endif
}

/**
 * @brief Common EPD/LCD模块显示刷新
 */ 
void mtepd_drv_display(mtepd_param_t *p, pos_u8_t *fbuf) {
  if( p->hw_rev == DRV_MTEPD_HW_REV_UC8276CBB )
    mtepd_drv_display_uc8276cbb(p, fbuf);
  else
    mtepd_drv_display_uc8276caa(p, fbuf);
  mtepd_drv_refresh(p);
}

/**
 * @brief LCD模块底层IO驱动初始化
 */
pos_status_t mtepd_drv_io_init(mtepd_param_t *p) {     
  const pos_lib_gpio_t *gpio = p->drv.os->gpio;
  ma_board_t *board = p->drv.board;
  
  /* GPIO PIN 初始化 */
  gpio->mode_set(board->pin_map(MB_PT_CTRL, MB_CTRL_ID_LED_DC), POS_GPIO_MODE_OUTPUT);
  gpio->mode_set(board->pin_map(MB_PT_CTRL, MB_CTRL_ID_LED_RST), POS_GPIO_MODE_OUTPUT);  

  if( !p->io ) {
    p->io = p->drv.os->io->init(MB_EPD_SPI, MB_EPD_SPI_SPEED, MB_EPD_SPI_MODE);
    if( !p->io )
      return POS_STATUS_E_INIT;
  }
  
  return POS_STATUS_OK;
}

/**
 * @brief LCD模块底层IO驱动释放
 */
pos_status_t mtepd_drv_io_deinit(mtepd_param_t *p) {     
  const pos_lib_gpio_t *gpio = p->drv.os->gpio;
  ma_board_t *board = p->drv.board;

  /* GPIO PIN 初始化 */
  gpio->mode_set(board->pin_map(MB_PT_CTRL, MB_CTRL_ID_LED_DC), POS_GPIO_MODE_OUTPUT); /* PIN should keep OUTPUT */
  gpio->mode_set(board->pin_map(MB_PT_CTRL, MB_CTRL_ID_LED_RST), POS_GPIO_MODE_OUTPUT); /* PIN should keep OUTPUT */  

  if( p->io ) {
    p->io->release(p->io);
    p->io = POS_NULL;
  }
  
  return POS_STATUS_OK;
}

/**
 * @brief 点亮屏幕Driver
 */
void mtepd_drv_on(mtepd_param_t *p) {
  const pos_lib_gpio_t *gpio = p->drv.os->gpio;
  pos_gpio_pin_t rst;
  ma_board_t *board = p->drv.board;
  pos_u8_t i;
  
  /* 打开IO */
  mtepd_drv_io_init(p);
  
  rst = board->pin_map(MB_PT_CTRL, MB_CTRL_ID_LED_RST);
  
  /* 初始化LCD模块供电 */
  board->pwr_set(MB_PWR_ID_VCC_LED, 1);
  gpio->set(rst, 1);
  p->drv.os->tick->sleep(10);
  for( i = 0; i <= 1; i++ ) {
    gpio->set(rst, i);
    p->drv.os->tick->sleep(100);
  }

  /* 初始化屏幕并点亮 */
  mtepd_drv_init(p);  

}

/**
 * @brief 关闭屏幕Driver
 */
void mtepd_drv_off(mtepd_param_t *p) {
  ma_board_t *board = p->drv.board;

  /* 关闭屏幕 */
  mtepd_drv_sleep(p);
  p->drv.os->tick->sleep(100);

  /* LCD模块断电 */
//  gpio->set(board->pin_map(MB_PT_CTRL, MB_CTRL_ID_LED_RST), 0);  
  board->pwr_set(MB_PWR_ID_VCC_LED, 0);

  /* 关闭IO */
  mtepd_drv_io_deinit(p);
}

#if 0
#define EPD_RST POS_PA11
#define EPD_RST POS_PA11
#define EPD_CS  POS_PB12
#define EPD_MOSI POS_PB13
#define EPD_SCK
void EPD_Reset(mtepd_param_t *p) {
  p->drv.os->gpio->set(EPD_RST, 0);
  p->drv.os->util->udelay(10000);
  p->drv.os->gpio->set(EPD_RST, 1);
  p->drv.os->util->udelay(10000);  
}

void EPD_Init(mtepd_param_t *p) {
  EPD_write_d(p,0x3f); 
  EPD_write_d(p,0x8a);

  EPD_write_i(p,0x01); 
  EPD_write_d(p,0x03); 
  EPD_write_d(p,0x10); 
  EPD_write_d(p,0x3f); 
  EPD_write_d(p,0x3f); 
  EPD_write_d(p,0x03);

  EPD_write_i(p,0x06);
  EPD_write_d(p,0xe7); 
  EPD_write_d(p,0xe7); 
  EPD_write_d(p,0x3d); 
  
  EPD_write_i(p,0x60); // TCON
  EPD_write_d(p,0x22); 

  EPD_write_i(p,0x82); // vcom
  EPD_write_d(p,0x00); 

  EPD_write_i(p,0x30); // 50HZ
  EPD_write_d(p,0x09); 

  EPD_write_i(p,0x50); // VCOM AND DATA INTERVAL
  EPD_write_d(p,0x17); 

  EPD_write_i(p,0xe3); // 
  EPD_write_d(p,0x88); 

  EPD_write_i(p,0x61); 
  EPD_write_d(p,0x01); 
  EPD_write_d(p,0x90); 
  EPD_write_d(p,0x01); 
  EPD_write_d(p,0x2c); 
}

void EPD_GPIO_Config(mtepd_param_t *p) {
}

void EPD_Main(mtepd_param_t *p) {
  EPD_GPIO_Config(p);
}

#endif


