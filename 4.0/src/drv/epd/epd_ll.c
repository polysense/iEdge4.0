/**
 * @file  epd_ll.c
 * @brief MT LCD液晶面板底层驱动
 * @author Runby F.
 * @date 2022-4-1
 * @copyright Polysense
 */

#include "drv_api.h"
#include "drv_epd.h"

/**
 * @brief LCD写命令
 */
#define write_i(p,v)  epd_drv_write(p, v, 0)

/**
 * @brief LCD写数据
 */
#define write_d(p,v)  epd_drv_write(p, v, 1)

/**
 * @brief 写数据或者命令流到LCD模块
 */
void epd_drv_write_stream(epd_param_t *p, pos_u8_t *buf, pos_size_t len, pos_u32_t is_data ) {
  p->drv.os->gpio->set(p->drv.board->pin_map(MB_PT_CTRL, MB_CTRL_ID_LED_DC), is_data);
  
  p->drv.os->util->udelay(1); /* very short delay */
  
  p->io->write(p->io, buf, len, POS_IO_WAIT_SPI_NORMAL(0));
}

/**
 * @brief 写单一数据或者命令到LCD模块
 */
void epd_drv_write(epd_param_t *p, pos_u8_t value, pos_u8_t is_data ) {
  epd_drv_write_stream(p, &value, 1, is_data);
}

/**
 * @brief Deep Sleep
 */
void epd_drv_sleep(epd_param_t *p) {
  if( p->hw_rev == DRV_EPD_HW_REV_UC8276 || p->hw_rev == DRV_EPD_HW_REV_UC8276CAA ) {
    write_i(p,0x07);
    write_d(p,0xa5);
  } else {
    /* SSD1683 */
    write_i(p,0x10);
    write_d(p,0x01);
  }
}

/**
 * @brief Wait busy for at most 6000ms
 */
pos_u32_t epd_drv_wait_busy(epd_param_t *p, pos_u32_t gran) {
  pos_u32_t s, v, ready_flag;
  s = p->drv.os->tick->get();
  v = 0;
  if( !gran )
    gran = 300;
  if( p->hw_rev == DRV_EPD_HW_REV_UC8276 || p->hw_rev == DRV_EPD_HW_REV_UC8276CAA )
    ready_flag = 1;
  else
    ready_flag = 0; // SSD1683 use ZERO for ready */
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
void epd_drv_refresh(epd_param_t *p) {
  if( p->hw_rev == DRV_EPD_HW_REV_UC8276 || p->hw_rev == DRV_EPD_HW_REV_UC8276CAA ) {
    write_i(p,0x17);
    write_d(p,0xa5);
  } else {
    /* SSD1683 */
#if 0    
    /* fast REFRESH (not work) */
    write_i(p, 0x22); //Display Update Control
    write_d(p, 0xC7);   
    write_i(p, 0x20); //Activate Display Update Sequence    
#else
    /* normal REFRESH */
    write_i(p, 0x22); //Display Update Control
    write_d(p, 0xF7);
    write_i(p, 0x20); //Activate Display Update Sequence    
#endif
  }

  /* refresh to normal by default */
  p->drv.s->ctrl |= MA_SENSOR_CTRL_STAT_NORMAL;
  p->drv.s->ctrl &= ~MA_SENSOR_CTRL_STAT_ERROR;

  /* wait until busy is not zero */
  {
    pos_u32_t v; 
    v = epd_drv_wait_busy(p, 0);
    if( p->drv.cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      p->drv.log->data("refresh time", v);
    }
    /* set error stat */
    if( v >= 6000 )
      p->drv.s->ctrl |= MA_SENSOR_CTRL_STAT_ERROR;    
  }
}


/**
 * @brief UC8276 LCD模块初始化
 */ 
void epd_drv_init_uc8276(epd_param_t *p) {
  write_i(p,0x00); 
#ifdef DRV_EPD_LUT_OTP_DEFAULT  
  write_d(p,0x1f); //USE Default LUT  
#else  
  write_d(p,0x3f); 
#endif

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
 * @brief SSD1683 LCD模块初始化
 */ 
void epd_drv_init_ssd1683(epd_param_t *p) {
  pos_u32_t v;
  v = epd_drv_wait_busy(p, 50);  
  write_i(p,0x12);     //SWRESET
  v = epd_drv_wait_busy(p, 50);
  if( p->drv.cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    p->drv.log->data("rst time", v);
  }

  write_i(p,0x21);
  write_d(p,0x40);
  write_d(p,0x00);

  write_i(p,0x3C); // board
  write_d(p,0x05); // 

#if 0
  /* fast init */
  write_i(p,0x1A); // Write to temperature register
  write_d(p,0x64);
  write_d(p,0x00); 

  write_i(p,0x22); // Load temperature value
  write_d(p,0x91);
  write_d(p,0x20);
  v = epd_drv_wait_busy(p, 50);
  if( p->drv.cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    p->drv.log->data("load tmp", v);
  }
#else
#endif

  write_i(p,0x11);  // Data entry mode
  write_d(p,0x01);

  write_i(p,0x44); 
  write_d(p,0x00); // RAM x address start at 0
  write_d(p,0x31); // RAM x address end at 31h(49+1)*8->400
  
  write_i(p,0x45);
  write_d(p,0x2B); // RAM y address start at 12Bh     
  write_d(p,0x01);
  write_d(p,0x00); // RAM y address end at 00h     
  write_d(p,0x00);

  write_i(p,0x4E);
  write_d(p,0x00);
  
  write_i(p,0x4F); 
  write_d(p,0x2B);
  write_d(p,0x01);

}

/**
 * @brief UC8276CAA LCD模块初始化
 */ 
void epd_drv_init_uc8276caa(epd_param_t *p) {
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
    for( i = 0; i < EPD_LCD_WIDTH * EPD_LCD_HEIGHT / 8; i++ )
      write_d(p, 0xff);
  }
  if( p->drv.cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    p->drv.log->data("v0805", 0); /* indicating V0805 revision */
  }
#endif
}

/**
 * @brief Common EPD/LCD模块初始化
 */ 
void epd_drv_init(epd_param_t *p) {
  if( p->hw_rev == DRV_EPD_HW_REV_UC8276 )
    epd_drv_init_uc8276(p);
  else if( p->hw_rev == DRV_EPD_HW_REV_UC8276CAA )
    epd_drv_init_uc8276caa(p);    
  else
    epd_drv_init_ssd1683(p);
}

/**
 * @brief reverse u8
 */
pos_u8_t epd_u8_reverse(pos_u8_t c) {
  static const pos_u8_t r[16] = {
    0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe,
    0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf,};
  return (r[c&0xf]<<4) + r[(c>>4)&0xf];
}

/**
 * @brief SSD1683 LCD模块显示刷新
 */
void epd_drv_display_fbuf_dir(epd_param_t *p, pos_u8_t *fbuf, pos_u8_t x_reverse, pos_u8_t y_reverse) {
  pos_u32_t i, pos, o;
  for( i = 0; i < EPD_LCD_HEIGHT; i++ ) {
    if( y_reverse )
      pos = EPD_LCD_HEIGHT - 1 - i;
    else
      pos = i;
    pos *= EPD_LCD_WIDTH/8;
    for( o = 0; o < EPD_LCD_WIDTH/8; o++ ) {
      if( x_reverse ) 
        write_d(p, epd_u8_reverse(~fbuf[pos+EPD_LCD_WIDTH/8-1-o]));
      else
        write_d(p, ~fbuf[pos+o]);
    }
  }
}

/**
 * @brief UC8276 LCD模块显示刷新
 */
void epd_drv_display_uc8276(epd_param_t *p, pos_u8_t *fbuf) {
#if 0
  write_i(p,0x50); 
  write_d(p,0xb7);
#endif

#if 1
  write_i(p,0x10); // Transfer new data
  epd_drv_display_fbuf_dir(p, fbuf, p->flags & DRV_EPD_FLAG_X_REVERSE, p->flags & DRV_EPD_FLAG_Y_REVERSE);

#endif

#if 1
  write_i(p,0x13); // Transfer new data
  epd_drv_display_fbuf_dir(p, fbuf, p->flags & DRV_EPD_FLAG_X_REVERSE, p->flags & DRV_EPD_FLAG_Y_REVERSE);
#endif

#ifdef DRV_EPD_LUT_OTP_DEFAULT
  /* default LUT: 
   * do NOTHING here
   */
#else
{
  pos_u32_t i;
  write_i(p,0x50); 
  write_d(p,0xb7);

  write_i(p,0x20); 
  for( i = 0; i < 56; i++ )
    write_d(p, 0);
  write_i(p,0x21); 
  for( i = 0; i < 56; i++ )
    write_d(p, 0);
  write_i(p,0x24); 
  for( i = 0; i < 56; i++ )
    write_d(p, 0);
  write_i(p,0x22); 
  for( i = 0; i < 56; i++ )
    write_d(p, 0);
  write_i(p,0x23); 
  for( i = 0; i < 56; i++ )
    write_d(p, 0);
}
#endif  
}

/**
 * @brief SSD1683 LCD模块显示刷新
 */
void epd_drv_display_ssd1683(epd_param_t *p, pos_u8_t *fbuf) {
  write_i(p,0x24); //write RAM for black(0)/white (1)
  epd_drv_display_fbuf_dir(p, fbuf, p->flags & DRV_EPD_FLAG_X_REVERSE, p->flags & DRV_EPD_FLAG_Y_REVERSE);

  write_i(p,0x26); //write RAM for black(0)/white (1)
  epd_drv_display_fbuf_dir(p, fbuf, p->flags & DRV_EPD_FLAG_X_REVERSE, p->flags & DRV_EPD_FLAG_Y_REVERSE);

}

/**
 * @brief UC8276CAA LCD模块显示刷新
 */
void epd_drv_display_uc8276caa(epd_param_t *p, pos_u8_t *fbuf) {
  pos_u32_t i;

#ifdef UC8276CAA_OLD
  write_i(p,0x10); // Transfer new data
  for( i = 0; i < EPD_LCD_WIDTH * EPD_LCD_HEIGHT / 8; i++ )
    write_d(p, 0xff);

  write_i(p,0x13); // Transfer new data
  epd_drv_display_fbuf_dir(p, fbuf, p->flags & DRV_EPD_FLAG_X_REVERSE, p->flags & DRV_EPD_FLAG_Y_REVERSE);
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
  epd_drv_display_fbuf_dir(p, fbuf, p->flags & DRV_EPD_FLAG_X_REVERSE, p->flags & DRV_EPD_FLAG_Y_REVERSE);
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
 * @brief Common EPD/LCD模块显示刷新
 */ 
void epd_drv_display(epd_param_t *p, pos_u8_t *fbuf) {
  if( p->hw_rev == DRV_EPD_HW_REV_UC8276 )
    epd_drv_display_uc8276(p, fbuf);
  else if( p->hw_rev == DRV_EPD_HW_REV_UC8276CAA )
    epd_drv_display_uc8276caa(p, fbuf);
  else
    epd_drv_display_ssd1683(p, fbuf);
  epd_drv_refresh(p);
}

/**
 * @brief LCD模块底层IO驱动初始化
 */
pos_status_t epd_drv_io_init(epd_param_t *p) {     
  const pos_lib_gpio_t *gpio = p->drv.os->gpio;
  ma_board_t *board = p->drv.board;
  
  /* GPIO PIN 初始化 */
  gpio->mode_set(board->pin_map(MB_PT_CTRL, MB_CTRL_ID_LED_DC), POS_GPIO_MODE_OUTPUT);
  gpio->mode_set(board->pin_map(MB_PT_CTRL, MB_CTRL_ID_LED_RST), POS_GPIO_MODE_OUTPUT);  

  if( !p->io ) {
    p->io = p->drv.os->io->init(EPD_LCD_SPI, EPD_LCD_SPI_SPEED, EPD_LCD_SPI_MODE);
    if( !p->io )
      return POS_STATUS_E_INIT;
  }
  
  return POS_STATUS_OK;
}

/**
 * @brief LCD模块底层IO驱动释放
 */
pos_status_t epd_drv_io_deinit(epd_param_t *p) {     
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
void epd_drv_on(epd_param_t *p) {
  const pos_lib_gpio_t *gpio = p->drv.os->gpio;
  pos_gpio_pin_t rst;
  ma_board_t *board = p->drv.board;
  pos_u8_t i;
  
  /* 打开IO */
  epd_drv_io_init(p);
  
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
  epd_drv_init(p);  

}

/**
 * @brief 关闭屏幕Driver
 */
void epd_drv_off(epd_param_t *p) {
  ma_board_t *board = p->drv.board;

  /* 关闭屏幕 */
  epd_drv_sleep(p);
  p->drv.os->tick->sleep(100);

  /* LCD模块断电 */
//  gpio->set(board->pin_map(MB_PT_CTRL, MB_CTRL_ID_LED_RST), 0);  
  board->pwr_set(MB_PWR_ID_VCC_LED, 0);

  /* 关闭IO */
  epd_drv_io_deinit(p);
}

#if 0
#define EPD_RST POS_PA11
#define EPD_RST POS_PA11
#define EPD_CS  POS_PB12
#define EPD_MOSI POS_PB13
#define EPD_SCK
void EPD_Reset(epd_param_t *p) {
  p->drv.os->gpio->set(EPD_RST, 0);
  p->drv.os->util->udelay(10000);
  p->drv.os->gpio->set(EPD_RST, 1);
  p->drv.os->util->udelay(10000);  
}

void EPD_Init(epd_param_t *p) {
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

void EPD_GPIO_Config(epd_param_t *p) {
}

void EPD_Main(epd_param_t *p) {
  EPD_GPIO_Config(p);
}

#endif

