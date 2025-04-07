/**
 * @file  drv_co2.c
 * @brief SENSOR COZIR CO2 driver
 * @author Runby F.
 * @date 2022-12-19
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * 版本修订历史
v1.0 @ 2022-12-19
  1) Support COZIR CO2 sensor by default

v1.1 @ 2022-12-20
  1) Support COZIR CO2 G calib through srsv = 1

v1.2 @ 2022-12-21
  1) Support BTN push triggered X calibration (regular clean fresh air)

v1.3 @ 2023-02-02
  1) Support record co2/ppm to history mv#7

v1.4 @ 2023-08-26
  1) Use explict IO init uart ctrl field

v1.5 @ 2023-11-18
  1) Using new PWR ID solution
  
*/

/**
 * @brief Driver version
 */
#define DRV_CO2_VERSION 0x0105

/**
 * @brief Driver name
 */
#define DRV_CO2_NAME  "CO2"

/**
 * @brief Operating timeout
 */
#define DRV_UART_IO_TIMEOUT  5000

/*
 * @brief CO2 IRQ Structure
 */
typedef struct {
  drv_api_t     drv;      /**< 驱动克隆，指向固定SLOT的驱动指针 */
  pos_u32_t     irq_ticks;    /**< 中断时刻 */
  pos_u32_t     calib_timeout;  /**< Calib timeout time */  
  pos_u32_t     baud;     /**< IO Baudrate */
  pos_u8_t      io_id;    /**< IO ID */
  pos_u8_t      pin;      /**< 按钮管脚 */
} co2_param_t;

/** 
 * Sleep without system deep sleep so that PWM can still work
 */
void sensor_co2_sleep(co2_param_t *p, pos_u32_t ticks) {
  const pos_lib_tick_t *t;  
  t = p->drv.os->tick;
  ticks = t->get() + ticks;
  while( !t->is_timeout(t->get(), ticks) ) {
    t->sleep(20);
  }
}

/** 
* Power control
* @param[in]  on   0:Power off, 1:Power on
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
* @note If sensor does NOT require any power control, this fucntion can be NULL
*/ 
pos_status_t sensor_co2_set(pos_u32_t on) {  
  pos_gpio_pin_t pwr;
  ma_board_t *b = g_drv->board;
  pwr = (g_drv->s->slot->io >> 8) & 0x7f;  
  if( !pwr )
    pwr = MB_PWR_ID_VCCN_PWR3;

  b->pwr_set(pwr, on);
  
  return POS_STATUS_OK;
}

/** 
 * Driver init
 * @param[in]  load   0:Unload driver, 1:Load driver
 * @return     0: Successful\n
             !=0: Failed, refer to @ref pos_status_t
 * @note This function can be NULL if it does NOT require any init operatoin
 */ 
pos_status_t sensor_co2_init(pos_u32_t load) {
  drv_api_t *drv = g_drv;
  co2_param_t *param;
  ma_sensor_ctrl_t *s = g_drv->s;
  pos_gpio_pin_t pin;
  pin = drv->board->pin_map(MB_PT_DIO,MB_DIO_ID_BTN);
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("co2 init", load);
  }  
  
  if( load ) {
    s->data.u32 = 0; /* clear counter by default */
    param = drv->os->malloc(sizeof(*param));
    if( !param )
      return POS_STATUS_E_MEM;
    drv->os->memset(param, 0, sizeof(*param));
    
    param->pin = pin;
    param->baud = s->slot->rsvd32 >> 8;
    param->io_id = s->slot->io;
    param->calib_timeout = drv->os->tick->get() + 60000; /* 60s */
    
    drv->os->memcpy(&param->drv, drv, sizeof(*drv)); /* clone a g_drv */    
    s->drv_buf = param;  /* record to drv_buf */
    
    /* 
     * turn on irq
     * warning: as GOT is implemented in DATA and DATA is not supported in mmap()
     * all GOT based pointer can NOT be used. So the IRQ SET should use the mapped
     * function pointer here (the export structure). 
     * the orignal sensor_fastcnt_irq() should NOT be used directly.
     */     
    drv->os->gpio->irq_set(
      pin, 
      POS_GPIO_MODE_INPUT_PU+POS_GPIO_MODE_IRQ_FALLING,
      (pos_func_t)drv->s->drv.u.sensor.irq_cb, /* has to be this. DO NOT use sensor_fastcnt_irq */
      param);

    /*
     * turn on poll
     */
    drv->proc->event_register((pos_func_t)drv->s->drv.u.sensor.poll_cb, param);

    /* 
     * Init ready BEEP 
     */      
    drv->board->beep_set(2700);
    sensor_co2_sleep(param, 2000);
    drv->board->beep_set(0);    
    
  } else {      
    /*
     * turn off poll
     */
    drv->proc->event_unregister((pos_func_t)drv->s->drv.u.sensor.poll_cb, s->drv_buf);

    /* 
     * turn off irq
     */
    drv->os->gpio->irq_set(pin, POS_GPIO_MODE_INPUT_PU+POS_GPIO_MODE_IRQ_DISABLE, POS_NULL, POS_NULL);
    if( s->drv_buf ) {      
      drv->os->free(drv->s->drv_buf);
      s->drv_buf = POS_NULL;
    }
  }
  return POS_STATUS_OK;
}

pos_status_t sensor_co2_calib(pos_io_handle_t *io) {
  pos_u32_t i,j, v;
  pos_u8_t response[32];
  /* 清空串口并等待50ms */
  while( io->read(io, POS_NULL, 1024, 50) >= 1024 ); 

  for( i = 0; i < 3; i++ ) {
    /* calib */
    io->write(io, (pos_u8_t*)"X 420\r\n", 7, 100);   /* jason suggests to use 420 @ 20221221 */

    for( j = 0; j < 10; j++ ) {
      v = io->read(io, response, sizeof(response), 100);
      if( response[v-1] != '\r' && response[v-1] != '\n' )
        continue;
      if( (response[0] == ' ' && response[1] == 'X') || response[0] == 'X' ) {
        return POS_STATUS_OK;
      }
    }
  }  
  return POS_STATUS_ERROR;
}

/** 
* Sensor collecting
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_co2_collect(void){
  pos_io_handle_t *io;
  pos_status_t ret = POS_STATUS_E_TIMEOUT;
  pos_u32_t v, t, b;
  drv_api_t *drv = g_drv;
  pos_u8_t response[32];
  co2_param_t *param;

  /* DISABLE IRQ when calib butten is timeout */
  param = (co2_param_t*)drv->s->drv_buf;
  if( param ) {
    if( drv->os->tick->is_timeout(drv->os->tick->get(), param->calib_timeout ) ) {    
      param->irq_ticks = 0;
      sensor_co2_init(0); /* unload IRQ handler */
    }
  }
  
  /* 初始化串口 */
  t = drv->s->slot->io;
  b = drv->s->slot->rsvd32;
  
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("co2 io", t);        
    drv->log->data("co2 baud", b>>8);    
  }

  t &= 0xff; /* keep io for uart only */
  
  io = POS_NULL;
  if( POS_IO_IS_SERIAL_PORT(t) ) {
    v = (b >> 8)&0xffffff;
    if( !v )
      v = 9600; /* 默认用9600 */
    io = drv->os->io->init(t, v, 0);
  }

  if( io ) {    
    /* 清空串口并等待50ms */
    while( io->read(io, POS_NULL, 1024, 50) >= 1024 ); 

    /* calib if b is set */
    if( (b & 0xff) == 1 ) {
      sensor_co2_calib(io);
      v = b & 0xffffff00;
      t = (pos_u32_t)&drv->s->slot->rsvd32 - MA_EEPROM_ADDR;
      g_drv->eeprom->update(t, &v, 4);
     if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {      
       drv->log->data("co2 calib ofs", t);
     }
    }
    
    b = drv->os->tick->get();  
    b += DRV_UART_IO_TIMEOUT;
    do {
      if( (v=io->read(io, response, sizeof(response), 100)) < 5 )
        continue;
      /* drv->log->buf("rx", response, v);*/
      if( response[0] != ' ' || response[1] != 'Z' || response[2] != ' ' || response[3] < '0' || response[3] > '9' )
        continue;
      if( response[v-1] != '\r' && response[v-1] != '\n' )
        continue;
      
//    drv->log->data((const char*)response,0);
      
      v = drv->os->util->str2u32((const char*)&response[3]);
      drv->history->mv[7] = v;  /* use #7 for CO2 PPM */

      drv->data->plss_put_u16(40, v); /* PLSS data format put, 40 for PLSS_PL_EXT_CO2_PPM: U16 */        
      ret = drv->data->put("\"%s\":%u,", "co2", v, 0, 0, 0);
      break;      
    } while( !drv->os->tick->is_timeout(drv->os->tick->get(), b) );   /* repeat until timeout */     
  } 

  if( ret != POS_STATUS_OK ) {
    drv->log->data("co2 err", t);
    /* 错误的Sensor, 上报出错 */
    drv->data->put("\"%s\":\"no co2#0x%x\",", "err", t, 0, 0, 0);
  }

  if( io ) {      
    /* shutdown IO after read */
    io->release(io);
  }
  return ret;
}



/** 
 * IRQ interrupt callback handler
 * @note This function can NOT call g_drv. When IRQ happening, g_drv might piont to any sensor drivers. It should call the param stored driver pointer.
 */
void sensor_co2_irq(co2_param_t *param) {
  pos_u32_t ticks;
  ticks = param->drv.os->tick->get();
  if( !ticks )
    ticks = 1; /* 至少是1 */
  param->irq_ticks = ticks; /* 记录中断时刻 */
  param->drv.os->gpio->irq_clr(param->pin);
}


/** 
 * Polling callback function
 * @note This function can NOT call g_drv. When polling, g_drv might piont to any sensor drivers. It should call the param stored driver pointer.
 */
void sensor_co2_poll(co2_param_t *p) {
  pos_u32_t t, baud, b0, b1, led;
  pos_gpio_pin_t pwr;
  pos_io_handle_t *io;  
  ma_board_t *b = p->drv.board;
  const pos_lib_tick_t *tick;
  
  /* 若从无中断则不处理任何内容 */
  if( !p->irq_ticks ) {
    return;
  }  

  /* do NOT response when calib-ticks is timeout */
  tick = p->drv.os->tick;
  if( tick->is_timeout(tick->get(), p->calib_timeout ) ) {
    p->irq_ticks = 0;
    return;
  }
  
  t = p->io_id&0xff; /* keep io for uart only */
  baud = p->baud;
  if( !baud )
    baud = 9600;
  
  if( p->drv.cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    p->drv.log->data("co2 calib io", t);        
    //p->drv.log->data("co2 bdrate", baud); 
  }
    
  /* power on module */
  pwr = (p->io_id >> 8) & 0x7f;  
  if( !pwr )
    pwr = MB_PWR_ID_VCCN_PWR3;
  b->pwr_set(pwr, 1);

  /* power on delay */
  p->drv.os->tick->sleep(1000);

  /* init io */
  io = POS_NULL;
  if( POS_IO_IS_SERIAL_PORT(t) ) {
    io = p->drv.os->io->init(t, baud, 0);
  }
  t = 0;
  if( io ) {
    if( sensor_co2_calib(io) == POS_STATUS_OK ) {
      t = 1;
    }
  }

  if( p->drv.cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    p->drv.log->data("co2 ok", t);    
  }
  if( t ) {
    /* 成功蜂鸣 */    
    led = MB_PIN_LED_G(p->drv.os);
    b0 = 1350;
    b1 = 2700;
  } else {
    /* Failed Beep */  
    led = MB_PIN_LED_R(p->drv.os);
    b0 = 2700;
    b1 = 1350;  
  }

  MB_PIN_LED_SET(p->drv.os, led, 1);    
  b->beep_set(b0);
  sensor_co2_sleep(p, 500);
  b->beep_set(b1);
  sensor_co2_sleep(p, 500);    
  MB_PIN_LED_SET(p->drv.os, led, 0);
  b->beep_set(0);        

  /* release io */
  if( io ) {
   io->release(io);    
  }
  
  /* power off module */
  b->pwr_set(pwr, 0);   

  /* 清除中断标志 */
  p->irq_ticks = 0;  
  
}


/*
 * Driver export
 */
ma_drv_export_t g_export = {
  .version = DRV_SENSOR_VERSION(MA_SENSOR_CO2, DRV_CO2_VERSION),
  .name = DRV_CO2_NAME,
  .u.sensor={
    .init = sensor_co2_init,
    .power_set = sensor_co2_set,
    .collect = sensor_co2_collect,
    .irq_cb = (pos_func_t)sensor_co2_irq,
    .poll_cb = (pos_func_t)sensor_co2_poll,     
  },
};

