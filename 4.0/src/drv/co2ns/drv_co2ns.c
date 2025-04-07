/**
 * @file  drv_co2ns.c
 * @brief SENSOR NS CO2 driver
 * @author Runby F.
 * @date 2022-12-19
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * 版本修订历史
v1.0 @ 2023-2-2
  1) Support NS CO2 sensor

v1.1 @ 2023-2-28
  1) Remove button trigger calibration by default
  2) Support AVG sampling 
  3) Support turn on auto calib and feeding by 30min interval
  4) Support Sliding filter

v1.2 @ 2023-3-17
  1) Support data re-calibation by manual offset (arg.i16[0]: positive/increase, negative/decrease)

v1.3 @ 2023-3-17
  1) Support data ratio adjustment (arg.u16[1]: final=(data+offset)*ratio/100)

v1.4 @ 2023-3-25  
  1) Fix sometime all zero content will return 0 issue

v1.5 @ 2023-5-08
  1) According to new HW, use CTRL3 to control EN pin

v1.6 @ 2023-07-23
  1) Use json_put for generic and onenet support
  2) Support long cycle scheduling mechanisms

v1.7 @ 2023-07-25
  1) Enhance debug
  2) Fix first time always use long sleep issue

v1.8 @ 2023-08-26
  1) Support RX/TX output low when power down
  2) Use explict IO init uart ctrl field  

v1.9 @ 2023-10-24
  1) Support stat normal/error

v1.10 @ 2023-11-18
  1) Using new PWR ID solution

v1.11 @ 2023-11-21
  1) Fix moving ratio address overlapping issue

v1.12 @ 20240229
  1) Support to set refresh flag indicating co2 updating

v1.13 @ 20240320
  1) Stop collecting when vbat < 3.0

*/

/**
 * @brief Driver version
 */
#define DRV_CO2NS_VERSION 0x010d

/**
 * @brief Driver name
 */
#define DRV_CO2NS_NAME  "CO2NS"

/**
 * @brief Operating timeout
 */
#define DRV_UART_IO_TIMEOUT  5000

/*
 * @brief CO2NS IRQ Structure
 */
typedef struct {
  drv_api_t     drv;      /**< 驱动克隆，指向固定SLOT的驱动指针 */
  pos_u32_t     irq_ticks;    /**< 中断时刻 */
  pos_u32_t     calib_timeout;  /**< Calib timeout time */  
  pos_u32_t     feed_timeout;  /**< Feed auto-calib timeout time */    
  pos_u32_t     baud;     /**< IO Baudrate */
  pos_u8_t      io_id;    /**< IO ID */
  pos_u8_t      pin;      /**< 按钮管脚 */
  pos_u8_t      rdy;      /**< rdy pin */
  pos_u8_t      rsvd1;
} co2ns_param_t;

/** 
 * Sleep without system deep sleep so that PWM can still work
 */
void sensor_co2ns_sleep(co2ns_param_t *p, pos_u32_t ticks) {
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
pos_status_t sensor_co2ns_set(pos_u32_t on) {  
  pos_gpio_pin_t pwr, en;
  drv_api_t *drv = g_drv;
  ma_board_t *b = drv->board;
  pwr = (drv->s->slot->io >> 8) & 0x7f;  
  if( !pwr )
    pwr = MB_PWR_ID_VCCN_PWR3;

  b->pwr_set(pwr, on);

  /* control EN through CTRL3 */
  /* accorging to jason@20230508
   * CTRL3 (PF0) is used to control EN 
   */
  en = drv->board->pin_map(MB_PT_CTRL,MB_CTRL_ID_UDF3);
  if( on ) {
    drv->os->gpio->mode_set(en, POS_GPIO_MODE_OUTPUT);
  } else {
    /* according to 20230826 debug on new a7 board
     * co2ns rx/tx pin has be output low, or else it will be abnormal in next round of running
     */
    /* output uart tx/rx to low */
    pos_u8_t ii, io_id;
    /* set rx/tx to low */
    io_id = drv->s->slot->io&0xff; /* uart io id */
    if( io_id < POS_IO_I2C0 ) {
      for( ii = 0; ii < 2; ii++ ) {
        pos_u8_t _pin;
        _pin = drv->os->pin->io[io_id*4+ii]; /* tx/rx uart pin */
        drv->os->gpio->mode_set(_pin, POS_GPIO_MODE_OUTPUT);
        drv->os->gpio->set(_pin, 0);
      }
    }    
  }
  drv->os->gpio->polarity_set(en, on);
  
  return POS_STATUS_OK;
}

/** 
 * Calib BTN IRQ set
 */
void sensor_co2ns_calib_irq_set(co2ns_param_t *p, pos_u32_t on) {
  drv_api_t *drv = &p->drv;

  if( on ) {
    /* 
     * turn on irq
     * warning: as GOT is implemented in DATA and DATA is not supported in mmap()
     * all GOT based pointer can NOT be used. So the IRQ SET should use the mapped
     * function pointer here (the export structure). 
     * the orignal sensor_fastcnt_irq() should NOT be used directly.
     */     
    drv->os->gpio->irq_set(
      p->pin, 
      POS_GPIO_MODE_INPUT_PU+POS_GPIO_MODE_IRQ_FALLING,
      (pos_func_t)drv->s->drv.u.sensor.irq_cb, /* has to be this. DO NOT use sensor_fastcnt_irq */
      p);
    
    /*
     * turn on poll
     */
    drv->proc->event_register((pos_func_t)drv->s->drv.u.sensor.poll_cb, p);
  } else {
    /*
     * turn off poll
     */
    drv->proc->event_unregister((pos_func_t)drv->s->drv.u.sensor.poll_cb, p);

    /* 
     * turn off irq
     */
    drv->os->gpio->irq_set(p->pin, POS_GPIO_MODE_INPUT_PU+POS_GPIO_MODE_IRQ_DISABLE, POS_NULL, POS_NULL);  
  }
}

/** 
 * Driver init
 * @param[in]  load   0:Unload driver, 1:Load driver
 * @return     0: Successful\n
             !=0: Failed, refer to @ref pos_status_t
 * @note This function can be NULL if it does NOT require any init operatoin
 */ 
pos_status_t sensor_co2ns_init(pos_u32_t load) {
  drv_api_t *drv = g_drv;
  co2ns_param_t *param;
  ma_sensor_ctrl_t *s = g_drv->s;
  pos_gpio_pin_t pin, rdy;
  pin = drv->board->pin_map(MB_PT_DIO,MB_DIO_ID_BTN);
  rdy = drv->board->pin_map(MB_PT_CTRL,MB_CTRL_ID_UDF2);  
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("co2ns init", load);
  }  
  
  if( load ) {
    s->data.u32 = 0; /* clear counter by default */
    param = drv->os->malloc(sizeof(*param));
    if( !param ) {
      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
        drv->log->data("co2ns mem error", sizeof(*param)); 
      }        
      return POS_STATUS_E_MEM;
    }
    drv->os->memset(param, 0, sizeof(*param));

    param->pin = pin;
    param->rdy = rdy;
    param->baud = s->slot->rsvd32 >> 8;
    param->io_id = s->slot->io;
    param->calib_timeout = drv->os->tick->get() + 60000; /* 60s */

    drv->os->memcpy(&param->drv, drv, sizeof(*drv)); /* clone a g_drv */    
    s->drv_buf = param;  /* record to drv_buf */    

    /* Init RDY */
    drv->os->gpio->mode_set(rdy, POS_GPIO_MODE_INPUT);

    /* if flag is not set, do NOT call IRQ calib trigger */
    if( (s->slot->rsvd32 & 0x80000000) != 0 ) {
      /* Turn ON Calib IRQ */
      sensor_co2ns_calib_irq_set(param, 1);

      /* 
       * Init ready BEEP 
       */      
      drv->board->beep_set(2700);
      sensor_co2ns_sleep(param, 2000);
      drv->board->beep_set(0);    
    }    
  } else if( s->drv_buf ) { 
    /* Turn OFF Calib IRQ */
    sensor_co2ns_calib_irq_set((co2ns_param_t *)s->drv_buf, 0); 

    /* Free mem */
    drv->os->free(drv->s->drv_buf);
    s->drv_buf = POS_NULL;
  }
  return POS_STATUS_OK;
}

pos_status_t sensor_co2ns_calib(pos_io_handle_t *io) {
  pos_u32_t i,j, v;
  pos_u8_t response[32];
  /* 清空串口并等待50ms */
  while( io->read(io, POS_NULL, 1024, 50) >= 1024 ); 

  for( i = 0; i < 3; i++ ) {
    const pos_u8_t cmd_calib[6] = {0x11, 0x03, 0x03, 0x01, 0x90, 0x58, }; // 400 PPM calib
    /* calib */
    io->write(io, (pos_u8_t*)cmd_calib, 6, 100); 

    for( j = 0; j < 10; j++ ) {      
      v = io->read(io, response, sizeof(response), 100);
      if( v != 4 )
        continue;
      if( response[0] == 0x16 || 
          response[1] == 0x01 ||
          response[2] == 0x03 ||
          response[3] == 0xe6 ) {
        return POS_STATUS_OK;
      }
    }
  }  
  return POS_STATUS_ERROR;
}

void sensor_co2ns_io_flush(pos_io_handle_t *io) {
  /* 清空串口并等待20ms */
  while( io->read(io, POS_NULL, 1024, 20) >= 1024 );   
}

pos_u32_t sensor_co2ns_send_buf(pos_io_handle_t *io, const pos_u8_t *buf, pos_u32_t buf_len, pos_u8_t *response, pos_u32_t res_len) {
  pos_u8_t c;
  pos_u32_t v, i;

  /* 清空串口*/
  sensor_co2ns_io_flush(io);
    
  io->write(io, (pos_u8_t*)buf, buf_len, 100);
  v = io->read(io, response, 1, 950);
  if( v ) {
    v += io->read(io, &response[1], res_len-1, 50);
    c = 0;
    for( i = 0; i < v; i++ ) {
      c += response[i];
    }
    if( c != 0 )
      v = 0; /* force V zero indicating failed crc */    
    if( v < 4 || response[1] != v - 3 )
      v = 0; /* Second byte (#1) should be payload length (V-3) */
  }
  return v;
}

pos_u32_t sensor_co2ns_send(pos_io_handle_t *io, pos_u8_t cmd, pos_u8_t *response, pos_u32_t res_len) {
  pos_u8_t buf[4];
  
    
  buf[0] = 0x11;
  buf[1] = 0x01;
  buf[2] = cmd;
  buf[3] = 0 - buf[0] - buf[1] - buf[2];
  return sensor_co2ns_send_buf(io, buf, 4, response, res_len);
}

/** 
* Sensor collecting
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_co2ns_collect(void){
  pos_io_handle_t *io;
  pos_status_t ret = POS_STATUS_E_TIMEOUT;
  pos_u32_t v, t, b, sum, cnt, min, max, longsleep;
  drv_api_t *drv = g_drv;
  pos_u8_t response[32];
  co2ns_param_t *param;

  /* DISABLE IRQ when calib butten is timeout */
  param = (co2ns_param_t*)drv->s->drv_buf;
  if( param ) {
    if( drv->os->tick->is_timeout(drv->os->tick->get(), param->calib_timeout ) ) {    
      param->irq_ticks = 0;
      sensor_co2ns_calib_irq_set(param, 0); /* unload IRQ handler */
    }
  }
  
  /* 初始化串口 */
  t = drv->s->slot->io;
  b = drv->s->slot->rsvd32;
  
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("co2ns io", t);        
    drv->log->data("co2ns rsvd", b);    
  }

  t &= 0xff; /* keep io for uart only */
  
  io = POS_NULL;
  if( POS_IO_IS_SERIAL_PORT(t) ) {
    io = drv->os->io->init(t, 9600, 0); /* 默认用9600 */
  }

  /* prepare avg variable */
  cnt = 0; 
  max = 0;
  min = 0xffffffff;
  sum = 0;

  /* refresh to normal by default */
  drv->s->ctrl |= MA_SENSOR_CTRL_STAT_NORMAL;
  drv->s->ctrl &= ~MA_SENSOR_CTRL_STAT_ERROR;

  /* stop collect when vbat < 3.0 */
  if( drv->history->mv[0] && drv->history->mv[0] < 3000 ) {
    return POS_STATUS_OK;
  }
  
  if( io ) {    
#if 1
    /* working status debug/check */
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {             
      const pos_u8_t pool[] = {0x51, 0x0f, 0x50};
      /*
      1602510097
            00 Work Mode (SingleShot)
      */
      /* 
      16070f64000201906479
              00 AutoCalib
                02 Calib Cycles (??2 Days??)
                  0190 Calib 400PPM
      */
      /*
      1604500078011d
            0078 SamplePeriod 120s
                01 MovingDataCnt 1
      */

      for( t = 0; t < 3; t++ ) {
        v = sensor_co2ns_send(io, pool[t], response, sizeof(response));
        drv->log->buf("co2ns dbg", response, v); 
        drv->os->tick->sleep(500);      
      }
    }
#endif
    /* work mode setup */
    v = sensor_co2ns_send(io, 0x0f, response, sizeof(response));
    if( v != 10 || response[4] != 0x00 || response[5] != 0x07 ) {
      const pos_u8_t cmd_mode[] = {0x11, 0x07, 0x10, 0x64, 0x00, 0x07, 0x01, 0x90, 0x64, 0x78};
      v = sensor_co2ns_send_buf(io, cmd_mode, 10, response, sizeof(response));
      if( v && (drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG) != 0 ) {
        drv->log->buf("co2ns mod", response, v);         
      }
    }
    
    /* calib if b is set */
    if( (b & 0xff) == 1 ) {
      sensor_co2ns_calib(io);
      v = b & 0xffffff00;
      t = (pos_u32_t)&drv->s->slot->rsvd32 - MA_EEPROM_ADDR;
      g_drv->eeprom->update(t, &v, 4);
     if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {      
       drv->log->data("co2ns calib ofs", t);
     }
    }

    /* setup timeout using AVG sampling durations */
    t = drv->os->tick->get();  
    v = (b >> 8) & 0xff;
    if( !v )
      v = 10; /* 10s default */
    t += DRV_UART_IO_TIMEOUT + (v)*1000; 
    do {
      /* wait 5s ready */
      for( v = 0; v<50; v++ ) {
        if( drv->os->gpio->get(param->rdy) == 0 ) 
          break; /* ready and quit */
        /* wait until ready */
        drv->os->tick->sleep(100);
      }
      if( v >= 50 )
        break; /* sensor wrong */
        
      v = sensor_co2ns_send(io, 0x01, response, sizeof(response));
#if 0      
      if( v ) {        
        if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {              
          drv->log->buf("co2ns buf", response, v);  
        }
      }
#endif      
      if( v < 6 ) {
        drv->os->tick->sleep(200);
        continue;
      }
      v = response[3] * 256 + response[4];
      /* data offset calib */
      if( drv->s->slot->arg.u16[0] ) {
        v += drv->s->slot->arg.u16[0];
        v &= 0xffff;
        if( v > 32767 )
          v = 0; /* at least report zero if negative */
      }
      /* data ratio calib */
      if( drv->s->slot->arg.u16[1] ) {
        v = v * drv->s->slot->arg.u16[1];
        v /= 100;
        if( v > 0xffff )
          v = 0xffff;
      }
#if 1
      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {      
        drv->log->data("co2ns rd", v);

      }
#endif
      sum += v;
      cnt++;
      if( v > max )
        max = v;
      if( v < min )
        min = v;

      /* if not AVG, return immediately */
      if( (b & 0xff00) == 0x100 )
        break;

      /* remove the TIMEOUT guard time for first valid data sampling */      
      if( cnt == 1 )
        t -= DRV_UART_IO_TIMEOUT;

      /* shutdown and trigger next */
      /* uart must be released as power down will set rx/tx to output low */
      io->release(io); /* release io */

      sensor_co2ns_set(0); /* power down */

      drv->os->tick->sleep(250);      

      /* restore IO before power up again */
      io->setup(io, 9600, 0); /* 默认用9600 */ 
      
      sensor_co2ns_set(1); /* power up */
      
      drv->os->tick->sleep(750);

      /* sleep if required */
      if( b & 0xff0000 )
        drv->os->tick->sleep(((b>>16)&0xff)*1000);
    } while( !drv->os->tick->is_timeout(drv->os->tick->get(), t) );   /* repeat until timeout */     
  } 

  /* init longsleep by default */  
  longsleep = 1; 
  if( cnt ) {
    int d, h, n;
    if( cnt > 2 )
      v = (sum - min - max)/(cnt-2);
    else
      v = sum/cnt;
    sum = v; /* store original value in sum */
    t = ((b >> 24) & 0x1f)*5; /* unit of 5% */
    if( !t )
      t = 50;
    if( param->rsvd1 == 0 ) {
      param->rsvd1 = 1; /* clear first done */
      /* keep v as it is */      
    } else if( t < 100 ) {
      /* perform sliding */
      h = drv->history->mv[7]; /* last data */
      n = v; /* new data */
      d = n - h; /* delta */     
      
      d *= t;
      t /= 2;
      if( d < 0 )
        d -= t; /* round up by -t% */
      else
        d += t; /* round up by +t% */
      n = h + d / 100; /* new = history + delta * ratio */
      v = n;
    }

  
    /* disable long when delta is bigger than valid delta thld */
    t = drv->s->slot->thld.u16[1];
#if 0    
    drv->log->data("co2ns delta", drv->os->util->abs(drv->history->mv[7] - v));      
    drv->log->data("co2ns deltat", t); 
#endif    
    n = drv->os->util->abs(drv->history->mv[7] - v);
    if( t && n >= t  ) {
      longsleep = 0;      
    } else if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {      
      drv->log->data("co2ns diff", n);
    }     
    drv->history->mv[7] = v;  /* use #7 for CO2 PPM */
    drv->history->rsvd[1] |= 1; /* set refresh flag */

    /* disable long when value is bigger than valid absolute thld */
    t = drv->s->slot->thld.u16[0];
    if( t && v >= t ) {
      longsleep = 0;           
    } else if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {      
      drv->log->data("co2ns thld", t);
    }     

    /* always force to use longsleep when out of timerange */
    if( !longsleep && drv->history->time.year ) { /* when year is not zero, indicating time sycned; or else, skip below when not synce */
      t = ( drv->history->time.hour << 8 ) + drv->history->time.minute;
      if( t <= drv->s->slot->thld.u16[2] || t >= drv->s->slot->thld.u16[3] ) {
        if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {      
          drv->log->data("co2ns tm", t);
        }        
        longsleep = 1;
      }
    }

    /* sync longsleep if it's defined */
    if( longsleep ) {
      longsleep = drv->s->slot->thld.u16[4] * 1000; 
      drv->call(DRV_CALL_FUNC_ID_SET_DUTY_MS, longsleep);
    }
    
    ret = drv->data->plss_json_put(v, 
      DRV_MA_PLSS_TRANS(PLSS_PL_EXT_CO2_PPM, 2, 0, 0, 0),
      "co2");
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {            
     drv->log->data("co2ns avg", cnt);
     drv->log->data("co2ns max", max);
     drv->log->data("co2ns min", min);    
     if( t < 100 )
      drv->log->data("co2ns raw", sum); 
     drv->log->data("co2ns lsleep", longsleep);
    }

    /* feed auto-calib */
    t = drv->os->tick->get();
    if( (b & 0x20000000) != 0 )
      v = 1; /* auto calib on each duty */
    else {
      v = drv->os->tick->is_timeout(t, param->feed_timeout); /* auto calib when timeout */
    }
    /* always timeout the time record even when auto calib is disabled,
    * so that the timeline is nearest and will not be rollover by a long gap */
    while( drv->os->tick->is_timeout(t, param->feed_timeout) ) {
      param->feed_timeout += 1800000; /* round to next 1800s interval */
    }

    /* perform auto calib record when not disabled and timeout cycle reached */ 
    if(  v && (b & 0x40000000) == 0 ) {
      /* auto calib record */
      v = sensor_co2ns_send(io, 0x11, response, sizeof(response));
      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
        drv->log->buf("co2ns feed", response, v);        
      }      
      /* sleep 100ms for auto-calib feed done. Vendor said 0ms is okay, but we leave 100ms here. */
      drv->os->tick->sleep(100);
    }
  } else {
    /* refresh to ERROR stat */
    drv->s->ctrl |= MA_SENSOR_CTRL_STAT_ERROR;
    drv->log->data("co2ns err", drv->s->slot->io);
    /* 错误的Sensor, 上报出错 */
    drv->data->put("\"%s\":\"no co2ns#0x%x\",", "err", drv->s->slot->io, 0, 0, 0);
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
void sensor_co2ns_irq(co2ns_param_t *param) {
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
void sensor_co2ns_poll(co2ns_param_t *p) {
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
    p->drv.log->data("co2ns calib io", t);        
    //p->drv.log->data("co2ns bdrate", baud); 
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
    if( sensor_co2ns_calib(io) == POS_STATUS_OK ) {
      t = 1;
    }
  }

  if( p->drv.cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    p->drv.log->data("co2ns ok", t);    
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
  sensor_co2ns_sleep(p, 500);
  b->beep_set(b1);
  sensor_co2ns_sleep(p, 500);    
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
  .version = DRV_SENSOR_VERSION(MA_SENSOR_CO2NS, DRV_CO2NS_VERSION),
  .name = DRV_CO2NS_NAME,
  .u.sensor={
    .init = sensor_co2ns_init,
    .power_set = sensor_co2ns_set,
    .collect = sensor_co2ns_collect,
    .irq_cb = (pos_func_t)sensor_co2ns_irq,
    .poll_cb = (pos_func_t)sensor_co2ns_poll,     
  },
};

