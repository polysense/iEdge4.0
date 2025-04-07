/**
 * @file  drv_co2z19.c
 * @brief SENSOR Z19 CO2 driver
 * @author Runby F.
 * @date 2022-12-19
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * Revision history
v1.3 @ 20240517
  1) Support Z19B CO2 sensor
  
*/

/**
 * @brief Driver version
 */
#define DRV_CO2Z19_VERSION 0x0100

/**
 * @brief Driver name
 */
#define DRV_CO2Z19_NAME  "CO2Z19"

/**
 * @brief Operating timeout
 */
#define DRV_UART_IO_TIMEOUT  5000

/*
 * @brief CO2Z19 Structure
 */
typedef struct {
  pos_u32_t     feed_timeout;  /**< Feed auto-calib timeout time */    
  pos_u16_t     rsvd;  
  pos_u8_t      init_flag;
  pos_u8_t      state_len; /**< store sensor state data */
  pos_u8_t      state[32];
} co2z19_param_t;

/** 
 * Sleep without system deep sleep so that PWM can still work
 */
void sensor_co2z19_sleep(pos_u32_t ticks) {
  const pos_lib_tick_t *t;  
  t = g_drv->os->tick;
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
pos_status_t sensor_co2z19_set(pos_u32_t on) {  
  pos_gpio_pin_t pwr;
  drv_api_t *drv = g_drv;
  ma_board_t *b = drv->board;
  pwr = (drv->s->slot->io >> 8) & 0x7f;  
  if( !pwr )
    pwr = MB_PWR_ID_VCC_PWRH;

  b->pwr_set(pwr, on);
  
  return POS_STATUS_OK;
}


/** 
* Sensor console flush
*/ 
void sensor_co2z19_io_flush(pos_io_handle_t *io) {
  while( io->read(io, POS_NULL, 1024, 20) >= 1024 );   
}

/** 
* Sensor command buffer write
*/ 
pos_u32_t sensor_co2z19_send_buf(pos_io_handle_t *io, const pos_u8_t *buf, pos_u32_t buf_len, pos_u8_t *response, pos_u32_t res_len) {
  pos_u8_t c;
  pos_u32_t v, i;

  if( g_drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    g_drv->log->buf("co2z19 wr", (void*)buf, buf_len);
    g_drv->os->flush();
  }

  /* 清空串口*/
  sensor_co2z19_io_flush(io);
    
  io->write(io, (pos_u8_t*)buf, buf_len, 100);
  if( !response )
    return 0;
  v = io->read(io, response, 1, 950);
  if( v ) {
    if( response[0] != 0xff )
      return 0;
    v += io->read(io, &response[1], res_len-1, 50);
#if 1
    if( g_drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      g_drv->log->buf("co2z19 rd", response, v);
    }
#endif      
    c = 0;
    for( i = 1; i < v; i++ ) {
      c += response[i];
    }
    if( c != 0 )
      v = 0; /* force V zero indicating failed crc */      
  }
    
  
  return v;
}


pos_u32_t sensor_co2z19_send(pos_io_handle_t *io, pos_u8_t cmd, pos_u8_t arg, pos_u8_t *response, pos_u32_t res_len) {
  union {
    pos_u8_t buf[9];
    pos_u32_t u32[3];
  } u; 
    
  u.buf[0] = 0xff;
  u.buf[1] = 0x01;
  u.buf[2] = cmd;
  u.buf[3] = arg;
  u.u32[1] = 0;
  u.buf[8] = 0xff - cmd - arg;
  
  return sensor_co2z19_send_buf(io, u.buf, 9, response, res_len);
}

/** 
* Sensor caliration
*/ 
pos_status_t sensor_co2z19_calib(pos_io_handle_t *io, pos_u8_t addr, pos_u8_t style, pos_u8_t *response) {
  pos_u8_t v;
  pos_u32_t t;
  drv_api_t *drv = g_drv;  
  pos_status_t ret = POS_STATUS_ERROR;
  switch( style ) {
  case 1: /* use 1 by default */
  default:
    /* Factory calibration */
    t = drv->os->tick->get() + 10000; /* 10s at most */
    do {
      v = sensor_co2z19_send(io, 0x87, 0, response, 9);
      if( v == 9 ) {
        ret = POS_STATUS_OK;
        break; /* Bit[2]: Factory calibration restored flag */
      }
      drv->os->tick->sleep(1000);
    } while( !drv->os->tick->is_timeout(drv->os->tick->get(), t));    
    break;
  }
  return ret;
}

/** 
* Sensor collecting
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_co2z19_collect(void){
  pos_io_handle_t *io;
  pos_status_t ret = POS_STATUS_E_TIMEOUT;
  pos_u32_t v, t, b, sum, cnt, min, max, longsleep;
  drv_api_t *drv = g_drv;
  pos_u8_t response[9], addr;
  co2z19_param_t *param;

  /* restore param */
  param = (co2z19_param_t*)drv->s->drv_buf;  
  
  /* 初始化串口 */
  t = drv->s->slot->io;
  b = drv->s->slot->rsvd32;
  addr = 0x68; /* fixed to use mbus addr 0x68 */
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("co2z19 io", t);        
    drv->log->data("co2z19 rsvd", b);    
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
    /* calib if b is set */
    if( (b & 0xff) == 1 ) {
      sensor_co2z19_calib(io, addr, b&0xff, response);
      v = b & 0xffffff00;
      t = (pos_u32_t)&drv->s->slot->rsvd32 - MA_EEPROM_ADDR;
      drv->eeprom->update(t, &v, 4);
     if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {      
       drv->log->data("co2z19 calib ofs", t);
     }
    }

    /* setup timeout using AVG sampling durations */
    t = drv->os->tick->get();  
    v = (b >> 8) & 0xff;
    if( !v )
      v = 10; /* 10s default */
    t += DRV_UART_IO_TIMEOUT + (v)*1000; 
    do {
      /* Step 5: read co2 data */
      v = sensor_co2z19_send(io, 0x86, 0, response, 9);     
      /* data is invalid, skip this round */      
      if( v != 9 ) {
        continue;
      }

      /* Process co2 value */      
      v = response[2]*256 + response[3]; /* save co2 */
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

      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {      
        drv->log->data("co2z19 v", v);

      }

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

      /* sleep if required */
      v = (b>>16)&0xff;
      if( v )
        v = v * 1000;
      else
        v = 1000;
      drv->os->tick->sleep(v);
      
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
    if( param->init_flag == 0 ) {
      param->init_flag = 1; /* clear first done */
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
    drv->log->data("co2z19 delta", drv->os->util->abs(drv->history->mv[7] - v));      
    drv->log->data("co2z19 deltat", t); 
#endif    
    n = drv->os->util->abs(drv->history->mv[7] - v);
    if( t && n >= t  ) {
      longsleep = 0;      
    } else if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {      
      drv->log->data("co2z19 diff", n);
    }     
    drv->history->mv[7] = v;  /* use #7 for CO2 PPM */
    drv->history->rsvd[1] |= 1; /* set refresh flag */

    /* disable long when value is bigger than valid absolute thld */
    t = drv->s->slot->thld.u16[0];
    if( t && v >= t ) {
      longsleep = 0;           
    } else if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {      
      drv->log->data("co2z19 thld", t);
    }     

    /* always force to use longsleep when out of timerange */
    if( !longsleep && drv->history->time.year ) { /* when year is not zero, indicating time sycned; or else, skip below when not synce */
      t = ( drv->history->time.hour << 8 ) + drv->history->time.minute;
      if( t <= drv->s->slot->thld.u16[2] || t >= drv->s->slot->thld.u16[3] ) {
        if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {      
          drv->log->data("co2z19 tm", t);
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
     drv->log->data("co2z19 avg", cnt);
     drv->log->data("co2z19 max", max);
     drv->log->data("co2z19 min", min);    
     if( t < 100 )
      drv->log->data("co2z19 raw", sum); 
     drv->log->data("co2z19 lsleep", longsleep);
     drv->os->flush();
    }

  } else {
    /* refresh to ERROR stat */
    drv->s->ctrl |= MA_SENSOR_CTRL_STAT_ERROR;
    drv->log->data("co2z19 err", drv->s->slot->io);
    /* 错误的Sensor, 上报出错 */
    drv->data->put("\"%s\":\"no co2z19#0x%x\",", "err", drv->s->slot->io, 0, 0, 0);
  }

  if( io ) {      
    /* shutdown IO after read */
    io->release(io);
  }
  return ret;
}



/** 
 * Driver init
 * @param[in]  load   0:Unload driver, 1:Load driver
 * @return     0: Successful\n
             !=0: Failed, refer to @ref pos_status_t
 * @note This function can be NULL if it does NOT require any init operatoin
 */ 
pos_status_t sensor_co2z19_init(pos_u32_t load) {
  drv_api_t *drv = g_drv;
  ma_sensor_ctrl_t *s = g_drv->s;  
  pos_gpio_pin_t en, rdy;
  co2z19_param_t *param;  
  en = drv->board->pin_map(MB_PT_CTRL,MB_CTRL_ID_UDF3);
  rdy = drv->board->pin_map(MB_PT_CTRL,MB_CTRL_ID_UDF2);  
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("co2z19 init", load);
  }  
  
  if( load ) {
    drv->os->gpio->mode_set(rdy, POS_GPIO_MODE_INPUT);  
    drv->os->gpio->mode_set(en, POS_GPIO_MODE_OUTPUT);
    drv->os->gpio->polarity_set(en, 0); /* disable en by default */    

    param = drv->os->malloc(sizeof(*param));
    if( !param ) {
      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
        drv->log->data("co2z19 mem error", sizeof(*param)); 
      }        
      return POS_STATUS_E_MEM;
    }
    drv->os->memset(param, 0, sizeof(*param));   
    param->state[1] = 1;
    param->state_len = 2;
    param->feed_timeout = 3600000; /* increase ABC hour by next hours */
    s->drv_buf = param;  /* record to drv_buf */       
  } else {
    drv->os->gpio->mode_set(rdy, POS_GPIO_MODE_ANALOG);  
    drv->os->gpio->mode_set(en, POS_GPIO_MODE_ANALOG);  
    if( s->drv_buf ) { 
      /* Free mem */
      drv->os->free(drv->s->drv_buf);
      s->drv_buf = POS_NULL;
    }
  }

  return POS_STATUS_OK;
}

/*
 * Driver export
 */
ma_drv_export_t g_export = {
  .version = DRV_SENSOR_VERSION(MA_SENSOR_CO2Z19, DRV_CO2Z19_VERSION),
  .name = DRV_CO2Z19_NAME,
  .u.sensor={
    .init = sensor_co2z19_init,
    .power_set = sensor_co2z19_set,
    .collect = sensor_co2z19_collect,
  },
};

