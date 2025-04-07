/**
 * @file  drv_co2ss.c
 * @brief SENSOR SS CO2 driver
 * @author Runby F.
 * @date 2022-12-19
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * Revision history
v1.0 @ 2023-12-27
  1) Support SS CO2 sensor

v1.1 @ 20240229
  1) Support to set refresh flag indicating co2 updating

v1.2 @ 20240319
  1) Support 48H ABC calibration mechanism
  2) Enhance start single procedure

v1.3 @ 20240320
  1) Stop collecting when vbat < 3.0
  
*/

/**
 * @brief Driver version
 */
#define DRV_CO2SS_VERSION 0x0103

/**
 * @brief Driver name
 */
#define DRV_CO2SS_NAME  "CO2SS"

/**
 * @brief Operating timeout
 */
#define DRV_UART_IO_TIMEOUT  5000

/*
 * @brief CO2SS Structure
 */
typedef struct {
  pos_u32_t     feed_timeout;  /**< Feed auto-calib timeout time */    
  pos_u16_t     rsvd;  
  pos_u8_t      init_flag;
  pos_u8_t      state_len; /**< store sensor state data */
  pos_u8_t      state[32];
} co2ss_param_t;

/** 
 * Sleep without system deep sleep so that PWM can still work
 */
void sensor_co2ss_sleep(pos_u32_t ticks) {
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
pos_status_t sensor_co2ss_set(pos_u32_t on) {  
  pos_gpio_pin_t pwr;
  drv_api_t *drv = g_drv;
  ma_board_t *b = drv->board;
  pwr = (drv->s->slot->io >> 8) & 0x7f;  
  if( !pwr )
    pwr = MB_PWR_ID_VCCN_PWR3;

  b->pwr_set(pwr, on);

  if( on ) {
  } else {
    /* according to 20230826 debug on new a7 board
     * co2ss rx/tx pin has be output low, or else it will be abnormal in next round of running
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
  
  return POS_STATUS_OK;
}


/** 
* Sensor console flush
*/ 
void sensor_co2ss_io_flush(pos_io_handle_t *io) {
  while( io->read(io, POS_NULL, 1024, 20) >= 1024 );   
}

/** 
* Sensor command buffer write
*/ 
pos_u32_t sensor_co2ss_write(pos_io_handle_t *io, pos_u8_t addr, pos_u8_t func, pos_u16_t reg, pos_u8_t *buf, pos_u32_t buf_len, pos_u8_t *response, pos_u32_t res_len) {
  pos_u8_t cmd[50];
  pos_u32_t v, i;
  pos_u16_t crc16;
  drv_api_t *drv = g_drv;  
  if( buf_len < 2 || buf_len > 40 || (buf_len & 0x1)) {    
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG )
      drv->log->buf("co2ss tx err", buf, buf_len);  
    return 0;
  }
  for( i = 0; i < 3; i++ ) {
    sensor_co2ss_io_flush(io);
    cmd[0] = addr;
    cmd[1] = func;
    cmd[2] = (reg >> 8)&0xff;
    cmd[3] = reg&0xff;
    cmd[4] = 0;
    cmd[5] = buf_len/2;
    cmd[6] = buf_len;
    drv->os->memcpy(&cmd[7], buf, buf_len);
    buf_len += 7;
    crc16 = drv->os->crc(POS_CRC16_MBUS, cmd , buf_len);
    cmd[buf_len++] = crc16 & 0xff;
    cmd[buf_len++] = (crc16>>8) & 0xff;
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      drv->log->buf("co2ss wr", cmd, buf_len);     
      drv->os->flush(); /* flush debug before read/write */
    }    
    io->write(io, cmd, buf_len, 100);
    v = io->read(io, response, res_len, 950);
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG )
      drv->log->buf("co2ss rd", response, v);         
    if( v > 2 ) {
      crc16 = drv->os->crc(POS_CRC16_MBUS, response , v-2);
      if( crc16 == (response[v-1]<<8)+response[v-2] ) /* crc16 用LE节序存放 */
          return v; /* successful return */
    }
  }
  return 0;
}

/** 
* Sensor record state
*/ 
pos_u32_t sensor_co2ss_record_state(pos_io_handle_t *io, co2ss_param_t *param, pos_u8_t addr, pos_u8_t *response) {
  pos_u32_t v;
  v = g_drv->data->modbus_read(io, response, addr, 3, 0x0022, 0x0c);
  if( v != 29 )
    return 0;
  
  g_drv->os->memcpy(&param->state[2], &response[3], 24);
  param->state_len = 24+2;
  return 26;
}

/** 
* Sensor start measurement
*/ 
pos_u32_t sensor_co2ss_start(pos_io_handle_t *io, co2ss_param_t *param, pos_u8_t addr, pos_u8_t *response) {
  pos_u32_t v, t;
  drv_api_t *drv = g_drv;
  if( param->state_len != 26 ) {
    /* first time load init state data */
    v = sensor_co2ss_write(io, addr, 0x10, 0x21, param->state, 2, response, 8);
    if( v == 8 ) {
      sensor_co2ss_record_state(io, param, addr, response);
    }
    return 0; /* always return 0 for first time */
  } 

  /* auto increase ABC auto-calib hours */
  t = drv->os->tick->get();
  v = drv->os->tick->is_timeout(t, param->feed_timeout); /* auto calib hour increasing when timeout */

  /* always timeout the time record even when auto calib is disabled,
  * so that the timeline is nearest and will not be rollover by a long gap */
  while( drv->os->tick->is_timeout(t, param->feed_timeout) ) {
    param->feed_timeout += 3600000; /* round to next 3600s interval */
  }

  /* increase hours when 1H timeout flag is set */
  if( v ) {
    param->state[1] += 1;
    if( !param->state[1] )
      param->state[0] += 1;
  }

  /* write state data */
  v = sensor_co2ss_write(io, addr, 0x10, 0x21, param->state, 26, response, 8);
  if( v != 8 ) {
    return 0;
  }

  return 8; /* successful response */
}

/** 
* Sensor caliration
*/ 
pos_status_t sensor_co2ss_calib(pos_io_handle_t *io, pos_u8_t addr, pos_u8_t style, pos_u8_t *response) {
  pos_u8_t cmd[8], v;
  pos_u32_t t;
  drv_api_t *drv = g_drv;  
  pos_status_t ret = POS_STATUS_ERROR;
  switch( style ) {
  case 1: /* use 1 by default */
  default:
    cmd[0] = 0x7c;
    cmd[1] = 0x02; /* Factory calibration */
    v = sensor_co2ss_write(io, addr, 0x10, 0x01, cmd, 2, response, 8);
    if( v != 8 ) {
      break;
    }
    t = drv->os->tick->get() + 10000; /* 10s at most */
    do {
      v = drv->data->modbus_read(io, response, addr, 3, 0, 1);
      if( v == 7 ) {
        if( response[4] & 0x4 ) {
          ret = POS_STATUS_OK;
          break; /* Bit[2]: Factory calibration restored flag */
        }
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
pos_status_t sensor_co2ss_collect(void){
  pos_gpio_pin_t en, rdy;
  pos_io_handle_t *io;
  pos_status_t ret = POS_STATUS_E_TIMEOUT;
  pos_u32_t v, t, b, sum, cnt, min, max, longsleep, tmp;
  drv_api_t *drv = g_drv;
  pos_u8_t response[48], addr;
  co2ss_param_t *param;

  /* restore param */
  param = (co2ss_param_t*)drv->s->drv_buf;  
  en = drv->board->pin_map(MB_PT_CTRL,MB_CTRL_ID_UDF3);
  rdy = drv->board->pin_map(MB_PT_CTRL,MB_CTRL_ID_UDF2);  
  
  /* 初始化串口 */
  t = drv->s->slot->io;
  b = drv->s->slot->rsvd32;
  addr = 0x68; /* fixed to use mbus addr 0x68 */
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("co2ss io", t);        
    drv->log->data("co2ss rsvd", b);    
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
      /* turn on en before calib */
      drv->os->gpio->polarity_set(en, 1); 
      sensor_co2ss_sleep(40);      
      sensor_co2ss_calib(io, addr, b&0xff, response);
      v = b & 0xffffff00;
      t = (pos_u32_t)&drv->s->slot->rsvd32 - MA_EEPROM_ADDR;
      drv->eeprom->update(t, &v, 4);
     if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {      
       drv->log->data("co2ss calib ofs", t);
     }
    }

    /* turn ON ABC by 48H */
    {
      v = drv->data->modbus_read(io, response, addr, 3, 0, 0x14);
      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
        drv->log->buf("co2ss cfg", response, v);
      }
      if( v == 5 + 0x14*2 ) {
        v = response[3+0x0d*2] * 256 + response[3+0x0d*2+1];
        /* set 0x0D ABC Period to 4 if it's not 48 */
        if( v != 48 ) {
          response[0] = 0;
          response[1] = 0x30;
          sensor_co2ss_write(io, addr, 0x10, 0x0d, response, 2, response, 8);
        }
      }
    }

    /* setup timeout using AVG sampling durations */
    t = drv->os->tick->get();  
    v = (b >> 8) & 0xff;
    if( !v )
      v = 10; /* 10s default */
    t += DRV_UART_IO_TIMEOUT + (v)*1000; 
    do {
      /* Step 0/7: drive EN low */
      drv->os->gpio->polarity_set(en, 0); /* disable en by default */  
      sensor_co2ss_sleep(40);
      
      /* Step 1: drive EN high */
      drv->os->gpio->polarity_set(en, 1); 

      /* Step 2: wait >35 ms */
      sensor_co2ss_sleep(40);

      /* Step 3: write start measurement command */
      if( sensor_co2ss_start(io, param, addr, response) != 8 )
        continue;
      
      /* Step 4: wait at most 5s ready */
      for( v = 0; v<50; v++ ) {
        if( drv->os->gpio->get(rdy) == 0 ) 
          break; /* ready and quit */
        /* wait until ready */
        drv->os->tick->sleep(100);
      }
      if( v >= 50 )
        break; /* sensor wrong */

      /* Step 5: read co2 data */
      v = drv->data->modbus_read(io, response, addr, 0x04, 0, 4);
      tmp = response[9]*256 + response[10]; /* save co2 as response will be destroyed by step 6 */

      /* step 6: read state for next */
      sensor_co2ss_record_state(io, param, addr, response);
      
      /* data is invalid, skip this round */      
      if( v != 13 ) {
        continue;
      }

      /* Process co2 value */      
      v = tmp;
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
        drv->log->data("co2ss v", v);

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
    drv->log->data("co2ss delta", drv->os->util->abs(drv->history->mv[7] - v));      
    drv->log->data("co2ss deltat", t); 
#endif    
    n = drv->os->util->abs(drv->history->mv[7] - v);
    if( t && n >= t  ) {
      longsleep = 0;      
    } else if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {      
      drv->log->data("co2ss diff", n);
    }     
    drv->history->mv[7] = v;  /* use #7 for CO2 PPM */
    drv->history->rsvd[1] |= 1; /* set refresh flag */

    /* disable long when value is bigger than valid absolute thld */
    t = drv->s->slot->thld.u16[0];
    if( t && v >= t ) {
      longsleep = 0;           
    } else if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {      
      drv->log->data("co2ss thld", t);
    }     

    /* always force to use longsleep when out of timerange */
    if( !longsleep && drv->history->time.year ) { /* when year is not zero, indicating time sycned; or else, skip below when not synce */
      t = ( drv->history->time.hour << 8 ) + drv->history->time.minute;
      if( t <= drv->s->slot->thld.u16[2] || t >= drv->s->slot->thld.u16[3] ) {
        if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {      
          drv->log->data("co2ss tm", t);
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
     drv->log->data("co2ss avg", cnt);
     drv->log->data("co2ss max", max);
     drv->log->data("co2ss min", min);    
     if( t < 100 )
      drv->log->data("co2ss raw", sum); 
     drv->log->data("co2ss lsleep", longsleep);
     drv->os->flush();
    }

#if 0 /* auto-calib is not supported now */
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
      v = sensor_co2ss_send(io, 0x11, response, sizeof(response));
      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
        drv->log->buf("co2ss feed", response, v);        
      }      
      /* sleep 100ms for auto-calib feed done. Vendor said 0ms is okay, but we leave 100ms here. */
      drv->os->tick->sleep(100);
    }
#endif

  } else {
    /* refresh to ERROR stat */
    drv->s->ctrl |= MA_SENSOR_CTRL_STAT_ERROR;
    drv->log->data("co2ss err", drv->s->slot->io);
    /* 错误的Sensor, 上报出错 */
    drv->data->put("\"%s\":\"no co2ss#0x%x\",", "err", drv->s->slot->io, 0, 0, 0);
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
pos_status_t sensor_co2ss_init(pos_u32_t load) {
  drv_api_t *drv = g_drv;
  ma_sensor_ctrl_t *s = g_drv->s;  
  pos_gpio_pin_t en, rdy;
  co2ss_param_t *param;  
  en = drv->board->pin_map(MB_PT_CTRL,MB_CTRL_ID_UDF3);
  rdy = drv->board->pin_map(MB_PT_CTRL,MB_CTRL_ID_UDF2);  
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("co2ss init", load);
  }  
  
  if( load ) {
    drv->os->gpio->mode_set(rdy, POS_GPIO_MODE_INPUT);  
    drv->os->gpio->mode_set(en, POS_GPIO_MODE_OUTPUT);
    drv->os->gpio->polarity_set(en, 0); /* disable en by default */    

    param = drv->os->malloc(sizeof(*param));
    if( !param ) {
      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
        drv->log->data("co2ss mem error", sizeof(*param)); 
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
  .version = DRV_SENSOR_VERSION(MA_SENSOR_CO2SS, DRV_CO2SS_VERSION),
  .name = DRV_CO2SS_NAME,
  .u.sensor={
    .init = sensor_co2ss_init,
    .power_set = sensor_co2ss_set,
    .collect = sensor_co2ss_collect,
  },
};

