/**
 * @file  drv_fs801.c
 * @brief SENSOR NS CO2 driver
 * @author Runby F.
 * @date 2022-12-19
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * 版本修订历史
v1.0 @ 2024-2-6
  1) Support FS801 sensor
  2) Support skipping co2=0 

*/

/**
 * @brief Driver version
 */
#define DRV_FS801_VERSION 0x0100

/**
 * @brief Driver name
 */
#define DRV_FS801_NAME  "FS801"

/**
 * @brief Operating timeout
 */
#define DRV_UART_IO_TIMEOUT  10000

/*
 * @brief FS801 IRQ Structure
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
} fs801_param_t;

/** 
 * Sleep without system deep sleep so that PWM can still work
 */
void sensor_fs801_sleep(fs801_param_t *p, pos_u32_t ticks) {
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
pos_status_t sensor_fs801_set(pos_u32_t on) {  
  pos_gpio_pin_t pwr;
  drv_api_t *drv = g_drv;
  ma_board_t *b = drv->board;
  pwr = (drv->s->slot->io >> 8) & 0x7f;  
  if( !pwr )
    pwr = MB_PWR_ID_PWRH_PWR3;

  b->pwr_set(pwr, on);
  
  return POS_STATUS_OK;
}

pos_status_t sensor_fs801_calib(pos_io_handle_t *io) {
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

void sensor_fs801_io_flush(pos_io_handle_t *io) {
  /* 清空串口并等待20ms */
  while( io->read(io, POS_NULL, 1024, 20) >= 1024 );   
}

pos_u32_t sensor_fs801_send_buf(pos_io_handle_t *io, const pos_u8_t *buf, pos_u32_t buf_len, pos_u8_t *response, pos_u32_t res_len) {
  pos_u8_t c;
  pos_u32_t v, i;

  /* 清空串口*/
  sensor_fs801_io_flush(io);
    
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

pos_u32_t sensor_fs801_send(pos_io_handle_t *io, pos_u8_t cmd, pos_u8_t *response, pos_u32_t res_len) {
  pos_u8_t buf[6];
  pos_u32_t v;  
    
  buf[0] = 0x11;
  buf[1] = 0x02;
  buf[2] = cmd;
  buf[3] = 0;
  buf[4] = 0 - buf[0] - buf[1] - buf[2] - buf[3];  
  v = sensor_fs801_send_buf(io, buf, 5, response, res_len);
  if( g_drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {              
    g_drv->log->buf("fs801 tx", buf, 5);
    g_drv->log->buf("fs801 rx", response, v);
  }    
  return v;
}

/** 
* Sensor collecting
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_fs801_collect(void){
  pos_io_handle_t *io;
  pos_status_t ret = POS_STATUS_E_TIMEOUT;
  pos_u32_t v, t, b, sum, cnt, min, max, longsleep;
  drv_api_t *drv = g_drv;
  pos_u8_t response[32];
  
  /* 初始化串口 */
  t = drv->s->slot->io;
  b = drv->s->slot->rsvd32;
  
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("fs801 io", t);        
    drv->log->data("fs801 rsvd", b);    
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
  
  if( io ) {        
    /* calib if b is set */
    if( (b & 0xff) == 1 ) {
      t = sensor_fs801_calib(io);
      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {      
        drv->log->data("fs801 calib", t);
      }      
      v = b & 0xffffff00;
      t = (pos_u32_t)&drv->s->slot->rsvd32 - MA_EEPROM_ADDR;
      g_drv->eeprom->update(t, &v, 4);
    }

    /* setup timeout using AVG sampling durations */
    t = drv->os->tick->get();  
    v = (b >> 8) & 0xff;
    t += DRV_UART_IO_TIMEOUT + (v)*1000; 
    do {        
      v = sensor_fs801_send(io, 0x01, response, sizeof(response));      
      if( v < 20 ) {
        drv->os->tick->sleep(200);
        continue;
      }
      v = response[3] * 256 + response[4];
      if( v == 0 )
        continue; /* skip all zero co2 */
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
        drv->log->data("fs801 co2", v);

      }
#endif
      sum += v;
      cnt++;
      if( v > max )
        max = v;
      if( v < min )
        min = v;

      /* if not AVG, return immediately */
      if( (b & 0xff00) <= 0x100 )
        break;

      /* remove the TIMEOUT guard time for first valid data sampling */      
      if( cnt == 1 )
        t -= DRV_UART_IO_TIMEOUT;

      /* sleep if required */
      if( b & 0xff0000 )
        drv->os->tick->sleep(((b>>16)&0xff)*1000);
      else
        drv->os->tick->sleep(200); /* minimal sleep */
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
      t = 100; /* default do not use sliding */
    if( drv->history->mv[7] && t < 100 ) {
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
    drv->log->data("fs801 delta", drv->os->util->abs(drv->history->mv[7] - v));      
    drv->log->data("fs801 deltat", t); 
#endif    
    n = drv->os->util->abs(drv->history->mv[7] - v);
    if( t && n >= t  ) {
      longsleep = 0;      
    } else if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {      
      drv->log->data("fs801 diff", n);
    }     
    drv->history->mv[7] = v;  /* use #7 for CO2 PPM */

    /* disable long when value is bigger than valid absolute thld */
    t = drv->s->slot->thld.u16[0];
    if( t && v >= t ) {
      longsleep = 0;           
    } else if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {      
      drv->log->data("fs801 thld", t);
    }     

    /* always force to use longsleep when out of timerange */
    if( !longsleep && drv->history->time.year ) { /* when year is not zero, indicating time sycned; or else, skip below when not synce */
      t = ( drv->history->time.hour << 8 ) + drv->history->time.minute;
      if( t <= drv->s->slot->thld.u16[2] || t >= drv->s->slot->thld.u16[3] ) {
        if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {      
          drv->log->data("fs801 tm", t);
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
     drv->log->data("fs801 avg", cnt);
     drv->log->data("fs801 max", max);
     drv->log->data("fs801 min", min);    
     if( t < 100 )
      drv->log->data("fs801 raw", sum); 
     drv->log->data("fs801 lsleep", longsleep);
    }

    /* report ch2o */
    v = response[5] * 256 + response[6];
    ret = drv->data->plss_json_put(v, 
      DRV_MA_PLSS_TRANS(PLSS_PL_EXT_CH2O, 2, 0, 0, 0),
      "ch2o");

    /* report humdity */
    v = response[7] * 256 + response[8];
    ret = drv->data->plss_json_put(v, 
      DRV_MA_PLSS_TRANS(PLSS_PL_HUMIDITY_HR, 2, 0, 0, 1),
      "hum");

    /* report temperature */
    v = response[9] * 256 + response[10];
    v -= 500;
    ret = drv->data->plss_json_puti((pos_i32_t)v, 
      DRV_MA_PLSS_TRANS(PLSS_PL_TEMPERATURE, 2, 0, 0, 1),
      "tmp");

    /* report pm2.5/10/1.0 (payload/response is pm2.5/1.0/10) */
    v = response[11] * 256 + response[12];
    drv->data->plss_json_put(v,
      DRV_MA_PLSS_TRANS(PLSS_PL_EXT_PM, 2, 0, 0, 0),
      "pm2_5");
    drv->data->plss_put_raw(2, &response[15]); /* pm10 */
    drv->data->plss_put_raw(2, &response[13]); /* pm1.0 */
    drv->data->put("\"%s\":%u,", "pm10", response[15]*256+response[16], 0, 0, 0);
    drv->data->put("\"%s\":%u,", "pm1", response[13]*256+response[14], 0, 0, 0);     
    
    /* report voc */
    v = response[17] * 256 + response[18];
    ret = drv->data->plss_json_puti((pos_i32_t)v, 
      DRV_MA_PLSS_TRANS(PLSS_PL_EXT_VOC, 2, 0, 0, 0),
      "voc");
    
  } else {
    /* refresh to ERROR stat */
    drv->s->ctrl |= MA_SENSOR_CTRL_STAT_ERROR;
    drv->log->data("fs801 err", drv->s->slot->io);
    /* 错误的Sensor, 上报出错 */
    drv->data->put("\"%s\":\"no fs801#0x%x\",", "err", drv->s->slot->io, 0, 0, 0);
  }

  if( io ) {      
    /* shutdown IO after read */
    io->release(io);
  }
  return ret;
}

/*
 * Driver export
 */
ma_drv_export_t g_export = {
  .version = DRV_SENSOR_VERSION(MA_SENSOR_FS801, DRV_FS801_VERSION),
  .name = DRV_FS801_NAME,
  .u.sensor={
    .power_set = sensor_fs801_set,
    .collect = sensor_fs801_collect,
  },
};

