/**
 * @file  drv_rfid.c
 * @brief SENSOR UltraHighSpeed RFID driver
 * @author Runby F.
 * @date 2022-12-19
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * 版本修订历史
v1.0 @ 2025-2-22
  1) Support UltraHighSpeed RFID sensor by default

v1.1 @ 2025-3-12
  1) Fix TID length too short issue
  2) Optimize first ready reading wait time
  3) Remove unnecessary json report err
  
*/

/**
 * @brief Driver version
 */
#define DRV_RFID_VERSION 0x0101

/**
 * @brief Driver name
 */
#define DRV_RFID_NAME  "RFID"

/**
 * @brief Operating timeout
 */
#define DRV_UART_IO_TIMEOUT  1000

/**
 * @brief Restart timeout
 */
#define DRV_RFID_RESTART_TIMEOUT  120000

/*
 * @brief RFID IRQ Structure
 */
typedef struct {
  drv_api_t     drv;      /**< 驱动克隆，指向固定SLOT的驱动指针 */
  pos_u32_t     baud;     /**< IO Baudrate */
  pos_io_handle_t *io;    /**< IO drv pointer */
  pos_u32_t     restart_to; /**< restart_timeout */
  pos_u16_t     addr16;   /**< Sensor 584 address */
  pos_u8_t      io_id;    /**< IO ID */
  pos_u8_t      mode;     /**< working mode */
  pos_u8_t      buf[256+1500]; /**< running buffer */
} rfid_param_t;

/** 
 * Sleep without system deep sleep so that PWM can still work
 */
void sensor_rfid_sleep(rfid_param_t *p, pos_u32_t ticks) {
  const pos_lib_tick_t *t;  
  t = p->drv.os->tick;
  ticks = t->get() + ticks;
  while( !t->is_timeout(t->get(), ticks) ) {
    t->sleep(20);
  }
}

/** 
* sensor crc8 get
* @param[in]  data: cmd data pointer
* @param[in]  data_len: cmd data length
* @return     0~255: CRC8
*/ 
pos_u8_t sensor_rfid_crc8_get(pos_u8_t *data, pos_u8_t data_len) {
  pos_u8_t c8 = 0;
  while( data_len-- )
    c8 -= *data++;
  return c8;
}

/** 
* sensor command send
* @param[in]  param: sensor param pointer
* @param[in]  cmd: cmd byte
* @param[in]  data: cmd data pointer
* @param[in]  data_len: cmd data length
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_rfid_send(rfid_param_t *param, pos_u8_t cmd, pos_u8_t *data, pos_size_t data_len) {
  pos_u8_t buf[32], len;
  drv_api_t *drv;
  drv = &param->drv;  
  if( data_len > sizeof(buf)-6 )
    data_len = sizeof(buf-6);
  buf[0] = 0xd9;
  len = 4 + data_len;
  buf[1] = len;
  buf[2] = (param->addr16 >> 8)&0xff;
  buf[3] = param->addr16 & 0xff;
  buf[4] = cmd;
  if( data_len )
    drv->os->memcpy(&buf[5], data, data_len);
  len++; /* 4* -> 5* */
  buf[len] = sensor_rfid_crc8_get(buf, len);
  len++; /* 5* -> 6* */

  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->buf("==>", buf, len);
    drv->os->flush();
  }
  param->io->write(param->io, buf, len, 100);
  return POS_STATUS_OK;
}

/** 
* sensor read
* @param[in]  param: sensor param pointer
* @param[in]  data: recv data buffer
* @param[in]  data_len: data buffer length
* @param[in]  timeout: read timeout is ms
* @return     0~0xffffffff: Read content length
*/ 
pos_size_t sensor_rfid_recv(rfid_param_t *param, pos_u8_t *data, pos_size_t data_len, pos_u32_t timeout) {
  drv_api_t *drv;
  drv = &param->drv;  
  data_len = param->io->read(param->io, data, data_len, timeout);
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->buf("<==", data, data_len);
    drv->os->flush();
  }

  if( data_len ) {
    pos_u8_t c8;
    c8 = sensor_rfid_crc8_get(data, data_len-1);
    if( data[data_len-1] != c8 ) {
      data_len = 0; /* force zero when CRC8 is wrong */
      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
        drv->log->data("err8", c8);
      }
    }
  }

  return data_len;
}

/** 
* sensor search&read mode
* @param[in]  param: sensor param pointer
* @param[in]  data: recv data buffer
* @param[in]  data_len: data buffer length
* @param[in]  timeout: read timeout is ms
* @return     0~0xffffffff: Read content length
*/ 
pos_size_t sensor_rfid_recv_search(rfid_param_t *param, pos_u8_t *data, pos_size_t data_len, pos_u32_t timeout) {
  drv_api_t *drv;
  pos_u32_t to;
  pos_size_t l;
  pos_u8_t c8;  
  drv = &param->drv;
  to = drv->os->tick->get() + timeout;
  do {
    /* d9 signature chk */
    l = param->io->read(param->io, data, 1, 100);
    if( !l || data[0] != 0xd9 ) 
      continue;

    /* read length */
    l += param->io->read(param->io, &data[1], 1, 100);
    if( l < 2 || data[1] > data_len - 2 || data[1] < 4 )
      continue;

    /* data read chk */
    l += param->io->read(param->io, &data[2], data[1], 100);
    if( l != data[1] + 2 )
      continue;

    /* log data */
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      drv->log->buf("<==", data, l);
      drv->os->flush();
    }

    /* c8 chk */
    c8 = sensor_rfid_crc8_get(data, l-1);
    if( data[l-1] != c8 ) {
      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
        drv->log->data("err8", c8);
        drv->os->flush();
      }
      continue;
    }

    /* return */
    return l;

  } while( !drv->os->tick->is_timeout(drv->os->tick->get(), to) );
  return 0;
}

/**
* sensor command query and response check
* @param[in]  param: sensor param pointer
* @param[in]  cmd: cmd byte
* @param[in]  data: cmd data pointer
* @param[in]  data_len: cmd data length
* @return     0: Successful\n
            !=0: Failed
*/ 
pos_u16_t sensor_rfid_cmd_query(rfid_param_t *param, pos_u8_t cmd, pos_u8_t *data, pos_size_t data_len) {
  pos_u32_t i, len;
  pos_u8_t buf[32];
  pos_u16_t ret = 0xffff;
  for( i = 0 ; i < 3; i++ ) {
    /* flush before cmd */
    while( param->io->read(param->io, POS_NULL, 1024, 5) >= 1024 ); 
    sensor_rfid_send(param, cmd, data, data_len);
    len = sensor_rfid_recv(param, buf, sizeof(buf), DRV_UART_IO_TIMEOUT);
    if( len >= 8 ) {
      ret = (buf[5] << 8) + buf[6];
      break;
    }
  }
  return ret;
}

/** 
* sensor EPC/TID data report
* @param[in]  param: sensor param pointer
* @param[in]  data: data pointer
* @param[in]  data_len: data length
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_rfid_report(rfid_param_t *param, pos_u8_t *data, pos_size_t data_len) {
  pos_status_t ret;
  pos_size_t i, d1, d2;
  drv_api_t *drv;
  drv = &param->drv;
  drv->data->put_raw("\"", 1);
  d1 = data[2];
  for( i = 0; i < d1; i++ ) {
    char s[4];
    drv->os->sprintf(s, "%02X", data[4+i]);
    drv->data->put_raw(s, 2);
  }  
  drv->data->put_raw("/", 1);
  d2 = data[3];
  for( i = 0; i < d2; i++ ) {
    char s[4];
    drv->os->sprintf(s, "%02X", data[4+d1+i]);
    drv->data->put_raw(s, 2);
  }
  drv->data->put_raw("\"", 1);

  ret = drv->data->plss_put_raw(data_len, data);
  return ret;
}

/** 
* Power control
* @param[in]  on   0:Power off, 1:Power on
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
* @note If sensor does NOT require any power control, this fucntion can be NULL
*/ 
pos_status_t sensor_rfid_set(pos_u32_t on) {  
  pos_gpio_pin_t pwr;
  ma_board_t *b = g_drv->board;
  pwr = (g_drv->s->slot->io >> 8) & 0x7f;  
  if( !pwr )
    pwr = MB_PWR_ID_VCC_PWRH;

  b->pwr_set(pwr, on);
  
  return POS_STATUS_OK;
}

pos_status_t sensor_rfid_start(rfid_param_t *param) {
  /* restart EPC+TID when timeout */
  drv_api_t *drv = &param->drv;
  pos_u32_t now;
  now = drv->os->tick->get();
  if( drv->os->tick->is_timeout(now, param->restart_to) ) {
    param->restart_to = now + DRV_RFID_RESTART_TIMEOUT;
    return sensor_rfid_cmd_query(param, 0x2b, POS_NULL, 0);
  } 
  return POS_STATUS_OK;
}

pos_status_t sensor_rfid_stop(rfid_param_t *param) {
  if( !param->mode )
    return POS_STATUS_OK;
  return sensor_rfid_cmd_query(param, 0x2f, POS_NULL, 0);
}

pos_u8_t *sensor_rfid_duplicate_chk(rfid_param_t *param, pos_u8_t *store, pos_size_t store_len, pos_u8_t *data, pos_size_t data_len) {
  pos_size_t l;
//  param->drv.log->buf("chk", data, data_len);  
  while( store_len >= data_len ) {
    l = store[1] + 2;
//    param->drv.log->buf("cmp", store, l);
    if( l == data_len ) {
      if( param->drv.os->memcmp(store, data, data_len) == 0 ) {
//        param->drv.log->data("hit", (pos_u32_t)store);
        return store;
      }
    }

    /* end of data */
    if( l >= store_len )
      break;

    /* move to next */
    store_len -= l;
    store += l;
  }
  return POS_NULL;
}

/** 
 * Driver init
 * @param[in]  load   0:Unload driver, 1:Load driver
 * @return     0: Successful\n
             !=0: Failed, refer to @ref pos_status_t
 * @note This function can be NULL if it does NOT require any init operatoin
 */ 
pos_status_t sensor_rfid_init(pos_u32_t load) {
  drv_api_t *drv = g_drv;
  rfid_param_t *param;
  ma_sensor_ctrl_t *s = g_drv->s;
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("rfid init", load);
  }  
  
  if( load ) {
    param = drv->os->malloc(sizeof(*param));
    if( !param )
      return POS_STATUS_E_MEM;
    drv->os->memset(param, 0, sizeof(*param));
    
    param->baud = s->slot->rsvd32 >> 8;
    param->io_id = s->slot->io;
    param->addr16 = 0x0100; /* default using 01-00 */
    param->restart_to = drv->os->tick->get();

    drv->os->memcpy(&param->drv, drv, sizeof(*drv)); /* clone a g_drv */    
    s->drv_buf = param;  /* record to drv_buf */

#if 0    
    /*
     * turn on poll
     */
    drv->proc->event_register((pos_func_t)drv->s->drv.u.sensor.poll_cb, param);
#endif

    /* 
     * Init ready BEEP 
     */      
    drv->board->beep_set(2700);
    sensor_rfid_sleep(param, 2000);
    drv->board->beep_set(0);    
    
  } else {      
#if 0
    /*
     * turn off poll
     */
    drv->proc->event_unregister((pos_func_t)drv->s->drv.u.sensor.poll_cb, s->drv_buf);

    /* 
     * turn off irq
     */
    drv->os->gpio->irq_set(pin, POS_GPIO_MODE_INPUT_PU+POS_GPIO_MODE_IRQ_DISABLE, POS_NULL, POS_NULL);
#endif

    if( s->drv_buf ) {
      drv->os->free(drv->s->drv_buf);
      s->drv_buf = POS_NULL;
    }
  }
  return POS_STATUS_OK;
}

/** 
* Sensor collecting
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_rfid_collect(void){
  pos_status_t ret = POS_STATUS_E_TIMEOUT;
  pos_u32_t v, t, b, collect_timeout, loop, sum;
  drv_api_t *drv = g_drv;
  pos_u8_t *response, *store, *data;
  rfid_param_t *param;

  /* DISABLE IRQ when calib butten is timeout */
  param = (rfid_param_t*)drv->s->drv_buf;
  
  /* 初始化串口 */
  t = drv->s->slot->io;
  b = drv->s->slot->rsvd32;
  
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("rfid io", t);        
    drv->log->data("rfid baud", b>>8);
    drv->log->data("mtu", drv->ext->mtu);
  }

  t &= 0xff; /* keep io for uart only */
  
  v = (b >> 8)&0xffffff;
  if( !v )
    v = 9600; /* 默认用9600 */
  param->io = drv->os->io->init(t, v, 1);
  param->mode = b & 0xff; /* not used currenty */
  response = param->buf;
  store = &response[256];

  if( param->io ) {
    ret = POS_STATUS_OK;
    loop = 0;
    sum = 0;
    collect_timeout = 0;
    while( param->io ) {
      pos_u32_t l, l2;

      l = drv->os->tick->get(); 
      /* break to report when timeout */
      if( collect_timeout && drv->os->tick->is_timeout(l, collect_timeout) ) {
        break;
      }
      
      /* restart EPC+TID when timeout */
      sensor_rfid_start(param); /* igmore mode and always issue cmd */

      /* scan EPC+TID */      
      v = sensor_rfid_recv_search(param, response, 224, DRV_UART_IO_TIMEOUT);

      if( v < 28 ) {
        /* when stopped, restart */
        if( v == 13 && response[5] == 0x02 ) {
          param->restart_to = drv->os->tick->get();
          sensor_rfid_start(param);
        }
        continue;
      }

      /* skip invalid formats */
      if( v != response[1] + 2 || response[4] != 0x2b )
        continue;

      /* renew restart after each successful reading */
      l = drv->os->tick->get();
      param->restart_to = l + DRV_RFID_RESTART_TIMEOUT;

      /* skip all empty epc+tid */
      if( v == 28 && response[v-2] == 0 )
        continue;

      /* set collect time when first rfid is scanned */      
      if( sum == 0 ) {
        l2 = drv->s->slot->arg.u8[0];
        if( !l2 )
          l2 = 1000;
        else
          l2 *= 1000; /* translate in seconds */
        collect_timeout = l + l2;
        if( !collect_timeout )
          collect_timeout = 1; /* at least 1 */
      }

      /*
       #0     D9
       #1     LEN
       #2..3  01-00
       #4     2B
       #5     FLAGS
       #6     FREQ
       #7     ANT
       #8..9  PC
       #10..10+N-1  N bytes of EPC
       #10+N..11+N CRC16
       #12+N  RSSI16
       #14+N..25+N  TID96 (12 Bytes)
       #26+N  TID_LEN
       #27+N  CRC8
       */
      l = v - 28; /* EPC length */
      l2 = response[v-2]; /* TID_LEN */
      if( l2 != 6 )
        l2 = 0; /* treat zero when invalid TID legnth */
      else
        l2 = 12; /* 6 will always indicate 12 bytes */

      /* break when lengh is full */
      if( sum + 4 + l + l2 + 20 > drv->ext->mtu ) /* give 20 guard chk */
        break;

      /* setup data */
      v = l+l2+2; /* v is RTU payload length only */
      data = &store[sum];
      data[0] = PLSS_PL_RTU_PAYLOAD;
      data[1] = v;
      data[2] = l;
      data[3] = l2;
      drv->os->memcpy(&data[4], &response[10], l); /* EPC value */
      drv->os->memcpy(&data[4+l], &response[14+l], l2); /* TID value */
      v += 2; /* v is full length now */
      /* ignore if duplicated */
      if( sensor_rfid_duplicate_chk(param, store, sum, data, v) != POS_NULL )
        continue;

      if( loop == 0 ) {
        drv->data->put_raw("\"rfids\":\[",9);
      } else {
        drv->data->put_raw(",",1);
      }

      sensor_rfid_report(param, data, v);

      sum += v; 
      loop++;

    } 

    sensor_rfid_stop(param); /* stop */
    if( loop )
      drv->data->put_raw("],",2);

  }


  if( ret != POS_STATUS_OK ) {
    drv->log->data("rfid err", t);
    /* 错误的Sensor, 上报出错 */
    drv->data->put("\"%s\":\"no rfid#0x%x\",", "err", t, 0, 0, 0);
  }

  if( param->io ) {
    /* shutdown IO after read */
    param->io->release(param->io);
  }
  return ret;
}

#if 0
/** 
 * IRQ interrupt callback handler
 * @note This function can NOT call g_drv. When IRQ happening, g_drv might piont to any sensor drivers. It should call the param stored driver pointer.
 */
void sensor_rfid_irq(rfid_param_t *param) {
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
void sensor_rfid_poll(rfid_param_t *p) {
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
    p->drv.log->data("rfid calib io", t);        
    //p->drv.log->data("rfid bdrate", baud); 
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
    if( sensor_rfid_calib(io) == POS_STATUS_OK ) {
      t = 1;
    }
  }

  if( p->drv.cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    p->drv.log->data("rfid ok", t);    
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
  sensor_rfid_sleep(p, 500);
  b->beep_set(b1);
  sensor_rfid_sleep(p, 500);    
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
#endif

/*
 * Driver export
 */
ma_drv_export_t g_export = {
  .version = DRV_SENSOR_VERSION(MA_SENSOR_RFID, DRV_RFID_VERSION),
  .name = DRV_RFID_NAME,
  .u.sensor={
    .init = sensor_rfid_init,
    .power_set = sensor_rfid_set,
    .collect = sensor_rfid_collect,
#if 0
    .irq_cb = (pos_func_t)sensor_rfid_irq,
    .poll_cb = (pos_func_t)sensor_rfid_poll,
#endif
  },
};

