/**
 * @file  drv_ipos.c
 * @brief SENSOR indoor position driver
 * @author Runby F.
 * @date 2022-3-16
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * 版本修订历史
v1.0 @ 2024-07-24
  1) First version ready

v1.1 @ 2024-08-03
  1) support d0.u8[1] for max ssid control (default 0 for 12)

v1.2 @ 2024-08-12
  1) support 60 SSID collecting and multiple report

v1.3 @ 2024-09-18
  1) support default 115200 baudrate
  2) support configurable baudrate by thw0 in unit of 100Hz

v1.4 @ 2024-09-19
  1) Changed to CLIENT mode to fix scan failure issue
  2) Fix multi seg report instable issue

v1.5 @ 2024-09-25
  1) Change POS16 to ID(8)+PKT_NUM(4)+PKT_ID(4) for packet reporting indication

v1.6 @ 2024-09-26
  1) Support d0.u8[2] for max ssid collected for each round
  2) Support to display N omit in dbg/json printing when mtu is too short
*/

/**
 * @brief Driver version
 */
#define DRV_IPOS_VERSION 0x0106

/**
 * @brief Driver name
 */
#define DRV_IPOS_NAME  "IPOS"

/**
 * @brief Driver flag for WIFI MODULE PWR ON control
 */
#define DRV_IPOS_FLAG_PWR_ON       0x01

/**
 * @brief Driver buffer length (actual length will be buffer length+1 for \0 pending)
 */
#define DRV_IPOS_BUF_LEN          2047

/**
 * @brief Driver payload length
 */
#define DRV_IPOS_PAYLOAD_LEN      (2+7*28)

/**
 * @brief MAX SSID count stored for each report rounds
 */
#define DRV_IPOS_MAX_SSID_CNT     80

/**
 * @brief MAX SSID count reported for each packet
 */
#define DRV_IPOS_MAX_SEG_SSID_CNT     32

/**
 * @brief IRQ driver structure
 */
typedef struct {
  drv_api_t     drv;
  pos_u32_t     irq_cnt;
  pos_u32_t     irq_tick;
  pos_u32_t     report_tick;
  pos_u8_t      pin;
  pos_u8_t      pwr;
  pos_u8_t      flag;
  pos_u8_t      gap_seconds;
  pos_u16_t     report_id;
  pos_u16_t     rsvd;
  pos_u16_t     ssid_pos;
  pos_u16_t     ssid_cnt;
  pos_u8_t      ssid[7*DRV_IPOS_MAX_SSID_CNT];
  pos_u8_t      pl_buf[DRV_IPOS_MAX_SEG_SSID_CNT*7+16];
} ipos_param_t;


/** 
* Driver dynamic pwr control
*/
void sensor_ipos_dyn_pwr_cntl(ipos_param_t *iq, pos_u32_t on) {
  if( !on ) {
    if( iq->flag & DRV_IPOS_FLAG_PWR_ON ) {
      iq->drv.board->pwr_set(iq->pwr, 0);
      iq->flag &= ~(DRV_IPOS_FLAG_PWR_ON);
    }
  } else {
    if( (iq->flag & DRV_IPOS_FLAG_PWR_ON) == 0 ) {
      iq->drv.board->pwr_set(iq->pwr, 1);
      iq->flag |= (DRV_IPOS_FLAG_PWR_ON);
    }
  }
}

/** 
* Driver at command send and response read
* @return     0: Nothing read
            !=0: Actual read length
*/
pos_u32_t sensor_ipos_cmd_issue(pos_io_handle_t *io, const char *cmd, pos_u8_t *buf, pos_u32_t buf_max, pos_u32_t timeout) {
  pos_u32_t rd, dbg;
  drv_api_t *drv = g_drv;
  const pos_u8_t ending[] = {'\r', '\n'};
  dbg = drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG;
  if( cmd ) {
    io->read(io, POS_NULL, 10240, 10); /* flush before write */
    if( dbg ) {
      drv->os->printf("==> %s\n", cmd);
      drv->os->flush();
    }
    io->write(io, (pos_u8_t*)cmd, drv->os->strlen(cmd), 1000);
    io->write(io, (pos_u8_t*)ending, 2, 50);
  }
  if( !buf_max )
    return 0;
  if( !timeout )
    timeout = 1000;
  rd = io->read(io, buf, 1, timeout);
  if( rd && rd < buf_max )
    rd += io->read(io, &buf[1], buf_max - rd, 100);
  buf[rd] = 0;
  if( dbg ) {
    drv->os->printf("<== \n");
    drv->os->puts((char*)buf);
    drv->os->flush();
  }
  return rd;
}

/** 
* Driver at command send
* @return     0: OK read
            !=0: ERROR read
*/
pos_u32_t sensor_ipos_rsp_check(pos_u8_t *buf, pos_u32_t buf_len) {
  pos_u32_t ret = 1;
  pos_u8_t *c;
  do {
    if( !buf_len )
      break;
    c = &buf[buf_len-1];
    while( buf_len && (*c == '\n' || *c == '\r') ) {
      c--;
      buf_len--;
    }
    if( buf_len < 2 )
      break;
    if( *c-- != 'K' )
      break;
    if( *c-- != 'O' )
      break;
    buf_len -= 2;
    ret = 0;
  } while(0);
  return ret;
}

/** 
* Driver at command query (retry until OK received)
* @return     0: Nothing read (ok not epxected)
            !=0: Actual read length
*/
pos_u32_t sensor_ipos_cmd_query(pos_io_handle_t *io, const char *cmd, pos_u8_t *buf, pos_u32_t buf_max, pos_u32_t timeout) {
  pos_u32_t rd, i;  
  for( i = 0; i < 3; i++ ) {
    rd = sensor_ipos_cmd_issue(io, cmd, buf, buf_max, timeout);
    if( sensor_ipos_rsp_check(buf, rd) == 0 ) /* OK Detected */
      return rd;
  }
  return 0;
}

/**
* Driver report from buffer pos to pos + max_ssid - 1
*/
pos_status_t sensor_ipos_buf_report(ipos_param_t *iq){
  drv_api_t *drv = &iq->drv;
  pos_status_t ret = POS_STATUS_E_NULL;
  pos_u8_t *pl;
  pos_u32_t i, v, seg_ssid_num;
  if( iq->ssid_pos >= iq->ssid_cnt )
    return ret; /* do NOTHING when report empty */
  seg_ssid_num = drv->s->slot->arg.u8[1];
  if( !seg_ssid_num )
    seg_ssid_num = 12;
  else if( seg_ssid_num > DRV_IPOS_MAX_SEG_SSID_CNT )
    seg_ssid_num = DRV_IPOS_MAX_SEG_SSID_CNT;
  pl = iq->pl_buf;

  if( iq->drv.cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    v = (seg_ssid_num << 16) + (iq->ssid_cnt << 8) + iq->ssid_pos;
    iq->drv.log->data("ipos brep", v);
  }

#if 0
  /* report bmp=0xCCPP (CC is sum of ssid count; PP is current 0 based position) */
  v = (iq->ssid_pos&0xff) + ((iq->ssid_cnt&0xff)<<8);
#else /* define v = REPORT_ID(8) + PKT_NUM(4) + PKT_ID(4) */
      v = (iq->report_id & 0xff) << 8;
      i = (iq->ssid_cnt + seg_ssid_num - 1) / seg_ssid_num;
      v += (i&0xf) << 4;
      i = (iq->ssid_pos + seg_ssid_num - 1) / seg_ssid_num;
      v += i & 0xf;
#endif

  drv->data->plss_json_put(v, DRV_MA_PLSS_TRANS(PLSS_PL_POS_BMP16,2,0,0,0), "bmp");

  /* json array start */
  drv->data->put_raw("\"ssid\":[", 8);

  /* report by segment size */
  do {
    v = seg_ssid_num;
    if( iq->ssid_pos + v > iq->ssid_cnt )
      v = iq->ssid_cnt - iq->ssid_pos;

    pl[0] = PLSS_PL_RTU_PAYLOAD; /* 0x78 */
    pl[1] = v*7;
    drv->os->memcpy(&pl[2], &iq->ssid[iq->ssid_pos * 7], pl[1]);
    ret = drv->data->plss_put_raw(2+pl[1],pl);

    /* report json (ignore return status) */
    for( i = 0; i < v; i++ ) {
      char buf[32];
      drv->os->sprintf(buf, "\"%7b\",", &iq->ssid[(iq->ssid_pos+i)*7]);
      if( drv->data->put_raw(buf, 17) != POS_STATUS_OK ) {
        ret = POS_STATUS_E_RESOURCE;
        break;
      }
      /* only print 10 grp with a omit string when mtu < 256 */
      if( drv->ext->mtu < 256 ) {
        if( i >= 9 && i + 1 < v ) {
          drv->os->sprintf(buf, "\"...%d omit...\",", v-i-1);
          drv->data->put_raw(buf, drv->os->strlen(buf));
          break;
        }
      }
    }

    /* advance ssid pos to next */
    iq->ssid_pos += v;

    if( ret != POS_STATUS_OK && MA_REPORT_TYPE_IS_LORA(drv->ext->report_type) )
      break; /* quit when LORA or NB/WIFI/4G report full */

  } while(0);

  /* json array stop */
  drv->data->put_array_stop();

  return ret;
}

/**
* Driver collecting and report handler
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/
pos_status_t sensor_ipos_duty_report(ipos_param_t *iq){
  drv_api_t *drv = &iq->drv;
  pos_status_t ret;
  pos_io_handle_t *io;
  pos_u8_t *buf, max_ssid_collected;
  pos_u32_t dbg;
  dbg = drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG;
  io = POS_NULL;
  buf = POS_NULL;

  drv->s->ctrl |= MA_SENSOR_CTRL_STAT_NORMAL;
  drv->s->ctrl &= ~MA_SENSOR_CTRL_STAT_ERROR;
  iq->gap_seconds = drv->s->slot->arg.u8[0];
  max_ssid_collected = drv->s->slot->arg.u8[2];
  if( !max_ssid_collected )
    max_ssid_collected = 60;
  if( max_ssid_collected > DRV_IPOS_MAX_SSID_CNT )
    max_ssid_collected = DRV_IPOS_MAX_SSID_CNT;
  if( !iq->gap_seconds )
    iq->gap_seconds = 10;
  iq->report_tick = drv->os->tick->get();
  if( !iq->report_tick )
    iq->report_tick = 1;

  if( dbg ) {
    iq->drv.log->data("ipos", iq->report_tick);
  }
  do {
    pos_u32_t rd, i;
    pos_u8_t *c, *pl_ssid;

    /* malloc buffer */
    buf = drv->os->malloc(DRV_IPOS_BUF_LEN+1);
    if( !buf ) {
      ret = POS_STATUS_E_MEM;
      break;
    }

    /* collect and report */
    iq->report_id++;
    i = drv->s->slot->thld.u16[0]; /* baudrate */
    if( !i ) 
      i = 115200;  /* 默认用9600 */
    else
      i = i * 100; /* unit 100Hz */
    if( dbg ) {
      iq->drv.log->data("baud", i);
      iq->drv.log->data("id", iq->report_id);      
    }
    io = drv->os->io->init(drv->s->slot->io&0x7f, i, 1);
    if( !io ) {
      ret = POS_STATUS_E_RESOURCE;
      break;
    }

    sensor_ipos_dyn_pwr_cntl(iq, 1); /* power on */

    sensor_ipos_cmd_query(io, "ATE0", buf, DRV_IPOS_BUF_LEN, 0);
    sensor_ipos_cmd_query(io, "AT+CWMODE=1", buf, DRV_IPOS_BUF_LEN, 0);
    sensor_ipos_cmd_query(io, "AT+CWLAPOPT=1,12", buf, DRV_IPOS_BUF_LEN, 0);
    for( i = 0; i < 2; i++ ) {
      rd = sensor_ipos_cmd_issue(io, "AT+CWLAP", buf, DRV_IPOS_BUF_LEN, 5000);
      if( rd >= 30 )
        break;
    }
#if 0    
    if( dbg ) {
      drv->log->buf("ssid", buf, rd);
    }
#endif    
    c = buf;
    i = 0;
    /* search '(' */
    pl_ssid = iq->ssid;
    iq->ssid_cnt = 0; /* clear before read */
    iq->ssid_pos = 0;

    while( i < rd && rd >= 30 ) {
      pos_u8_t m;
      while( i < rd ) {
        if( *c == '(' ) {
          break;
        }
        c++;
        i++;
      }
      if( *c != '(' )
        break;
      c++;
      i++;

      if( i + 22 > rd ) 
        break; /* not enough length for 0,"d8:c8:e9:7a:34:82") */
      pl_ssid[6] = drv->os->util->str2i32((char*)c);
      while( i < rd ) {
        if( *c == '"' ) {
          break;
        }
        c++;
        i++;
      }
      if( *c != '"' )
        break;
      if( i + 18 > rd ) 
        break; /* not enough length for d8:c8:e9:7a:34:82") */
      c++;
      i++;

      for( m = 0; m < 6; m++ ) {
        pos_u32_t s_len;
        s_len = rd - i;
        drv->os->str2bin((char*)c, (pos_u8_t*)&pl_ssid[m], &s_len, DECODE_HEXSTR);
        c += 3;
        i += 3;
      } 

      /* advance to next */
      pl_ssid += 7;
      iq->ssid_cnt ++;
      if( iq->ssid_cnt >= max_ssid_collected )
        break;
    }    

    sensor_ipos_dyn_pwr_cntl(iq, 0); /* power off */

    /* report after collecting */
    ret = sensor_ipos_buf_report(iq);
  
  } while(0);

  /* Release and return */
  if( io )
    io->release(io);

  if( buf )
    drv->os->free(buf);

  /* set error flag */
  if( ret != POS_STATUS_OK )
    drv->s->ctrl |= MA_SENSOR_CTRL_STAT_ERROR;

  /* clear IRQ after report */
  iq->irq_tick = 0;
  
  return ret;
  
}

/**
* Driver polling
*/
void sensor_ipos_poll(ipos_param_t *iq){
  pos_u32_t to;
  /* report buffer when ssid pos < cnt */
  if( iq->ssid_pos < iq->ssid_cnt ) {
    sensor_ipos_buf_report(iq);
    return;
  }

  if( !iq->irq_tick ) {
    return;
  }

  if( iq->drv.cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    iq->drv.log->data("ipos irq", iq->irq_tick);
  }

  to = iq->gap_seconds;
  if( to ) {
    pos_u32_t ms;
    /* make sure report gap is guaranteed */
    to *= 1000;
    ms = iq->drv.os->tick->elaps(iq->report_tick); /* elaps time from last report */
    if( to > ms ) {
      to -= ms;
      if( iq->drv.cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
        iq->drv.log->data("ipos gap", to);
      }
      iq->drv.os->tick->sleep(to); /* sleep when setting gap is longer than elaps time */
    }
  }

  sensor_ipos_duty_report(iq);
}

/**
* Driver collecting
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/
pos_status_t sensor_ipos_collect(void){
  drv_api_t *drv = g_drv;
  pos_status_t ret;
  ipos_param_t *iq;
  iq = (ipos_param_t*)drv->s->drv_buf;
  ret = sensor_ipos_duty_report(iq);
  return ret;
}

/** 
 * IRQ interrupt callback handler
 * @note This function can NOT call g_drv. When IRQ happening, g_drv might piont to any sensor drivers. It should call the param stored driver pointer.
 */
void sensor_ipos_irq(ipos_param_t *param) {
  pos_u32_t tick;
  tick = param->drv.os->tick->get();
  if( !tick )
    tick = 1;
  param->irq_cnt++;
  param->irq_tick = tick; /* record tick at least to be 1 */
  param->drv.os->gpio->irq_clr(param->pin);
}

/**
* Power control
* @param[in]  on   0:Power off, 1:Power on
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
* @note This function can be NULL if it does NOT require any power operatoin
*/
pos_status_t sensor_ipos_set(pos_u32_t on) {  
  return POS_STATUS_OK;
}

/** 
 * Driver init
 * @param[in]  load   0:Unload driver, 1:Load driver
 * @return     0: Successful\n
             !=0: Failed, refer to @ref pos_status_t
 * @note This function can be NULL if it does NOT require any init operatoin
 */ 
pos_status_t sensor_ipos_init(pos_u32_t load) {
  drv_api_t *drv = g_drv;
  ipos_param_t *param;
  ma_sensor_ctrl_t *s = g_drv->s;
  pos_gpio_pin_t pin;
  pos_u8_t pwr;  
  
  /* collect pwr/pin */
  pwr = (s->slot->io >> 8) & 0x7f;
  if( !pwr ) {
    pwr = MB_PWR_ID_PWRH_ONLY;
  }
  pin = s->slot->rsvd32 & 0xffff;
  if( !pin )
    pin = drv->board->pin_map(MB_PT_WIRE1, 0); /* default is WIRE1 PIN */
  else
    pin = drv->board->pin_map(MB_PT_DIO, pin);

  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("ipos init", load);
  }  

  if( load ) {
    pos_u32_t v;
    param = drv->os->malloc(sizeof(*param));
    if( !param )
      return POS_STATUS_E_MEM;

    drv->os->memset(param, 0, sizeof(*param));
    param->report_id = 0xffff; /* inc by 1 and start by #0 */
    param->pwr = pwr;
    param->pin = pin;
    param->flag = 0;
    drv->os->memcpy(&param->drv, drv, sizeof(*drv)); /* clone a g_drv */    
    s->drv_buf = param;  /* record to drv_buf */

    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      drv->log->buf("ipos param", &param->pin, 4);
    }

    /* set PD+RISING for Single Polarity, Rising
     * set PU+FALLING for Single Polarity, !Rising
     * set RISING+FALLSING for !Single Polarity
     */
    v = POS_GPIO_MODE_INPUT_IRQ_RISING_FALLING;
 
    /* 
     * turn on irq
     * warning: as GOT is implemented in DATA and DATA is not supported in mmap()
     * all GOT based pointer can NOT be used. So the IRQ SET should use the mapped
     * function pointer here (the export structure). 
     * the orignal sensor_ipos_irq() should NOT be used directly.
     */     
    drv->os->gpio->irq_set(
      pin, 
      v,
      (pos_func_t)drv->s->drv.u.sensor.irq_cb, /* has to be this. DO NOT use sensor_ipos_irq */
      param);

    /*
     * turn on poll
     */
    drv->proc->event_register((pos_func_t)drv->s->drv.u.sensor.poll_cb, param);

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

/*
 * Driver export
 */
ma_drv_export_t g_export = {
  .version = DRV_SENSOR_VERSION(MA_SENSOR_IPOS, DRV_IPOS_VERSION),
  .name = DRV_IPOS_NAME,
  .u.sensor={
    .init = sensor_ipos_init,
    .power_set = sensor_ipos_set, 
    .collect = sensor_ipos_collect,
    .irq_cb = (pos_func_t)sensor_ipos_irq,
    .poll_cb = (pos_func_t)sensor_ipos_poll,
  },
};

