/**
 * @file  drv_mn316.c
 * @brief NET MN316 driver
 * @author Runby F.
 * @date 2021-1-28
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * Revision History
v1.0 @ 2023-09-14
  1) First version draft

v1.1 @ 2023-09-16
  1) Support ROAMING
  2) Support LWM2M dereg chk

*/

/**
 * @brief Driver version
 */
#define DRV_MN316_VERSION 0x0101

/**
 * @brief Driver name
 */
#define DRV_MN316_NAME  "MN316"

/**
 * @brief Wakeup PIN normal state
 */
#define DRV_MN316_WAKEUP_NORMAL  1

/**
 * @brief Wakeup PIN trigger state
 */
#define DRV_MN316_WAKEUP_TRIG  0

/**
* @brief MN316 working state wakeup
*/
void drv_mn316_wakeup(void) {
  drv_api_t *drv = g_drv;
  pos_gpio_pin_t pin, pwrkey;
  pin = drv->board->pin_map(MB_PT_CTRL, MB_CTRL_ID_MODULE_WAKEUP);    
  pwrkey = drv->board->pin_map(MB_PT_CTRL, MB_CTRL_ID_MODULE_PWRKEY);   
  drv->os->gpio->polarity_set(pin, DRV_MN316_WAKEUP_NORMAL); 
  drv->os->gpio->polarity_set(pwrkey, DRV_MN316_WAKEUP_NORMAL);     
  drv->os->tick->sleep(2);
  drv->os->gpio->polarity_set(pin, DRV_MN316_WAKEUP_TRIG);
  drv->os->gpio->polarity_set(pwrkey, DRV_MN316_WAKEUP_TRIG);    
  drv->os->tick->sleep(2);    
  drv->os->gpio->polarity_set(pin, DRV_MN316_WAKEUP_NORMAL);
  drv->os->gpio->polarity_set(pwrkey, DRV_MN316_WAKEUP_NORMAL);     
}

/**
* @brief MN316 release for PSM
*/
pos_status_t drv_mn316_release(void) {
  pos_u32_t i;      
  pos_status_t ret = POS_STATUS_E_TIMEOUT;
  for( i  = 0; i < 3; i++ ) {
    ret = g_drv->ext->cmd_query((pos_u8_t*)"AT+WORKLOCK=0",0);
    if( ret == POS_STATUS_OK ) /* MN316 release worklock to enter PSM */
      break;
  }
  return ret;
}

/**
* @brief MN316 working state set
*/
pos_status_t drv_mn316_run_set(pos_run_level_t level) {
  drv_api_t *drv = g_drv;
  pos_status_t ret = POS_STATUS_OK;  
  if( level == RUN_LEVEL_POWER_OFF ) {      
    /* stop all before power off */
    ret = drv->ext->cmd_query((pos_u8_t*)"AT+CFUN=0", 0); 
  } else if( level == RUN_LEVEL_POWER_SAVING ) {    
    /* PSMÄ£Ê½ */
    if(  (drv->ext->flag & MA_EXT_FLAG_MODULE_PSM_ACK) == 0 ) {
      if( drv_mn316_release() == POS_STATUS_OK ) {
        pos_u32_t i;      
        /* wait for PSM notificaiton */
        for( i = 0; i < 3; i++ ) {
          ret = drv->ext->cmd_query_check(POS_NULL, 0, (pos_u8_t*)"+POWERDOWN:", 16000);
          if( ret == POS_STATUS_OK )
            break;
        }
        if( ret == POS_STATUS_OK ) {
          drv->ext->flag |= MA_EXT_FLAG_MODULE_PSM_ACK; /* this flag will be flush by DATA_REPORT in the next round */
        } 
      }
    }
   
  } else {
    /* Give a Rising signal to wakeup from sleeping/psm mode */
    drv_mn316_wakeup();

  }
  return ret;
}


/**
* @brief MN316 AT check
*/
pos_status_t drv_mn316_at_chk(pos_u32_t cnt) {
  pos_status_t ret = POS_STATUS_ERROR;
  pos_u32_t i = 0;
  do {
    if( (ret=g_drv->ext->cmd_query((pos_u8_t*)"ATE0", 1000)) == POS_STATUS_OK ) {
      break;
    }
  } while(++i < cnt);
  return ret;
}


/**
* @brief module LWM2M disconnect
*/
pos_status_t drv_mn316_lwm2m_disconnect(void) {
  ma_ext_ctrl_t *ext = g_drv->ext;
  ext->cmd_query((pos_u8_t*)"AT+MIPLCLOSE=0",0);
  ext->cmd_query_check(POS_NULL, 0, (pos_u8_t*)"+MIPLEVENT:0,15", 10000);
  ext->cmd_query((pos_u8_t*)"AT+MIPLDELETE=0",0);  
  ext->one.arg[0] = 0;
  ext->one.arg[1] = 0;
  ext->flag &= ~MA_EXT_FLAG_MODULE_CONNECT;
  return POS_STATUS_OK;
}

/**
* @brief Module LWM2M connect
*/
pos_status_t drv_mn316_lwm2m_connect(pos_u32_t timeout) {
  pos_status_t ret;
  drv_api_t *drv = g_drv;  
  pos_u32_t v;
  const char *cb;
  ma_ext_ctrl_t *ext = drv->ext;
  /* when arg[0] is defined (connection is valid),
   * check status and return OK
   */
  if( ext->one.arg[0] ) {
    /* check status as sometime it might be denied by onenet when topo/product change */
    ret = ext->cmd_query((pos_u8_t*)"AT+MIPLOPEN?",0);
    if( ret == POS_STATUS_OK ) {
      ext->cmd_query_check(POS_NULL, 0, (pos_u8_t*)"+MIPLEVENT:0,6", 5000); /* eat up the information prompt */
      return POS_STATUS_OK;
    }
    /* disconnect old when session is error */
    drv_mn316_lwm2m_disconnect();
  }
  
  do {
    ret = ext->cmd_query((pos_u8_t*)"AT+MIPLCREATE", 5000); 
    if( ret != POS_STATUS_OK )
      break;

    /* bind generic sensor */
    ret = ext->cmd_query((pos_u8_t*)"AT+MIPLADDOBJ=0,3300,1,\"1\",1,1", 0);  
    if( ret != POS_STATUS_OK )
      break;

    /* add discover response */
    ret = ext->cmd_query((pos_u8_t*)"AT+MIPLDISCOVERRSP=0,3300,1,9,\"5527;5750\"", 0);  
    if( ret != POS_STATUS_OK )
      break;


    ret = ext->cmd_query((pos_u8_t*)"AT+MIPLOPEN=0,86400", 0);  
    if( ret != POS_STATUS_OK )
      break;
                
  } while(0);
  if( !timeout )
    timeout = 30000;
  timeout = drv->os->tick->get() + timeout;
  ret = POS_STATUS_E_RESOURCE;
  do {
    v = ext->module_opr(MA_EXT_MODULE_OPR_READLN, 3000);
    if( v == 0 )
      continue;   

    /* check register failure */
    cb = drv->os->util->strfind_skip((const char*)ext->buf, "+MIPLEVENT:0,7", v, 0 );    
    if( cb ) {
      break;
    }
    
    /* check +MIPLDISCOVER:0,9813,3300*/
    cb = drv->os->util->strfind_skip((const char*)ext->buf, "+MIPLDISCOVER:", v, 0 );    
    if( cb ) {
      ext->one.arg[0] = 1; /* MN316 does not need to use the msgid. record 1 here */
      ret = POS_STATUS_OK;
      break;
    } 
  } while(!drv->os->tick->is_timeout(drv->os->tick->get(), timeout));

  /* release connection when failed */
  if( ret != POS_STATUS_OK ) {
    drv_mn316_lwm2m_disconnect();
  }
  return ret;
}

/**
* @brief MN316 module init
*/
pos_status_t drv_mn316_init(void) {
  pos_u8_t flag;
  pos_u32_t timeout;
  drv_api_t *drv = g_drv;  
  pos_gpio_pin_t pin;
  ma_ext_ctrl_t *ext = drv->ext;

  ext->one.arg[0] = 0;
  ext->one.arg[1] = 0;

  /* init wakup pin and set to low by default */
  pin = drv->board->pin_map(MB_PT_CTRL, MB_CTRL_ID_MODULE_WAKEUP);
  drv->os->gpio->mode_set(pin, POS_GPIO_MODE_OUTPUT);
  drv->os->gpio->polarity_set(pin, DRV_MN316_WAKEUP_NORMAL); 

  /* init pwrkey pin and set to low by default */
  pin = drv->board->pin_map(MB_PT_CTRL, MB_CTRL_ID_MODULE_PWRKEY);
  drv->os->gpio->mode_set(pin, POS_GPIO_MODE_OUTPUT);
  drv->os->gpio->polarity_set(pin, DRV_MN316_WAKEUP_NORMAL); 

  ext->mtu = 400; /* use middle mtu */
  ext->tx_rt[0] = 1; /* \r only */
  do {
    /* reset */
    if( ext->run_level > RUN_LEVEL_POWER_OFF ) {
      drv_mn316_run_set(RUN_LEVEL_POWER_OFF); /* stop all before reset */
    }
#if 0
    ext->cmd_query((pos_u8_t*)"AT+CFUN=0", 3000); /* Clear state */
#endif
    timeout = 0;
    while(1) {
      ext->module_reset();
      drv_mn316_wakeup(); /* mn316 requires a wakeup to avoid PSM unable to reset by pwr cycle */
      drv_mn316_at_chk(16);
      ext->cmd_query((pos_u8_t*)"AT+NRB",0); /* sometime pwr reset not work, use SW reset to clear running status */
      if( drv_mn316_at_chk(16) == POS_STATUS_OK ) 
        break;
#if 0
      /* reset 115200 to 9600 */
      ext->io->release(ext->io);
      ext->io->setup(ext->io, 115200, 0);
      ext->cmd_query((pos_u8_t*)"AT+IPR=9600;&W", 0);
      ext->io->release(ext->io);
      ext->io->setup(ext->io, MB_EXT_UART_BAUD, 0);
      if( drv_mn316_at_chk(3) == POS_STATUS_OK ) 
        break;
#endif        
      /* power off 3s and try again */
      ext->module_power_set(0);

      /* Support NULL report when too much failed */
      if( ++timeout >= 8 ) {
        /* about 8*16 + 7*3 = 149s, 2.5min later, use NULL report */
        ext->report_type = MA_REPORT_NATIVE_NULL;
        return POS_STATUS_OK;
      }
      
      drv->os->tick->sleep(3000);
    }
    do {
      if( ext->cmd_query((pos_u8_t*)"ATE0", 0) == POS_STATUS_OK )
        break;
    } while(1);

    /* print IMEI and CIMI */
    drv->ext->module_opr(MA_EXT_MODULE_OPR_PRINT_IMEI_CIMI, 0);  

#if 0
    /* Turn ON HIBNATE notification */
    ext->cmd_query((pos_u8_t*)"AT+ECPURC=\"HIBNATE\",1", 0); 

    /* Turn OFF PSM notification */
    ext->cmd_query((pos_u8_t*)"AT+ECPSMR=0", 0); 
#endif

    /* PSM/None-PSM mode */
#if 0
    drv->os->sprintf((char*)g_drv->ext->buf, "AT+CPSMS=%u,,,,00011111", 
      (drv->cfg->ctrl & MA_CFG_CTRL_POWER_SAVING) == 0 ? 0 : 1);
    ext->cmd_query(g_drv->ext->buf, 0);

#else
    ext->cmd_query((pos_u8_t*)"AT+CPSMS=1,,,\"00111000\",\"00010000\"", 0); /*TAU=24H, ACT=16 *2 (000=2s interval) = 32s */
#endif
    
    /* check APN/BAND */
    flag = 0;
    do {
      if( ext->cmd_query_check((pos_u8_t*)"AT+CGDCONT?", 17, (pos_u8_t*)g_drv->cfg->net.apn, 0) != POS_STATUS_OK ) {
        break;
      }
      flag |= 1;  
      {
        pos_u8_t bbuf[sizeof(g_drv->cfg->net.band)+8];
        drv->os->sprintf((char*)bbuf, "%s\r", g_drv->cfg->net.band);
        if( ext->cmd_query_check((pos_u8_t*)"AT+NBAND?", 7, (pos_u8_t*)bbuf, 0) != POS_STATUS_OK ) {
          break;
        } 
      }
      flag |= 2;
    } while(0);
    if( flag == 3 )
      break; /* quit if APN/BAND are matched */

    /* set APN/BAND */
    ext->cmd_query((pos_u8_t*)"AT+CGATT=0", 0); /* detach before APN/BAND change */
    drv->os->sprintf((char*)g_drv->ext->buf, "AT+CGDCONT=1,\"IP\",\"%s\"", g_drv->cfg->net.apn);
    ext->cmd_query(g_drv->ext->buf, 0);
    ext->cmd_query((pos_u8_t*)"AT+CGATT=1", 0); /* re-attach*/
    drv->os->sprintf((char*)g_drv->ext->buf, "AT+NBAND=%s", g_drv->cfg->net.band);
    ext->cmd_query(g_drv->ext->buf, 0);
    ext->cmd_query((pos_u8_t*)"AT+NRB", 0); /* use NRB to make change take effect */

  } while(1);

#if 1
  /* start network */
  drv->ext->cmd_query((pos_u8_t*)"AT+CEREG=5", 0);
#endif

  drv->history->signal = drv->ext->module_opr(MA_EXT_MODULE_OPR_GET_SIGNAL,0);
  return POS_STATUS_OK;
}

/**
* @brief MN316 module link ready check
*/
pos_status_t drv_mn316_net_ready(pos_u32_t timeout_ms) {
  pos_status_t ret;
  drv_api_t *drv = g_drv;
  pos_u16_t v;  
  pos_u32_t old;
#if 0  
  pos_u8_t i;  
  i = 0;
#endif  

  drv_mn316_run_set(RUN_LEVEL_NORMAL); /* Wakeup PIN */
  if( drv_mn316_at_chk(timeout_ms/1000) == POS_STATUS_OK ) { /* Wait until wakeup from PSM */
    drv->ext->flag &= ~MA_EXT_FLAG_MODULE_PSM_ACK;  /* clear PSM ACK */
  }

#if 1
  drv->ext->cmd_query((pos_u8_t*)"AT+CEREG=5", 0); /* ¼¤»î×¢²á */
#endif
  old = drv->os->tick->get(); 

  /* Register */
  do {
    ret = drv->ext->cmd_query_check((pos_u8_t*)"AT+CEREG?", 0, (pos_u8_t*)"+CEREG:5,",0);
    if( ret == POS_STATUS_OK ) {
      v = drv->ext->module_opr(MA_EXT_MODULE_OPR_GET_U32(1),(pos_u32_t)drv->ext->buf);      
      if( v == 1 || v == 5 ) { /* 1 for LOCAL, 5 for roaming */
        /* drv->log->data("link", v); */ // DBG
        break;
      }
    }
    drv->os->tick->sleep(2000);
  } while( drv->os->tick->elaps(old) < timeout_ms );

#if 0
  /* PDP Attach */
  if( ret == POS_STATUS_OK ) {
    do {
      ret = drv->ext->cmd_query((pos_u8_t*)"AT+CGACT=1,1", 0);
        if( ret == POS_STATUS_OK )
          break;
        drv->os->tick->sleep(2000);
    } while( drv->os->tick->elaps(old) < timeout_ms );
  }

  /* PDP Attach */
  if( ret == POS_STATUS_OK ) {
    do {
      ret = drv->ext->cmd_query_check((pos_u8_t*)"AT+CGCONTRDP", 0, (pos_u8_t*)"+CGCONTRDP:", 0);
      if( ret == POS_STATUS_OK )
        break;
      drv->os->tick->sleep(2000);
    } while( drv->os->tick->elaps(old) < timeout_ms );
  }
#endif

  drv->ext->cmd_query((pos_u8_t*)"AT+CGCONTRDP", 0);
  
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("mn316 link status", ret);
  }
  v = 0;
  if( ret == POS_STATUS_OK )
    v = drv->ext->module_opr(MA_EXT_MODULE_OPR_GET_SIGNAL,0);


  if( ret == POS_STATUS_OK &&
      drv->ext->cmd_query_check((pos_u8_t*)"AT+CCLK?", 0, (pos_u8_t*)"+CCLK:",0) == POS_STATUS_OK ) {
      /* +CCLK: "2023/02/09,08:51:58+32" */
    if( drv->ext->buf[7] == ':' ) {
      char *n;
      drv_data_time_t _t, *t = &_t;
      pos_i32_t adj;
      t->tick = drv->os->tick->get();
      if( !t->tick )
        t->tick = 1;
      n = (char *)drv->os->sscan( (char*)&drv->ext->buf[8], "%2n/%n/%n,%n:%n:%n", 
        &t->year, &t->month, &t->day,
        &t->hour, &t->minute, &t->second );
      if( *n == '+' || *n == '-' ) {
        drv->os->sscan(n+1, "%n", &t->tz_quarter );
        if( *n == '-' )
          t->tz_quarter = 0 - t->tz_quarter;
        adj = t->tz_quarter;
        adj *= 15*60; /* quarter to seconds */
        drv->data->plss_time_adj(t, adj);
        drv->os->memcpy(&drv->history->time, t, sizeof(*t));
#if 1
        if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
          drv->os->printf("%u-%u-%u %02u:%02u:%02u RF:%u", t->year, t->month, t->day, t->hour, t->minute, t->second, v);
        }
#endif        
      }
    }
  }
  drv->history->signal = v;
  return ret;
}

/**
* @brief MN316 module socket open
*/
pos_status_t drv_mn316_sock_open(pos_u32_t *p_sock, pos_net_trans_t tcp, pos_u16_t port) {
  g_drv->os->sprintf((char*)g_drv->ext->buf, "AT+NSOCR=%s,%u,%u,1", tcp ? "STREAM" : "DGRAM", tcp ? 6 : 17, port); 
  if( g_drv->ext->cmd_query(g_drv->ext->buf,0) == POS_STATUS_OK ) {
    if( g_drv->ext->buf[7] >= '0' && g_drv->ext->buf[7] <= '9' ) {
      *p_sock = g_drv->ext->buf[7] - '0';
      return POS_STATUS_OK;
    }
  }
  *p_sock = 0; /* always use 0 when failed */
  return POS_STATUS_ERROR;
}

/**
* @brief MN316 module socket close
*/
pos_status_t drv_mn316_sock_close(pos_u32_t sock) {  
  g_drv->os->sprintf((char*)g_drv->ext->buf, "AT+NSOCL=%u", sock);  
  g_drv->ext->cmd_query(g_drv->ext->buf,0);
  return g_drv->ext->cmd_query_check(POS_NULL,0,(pos_u8_t*)"+NSOCLI:",5000);
}

/**
* @brief MN316 module socket connect
*/
pos_status_t drv_mn316_connect(pos_u32_t sn, pos_u8_t * addr, pos_u16_t port) {
  g_drv->os->sprintf((char*)g_drv->ext->buf, "AT+NSOCO=%u,%u.%u.%u.%u,%u", sn, addr[0],addr[1],addr[2],addr[3],port);
  g_drv->ext->cmd_query(g_drv->ext->buf,0);
  return g_drv->ext->cmd_query_check(POS_NULL,0,(pos_u8_t*)"CONNECT OK",15000);
}

/**
* @brief MN316 module sendto
*/
pos_i32_t drv_mn316_sendto(pos_u32_t sn, void * vbuf, pos_size_t len, pos_u8_t * addr, pos_u16_t port) {
  pos_size_t slen;
  drv_api_t *drv = g_drv;  
  if( MA_REPORT_TYPE_IS_NATIVE(drv->ext->report_type) ) {
    /* LWM2M */
    if( drv_mn316_lwm2m_connect(30000) != POS_STATUS_OK )
      return -1;
    
    /* AT+MIPLNOTIFY=0,0,3300,0,5750,1,8,"aabbccdd00112233",0,0 */
    drv->os->sprintf((char*)drv->ext->buf,"AT+MIPLNOTIFY=0,0,3300,0,5750,1,%u,\"",len*2);
    slen = drv->os->strlen((char*)drv->ext->buf);
    drv->os->bin2str((char*)&drv->ext->buf[slen], (pos_u8_t*)vbuf, len, ENCODE_HEXSTR_UPPER); 
    slen = drv->os->strlen((char*)drv->ext->buf);
    drv->os->strcpy((char*)&drv->ext->buf[slen], "\",0,0");
    slen = drv->os->strlen((char*)drv->ext->buf);  
    return drv->ext->cmd_query(drv->ext->buf,0) == POS_STATUS_OK ? len : -1;
  }
  
  if( addr && port )
    /* UDP SENDTO */
    drv->os->sprintf((char*)drv->ext->buf, "AT+NSOST=%u,%u.%u.%u.%u,%u,%u,", sn, addr[0],addr[1],addr[2],addr[3],port,len);
  else
    /* TCP SEND */
    drv->os->sprintf((char*)drv->ext->buf, "AT+NSOSD=%u,%u,", sn, len);
  slen = drv->os->strlen((char*)drv->ext->buf);
  drv->os->bin2str((char*)&drv->ext->buf[slen], (pos_u8_t*)vbuf, len, ENCODE_HEXSTR_UPPER);
  return drv->ext->cmd_query(drv->ext->buf,0) == POS_STATUS_OK ? len : -1;
}

/**
* @brief MN316 module receive from
*/
pos_i32_t drv_ext_mn316_recvfrom_nb(pos_u32_t sn, pos_u8_t * buf, pos_size_t max_len, pos_u8_t * addr, pos_u16_t *port) {
  pos_status_t ret;
  pos_u32_t len32;
  pos_u8_t *rbuf;
  pos_u16_t flush_len;
  flush_len = max_len;
  drv_api_t *drv = g_drv;
  if( flush_len > 4 ) /* some app (like mqtt) will try to read 1/2/3/4 bytes for special check, this way, do NOT flush all */
    flush_len = 1358;
  drv->os->sprintf((char*)drv->ext->buf, "AT+NSORF=%u,%u", sn, flush_len ); /* MN316 uses 1358 to flush all */
  ret = drv->ext->cmd_query(drv->ext->buf,200);
  if( ret != POS_STATUS_OK ) {
//    drv->log->buf("RXERR", drv->ext->buf, drv->ext->rlen);        
    return 0; /* return 0*/
  }

//  drv->log->buf("RX", drv->ext->buf, drv->ext->rlen);

  /* check feedback */
  len32 = 0;    
  if( drv->ext->buf[9] == sn+'0' && drv->ext->buf[10] == ',' ) {
    rbuf = (pos_u8_t*)drv->os->sscan( (char*)&drv->ext->buf[11], "%4n,%2n,%4n,", addr, port, &len32 );
    drv->os->nps->ntohl((pos_u32_t*)addr,1);
    if( len32 > max_len )
      len32 = max_len;
    if( len32 ) {
      pos_u32_t i;
      i = len32;
      drv->os->str2bin((char*)rbuf, buf, &i,DECODE_HEXSTR);
      if( i != len32 )
        len32 = 0; /* wrong payload */
    }
  }
  return len32;
}

/**
* @brief MN316 module LWM2M receive process
*/
pos_i32_t drv_mn316_lwm2m_rx(void * vbuf, pos_size_t max_len, pos_u32_t timeout_ms) {
  pos_u32_t v, msgid, slen, dbg;
  const char *cb, *end;
  pos_i32_t len32;
  drv_api_t *drv = g_drv;      
  ma_ext_ctrl_t *ext = drv->ext;
  timeout_ms = drv->os->tick->get() + timeout_ms;
  len32 = 0;
  
  do {
    v = ext->module_opr(MA_EXT_MODULE_OPR_READLN, 1000);
    if( v == 0 )
      continue;
    end = (const char *)&ext->buf[v];

    /* WHEN PSM is entered, quick immediately */    
    cb = drv->os->util->strfind_skip((char*)ext->buf, "+POWERDOWN:", v, 0 ); 
    if( cb )  {
      drv->ext->flag |= MA_EXT_FLAG_MODULE_PSM_ACK;
      break;
    }
    
    /* check +MIPLWRITE:0,25052,3300,0,5750,1,6,123412,0 */    
    cb = drv->os->util->strfind_skip((char*)ext->buf, "+MIPLWRITE:", v, 0 ); 
    if( !cb ) 
      continue;

    dbg = 0;
    do {
      msgid = ext->module_opr(MA_EXT_MODULE_OPR_GET_U32(1),(pos_u32_t)cb);
      if( !msgid ) {
        dbg = 1;
        break; /* wrong fmt: no msgid */
      }

      slen = ext->module_opr(MA_EXT_MODULE_OPR_GET_U32(6),(pos_u32_t)cb);
      if( slen < 2 ) {
        dbg = 2;
        break; /* wrong fmt: not enough length */
      }

      cb = drv->os->util->strfind_skip((char*)cb, ",", end-cb, 6);
      if( !cb ) {
        dbg = 3;
        break; /* wrong fmt: no payload */
      }
      cb++; /* skip ',' to next char */

      if( cb + len32 >= end ) {
        dbg = 4;
        break; /* wrong fmt: string length exceeding */
      }

      if( slen > max_len * 2 )
        slen = max_len * 2;
      if( slen > ext->buf_size )
        slen = ext->buf_size & 0xfffe;

      slen /= 2;
      v = slen;
      drv->os->str2bin((char*)cb, (pos_u8_t*)vbuf, &v, DECODE_HEXSTR);
#if 0
      drv->log->data("v", v);
      drv->log->data("slen", slen); 
      drv->log->buf("rx", (pos_u8_t*)cb, end-cb); 
#endif        
      if( v != slen ) {
        dbg = 5;
        break; /* wrong fmt: string is not hex string */
      }

      /* successful */
      len32 = slen; /* update length */
      drv->os->sprintf((char*)ext->buf, "AT+MIPLWRITERSP=0,%u,2", msgid);
      ext->cmd_query(ext->buf, 0); /* response SUCCESS */
    } while(0);
    if( dbg ) {
      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
        drv->log->data("event err", dbg);
      }
    }

  } while(
    !len32 && /* quit when len32 is readed */
    !drv->os->tick->is_timeout(drv->os->tick->get(), timeout_ms)); /* or timeout */
  
  /* to-do */
  return len32;
}

/**
* @brief MN316 module receive (blocking)
*/
pos_i32_t drv_mn316_recvfrom(pos_u32_t sn, void * vbuf, pos_size_t max_len, pos_u8_t * addr, pos_u16_t *port, pos_u32_t timeout_ms) {
  pos_i32_t len32;
  drv_api_t *drv = g_drv;  
  
  if( !max_len )
    return 0;
  if( MA_REPORT_TYPE_IS_NATIVE(drv->ext->report_type) ) {
    len32 = drv_mn316_lwm2m_rx(vbuf, max_len, timeout_ms);
    if( !len32 && (drv->ext->flag & MA_EXT_FLAG_MODULE_PSM_ACK) == 0 ) {
      /* perform another round when len32 is zero and !PSM */
      drv_mn316_release(); /* make ready for PSM */
      len32 = drv_mn316_lwm2m_rx(vbuf, max_len, 60000);  
    }
    return len32;
  }
  
  len32 = drv_ext_mn316_recvfrom_nb(sn, vbuf, max_len, addr, port);
  if( timeout_ms && len32 == 0 ) {
    if( drv->ext->cmd_query_check(POS_NULL, 0, (pos_u8_t*)"+NSONMI:", timeout_ms) == POS_STATUS_OK )
      len32 = drv_ext_mn316_recvfrom_nb(sn, vbuf, max_len, addr, port);    
  }
  return len32;
}


ma_drv_export_t g_export = {
  .version = DRV_NET_VERSION(MA_REPORT_MQTT_MN316, DRV_MN316_VERSION),
  .name = DRV_MN316_NAME,
  .u.net={
    drv_mn316_init,
    drv_mn316_run_set,
    drv_mn316_net_ready,
    drv_mn316_sock_open,
    drv_mn316_sock_close,
    drv_mn316_connect,
    drv_mn316_sendto,
    drv_mn316_recvfrom,
  },
};

