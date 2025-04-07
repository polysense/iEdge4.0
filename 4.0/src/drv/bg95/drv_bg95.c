/**
 * @file  drv_bg95.c
 * @brief NET BG95 driver
 * @author Runby F.
 * @date 2021-1-28
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * Revision History
v1.0 @ 2023-04-22
  1) First version draft

v1.1 @ 2023-04-26
  1) Debug enhance

v1.2 @ 2023-06-20
  1) Support BG95 PWRKEY/WAKEUP key
  2) Support BG95 auto reset IPR=9600

v1.3 @ 2023-07-27
  1) Use generic CSQ update

v1.4 @ 2023-09-11
  1) Support PWRKEY wakeup
  2) Support PSM URC capture

v1.5 @ 2023-11-10
  1) Support to trigger RAI relesae when PSM is failed in first detection cycle

v1.6 @ 2024-08-16
  1) Enhance MQTT report for longer SOCK CONNECT waiting
  2) Enhance SOCK OPEN retry for better connecting rate
  3) Enhance SOCK READ for continuous waiting (ret 0 instead of -1)
  4) Support new BG95 RX for +QIRD: X direclty and re-read RX at once when QIURC/recv detetcted
  5) Fix RTC year issue (24 should be 2024)

v1.7 @ 2024-08-19
  1) Fix APN can not set to ABCD when current is ABCDX
  2) Optimize PSM check mechanism for 80s+RAI+40s style
  3) Read more signals when it's too low (0%)
  4) Enlarge CLOSE wait to 30s

v1.8 @ 2024-09-14
  1) Increase additional 1 retry/re-link durng OPEN 

*/

/**
 * @brief Driver version
 */
#define DRV_BG95_VERSION 0x0108

/**
 * @brief Driver name
 */
#define DRV_BG95_NAME  "BG95"

/**
* @brief BG95 working state set
*/
pos_status_t drv_bg95_run_set(pos_run_level_t level) {
  drv_api_t *drv = g_drv;
  pos_status_t ret = POS_STATUS_OK;  
  if( level == RUN_LEVEL_POWER_OFF ) {      
    /* stop all before power off */
    ret = drv->ext->cmd_query((pos_u8_t*)"AT+QPOWD=0", 0); 
  } else if( level == RUN_LEVEL_POWER_SAVING ) {    
    /* PSM模式 */
    /* BG95 does NOT require any special actions to enter PSM */
#if 1    
    if(  (drv->ext->flag & MA_EXT_FLAG_MODULE_PSM_ACK) == 0 ) {
      pos_u32_t i;      

      /* wait for PSM notificaiton */
      for( i = 0; i < 3; i++ ) {
        ret = drv->ext->cmd_query_check(POS_NULL, 0, (pos_u8_t*)"PSM POWER DOWN", 40000);
        if( ret == POS_STATUS_OK )
          break;
        if( i == 1 )
          drv->ext->module_opr(MA_EXT_MODULE_OPR_DUMMY_TX,0); /* trigger RAI to release */
      }
      if( ret == POS_STATUS_OK ) {
        drv->ext->flag |= MA_EXT_FLAG_MODULE_PSM_ACK; /* this flag will be flush by DATA_REPORT in the next round */
      } else {
        /* no nitification */
        /* do nothing now */
      }
    }
#endif
    
  } else {
    /* Give a Rising signal to wakeup from sleeping/psm mode */
    pos_gpio_pin_t pin, pwrkey;
    pin = drv->board->pin_map(MB_PT_CTRL, MB_CTRL_ID_MODULE_WAKEUP);    
    pwrkey = drv->board->pin_map(MB_PT_CTRL, MB_CTRL_ID_MODULE_PWRKEY);   
    drv->os->gpio->polarity_set(pin, 0); 
    drv->os->gpio->polarity_set(pwrkey, 0);     
    drv->os->tick->sleep(50);
    drv->os->gpio->polarity_set(pin, 1);
    drv->os->gpio->polarity_set(pwrkey, 1);    
    drv->os->tick->sleep(50);    
    drv->os->gpio->polarity_set(pin, 0);
    drv->os->gpio->polarity_set(pwrkey, 0);    
  }
  return ret;
}

/**
* @brief BG95 PowerKey Switch
*/
void drv_bg95_pwr_switch(pos_gpio_pin_t pin) {
  const pos_lib_t *os = g_drv->os;  
  os->tick->sleep(100);
  os->gpio->polarity_set(pin, 1);
  os->tick->sleep(600);
  os->gpio->polarity_set(pin, 0);
}


/**
* @brief BG95 AT check
*/
pos_status_t drv_bg95_at_chk(pos_u32_t cnt) {
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
* @brief BG95 module init
*/
pos_status_t drv_bg95_init(void) {
  pos_u8_t flag;
  pos_u32_t timeout;
  drv_api_t *drv = g_drv;  
  pos_gpio_pin_t pin;
  ma_ext_ctrl_t *ext = drv->ext;

  /* init wakup pin and set to low by default */
  pin = drv->board->pin_map(MB_PT_CTRL, MB_CTRL_ID_MODULE_WAKEUP);
  drv->os->gpio->mode_set(pin, POS_GPIO_MODE_OUTPUT);
  drv->os->gpio->polarity_set(pin, 0);

  /* init pwrkey pin and set to low by default */
  pin = drv->board->pin_map(MB_PT_CTRL, MB_CTRL_ID_MODULE_PWRKEY);
  drv->os->gpio->mode_set(pin, POS_GPIO_MODE_OUTPUT);
  drv->os->gpio->polarity_set(pin, 0);

  ext->mtu = 400; /* use middle mtu */
  ext->tx_rt[0] = 1; /* \r only */
  do {
    /* reset */
    if( ext->run_level > RUN_LEVEL_POWER_OFF ) {
      drv_bg95_run_set(RUN_LEVEL_POWER_OFF); /* stop all before reset */
    }

    ext->cmd_query((pos_u8_t*)"AT+CFUN=0", 3000); /* Clear state */

    timeout = 0;
    while(1) {
      ext->module_reset();
      drv_bg95_pwr_switch(pin); /* bg95 requires a PWRKEY push/switch after each power on */
      if( drv_bg95_at_chk(16) == POS_STATUS_OK ) 
        break;

      /* reset 115200 to 9600 */
      ext->io->release(ext->io);
      ext->io->setup(ext->io, 115200, 0);
      ext->cmd_query((pos_u8_t*)"AT+IPR=9600;&W", 0);
      ext->io->release(ext->io);
      ext->io->setup(ext->io, MB_EXT_UART_BAUD, 0);
      if( drv_bg95_at_chk(3) == POS_STATUS_OK ) 
        break;
        
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
      char abuf[32+8];
      drv->os->sprintf(abuf, "%s\"", g_drv->cfg->net.apn); /* add \" for full matching */
      if( ext->cmd_query_check((pos_u8_t*)"AT+CGDCONT?", 18, (pos_u8_t*)abuf, 0) != POS_STATUS_OK ) {
        break;
      }
      flag |= 1;
#if 0      
      if( ext->cmd_query_check((pos_u8_t*)"AT+ECBAND?", 11, (pos_u8_t*)g_drv->cfg->net.band, 0) != POS_STATUS_OK ) {
        break;
      }
#endif      
      flag |= 2;
    } while(0);
    if( flag == 3 )
      break; /* quit if APN/BAND are matched */

    /* set APN/BAND */
    ext->cmd_query((pos_u8_t*)"AT+CGACT=0,1", 60000); /* stop func before APN/BAND change */
    drv->os->sprintf((char*)g_drv->ext->buf, "AT+CGDCONT=1,\"IP\",\"%s\"", g_drv->cfg->net.apn);
    ext->cmd_query(g_drv->ext->buf, 0);
    ext->cmd_query((pos_u8_t*)"AT+CGACT=1,1", 60000); /* full stop func before APN/BAND change */
#if 0
    drv->os->sprintf((char*)g_drv->ext->buf, "AT+ECBAND=0,%s", g_drv->cfg->net.band);
    ext->cmd_query(g_drv->ext->buf, 0);
#endif    
  } while(1);

#if 1
  /* start network */
  drv->ext->cmd_query((pos_u8_t*)"AT+CEREG=4", 0);
#endif

  drv->history->signal = drv->ext->module_opr(MA_EXT_MODULE_OPR_GET_SIGNAL,0);
  return POS_STATUS_OK;
}

/**
* @brief BG95 module link ready check
*/
pos_status_t drv_bg95_net_ready(pos_u32_t timeout_ms) {
  pos_status_t ret;
  drv_api_t *drv = g_drv;
  pos_u16_t v;  
  pos_u32_t old;
#if 0  
  pos_u8_t i;  
  i = 0;
#endif  

  drv_bg95_run_set(RUN_LEVEL_NORMAL); /* Wakeup PIN */
  if( drv_bg95_at_chk(timeout_ms/1000) == POS_STATUS_OK ) { /* Wait until wakeup from PSM */
    drv->ext->flag &= ~MA_EXT_FLAG_MODULE_PSM_ACK;  /* clear PSM ACK */
  }

#if 1
  drv->ext->cmd_query((pos_u8_t*)"AT+CEREG=4", 0); /* 激活注册 */
#endif
  old = drv->os->tick->get(); 

  /* Register */
  do {
    ret = drv->ext->cmd_query_check((pos_u8_t*)"AT+CEREG?", 0, (pos_u8_t*)"+CEREG: 4,5",0);
    if( ret != POS_STATUS_OK )
      ret = drv->ext->cmd_query_check((pos_u8_t*)"AT+CEREG?", 0, (pos_u8_t*)"+CEREG: 4,1",0);
    if( ret == POS_STATUS_OK )
      break;
    drv->os->tick->sleep(2000);
  } while( drv->os->tick->elaps(old) < timeout_ms );

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
      ret = drv->ext->cmd_query_check((pos_u8_t*)"AT+CGPADDR=1", 0, (pos_u8_t*)"+CGPADDR: 1,", 0);
      if( ret == POS_STATUS_OK )
        break;
      drv->os->tick->sleep(2000);
    } while( drv->os->tick->elaps(old) < timeout_ms );
  }
  
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("bg95 link status", ret);
  }
  v = 0;
  if( ret == POS_STATUS_OK ) {
    pos_u16_t i;
    v = 0;
    for( i = 0; i < 10; i++ ) {
      v = drv->ext->module_opr(MA_EXT_MODULE_OPR_GET_SIGNAL,0);
      if( v >= 20 )
        break;
      drv->os->tick->sleep(2000);
    }
    if( v < 5 )
      v = 5; /* at least 5% when REGISTER is okay */
  }

  if( ret == POS_STATUS_OK &&
      drv->ext->cmd_query_check((pos_u8_t*)"AT+CCLK?", 0, (pos_u8_t*)"+CCLK: ",0) == POS_STATUS_OK ) {
      /* +CCLK: "23/02/09,08:51:58+32" */
    if( drv->ext->buf[9] == '"' ) {
      char *n;
      drv_data_time_t _t, *t = &_t;
      pos_i32_t adj;
      t->tick = drv->os->tick->get();
      if( !t->tick )
        t->tick = 1;
      n = (char *)drv->os->sscan( (char*)&drv->ext->buf[10], "%2n/%n/%n,%n:%n:%n", 
        &t->year, &t->month, &t->day,
        &t->hour, &t->minute, &t->second );
      if( t->year < 2000 )
        t->year += 2000; /* translate from 24 to 2024 */
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
          drv->os->printf("%u-%u-%u %02u:%02u:%02u", t->year, t->month, t->day, t->hour, t->minute, t->second);
        }
#endif        
      }
    }
  }
  drv->history->signal = v;
  return ret;
}

/**
* @brief BG95 module socket open
*/
pos_status_t drv_bg95_sock_open(pos_u32_t *p_sock, pos_net_trans_t tcp, pos_u16_t port) {
  if( tcp == NET_TRANS_UDP )
    *p_sock = 0; /* UDP always use 0 */  
  else
    *p_sock = 1; /* TCP always use 1 */

  return POS_STATUS_OK;
}

/**
* @brief BG95 module socket close
*/
pos_status_t drv_bg95_sock_close(pos_u32_t sock) {  
  drv_api_t *drv = g_drv;

  drv->ext->flag &= ~MA_EXT_FLAG_MODULE_CONNECT; // REMOVE connect flag during close
    
  drv->os->sprintf((char*)drv->ext->buf, "AT+QICLOSE=%u", sock);  
  return drv->ext->cmd_query(drv->ext->buf,30000);
}

/**
* @brief BG95 module socket connect
*/
pos_status_t drv_bg95_connect(pos_u32_t sn, pos_u8_t * addr, pos_u16_t port) {
  pos_status_t ret;
  drv_api_t *drv = g_drv;
  pos_u8_t i, retry2;
  ma_ext_ctrl_t *ext = drv->ext;
  
  if( !port )
    return POS_STATUS_E_PARAMETER;

  ret = POS_STATUS_OK;
  for( retry2 = 0; retry2 < 2; retry2++ ) {
    if( retry2 ) {
      ret = drv_bg95_net_ready(30000);
      if( ret != POS_STATUS_OK )
        continue;
    }
    for( i = 0; i < 3; i++ ) {
      drv->os->sprintf((char*)g_drv->ext->buf, "AT+QIOPEN=1,%u,\"%s\",\"%u.%u.%u.%u\",%u,0,0", sn, sn == 0 ? "UDP" : "TCP", addr[0],addr[1],addr[2],addr[3],port);

      ret = ext->cmd_query(g_drv->ext->buf,10000); 
      if( ret == POS_STATUS_OK ) {
        static const char c[] = "+QIOPEN";
        if( ext->rlen < 8 || drv->os->memcmp(&ext->buf[8], c, 7) != 0 ) {
          ret = drv->ext->cmd_query_check(POS_NULL, 0, (pos_u8_t*)c, 60000);
        }
        if( ret == POS_STATUS_OK ) {
          /* check OPEN status */
          char pattern[32];
          drv->os->sprintf(pattern, "%s: %u,0", c, sn);
          if( ext->buf_search_pos(ext->buf, ext->rlen, 0, (pos_u8_t*)pattern, drv->os->strlen(pattern), (pos_u8_t*)"") ) {
            ext->flag |= MA_EXT_FLAG_MODULE_CONNECT; /* SUCCESSFUL return */
            break;
          }
          ret = POS_STATUS_ERROR;
          if( i + 1 < 3 ) {
            drv_bg95_sock_close(sn); /* close before next retrial, last retrial do not need to close */
          }
        }
      }
    }
    if( ret == POS_STATUS_OK )
      break;
  }
  return ret;
}

/**
* @brief BG95 module sendto
*/
pos_i32_t drv_bg95_sendto(pos_u32_t sn, void * vbuf, pos_size_t len, pos_u8_t * addr, pos_u16_t port) {
  pos_size_t slen;
  drv_api_t *drv = g_drv;
  ma_ext_ctrl_t *ext = drv->ext;   
  pos_u8_t i;

  if( sn == 0 && (ext->flag & MA_EXT_FLAG_MODULE_CONNECT) == 0 ) {
    /* Connect if not connect flags set AND addr is valid */
    drv_bg95_connect(sn, addr, port);
  }

  /* 如果模式没有CONNECT, 代表进入失败，返回出错 */
  if( (ext->flag & MA_EXT_FLAG_MODULE_CONNECT) == 0 ){
    return -1;
  }

  for( i = 0; i < 3; i++ ) {
    /* TCP SEND */
    drv->os->sprintf((char*)drv->ext->buf, "AT+QISENDEX=%u,\"", sn);
    slen = drv->os->strlen((char*)ext->buf);
    drv->os->bin2str((char*)&ext->buf[slen], (pos_u8_t*)vbuf, len, ENCODE_HEXSTR_UPPER);
    slen = drv->os->strlen((char*)ext->buf);
    drv->os->sprintf((char*)&ext->buf[slen], "\",%u", len==1?1:0); /* RAI = 0, no RAI information (len>1) or 1 (Release immediately), when len==0*/
    drv->ext->cmd_send(ext->buf, 0);
    if( drv->ext->cmd_query_check(POS_NULL, 0, (pos_u8_t*)"SEND OK", 10000 ) == POS_STATUS_OK ) {
      return len;
    }
    if( i + 1 < 3 )
      drv->os->tick->sleep(2000); /* sleep 2s and try again */
  }
  return -1;
}

/**
* @brief BG95 module receive from
*/
pos_i32_t drv_ext_bg95_recvfrom_nb(pos_u32_t sn, pos_u8_t * buf, pos_size_t max_len, pos_u8_t * addr, pos_u16_t *port) {
  pos_u32_t len32, r_len, u;
  pos_u8_t *rbuf;
  pos_u16_t flush_len, loop;
  drv_api_t *drv = g_drv;  
  flush_len = max_len;
  for( loop = 0; loop < 2; loop++ ) {
    if( flush_len > 4 ) /* some app (like mqtt) will try to read 1/2/3/4 bytes for special check, this way, do NOT flush all */
      flush_len = 1358;
    
    drv->os->sprintf((char*)drv->ext->buf, "AT+QIRD=%u,%u", sn, flush_len ); /* BC28 uses 1358 to flush all */
    drv->ext->rlen = 0; /* clear receiving length */
    drv->ext->cmd_query(drv->ext->buf,0);
    rbuf = drv->ext->buf;
    r_len = drv->ext->rlen;
#if 1
    if( drv->cfg->ctrl & MA_CFG_CTRL_DBG ) {
      drv->log->buf("bg95 rxbuf", rbuf, r_len);
    }
#endif
    /* check feedback */
    len32 = 0;
    do {
      /*
      \r\n+QIRD: XXX\r\n
      ...RAW PAYLOAD...
      \r\nOK\r\n
      or
      \r\n+QIRD: 0\r\n
      \r\nOK\r\n
      */
      for( u = 9; u < r_len; u++ ) {
        /* filter \r\n+QIRD: */
        if(drv->os->memcmp(&rbuf[u-9], "\r\n+QIRD: ", 9) == 0 ) {
          break;
        }
      }
      if( u >= r_len )
        break; /* search failed */
      r_len -= u;
      rbuf += u;

#if 0
      if( drv->cfg->ctrl & MA_CFG_CTRL_DBG ) {
        drv->log->buf("rx raw", rbuf, r_len);
      }
#endif
      /* filter XXX\r\n */    
      u = (pos_u8_t*)drv->os->sscan((char*)rbuf, "%n", &len32) - rbuf;
      u += 2; /* add \r\n */
      if( u >= r_len )
        break;
      rbuf += u;
      r_len -= u;
#if 0
      drv->log->buf("bg95 rx raw2", rbuf, r_len);
      drv->os->printf("u=%u r_len=%u len32=%u\n", u, r_len, len32);
#endif

      /* check length */
      if( len32 > r_len )
        len32 = 0;

      if( !len32 )
        break;

      /* copy to buf */
      drv->os->memcpy(buf, rbuf, len32);
      if( drv->cfg->ctrl & MA_CFG_CTRL_DBG ) {
        drv->log->buf("bg95 rx", buf, len32);
      }
    } while(0);

    if( len32 )
      break;
    
    if( loop == 0 ) {
      /* check +QIURC: "recv" */
      if( drv->ext->buf_search_pos(drv->ext->buf, drv->ext->rlen, 0, (pos_u8_t*)"+QIURC: \"recv\"", 14, (pos_u8_t*)"") ) {
        continue; /* SUCCESSFUL QIURC/RECV, re-read again*/
      }
      break; /* do NOT continue, if not QIURC detected */
    }
  }
  return len32;
}

/**
* @brief BG95 module receive (blocking)
*/
pos_i32_t drv_bg95_recvfrom(pos_u32_t sn, void * vbuf, pos_size_t max_len, pos_u8_t * addr, pos_u16_t *port, pos_u32_t timeout_ms) {
  pos_i32_t len32;
  if( !max_len )
    return 0;
  len32 = drv_ext_bg95_recvfrom_nb(sn, vbuf, max_len, addr, port);
  if( timeout_ms && len32 == 0 ) {
    g_drv->ext->cmd_query_check(POS_NULL, 0, (pos_u8_t*)"+QIURC: \"recv\"", timeout_ms);
    len32 = drv_ext_bg95_recvfrom_nb(sn, vbuf, max_len, addr, port);
  }
  return len32;
}

ma_drv_export_t g_export = {
  .version = DRV_NET_VERSION(MA_REPORT_MQTT_BG95, DRV_BG95_VERSION),
  .name = DRV_BG95_NAME,
  .u.net={
    drv_bg95_init,
    drv_bg95_run_set,
    drv_bg95_net_ready,
    drv_bg95_sock_open,
    drv_bg95_sock_close,
    drv_bg95_connect,
    drv_bg95_sendto,
    drv_bg95_recvfrom,
  },
};

