/**
 * @file  drv_n306.c
 * @brief NET N306 driver
 * @author Runby F.
 * @date 2021-1-28
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * Revision History
v1.0 @ 2023-02-07
  1) First version draft
  2) Add PSM init
  3) Add AT polling during power cycle
  4) Support CSQ to signal during ready 

v1.1 @ 2023-02-09
  1) Fix signal not correct issue
  2) Support report time record
  3) Enhance INIT by ECBAND  

v1.2 @ 2023-02-11
  1) Correct NB not register issue under new modules

v1.3 @ 2023-02-13
  1) Decrease PSM active timer to 32s and set TAU to 24H
  2) Fix sometime Link check timer stop too early issue

v1.4 @ 2023-02-14
  1) Support new PSM+HIBNATE logic

v1.5 @ 2023-02-18
  1) Enhance PSM+RUN set logic for better detecting HIB state
  2) Decrease MAX HIB time from 5s to 2s

v1.6 @ 2023-03-01
  1) Fix mqtt connect instable issue by 30s connect waiting
  2) Reset module in 16s with 3s power off time to avoid chip hang during init
  3) Avoid to call connect with NULL addr during sendto functions

v1.7 @ 2023-03-16
  1) Support NULL report after module init failed after about 2.5 minutes init

v1.8 @ 2023-05-06
  1) Support CT LWM2M

v1.9 @ 2023-07-27
  1) Try long delay in init
  2) Support to get csq during init

v1.10 @ 2023-07-31
  1) Support LWM2M when report is 0x42xx

v1.11 @ 2023-09-26
  1) Support OneNET OneJSON OTA command
  2) Use 30s send sync waiting
  3) Support OneNET IPSO mode
  
*/

/**
 * @brief Driver version
 */
#define DRV_N306_VERSION 0x010b

/**
 * @brief Driver name
 */
#define DRV_N306_NAME  "N306"

/**
* @brief N306 module LWM2M event result
*/
pos_u32_t drv_n306_lwm2m_event_rslt(char *s, pos_size_t s_len) {
  drv_api_t *drv = g_drv;  
  ma_ext_ctrl_t *ext = drv->ext;
  pos_u32_t v = 0;
  const char *cb;
  cb = drv->os->util->strfind_skip(s, "+MIPLEVENT:", s_len, 0 );      
  if( cb )
      v = ext->module_opr(MA_EXT_MODULE_OPR_GET_U32(1),(pos_u32_t)cb);    
  return v;
}

/**
* @brief N306 module LWM2M disconnect
*/
pos_status_t drv_n306_lwm2m_disconnect(void) {
  ma_ext_ctrl_t *ext = g_drv->ext;
  ext->cmd_query((pos_u8_t*)"AT+MIPLCLOSE=0",0);
  ext->cmd_query((pos_u8_t*)"AT+MIPLDELETE=0",0);  
  ext->one.arg[0] = 0;
  ext->one.arg[1] = 0;
  ext->flag &= ~MA_EXT_FLAG_MODULE_CONNECT;
  return POS_STATUS_OK;
}

/**
* @brief N306 module LWM2M send sync
*/
pos_status_t drv_n306_lwm2m_send_sync(pos_u32_t timeout) {
  drv_api_t *drv = g_drv;  
  pos_u32_t v, code;
  ma_ext_ctrl_t *ext = drv->ext;
  if( !timeout )
    timeout = 30000;
  timeout = drv->os->tick->get() + timeout;
  do {
    v = ext->module_opr(MA_EXT_MODULE_OPR_READLN, 3000);
    if( v == 0 )
      continue;
    
    /* check +MIPLEVENT: 0,26 */
    code = drv_n306_lwm2m_event_rslt((char*)ext->buf, v);
    if( !code )
      continue;
    
    switch( code ) {
    case 26:
      return POS_STATUS_OK;
      break; /* register okay */

    case 25:
      return POS_STATUS_E_RESOURCE;
      break;
      
    default:
      return POS_STATUS_ERROR;
      break;
    }     
  } while(!drv->os->tick->is_timeout(drv->os->tick->get(), timeout));

  return POS_STATUS_E_TIMEOUT;
}

/**
* @brief N306 module LWM2M event WRITE
*/
pos_u32_t drv_n306_lwm2m_rx_chk(char *s, pos_size_t s_len, pos_u8_t *vbuf, pos_size_t max_len) {
  drv_api_t *drv = g_drv;  
  ma_ext_ctrl_t *ext = drv->ext;
  pos_u32_t v = 0;
  const char *cb;
/*
+MIPLWRITE: 0,40236,19,1,0,2,86,7B226964223A223234222C2276657273696F6E223A22312E30222C22706172616D73223A7B226474756D223A223131323233333434227D2C226D6574686F64223A227468696E672E70726F70657274792E736574227D,0,0
*/
  cb = drv->os->util->strfind_skip(s, "+MIPLWRITE:", s_len, 0 );      
  do {
    pos_u32_t rsp_id, pl_len;    
    const char *end;
    if( !cb ) 
      break;
    v = ext->module_opr(MA_EXT_MODULE_OPR_GET_U32(1),(pos_u32_t)cb);    
    if( !v )
      break;

    rsp_id = v;
    cb = drv->os->util->strfind_skip(cb, ",", drv->os->strlen(cb), 6 );
    if( !cb )
      break;

    v = max_len;
    drv->os->str2bin((char*)&cb[1], vbuf, &v,DECODE_HEXSTR);

    /* oneJSON processing for dtum */
    if(drv->cfg->ctrl & MA_CFG_CTRL_ONENET_MODE)	{
      if( v >= max_len )
        v--;
      vbuf[v] = 0;
      cb = drv->os->util->strfind_skip((const char*)vbuf, "\"dtum\":\"", v, 0 );
      if( !cb ) {
        v = 0;
        break; /* signature error */
      }

      cb = &cb[8]; /* length of "dtum":" */
      end = drv->os->util->strfind_skip(cb, "\"", drv->os->strlen(cb), 0);
      if( end < cb ) {
        v = 0;
        break; /* payload error */
      }

      /* translate from 
       * {"id":"26","version":"1.0","params":{"dtum":"11223344"},"method":"thing.property.set"}
       * to
       * {"cmd":"11223344"}
       */
      pl_len = end - cb;
      drv->os->strcpy((char*)vbuf, "{\"cmd\":\"");
      drv->os->memcpy(&vbuf[8], cb, pl_len);
      v = pl_len + 8;     
      drv->os->strcpy((char*)&vbuf[v], "\"}");
      v += 2;
    }
    
    /* write rsp */
    drv->os->sprintf((char*)ext->buf, "AT+MIPLWRITERSP=0,%u,2", rsp_id);
    ext->cmd_query(ext->buf, 0);      
    
  } while(0);
  return v;
}

/**
* @brief N306 module LWM2M connect
*/
pos_status_t drv_n306_lwm2m_connect(pos_u32_t timeout) {
  pos_status_t ret;
  drv_api_t *drv = g_drv;  
  pos_u32_t v;
  const char *cb;
  ma_ext_ctrl_t *ext = drv->ext;
  /* when arg[0] is defined (connection is valid),
   * return OK and do nothing 
   */
  if( ext->one.arg[0] )
    return POS_STATUS_OK;
  
  do {
    ret = ext->cmd_query((pos_u8_t*)"AT+MIPLCREATE", 0);  
    if( ret != POS_STATUS_OK )
      break;

#if 1
    /* bind 19/0/0 and 19/1/0 for OneJSON */
    if(drv->cfg->ctrl & MA_CFG_CTRL_ONENET_MODE) {
      ret = ext->cmd_query((pos_u8_t*)"AT+MIPLADDOBJ=0,19,2,\"11\",2,2", 0);  
    } else {
    /* bind 3300/0 for IPSO */		
      ret = ext->cmd_query((pos_u8_t*)"AT+MIPLADDOBJ=0,3300,1,\"1\",1,1", 0);  
    }
#else
    /* bind 19/0/0 */
    ret = ext->cmd_query((pos_u8_t*)"AT+MIPLADDOBJ=0,19,1,\"1\",1,1", 0);  
#endif
    if( ret != POS_STATUS_OK )
      break;
    
    ret = ext->cmd_query((pos_u8_t*)"AT+MIPLOPEN=0,86400", 0);  
    if( ret != POS_STATUS_OK )
      break;
    
  } while(0);
  if( !timeout )
    timeout = 30000;
  timeout = drv->os->tick->get() + timeout;
  do {
    v = ext->module_opr(MA_EXT_MODULE_OPR_READLN, 3000);
    if( v == 0 )
      continue;
    
    /* check +MIPLOBSERVE: 0,97442,1,19,0,-1 */
    cb = drv->os->util->strfind_skip((const char*)ext->buf, "+MIPLOBSERVE:", v, 0 );
    if( cb ) {
      pos_u8_t id;
      v = drv->ext->module_opr(MA_EXT_MODULE_OPR_GET_U32(1),(pos_u32_t)cb);
      id = drv->ext->module_opr(MA_EXT_MODULE_OPR_GET_U32(4),(pos_u32_t)cb); /* instance */
      drv->os->sprintf((char*)ext->buf, "AT+MIPLOBSERVERSP=0,%u,1", v);
      if( id < POS_ARRAY_CNT(ext->one.arg) )
        ext->one.arg[id] = v; /* record OBSERVE msgid into specified instance id */
      ext->cmd_query(ext->buf, 0);
      continue;
    } 

    /* check +MIPLDISCOVER: 0,31907,19 */
    cb = drv->os->util->strfind_skip((const char*)ext->buf, "+MIPLDISCOVER:", v, 0 );    
    if( cb ) {
      v = drv->ext->module_opr(MA_EXT_MODULE_OPR_GET_U32(1),(pos_u32_t)cb);
      drv->os->sprintf((char*)ext->buf, "AT+MIPLDISCOVERRSP=0,%u,1,1,\"1\"", v);
      ext->cmd_query(ext->buf, 0);
      continue;
    } 

    /* check +MIPLEVENT: 0,11 */
    if( drv_n306_lwm2m_event_rslt((char*)ext->buf, v) == 11 ) {
      ret = POS_STATUS_OK;
      break; /* register okay */
    }     
  } while(!drv->os->tick->is_timeout(drv->os->tick->get(), timeout));

  /* release connection when failed */
  if( ret != POS_STATUS_OK ) {
    drv_n306_lwm2m_disconnect();
  }
  return ret;
}

/**
* @brief N306 module init
*/
pos_status_t drv_n306_init(void) {
  pos_u8_t flag;
  pos_u32_t timeout;
  drv_api_t *drv = g_drv;  
  ma_ext_ctrl_t *ext = drv->ext;
  ext->mtu = 400; /* use middle mtu */
  ext->tx_rt[0] = 1; /* \r only */
  ext->one.arg[0] = 0;
  ext->one.arg[1] = 0;  
  do {
    /* reset */
    if( ext->run_level > RUN_LEVEL_POWER_OFF ) {
      ext->cmd_query((pos_u8_t*)"AT+CFUN=0", 0); /* stop all before reset */
    }

    if( drv->cfg->ctrl & MA_CFG_CTRL_POWER_SAVING )
      ext->cmd_query((pos_u8_t*)"AT+ECRST", 500); /* PSM might not reset module correctly, reboot manually */

    timeout = 0;
    ext->module_power_set(0);    
    while(1) {
      pos_u32_t i;
      drv->os->tick->sleep(10000);      
      ext->module_power_set(1);    
      drv->os->tick->sleep(6000);      /* power on sleep */
      for( i  = 0; i < 5; i++ ) {
        if( ext->cmd_query((pos_u8_t*)"AT", 3000) == POS_STATUS_OK ) {
          break;
        }
      }
      if( i < 5 ) 
        break;
      /* power off 10s and try again */
      ext->module_power_set(0);

      /* Support NULL report when too much failed */
      if( ++timeout >= 4 ) {
        /* about 8*16 + 7*3 = 149s, 2.5min later, use NULL report */
        ext->report_type = MA_REPORT_NATIVE_NULL;
        return POS_STATUS_OK;
      }
      
    }
    do {
      if( ext->cmd_query((pos_u8_t*)"ATE0", 0) == POS_STATUS_OK )
        break;
    } while(1);
//    ext->cmd_query((pos_u8_t*)"AT+QREGSWT=2", 0); /* normal register */

    /* print IMEI and CIMI */
    drv->ext->module_opr(MA_EXT_MODULE_OPR_PRINT_IMEI_CIMI, 0);  

    /* Turn ON HIBNATE notification */
    ext->cmd_query((pos_u8_t*)"AT+ECPURC=\"HIBNATE\",1", 0); 

    /* Turn OFF PSM notification */
    ext->cmd_query((pos_u8_t*)"AT+ECPSMR=0", 0); 

    /* PSM/None-PSM mode */
#if 0
    drv->os->sprintf((char*)g_drv->ext->buf, "AT+CPSMS=%u,,,,00011111", 
      (drv->cfg->ctrl & MA_CFG_CTRL_POWER_SAVING) == 0 ? 0 : 1);
    ext->cmd_query(g_drv->ext->buf, 0);

#else
    ext->cmd_query((pos_u8_t*)"AT+CPSMS=1,,,00111000,00010000", 0); /*TAU=24H, ACT=16 *2 (000=2s interval) = 32s */
    ext->cmd_query((pos_u8_t*)"AT+ECCFG=\"T3324MaxValueS\",2", 0); /* It can be very fast, as HIB is disabled now. */

#endif


    /* Use AutoAPN */
    ext->cmd_query((pos_u8_t*)"AT+ECCFG=\"AutoApn\",1", 0);
    
    /* check APN/BAND */
    flag = 0;
    do {
#if 0      
      if( ext->cmd_query_check((pos_u8_t*)"AT+CGDCONT?", 22, (pos_u8_t*)g_drv->cfg->net.apn, 0) != POS_STATUS_OK ) {
        break;
      }
#endif      
      flag |= 1;
      if( ext->cmd_query_check((pos_u8_t*)"AT+ECBAND?", 11, (pos_u8_t*)drv->cfg->net.band, 0) != POS_STATUS_OK ) {
        break;
      }
      flag |= 2;
    } while(0);
    if( flag == 3 )
      break; /* quit if APN/BAND are matched */

    /* set APN/BAND */
#if 0 /* according to AE from N306, it does NOT support APN change and use AutoAPN feature */   
    ext->cmd_query((pos_u8_t*)"AT+CGATT=0", 60000); /* stop func before APN/BAND change */
    drv->os->sprintf((char*)g_drv->ext->buf, "AT+CGDCONT=0,\"IP\",\"%s\"", g_drv->cfg->net.apn);
    ext->cmd_query(g_drv->ext->buf, 0);
    ext->cmd_query((pos_u8_t*)"AT+CFUN=0", 60000); /* full stop func before APN/BAND change */
#endif    
    drv->os->sprintf((char*)drv->ext->buf, "AT+ECBAND=0,%s", drv->cfg->net.band);
    ext->cmd_query(drv->ext->buf, 0);
  } while(1);

#if 1
  /* start network */
  g_drv->ext->cmd_query((pos_u8_t*)"AT+CEREG=5", 0);
#endif

  if( MA_REPORT_TYPE_IS_NATIVE(ext->report_type) ) {
    /* ONENET LWM2M INIT */
    /*    ext->cmd_query((pos_u8_t*)"AT+MIPLCONFIG=7,3,1", 0); do NOT enable buffer URC, it does NOT work!!! */
  } else if( drv->cfg->net.rsvr_port == 5683  ) {
    /* CT LWM2M INIT */
    /* start CTLWM2M */
    drv->ext->cmd_query((pos_u8_t*)"AT+CTM2MSETPM=180.101.147.115,5683,86400", 0);
  }

  /* get csq */
  drv->history->signal = drv->ext->module_opr(MA_EXT_MODULE_OPR_GET_SIGNAL,0);  
  
  return POS_STATUS_OK;
}

/**
* @brief N306 working state set
*/
pos_status_t drv_n306_run_set(pos_run_level_t level) {
  drv_api_t *drv = g_drv;
  pos_u32_t i;
  pos_status_t ret = POS_STATUS_OK;  
  if( level == RUN_LEVEL_POWER_OFF ) {
    // NOTHING to do g_drv->ext->cmd_query((pos_u8_t*)"AT+CFUN=0", 0); /* stop all before power off */
  } else if( level == RUN_LEVEL_POWER_SAVING ) {    
    /* PSM模式 */
    if(  (g_drv->ext->flag & MA_EXT_FLAG_MODULE_PSM_ACK) == 0 ) {
      drv->ext->cmd_query((pos_u8_t*)"AT+ECNBIOTRAI=1", 0); /* Release RRC Quickly */    

      /* wait for PSM notificaiton */
      for( i = 0; i < 3; i++ ) {
        drv->ext->cmd_send((pos_u8_t*)"AT+ECPMUCFG=1,4", 15); /* Re-Enable Hibnate */        
        if( i >= 2 ) /* wait at most two rounds */
          break;
        ret = drv->ext->cmd_query_check(POS_NULL, 0, (pos_u8_t*)"+HIB Enter", 32000);
        if( ret == POS_STATUS_OK )
          break;
      }
      if( ret == POS_STATUS_OK ) {
        g_drv->ext->flag |= MA_EXT_FLAG_MODULE_PSM_ACK; /* this flag will be flush by DATA_REPORT in the next round */
      } else {
        /* no nitification */
        /* do nothing now */
      }
    }
  } else {
  }
  return ret;
}

/**
* @brief N306 module link ready check
*/
pos_status_t drv_n306_net_ready(pos_u32_t timeout_ms) {
  pos_status_t ret;
  drv_api_t *drv = g_drv;
  pos_u16_t v;  
  pos_u32_t old, ctlwm2m = 0;
#if 0  
  pos_u8_t i;  
  i = 0;
#endif  

  drv->ext->cmd_query((pos_u8_t*)"AT",500); /* Wakeup from PSM */
  ret = drv->ext->cmd_query((pos_u8_t*)"AT+ECPMUCFG=1,1", 0); /* Disable Hibnate */
  drv->ext->flag &= ~MA_EXT_FLAG_MODULE_PSM_ACK;  /* clear PSM ACK */

#if 1
  drv->ext->cmd_query((pos_u8_t*)"AT+CEREG=5", 0); /* 激活注册 */
#endif

  /* active CTLWM2M */
  if( drv->cfg->net.rsvr_port == 5683 && !MA_REPORT_TYPE_IS_NATIVE(drv->ext->report_type) ) {
    ctlwm2m = 1;
  }

  old = drv->os->tick->get();
  do {
    ret = drv->ext->cmd_query_check((pos_u8_t*)"AT+CEREG?", 0, (pos_u8_t*)"+CEREG: 5,1",0);
#if 0 
    if( !timeout_ms )
      break;
    if( ret != POS_STATUS_OK && i == 0 ) {
      drv->ext->cmd_query_check(POS_NULL, 0, (pos_u8_t*)"+CEREG: 1", timeout_ms);
    }    
    ++i;
  } while( ++i < 2 );
#else
    if( ret == POS_STATUS_OK && ctlwm2m ) {
      ret = drv->ext->cmd_query_check((pos_u8_t*)"AT+CTM2MREG?", 0, (pos_u8_t*)"+CTM2MREG: 1",0);
      if( ret != POS_STATUS_OK &&  ctlwm2m == 1 ) {
        drv->ext->cmd_query((pos_u8_t*)"AT+CTM2MREG", 0);
        ctlwm2m = 2; /* do NOT call REG again */
      }
    }
    
    if( ret == POS_STATUS_OK )
      break;
    drv->os->tick->sleep(2000);
  } while( drv->os->tick->elaps(old) < timeout_ms );
#endif      
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("n306 link status", ret);
  }
  v = 0;
  if( ret == POS_STATUS_OK )
    v = drv->ext->module_opr(MA_EXT_MODULE_OPR_GET_SIGNAL,0);

  if( ret == POS_STATUS_OK &&
      drv->ext->cmd_query_check((pos_u8_t*)"AT+CCLK?", 0, (pos_u8_t*)"+CCLK: ",0) == POS_STATUS_OK ) {
      /* +CCLK: "2023/02/09,08:51:58+32" */
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
      if( *n == '+' || *n == '-' ) {
        drv->os->sscan(n+1, "%n", &t->tz_quarter );
        if( *n == '-' )
          t->tz_quarter = 0 - t->tz_quarter;
        adj = t->tz_quarter;
        adj *= 15*60; /* quarter to seconds */
        drv->data->plss_time_adj(t, adj);
        drv->os->memcpy(&drv->history->time, t, sizeof(*t));
#if 0
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
* @brief N306 module socket open
*/
pos_status_t drv_n306_sock_open(pos_u32_t *p_sock, pos_net_trans_t tcp, pos_u16_t port) {
  drv_api_t *drv = g_drv;
  if( drv->cfg->net.rsvr_port == 5683 ) {
    return POS_STATUS_OK;
  }
  
  drv->os->sprintf((char*)drv->ext->buf, "AT+SKTCREATE=1,%u,%u", tcp ? 1 : 2, tcp ? 6 : 17); 

  if( drv->ext->cmd_query(drv->ext->buf,0) == POS_STATUS_OK ) {
    if( drv->ext->buf[14] >= '0' && drv->ext->buf[14] <= '9' ) {
      *p_sock = drv->ext->buf[14] - '0';
      return POS_STATUS_OK;
    }
  }
  *p_sock = 0; /* always use 0 when failed */
  return POS_STATUS_ERROR;
}

/**
* @brief N306 module socket close
*/
pos_status_t drv_n306_sock_close(pos_u32_t sock) {  
  drv_api_t *drv = g_drv;

  drv->ext->flag &= ~MA_EXT_FLAG_MODULE_CONNECT; // REMOVE connect flag during close

  if( drv->cfg->net.rsvr_port == 5683 ) {
    return POS_STATUS_OK;
  }
    
  drv->os->sprintf((char*)drv->ext->buf, "AT+SKTDELETE=%u", sock);  
  return drv->ext->cmd_query(drv->ext->buf,0);
}

/**
* @brief N306 module socket connect
*/
pos_status_t drv_n306_connect(pos_u32_t sn, pos_u8_t * addr, pos_u16_t port) {
  pos_status_t ret;
  drv_api_t *drv = g_drv;
  ma_ext_ctrl_t *ext = drv->ext;    
  if( MA_REPORT_TYPE_IS_NATIVE(ext->report_type) ) {
    /* ONENET LWM2M */
    ret = drv_n306_lwm2m_connect(30000);
  }
  else if( drv->cfg->net.rsvr_port == 5683 ) {
    /* CTLWM2M do nothing */
    ret = POS_STATUS_OK;
  } else {
    drv->os->sprintf((char*)g_drv->ext->buf, "AT+SKTCONNECT=%u,%u.%u.%u.%u,%u", sn, addr[0],addr[1],addr[2],addr[3],port);
    ret = ext->cmd_query(g_drv->ext->buf,30000); 
  }
  if( ret == POS_STATUS_OK ) {
    ext->flag |= MA_EXT_FLAG_MODULE_CONNECT;
  }
  return ret;
}

/**
* @brief N306 module sendto
*/
pos_i32_t drv_n306_sendto(pos_u32_t sn, void * vbuf, pos_size_t len, pos_u8_t * addr, pos_u16_t port) {
  pos_size_t v;
  drv_api_t *drv = g_drv;
  ma_ext_ctrl_t *ext = drv->ext;   

  if( (ext->flag & MA_EXT_FLAG_MODULE_CONNECT) == 0 ) {
    /* Connect if not connect flags set AND addr is valid */
    drv_n306_connect(sn, addr, port);
  }  

  /* 如果模式没有CONNECT, 代表进入失败，返回出错 */  
  if( (ext->flag & MA_EXT_FLAG_MODULE_CONNECT) == 0 ){
    return -1;
  }  

  /* ONENET L2M2M Processing */
  if( MA_REPORT_TYPE_IS_NATIVE(ext->report_type) ) {
    if(drv->cfg->ctrl & MA_CFG_CTRL_ONENET_MODE) {
      /* ONEJSON LWM2M SEND 2:OPAQUE */
      /* translate: {"id":"123","params":{"co2":{"value":500},"hum":{"value":60},"tmp":{"value":28},"vbat":{"value":3300}},"method":"thing.property.post"} */
      /* cmd: AT+MIPLNOTIFY=0,9062,19,0,0,2,134,"7b226964223a22313233222c22706172616d73223a7b22636f32223a7b2276616c7565223a3530307d2c2268756d223a7b2276616c7565223a36307d2c22746d70223a7b2276616c7565223a32387d2c2276626174223a7b2276616c7565223a333330307d7d2c226d6574686f64223a227468696e672e70726f70657274792e706f7374227d",0,0,4 */
      drv->os->sprintf((char*)ext->buf, "AT+MIPLNOTIFY=0,%u,19,0,0,2,%u,\"",ext->one.arg[0], len+31);
      /*31B: ,"method":"thing.property.post" */
      v = drv->os->strlen((char*)ext->buf);
      if( v + (31+len+22)*2 >= ext->buf_size ) {
        return -1; /* length overflow */
      }
      drv->os->bin2str((char*)&ext->buf[v], (pos_u8_t*)vbuf, len-1, ENCODE_HEXSTR_UPPER); /* final } should be removed */
      v += (len-1)*2;
      vbuf = ",\"method\":\"thing.property.post\"}"; /* append another } */    
      drv->os->bin2str((char*)&ext->buf[v], (pos_u8_t*)vbuf, 31+1, ENCODE_HEXSTR_UPPER); 
      v += (31+1)*2;
    } else {
      /* IPSO LWM2M SEND 2:OPAQUE */
      /* AT+MIPLNOTIFY=0,XXX,3300,0,5750,2,8,"aabbccdd00112233",0,0 */
      drv->os->sprintf((char*)drv->ext->buf,"AT+MIPLNOTIFY=0,%u,3300,0,5750,1,%u,\"",ext->one.arg[0],len*2);
      v = drv->os->strlen((char*)drv->ext->buf);
      drv->os->bin2str((char*)&drv->ext->buf[v], (pos_u8_t*)vbuf, len, ENCODE_HEXSTR_UPPER); 
      v = drv->os->strlen((char*)drv->ext->buf);
    }    

    drv->os->sprintf((char*)&ext->buf[v], "\",0,0,%u", drv->duty_loop);
    if( drv->ext->cmd_query(ext->buf,0) == POS_STATUS_OK ) {
      if( drv_n306_lwm2m_send_sync(0) == POS_STATUS_OK ) {
        drv->call(DRV_CALL_FUNC_ID_SET_ACK_CLEAR, 0); /* clear ack timeout when synced */
        return len;
      }
    }
    return -1;
  } 

  /* TCP SEND */  
  
  if( drv->cfg->net.rsvr_port == 5683 ) {
    /* CT LWM2M will carry a U8 length in the beginning of payload */
    drv->os->sprintf((char*)ext->buf, "AT+CTM2MSEND=%02X",len&0xff);
  } else {
    drv->os->sprintf((char*)ext->buf, "AT+SKTSEND=%u,%u,", sn, len);
  }
  v = drv->os->strlen((char*)ext->buf);
  drv->os->bin2str((char*)&ext->buf[v], (pos_u8_t*)vbuf, len, ENCODE_HEXSTR_UPPER);
  return drv->ext->cmd_query(ext->buf,0) == POS_STATUS_OK ? len : -1;
}

/**
* @brief N306 module receive (blocking)
*/
pos_i32_t drv_n306_recvfrom(pos_u32_t sn, void * vbuf, pos_size_t max_len, pos_u8_t * addr, pos_u16_t *port, pos_u32_t timeout_ms) {
  pos_i32_t len32 = 0;
  drv_api_t *drv = g_drv;
  ma_ext_ctrl_t *ext = drv->ext;
  pos_u32_t tick, elaps, t;
  if( !max_len )
    return 0;

  tick = drv->os->tick->get();
  t = timeout_ms;
  while( 1 ) {  
    if( MA_REPORT_TYPE_IS_NATIVE(ext->report_type) ) {
      /* 
       * filter rx
       */      
      ext->module_opr(MA_EXT_MODULE_OPR_READLN, 3000); /* readln */
      if( ext->rlen ) {
        len32 = drv_n306_lwm2m_rx_chk((char *)ext->buf, ext->rlen, (pos_u8_t*)vbuf, max_len);
      }
      
    } else if( drv->cfg->net.rsvr_port == 5683 ) {
      if( ext->cmd_query_check(POS_NULL, 0, (pos_u8_t*)"+CTM2MRECV: ", t) == POS_STATUS_OK ) {
        pos_u8_t *rbuf;    
        pos_size_t s;
        rbuf = (pos_u8_t*)&ext->buf[14]; /* 0d 0a +CTM2MRECV */        
        if( ext->rlen > 14 )
          s = ext->rlen - 14;
        else
          s = 0;
        drv->os->str2bin((char*)rbuf, (pos_u8_t*)vbuf, &s,DECODE_HEXSTR);
        len32 = s;
        if( len32 ) {
          pos_u8_t *b8 = (pos_u8_t*)vbuf;
          if( len32 != b8[0]+1 ) {
            len32 = 0; /* bad payload. the first u8 should be length */
          } else {
            len32--;
            drv->os->memcpy(b8, b8+1, len32); /* adjust payload */
          }
          
        }
      }
    } else if( ext->cmd_query_check(POS_NULL, 0, (pos_u8_t*)"+SKTRECV: ", t) == POS_STATUS_OK ) {
      pos_u8_t *rbuf;    
      pos_u32_t sock;
      rbuf = (pos_u8_t*)drv->os->sscan( (char*)&ext->buf[12], "%4n,%4n,\"", &sock, &len32 );
      if( len32 > max_len )
        len32 = max_len;
      else if( len32 < 0 )
        len32 = 0;
      if( sock != sn ) {
        len32 = 0; /* ignore wrong socket recv */
      }
      
      if( len32 ) {
        pos_u32_t i;
        i = len32;
        drv->os->str2bin((char*)rbuf, (pos_u8_t*)vbuf, &i,DECODE_HEXSTR);
        if( i != len32 )
          len32 = 0; /* wrong payload */
      }
    }

    /* recv quit */
    if( len32 )
      break;

    /* timeout quit */
    elaps = drv->os->tick->elaps(tick);
    if( elaps >= timeout_ms )
      break; 

    /* recheck using new timeout ms */
    t = timeout_ms - elaps;
  }
  
  return len32;
}


ma_drv_export_t g_export = {
  .version = DRV_NET_VERSION(MA_REPORT_MQTT_N306, DRV_N306_VERSION),
  .name = DRV_N306_NAME,
  .u.net={
    drv_n306_init,
    drv_n306_run_set,
    drv_n306_net_ready,
    drv_n306_sock_open,
    drv_n306_sock_close,
    drv_n306_connect,
    drv_n306_sendto,
    drv_n306_recvfrom,
  },
};

