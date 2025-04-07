/**
 * @file  drv_a780.c
 * @brief NET A780 driver
 * @author Runby F.
 * @date 2022-4-25
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * Revision History
v1.0 @ 2022-12-26
  1) Init first version

v1.1 @ 2023-03-08
  1) Reset module in 16s with 3s power off time to avoid chip hang during init

v1.2 @ 2023-05-11
  1) Support TIMESYNC and CSQ signal sync

v1.3 @ 2023-05-17
  1) increase mtu to 1024

v1.4 @ 2023-06-01
  1) Add booting up delay

v1.5 @ 2023-070-07
  1) Use generic CSQ
  
*/

/**
 * @brief Driver version
 */
#define DRV_A780_VERSION 0x0105

/**
 * @brief Driver name
 */
#define DRV_A780_NAME  "A780"

/**
 * @brief UDP Socket
 */
#define DRV_A780_SOCK_UDP 0

/**
 * @brief TCP Socket
 */
#define DRV_A780_SOCK_TCP 1

/**
* @brief A780 module init
*/
pos_status_t drv_a780_init(void) {
  drv_api_t *drv = g_drv;  
  ma_ext_ctrl_t *ext = drv->ext;
  pos_u8_t i;
  ext->mtu = 1024; /* use large mtu. More than 1024 requires to change TRANS CFG */  
  ext->tx_rt[0] = 1; /* \r only */
  do {
    /* reset */
#if 0    
    if( ext->run_level > RUN_LEVEL_POWER_OFF || (drv->cfg->ctrl & MA_CFG_CTRL_POWER_SAVING ) ) {
      ext->cmd_query((pos_u8_t*)"AT+CFUN=0,1", 0); /* stop all before reset */
    }
#endif    
    while(1) {
      pos_u32_t i;
      ext->module_reset();
      for( i  = 0; i < 16; i++ ) {
        if( ext->cmd_query((pos_u8_t*)"AT", 1000) == POS_STATUS_OK ) {
          break;
        }
      }
      if( i < 16 ) 
        break;
      /* power off 3s and try again */
      ext->module_power_set(0);
      drv->os->tick->sleep(3000);
    }

    for( i = 0; i < 10; i++ ) {
      drv->os->tick->sleep(2000); /* system bootup delay */ 
      if( ext->cmd_query((pos_u8_t*)"ATE0", 0) == POS_STATUS_OK ) /* Echo OFF */
        break;      
    } 

    for( i = 0; i < 10; i++ ) {
      drv->os->tick->sleep(2000); /* system bootup delay */ 
      /* set transparent mode */
      if( ext->cmd_query((pos_u8_t*)"AT+CIPMODE=1",0) == POS_STATUS_OK )
        break;
    }
#if 0    
    ext->cmd_query((pos_u8_t*)"AT+CIPCCFG=5,2,1024,1",0);
#endif    
    /* set APN/BAND */
    for( i = 0; i < 10; i++ ) {
      drv->os->tick->sleep(2000); /* system bootup delay */ 
      drv->os->sprintf((char*)g_drv->ext->buf, "AT+CSTT=\"%s\",\"\",\"\"", g_drv->cfg->net.apn);
      if( ext->cmd_query(g_drv->ext->buf, 0) == POS_STATUS_OK )
        break;
    }
    break;// break, as it sometime does returns error. break;
  } while(1);

  /* start network */  
  drv->history->signal = drv->ext->module_opr(MA_EXT_MODULE_OPR_GET_SIGNAL,4);  
  
  return POS_STATUS_OK;
}

/**
* @brief A780 module working state set
*/
pos_status_t drv_a780_run_set(pos_run_level_t level) {
  if( level == RUN_LEVEL_POWER_OFF ) {
    // DO NOTHING g_drv->ext->cmd_query((pos_u8_t*)"AT+CFUN=0", 0); /* stop all before power off */
  } else {
    /* 无需做任何动作, 模组自身通过APN自动进入PSM模式 */
    
  }
  return POS_STATUS_OK;
}

/**
* @brief A780 module link ready check
*/
pos_status_t drv_a780_net_ready(pos_u32_t timeout_ms) {
  pos_status_t ret;
  drv_api_t *drv = g_drv;
  ma_ext_ctrl_t *ext = drv->ext;
  const pos_lib_tick_t *tick = drv->os->tick;
  pos_u16_t v;
  timeout_ms +=tick->get();
  do {
    ret = ext->cmd_query_check((pos_u8_t*)"AT+CREG?", 0, (pos_u8_t*)"+CREG: 0,1",0);
    if( ret == POS_STATUS_OK )
      break;
   
    tick->sleep(1000);
  } while( !tick->is_timeout(tick->get(), timeout_ms) );

  if( ret == POS_STATUS_OK ) {
    ret = POS_STATUS_E_TIMEOUT;
    ext->cmd_query((pos_u8_t*)"AT+CIICR", 1000); /* 激活链路 */
    do {
      ext->cmd_send((pos_u8_t*)"AT+CIFSR",8);
      ext->rlen = ext->io->read(ext->io, ext->buf, ext->buf_size, 400);
      ext->buf[ext->rlen] = 0;    
      if( ext->rlen ) {
        pos_u32_t i, ip;
        /* dbg show rx */
        if( drv->cfg->ctrl & MA_CFG_CTRL_DBG ) {    
          drv->os->printf( "\r\n<==%s\r\n", ext->buf );
          drv->os->flush();
        }        
        /* filter IP address */
        for( i = 0; i < ext->rlen; i++ ) {
          if( ext->buf[i] >= '0' && ext->buf[i] <= '9' ) {
            ip = 0;
            ip = drv->os->util->str2u32((const char*)&ext->buf[i]);
            if( drv->cfg->ctrl & MA_CFG_CTRL_DBG ) {
              drv->log->data("a780 ip",ip);
              drv->log->buf("ip str", &ext->buf[i], ext->rlen - i );
            }
            if( ip > 0x01000000 ) {
              ret = POS_STATUS_OK; /* set VALID IP flag */
            }
            break;
          }
        }
      }  
      if( ret == POS_STATUS_OK )
        break;
      tick->sleep(1000);      
    } while( !tick->is_timeout(tick->get(), timeout_ms) );
  }
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("a780 link status", ret);
  }
  v = 0;
  if( ret == POS_STATUS_OK )
    v = drv->ext->module_opr(MA_EXT_MODULE_OPR_GET_SIGNAL,4); /* CAT-1 use x4 signal ratio */
  drv->history->signal = v;

  if( ret == POS_STATUS_OK &&
      drv->ext->cmd_query_check((pos_u8_t*)"AT+CCLK?", 0, (pos_u8_t*)"+CCLK: ",0) == POS_STATUS_OK ) {
      /* +CCLK: "2023/02/09,08:51:58+32" */
    if( drv->ext->buf[9] == '"' ) {
      char *n;
      drv_data_time_t _t, *t = &_t;
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
/* A780 is already using local timestamp. 
 * So there's no need to translate by timezone
 */
#if 0        
        {
          pos_i32_t adj;
          adj = t->tz_quarter;
          adj *= 15*60; /* quarter to seconds */
          drv->data->plss_time_adj(t, adj);
        }
#endif        
        drv->os->memcpy(&drv->history->time, t, sizeof(*t));
#if 0
        if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
          drv->os->printf("%u-%u-%u %02u:%02u:%02u", t->year, t->month, t->day, t->hour, t->minute, t->second);
        }
#endif        
      }
    }
  }

#if 0
  drv->log->data("signal", v);
  drv->log->buf("time", &drv->history->time, 8);
#endif  
  return ret;
}

/**
* @brief A780 module socket open
*/
pos_status_t drv_a780_sock_open(pos_u32_t *p_sock, pos_net_trans_t tcp, pos_u16_t port) {
  if( tcp )
    *p_sock = DRV_A780_SOCK_TCP;
  else
    *p_sock = DRV_A780_SOCK_UDP;
  return POS_STATUS_OK;
}

/**
* @brief A780 module socket close
*/
pos_status_t drv_a780_sock_close(pos_u32_t sock) {  
  drv_api_t *drv = g_drv;
  ma_ext_ctrl_t *ext = drv->ext;  
  ext->flag &= ~MA_EXT_FLAG_MODULE_CONNECT;
  if( ext->flag & MA_EXT_FLAG_MODULE_TRANS ) {
    ext->quit_trans_mode();
    ext->flag &= ~MA_EXT_FLAG_MODULE_TRANS;
  }
  ext->cmd_send((pos_u8_t*)"AT+CIPCLOSE", 0);
  return ext->cmd_query_check(POS_NULL, 0, (pos_u8_t*)"CLOSE OK", 0);
}

/**
* @brief A780 module socket connect
*/
pos_status_t drv_a780_connect(pos_u32_t sn, pos_u8_t * addr, pos_u16_t port) {
  drv_api_t *drv = g_drv;
  ma_ext_ctrl_t *ext = drv->ext;
  pos_status_t ret;
  drv->os->sprintf((char*)ext->buf, "AT+CIPSTART=%s,%u.%u.%u.%u,%u", 
    sn == DRV_A780_SOCK_UDP ? "UDP" : "TCP",
    addr[0],addr[1],addr[2],addr[3],port);
  ret = ext->cmd_query_check(ext->buf, 0, (pos_u8_t*)"CONNECT\r", 0);
  if( ret != POS_STATUS_OK ) {
    /* 30s connect timeout */
    ret = ext->cmd_query_check(POS_NULL, 0, (pos_u8_t*)"CONNECT\r", 30000);
  }
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("a780 conn", ret);
  }
  ext->flag |= MA_EXT_FLAG_MODULE_TRANS; /* set trans flag */
  if( ret == POS_STATUS_OK )
    ext->flag |= MA_EXT_FLAG_MODULE_CONNECT;
  return ret;
}

/**
* @brief A780 module socket sendto (noneblocking)
*/
pos_i32_t drv_a780_sendto(pos_u32_t sn, void * vbuf, pos_size_t len, pos_u8_t * addr, pos_u16_t port) {
  drv_api_t *drv = g_drv;
  ma_ext_ctrl_t *ext = drv->ext;  
  pos_i32_t ret = -1;
  
  if( sn == DRV_A780_SOCK_UDP && (ext->flag & MA_EXT_FLAG_MODULE_CONNECT) == 0 ) {
    /* N58 UDP也需要connect */
    drv_a780_connect(sn, addr, port);
  }  

  do {
    /* 如果模式没有CONNECT, 代表进入失败，返回出错 */  
    if( (ext->flag & MA_EXT_FLAG_MODULE_CONNECT) == 0 ){
      break;  
    }

    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      drv->log->buf("a780 tx", vbuf, len);
    }

    ext->io->write(ext->io, vbuf, len, 5000); 
    ret = len;
  } while(0);

  return ret;
}

/**
* @brief A780 module socket receive from
*/
pos_i32_t drv_a780_recvfrom(pos_u32_t sn, void * vbuf, pos_size_t max_len, pos_u8_t * addr, pos_u16_t *port, pos_u32_t timeout_ms) {
  pos_i32_t len32, t;
  drv_api_t *drv = g_drv;
  ma_ext_ctrl_t *ext = g_drv->ext;    
  if( !max_len )
    return 0;
  t = drv->os->tick->get() + timeout_ms;
  do {
    len32 =  ext->io->read(ext->io, vbuf, max_len, 100);
    if( len32 )
      break; /* quit immediately if read */
  } while( !drv->os->tick->is_timeout(drv->os->tick->get(), t) );
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->buf("a780 rx", vbuf, len32);
#if 0    
    drv->log->data("a780 mlen", max_len);
    drv->log->data("a780 to", timeout_ms);
#endif    
  }  
  return len32;
}

/*
 * Driver export
 */
ma_drv_export_t g_export = {
  .version = DRV_NET_VERSION(MA_REPORT_MQTT_CAT1, DRV_A780_VERSION),
  .name = DRV_A780_NAME,
  .u.net={
    drv_a780_init,
    drv_a780_run_set,
    drv_a780_net_ready,
    drv_a780_sock_open,
    drv_a780_sock_close,
    drv_a780_connect,
    drv_a780_sendto,
    drv_a780_recvfrom,
  },
};

