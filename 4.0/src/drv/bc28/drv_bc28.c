/**
 * @file  drv_bc28.c
 * @brief NET BC28 driver
 * @author Runby F.
 * @date 2022-3-12
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * 版本修订历史
v1.0 @ 2022-03-10
  1) 完成第一版驱动

v1.1 @ 2022-03-15
  1) 支持RT-Thread

v1.2 @ 2022-04-20
  1) 支持联网状态调试打印

v1.3 @ 2022-04-22
  1) 支持PSM/NONE-PSM配置切换
  2) 修复PSM时无法注册问题

v1.4 @ 2022-04-27
  1) 支持mtu初始化

v1.5 @ 2023-09-14
  1) Support BC28 roaming connecting

v1.6 @ 2024-08-20
  1) When APN is locked, only retry 2 times
  2) Fix dummy CEREG check issues
  3) Support RF signal query
  4) Support RTC clock query

v1.7 @ 2024-10-31
  1) Optimize APN init issue and inrease retry count
  
*/

/**
 * @brief Driver version
 */
#define DRV_BC28_VERSION 0x0107

/**
 * @brief Driver name
 */
#define DRV_BC28_NAME  "BC28"


/**
* @brief BC28 module init
*/
pos_status_t drv_bc28_init(void) {
  pos_u8_t flag, loop, i;
  drv_api_t *drv = g_drv;  
  ma_ext_ctrl_t *ext = drv->ext;
  ext->mtu = 400; /* use middle mtu */
  ext->tx_rt[0] = 1; /* \r only */
  for(loop = 0; loop < 3; loop++) {
    /* reset */
    if( ext->run_level > RUN_LEVEL_POWER_OFF ) {
      ext->cmd_query((pos_u8_t*)"AT+CFUN=0", 0); /* stop all before reset */
    }

    if( drv->cfg->ctrl & MA_CFG_CTRL_POWER_SAVING )
      ext->cmd_query((pos_u8_t*)"AT+NRB", 500); /* PSM might not reset module correctly, reboot manually */
    
    ext->module_reset();
    drv->os->tick->sleep(5000);
    do {
      if( ext->cmd_query((pos_u8_t*)"AT+CMEE=0", 0) == POS_STATUS_OK )
        break;
    } while(1);
    ext->cmd_query((pos_u8_t*)"AT+QREGSWT=2", 0); /* normal register */

    /* PSM/None-PSM mode */
    if( drv->cfg->ctrl & MA_CFG_CTRL_POWER_SAVING ) {
      ext->cmd_query((pos_u8_t*)"AT+CPSMS=1,,,\"00111000\",\"00010000\"", 0);
    } else {
      ext->cmd_query((pos_u8_t*)"AT+CPSMS=0", 0);
    }

    /* do not perform following check when loop >=2 */
    if( loop >= 2 )
      break;
    
    /* check APN/BAND */
    flag = 0;
    do {
      char abuf[32+8];
      drv->os->sprintf(abuf, "%s\"", g_drv->cfg->net.apn); /* add \" for full matching */
      if( ext->cmd_query_check((pos_u8_t*)"AT+CGDCONT?", 17, (pos_u8_t*)abuf, 0) == POS_STATUS_OK ) {
        flag |= 1; /* +CGDCONT:0,"IP","XXXXX" */
      } else if( ext->buf_search_pos(drv->ext->buf, drv->ext->rlen, 21, (pos_u8_t*)abuf, drv->os->strlen(abuf), (pos_u8_t*)"") ) {
        flag |= 1; /* +CGDCONT:0,"IPV4V6","XXXXX" */
      }
      if( (flag & 1) == 0 ) {
        break;
      }

      drv->os->sprintf(abuf, "%s\r", g_drv->cfg->net.band); /* add \r for full matching */
      if( ext->cmd_query_check((pos_u8_t*)"AT+NBAND?", 7, (pos_u8_t*)g_drv->cfg->net.band, 0) != POS_STATUS_OK ) {
        break;
      }
      flag |= 2;
    } while(0);
    if( flag == 3 )
      break; /* quit if APN/BAND are matched */

    /* set APN/BAND */
    for( i = 0; i < 10; i++ ) {
      ext->cmd_query((pos_u8_t*)"AT+CGATT=0", 60000); /* stop func before APN/BAND change */
      drv->os->sprintf((char*)g_drv->ext->buf, "AT+CGDCONT=0,\"IPV4V6\",\"%s\"", g_drv->cfg->net.apn);
      if( ext->cmd_query(g_drv->ext->buf, 0) == POS_STATUS_OK )
        break;
      drv->os->tick->sleep(3000);
    }
    ext->cmd_query((pos_u8_t*)"AT+CFUN=0", 60000); /* full stop func before APN/BAND change */
    drv->os->sprintf((char*)g_drv->ext->buf, "AT+NBAND=%s", g_drv->cfg->net.band);
    ext->cmd_query(g_drv->ext->buf, 0);
  }

  /* start network */
  g_drv->ext->cmd_query((pos_u8_t*)"AT+CEREG=5", 0);
  
  return POS_STATUS_OK;
}

/**
* @brief BC28 working state set
*/
pos_status_t drv_bc28_run_set(pos_run_level_t level) {
  if( level == RUN_LEVEL_POWER_OFF ) {
    g_drv->ext->cmd_query((pos_u8_t*)"AT+CFUN=0", 0); /* stop all before power off */
  } else {
    /* 无需做任何动作, 模组自身通过APN自动进入PSM模式 */
    
  }
  return POS_STATUS_OK;
}

/**
* @brief BC28 module link ready check
*/
pos_status_t drv_bc28_net_ready(pos_u32_t timeout_ms) {
  pos_u8_t i;
  pos_u16_t v;
  pos_status_t ret;
  drv_api_t *drv = g_drv;
  i = 0;
  drv->ext->cmd_query((pos_u8_t*)"AT+CEREG=5", 0); /* 激活注册 */
  do {
    ret = drv->ext->cmd_query_check((pos_u8_t*)"AT+CEREG?", 0, (pos_u8_t*)"+CEREG:5,1",0);
    if( ret == POS_STATUS_OK ) 
      break;
    ret = drv->ext->cmd_query_check((pos_u8_t*)"AT+CEREG?", 0, (pos_u8_t*)"+CEREG:5,5",0);    
    if( ret == POS_STATUS_OK ) 
      break;

    if( !timeout_ms )
      break;
    if( i == 0 ) {
      drv->ext->cmd_query_check(POS_NULL, 0, (pos_u8_t*)"+CEREG:1", timeout_ms);
    }    
  } while( ++i < 2 );

  /* update rf signal */
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
  drv->history->signal = v;

  if( ret == POS_STATUS_OK &&
      drv->ext->cmd_query_check((pos_u8_t*)"AT+CCLK?", 0, (pos_u8_t*)"+CCLK:",0) == POS_STATUS_OK ) {
      /* +CCLK:23/02/09,08:51:58+32 */
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
      if( t->year < 100 )
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
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("bc28 link status", ret);
  }
  return ret;
}

/**
* @brief BC28 module socket open
*/
pos_status_t drv_bc28_sock_open(pos_u32_t *p_sock, pos_net_trans_t tcp, pos_u16_t port) {
  g_drv->os->sprintf((char*)g_drv->ext->buf, "AT+NSOCR=%s,%u,%u,1", tcp ? "STREAM" : "DGRAM", tcp ? 6 : 17, port); 
  if( g_drv->ext->cmd_query(g_drv->ext->buf,0) == POS_STATUS_OK ) {
    if( g_drv->ext->buf[2] >= '0' && g_drv->ext->buf[2] <= '9' ) {
      *p_sock = g_drv->ext->buf[2] - '0';
      return POS_STATUS_OK;
    }
  }
  *p_sock = 1; /* always use 1 when failed */
  return POS_STATUS_ERROR;
}

/**
* @brief BC28 module socket close
*/
pos_status_t drv_bc28_sock_close(pos_u32_t sock) {  
  g_drv->os->sprintf((char*)g_drv->ext->buf, "AT+NSOCL=%u", sock);  
  return g_drv->ext->cmd_query(g_drv->ext->buf,0);
}

/**
* @brief BC28 module socket connect
*/
pos_status_t drv_bc28_connect(pos_u32_t sn, pos_u8_t * addr, pos_u16_t port) {
  g_drv->os->sprintf((char*)g_drv->ext->buf, "AT+NSOCO=%u,%u.%u.%u.%u,%u", sn, addr[0],addr[1],addr[2],addr[3],port);
  return g_drv->ext->cmd_query(g_drv->ext->buf,0);
}

/**
* @brief BC28 module sendto
*/
pos_i32_t drv_bc28_sendto(pos_u32_t sn, void * vbuf, pos_size_t len, pos_u8_t * addr, pos_u16_t port) {
  pos_size_t slen;
  if( addr && port )
    /* UDP SENDTO */
    g_drv->os->sprintf((char*)g_drv->ext->buf, "AT+NSOST=%u,%u.%u.%u.%u,%u,%u,", sn, addr[0],addr[1],addr[2],addr[3],port,len);
  else
    /* TCP SEND */
    g_drv->os->sprintf((char*)g_drv->ext->buf, "AT+NSOSD=%u,%u,", sn, len);
  slen = g_drv->os->strlen((char*)g_drv->ext->buf);
  g_drv->os->bin2str((char*)&g_drv->ext->buf[slen], (pos_u8_t*)vbuf, len, ENCODE_HEXSTR_UPPER);
  return g_drv->ext->cmd_query(g_drv->ext->buf,0) == POS_STATUS_OK ? len : -1;
}

/**
* @brief BC28 module receive from
*/
pos_i32_t drv_ext_bc28_recvfrom_nb(pos_u32_t sn, pos_u8_t * buf, pos_size_t max_len, pos_u8_t * addr, pos_u16_t *port) {
  pos_status_t ret;
  pos_u32_t len32;
  pos_u8_t *rbuf;
  pos_u16_t flush_len;
  flush_len = max_len;
  if( flush_len > 4 ) /* some app (like mqtt) will try to read 1/2/3/4 bytes for special check, this way, do NOT flush all */
    flush_len = 1358;
  g_drv->os->sprintf((char*)g_drv->ext->buf, "AT+NSORF=%u,%u", sn, flush_len ); /* BC28 uses 1358 to flush all */
  ret = g_drv->ext->cmd_query(g_drv->ext->buf,0);
  if( ret != POS_STATUS_OK )
    return -1; /* indicate socket error */

  /* check feedback */
  len32 = 0;  
  if( g_drv->ext->buf[2] == sn+'0' && g_drv->ext->buf[3] == ',' ) {
    rbuf = (pos_u8_t*)g_drv->os->sscan( (char*)&g_drv->ext->buf[4], "%4n,%2n,%4n,", addr, port, &len32 );
    g_drv->os->nps->ntohl((pos_u32_t*)addr,1);
    if( len32 > max_len )
      len32 = max_len;
    if( len32 ) {
      pos_u32_t i;
      i = len32;
      g_drv->os->str2bin((char*)rbuf, buf, &i,DECODE_HEXSTR);
      if( i != len32 )
        len32 = 0; /* wrong payload */
    }
  }
  return len32;
}

/**
* @brief BC28 module receive (blocking)
*/
pos_i32_t drv_bc28_recvfrom(pos_u32_t sn, void * vbuf, pos_size_t max_len, pos_u8_t * addr, pos_u16_t *port, pos_u32_t timeout_ms) {
  pos_i32_t len32;
  if( !max_len )
    return 0;
  len32 = drv_ext_bc28_recvfrom_nb(sn, vbuf, max_len, addr, port);
  if( timeout_ms && len32 == 0 ) {
    if( g_drv->ext->cmd_query_check(POS_NULL, 0, (pos_u8_t*)"+NSONMI:", timeout_ms) == POS_STATUS_OK )
      len32 = drv_ext_bc28_recvfrom_nb(sn, vbuf, max_len, addr, port);    
  }
  return len32;
}

ma_drv_export_t g_export = {
  .version = DRV_NET_VERSION(MA_REPORT_MQTT_NBIOT, DRV_BC28_VERSION),
  .name = DRV_BC28_NAME,
  .u.net={
    drv_bc28_init,
    drv_bc28_run_set,
    drv_bc28_net_ready,
    drv_bc28_sock_open,
    drv_bc28_sock_close,
    drv_bc28_connect,
    drv_bc28_sendto,
    drv_bc28_recvfrom,
  },
};

