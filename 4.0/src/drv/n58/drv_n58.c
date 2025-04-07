/**
 * @file  drv_n58.c
 * @brief NET N58 driver
 * @author Runby F.
 * @date 2022-4-25
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * 版本修订历史
v1.0 @ 2022-04-25
  1) 完成第一版驱动

v1.1 @ 2022-04-27
  1) 支持mtu初始化  

v1.2 @ 2022-05-20
  1) 根据命令模式调整初始化和收发动作
  2) 支持10s连接超时判断
  3) 优化读响应当快速响应时减少读等待
  
*/

/**
 * @brief Driver version
 */
#define DRV_N58_VERSION 0x0102

/**
 * @brief Driver name
 */
#define DRV_N58_NAME  "N58"

/**
 * @brief UDP Socket
 */
#define DRV_N58_SOCK_UDP 0

/**
 * @brief TCP Socket
 */
#define DRV_N58_SOCK_TCP 1

/**
* @brief N58 module init
*/
pos_status_t drv_n58_init(void) {
  drv_api_t *drv = g_drv;  
  ma_ext_ctrl_t *ext = drv->ext;
  ext->mtu = 400; /* use middle mtu */  
  ext->tx_rt[0] = 1; /* \r only */
  do {
    /* reset */
    if( ext->run_level > RUN_LEVEL_POWER_OFF || (drv->cfg->ctrl & MA_CFG_CTRL_POWER_SAVING ) ) {
      ext->cmd_query((pos_u8_t*)"AT+CFUN=0,1", 0); /* stop all before reset */
    }
    
    ext->module_reset();
    drv->os->tick->sleep(5000);
    do {
      if( ext->cmd_query((pos_u8_t*)"ATE0", 0) != POS_STATUS_OK ) /* Echo OFF */
        continue;
      
      if( ext->cmd_query((pos_u8_t*)"AT+RECVMODE=0,1", 0) == POS_STATUS_OK ) /* CMD RECEIVE, HEX MODE */
        break;
    } while(1);


    /* set APN/BAND */
    drv->os->sprintf((char*)g_drv->ext->buf, "AT+NETAPN=\"%s\",\"\",\"\"", g_drv->cfg->net.apn);
    if( ext->cmd_query(g_drv->ext->buf, 0) == POS_STATUS_OK )
      break;
  } while(1);

  /* start network */
  g_drv->ext->cmd_query((pos_u8_t*)"AT+CEREG=1", 0);
  
  return POS_STATUS_OK;
}

/**
* @brief N58 module working state set
*/
pos_status_t drv_n58_run_set(pos_run_level_t level) {
  if( level == RUN_LEVEL_POWER_OFF ) {
    g_drv->ext->cmd_query((pos_u8_t*)"AT+CFUN=0", 0); /* stop all before power off */
  } else {
    /* 无需做任何动作, 模组自身通过APN自动进入PSM模式 */
    
  }
  return POS_STATUS_OK;
}

/**
* @brief N58 module link ready check
*/
pos_status_t drv_n58_net_ready(pos_u32_t timeout_ms) {
  pos_status_t ret;
  drv_api_t *drv = g_drv;
  ma_ext_ctrl_t *ext = drv->ext;
  const pos_lib_tick_t *tick = drv->os->tick;
  ext->cmd_query((pos_u8_t*)"AT+CEREG=1", 0); /* 激活注册 */
  timeout_ms +=tick->get();
  do {
    ret = ext->cmd_query_check((pos_u8_t*)"AT+CEREG?", 0, (pos_u8_t*)"+CEREG: ",0);
    if( ret != POS_STATUS_OK )
      break;
   
    if( ext->buf[10] == '1' || ext->buf[10] == '5' )
      break; /* +CEREG:1/5,1 are both accepted */
    tick->sleep(2000);
  } while( !tick->is_timeout(tick->get(), timeout_ms) );

  if( ret == POS_STATUS_OK ) {
    ext->cmd_query((pos_u8_t*)"AT+XIIC=1", 0); /* 激活链路 */
    do {
      ret = ext->cmd_query_check((pos_u8_t*)"AT+XIIC?", 0, (pos_u8_t*)"+XIIC:    1",0);
      if( ret == POS_STATUS_OK )
        break;
      tick->sleep(1000);      
    } while( !tick->is_timeout(tick->get(), timeout_ms) );
  }
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("n58 link status", ret);
  }
  return ret;
}

/**
* @brief N58 module socket open
*/
pos_status_t drv_n58_sock_open(pos_u32_t *p_sock, pos_net_trans_t tcp, pos_u16_t port) {
  if( tcp )
    *p_sock = DRV_N58_SOCK_TCP;
  else
    *p_sock = DRV_N58_SOCK_UDP;
  return POS_STATUS_OK;
}

/**
* @brief N58 module socket close
*/
pos_status_t drv_n58_sock_close(pos_u32_t sock) {  
  drv_api_t *drv = g_drv;
  ma_ext_ctrl_t *ext = drv->ext;  
  if( sock == DRV_N58_SOCK_UDP )
    drv->os->sprintf((char*)ext->buf, "AT+UDPCLOSE=0");
  else
    drv->os->sprintf((char*)ext->buf, "AT+TCPCLOSE=0");
  ext->flag &= ~MA_EXT_FLAG_MODULE_CONNECT;
  return g_drv->ext->cmd_query(g_drv->ext->buf,0);
}

/**
* @brief N58 module socket connect
*/
pos_status_t drv_n58_connect(pos_u32_t sn, pos_u8_t * addr, pos_u16_t port) {
  drv_api_t *drv = g_drv;
  ma_ext_ctrl_t *ext = drv->ext;
  pos_status_t ret;
  drv->os->sprintf((char*)ext->buf, "AT+%sSETUP=0,%u.%u.%u.%u,%u", 
    sn == DRV_N58_SOCK_UDP ? "UDP" : "TCP",
    addr[0],addr[1],addr[2],addr[3],port);
  ret = ext->cmd_query_check(ext->buf,4, (pos_u8_t*)"SETUP: 0",10000); /* 10s connect timeout */
  if( ret == POS_STATUS_OK )
    ext->flag |= MA_EXT_FLAG_MODULE_CONNECT;
  return ret;
}

/**
* @brief N58 module socket sendto (noneblocking)
*/
pos_i32_t drv_n58_sendto(pos_u32_t sn, void * vbuf, pos_size_t len, pos_u8_t * addr, pos_u16_t port) {
  drv_api_t *drv = g_drv;
  ma_ext_ctrl_t *ext = drv->ext;  
  pos_size_t slen;
  pos_i32_t ret = -1;
  
  if( sn == DRV_N58_SOCK_UDP && (ext->flag & MA_EXT_FLAG_MODULE_CONNECT) == 0 ) {
    /* N58 UDP也需要connect */
    drv_n58_connect(sn, addr, port);
  }  

  do {
    /* 如果模式没有CONNECT, 代表进入失败，返回出错 */  
    if( (ext->flag & MA_EXT_FLAG_MODULE_CONNECT) == 0 ){
      break;  
    }

    drv->os->sprintf((char*)ext->buf, "AT+%sSEND=0,%u,\"",
      sn == DRV_N58_SOCK_UDP ? "UDP" : "TCP",  len);
    slen = drv->os->strlen((char*)ext->buf);
    drv->os->bin2str((char*)&ext->buf[slen], (pos_u8_t*)vbuf, len, ENCODE_HEXSTR_UPPER);
    slen += len * 2;
    drv->os->strcpy((char*)&ext->buf[slen], "\",1");
    if( ext->cmd_query(ext->buf,0) == POS_STATUS_OK )
      ret = len;
  } while(0);

  return ret;
}

/**
* @brief N58 module socket receive from (noneblocking)
*/
pos_i32_t drv_ext_n58_recvfrom_nb(pos_u32_t sn, pos_u8_t * buf, pos_size_t max_len, pos_u8_t * addr, pos_u16_t *port) {
  drv_api_t *drv = g_drv;
  ma_ext_ctrl_t *ext = drv->ext;  
  pos_status_t ret;
  pos_u32_t len32;
  pos_u8_t *rbuf;
  pos_u16_t flush_len;
  flush_len = max_len;
  if( flush_len > 4 ) /* some app (like mqtt) will try to read 1/2/3/4 bytes for special check, this way, do NOT flush all */
    flush_len = 2038; /* N58 uses 1358 to flush all */
  drv->os->sprintf((char*)ext->buf, "AT+%sREAD=0,%u", 
    sn == DRV_N58_SOCK_UDP ? "UDP" : "TCP", flush_len ); 
  ret = ext->cmd_query_check(ext->buf,4, (pos_u8_t*)"READ: 0",0);
  if( ret != POS_STATUS_OK )
    return -1; /* indicate socket error */

  /* check feedback */
  rbuf = ext->buf_search_pos(ext->buf, ext->rlen, 4, (pos_u8_t*)"READ: 0,", 8, (pos_u8_t*)"");
  if( !rbuf )
    return 0;
  
  len32 = 0;  
  rbuf = (pos_u8_t*)drv->os->sscan( (const char*)rbuf, "%4n,", &len32 );
  
  if( len32 > max_len )
    len32 = max_len;
  if( len32 ) {
    pos_u32_t i;
    i = len32;
    drv->os->str2bin((char*)rbuf, buf, &i, DECODE_HEXSTR);
    if( i != len32 )
      len32 = 0; /* wrong payload */
  }
  return len32;
}

/**
* @brief N58 module socket receive from
*/
pos_i32_t drv_n58_recvfrom(pos_u32_t sn, void * vbuf, pos_size_t max_len, pos_u8_t * addr, pos_u16_t *port, pos_u32_t timeout_ms) {
  pos_i32_t len32, i;
  if( !max_len )
    return 0;
  
  /* 有时候N58会快速响应并收到数据，此时判断两次recvfrom就能收到数据 */    
  for( i = 0; i < 2; i++ ) {
    len32 = drv_ext_n58_recvfrom_nb(sn, vbuf, max_len, addr, port);
    if( len32 )
      break;
  }
  
  if( timeout_ms && len32 == 0 ) {
    if( g_drv->ext->cmd_query_check(POS_NULL, 0, 
        sn == DRV_N58_SOCK_UDP ? (pos_u8_t*)"+UDPRECV:" : (pos_u8_t*)"+TCPRECV:", 
        timeout_ms) == POS_STATUS_OK )
      len32 = drv_ext_n58_recvfrom_nb(sn, vbuf, max_len, addr, port);    
  }
  return len32;
}

/*
 * Driver export
 */
ma_drv_export_t g_export = {
  .version = DRV_NET_VERSION(MA_REPORT_MQTT_LTE, DRV_N58_VERSION),
  .name = DRV_N58_NAME,
  .u.net={
    drv_n58_init,
    drv_n58_run_set,
    drv_n58_net_ready,
    drv_n58_sock_open,
    drv_n58_sock_close,
    drv_n58_connect,
    drv_n58_sendto,
    drv_n58_recvfrom,
  },
};

