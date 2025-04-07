/**
 * @file  drv_esp.c
 * @brief NET ESP WIFI driver
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
  1) 解决注册类型不对问题
  2) 修正结尾为\r\n
  3) 解决初始化异常问题
  4) 支持标准模式操作(非透传)
  5) 支持STATUS:2/4都当做联网正常

v1.2 @ 2023-03-27
  1) Support auto set module uart from 115200 to 9600
  
*/

/**
 * @brief Driver version
 */
#define DRV_ESP_VERSION 0x0102

/**
 * @brief Driver name
 */
#define DRV_ESP_NAME  "ESP"

/**
 * @brief UDP Socket
 */
#define DRV_ESP_SOCK_UDP 0

/**
 * @brief TCP Socket
 */
#define DRV_ESP_SOCK_TCP 1


/**
* @brief ESP module init
*/
pos_status_t drv_esp_init(void) {
  drv_api_t *drv = g_drv;  
  pos_u8_t i;
  ma_ext_ctrl_t *ext = drv->ext;
  char b[48];
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("esp init", 0);
  }  
  ext->mtu = 900; /* use large mtu */
  ext->tx_rt[0] = 2; /* \r\n ESP必须\r\n */
  for( i = 0; i < 2; i++)  {    
    pos_u8_t r;
    ext->module_reset();
    drv->os->tick->sleep(100); 
    ext->io_flush();    
    ext->cmd_query_check(POS_NULL, 0, (pos_u8_t*)"ready", 2000);
    r = 0;
    do {
      if( ext->cmd_query((pos_u8_t*)"ATE0", 0) == POS_STATUS_OK )
        break;
      if( ++r >= 8 ) {
        r = 0;
        /* reset 115200 to 9600 */
        ext->io->release(ext->io);
        ext->io->setup(ext->io, 115200, 0);
        ext->cmd_query((pos_u8_t*)"AT+UART_DEF=9600,8,1,0,0", 0);
        ext->io->release(ext->io);
        ext->io->setup(ext->io, MB_EXT_UART_BAUD, 0);
      }
    } while(1);

    /* 第二遍什么都不做 */
    if( i )
      break;

    /* 第一遍设置各种模式 */
    ext->cmd_query((pos_u8_t*)"AT+CWMODE=1", 0); /* WIFI Station mode*/

    /* 判断是否已经联网并且SSID相符 */
    drv->os->sprintf(b, "+CWJAP:\"%s\"", drv->cfg->net.ssid);
    if( ext->cmd_query_check((pos_u8_t*)"AT+CWJAP?", 0, (pos_u8_t*)b, 0) == POS_STATUS_OK )
      break; /* SSID相符退出 */
    
    /* 设置新的SSID并重启生效 */
    drv->os->sprintf((char*)ext->buf, "AT+CWJAP=\"%s\",\"%s\"", drv->cfg->net.ssid, drv->cfg->net.wpwd);
    ext->cmd_query(ext->buf, 0);
  } 
  
  return POS_STATUS_OK;
}

/**
* @brief ESP module state set
*/
pos_status_t drv_esp_run_set(pos_run_level_t level) {
  /* 无需做任何动作, 模组自身通过APN自动进入PSM模式 */
  return POS_STATUS_OK;
}

/**
* @brief ESP module link ready check
*/
pos_status_t drv_esp_net_ready(pos_u32_t timeout_ms) {
  pos_status_t ret;
  drv_api_t *drv = g_drv;
  ma_ext_ctrl_t *ext = drv->ext;
  timeout_ms = drv->os->tick->get() + timeout_ms;
  do {
    ret = ext->cmd_query_check((pos_u8_t*)"AT+CIPSTATUS", 0, (pos_u8_t*)"STATUS:",0);
    if( ret == POS_STATUS_OK ) {
      pos_u8_t *c;
      c = ext->buf_search_pos(ext->buf, ext->rlen, 0, (pos_u8_t*)"STATUS:", 7, (pos_u8_t*)"");
      if( c && (*c == '2' || *c == '4') ) {
        /* 2/4 都当做联网正常 */
        break;
      } else
        ret = POS_STATUS_E_TIMEOUT;
    }

    drv->os->tick->sleep(1000);
  } while( !drv->os->tick->is_timeout(drv->os->tick->get(), timeout_ms));

#if 0
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("esp ready", ret);
  }  
#endif  
  return ret;
}

/**
* @brief ESP module socket open
*/
pos_status_t drv_esp_sock_open(pos_u32_t *p_sock, pos_net_trans_t tcp, pos_u16_t port) {
  if( tcp )
    *p_sock = DRV_ESP_SOCK_TCP;
  else
    *p_sock = DRV_ESP_SOCK_UDP;
  return POS_STATUS_OK;
}

/**
* @brief ESP module socket close
*/
pos_status_t drv_esp_sock_close(pos_u32_t sock) {  
  ma_ext_ctrl_t *ext = g_drv->ext;
  ext->flag &= ~MA_EXT_FLAG_MODULE_CONNECT;
  return ext->cmd_query((pos_u8_t*)"AT+CIPCLOSE", 0);
}

/**
* @brief ESP module socket connect
*/
pos_status_t drv_esp_connect(pos_u32_t sn, pos_u8_t * addr, pos_u16_t port) {
  pos_status_t ret;
  drv_api_t *drv = g_drv;
  ma_ext_ctrl_t *ext = drv->ext;  
  drv->os->sprintf((char*)ext->buf, "AT+CIPSTART=\"%s\",\"%u.%u.%u.%u\",%u",  
    sn ? "TCP" : "UDP", addr[0],addr[1],addr[2],addr[3],port);  
  ret = ext->cmd_query(ext->buf, 30000);
  if( ret == POS_STATUS_OK ) {
    ext->flag |= MA_EXT_FLAG_MODULE_CONNECT;
  }
#if 0  
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("esp connect", ret);
    if( ret != POS_STATUS_OK )
      drv->log->buf("esp dbg", ext->buf, ext->buf_size);
  }  
#endif  
  return ret;
}

/**
* @brief ESP module socket send (noneblocking)
*/
pos_i32_t drv_esp_sendto(pos_u32_t sn, void * vbuf, pos_size_t len, pos_u8_t * addr, pos_u16_t port) {
  drv_api_t *drv = g_drv;
  ma_ext_ctrl_t *ext = drv->ext;  
  pos_i32_t ret = -1;
#if 0  
  pos_u32_t t;
  t = drv->os->tick->get();
#endif

  if( sn == DRV_ESP_SOCK_UDP && (ext->flag & MA_EXT_FLAG_MODULE_CONNECT) == 0 ) {
    /* 尝试进入透传模式 */
    drv_esp_connect(sn, addr, port);
  }

  do {
    /* 如果模式没有CONNECT, 代表进入失败，返回出错 */  
    if( (ext->flag & MA_EXT_FLAG_MODULE_CONNECT) == 0 ){
      break;  
    }

    drv->os->sprintf((char*)ext->buf, "AT+CIPSEND=%u", len);
    ext->cmd_send(ext->buf, 0);
    if( ext->cmd_query_check(POS_NULL, 0, (pos_u8_t*)"> ", 0) != POS_STATUS_OK )
      break;

    if( drv->cfg->ctrl & MA_CFG_CTRL_DBG ) {
      drv->log->buf("esp tx", vbuf, len);
    }
    
    /* direct send out */
    ext->io->write(ext->io, vbuf, len, 5000);

    ret = len;
  }while(0);

#if 0
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("esp tx", ret);
    drv->log->data("esp tx time", drv->os->tick->elaps(t));
  }  
#endif

  return ret;
}


/**
* @brief ESP module receive from (blocking)
*/
pos_i32_t drv_esp_recvfrom(pos_u32_t sn, void * vbuf, pos_size_t max_len, pos_u8_t * addr, pos_u16_t *port, pos_u32_t timeout_ms) {
  drv_api_t *drv = g_drv;
  ma_ext_ctrl_t *ext = drv->ext;  
  pos_u8_t *c, *end;
  pos_u32_t clen;
  /* 无阻塞方式用10ms来模拟， 直接传递0到cmd_query将引入3s延迟 */
  if( !timeout_ms )
    timeout_ms = 10;
  if( ext->cmd_query_check(POS_NULL,0, (pos_u8_t*)"+IPD,", timeout_ms) != POS_STATUS_OK )
    return 0;

  if( drv->cfg->ctrl & MA_CFG_CTRL_DBG ) {
    drv->log->buf("esp rx uart", ext->buf, ext->rlen);
  }

  do {
    /* search signature */
    c = ext->buf_search_pos(ext->buf, ext->rlen, 0, (pos_u8_t*)"+IPD,", 5, (pos_u8_t*)"");
    if( !c ) {
      drv->log->buf("esp no ipd", ext->buf, ext->rlen);
      break;
    }
    
    /* search payload start */
    end = &ext->buf[ext->rlen];
    while( c < end ) {
      if( *c++ == ':' ) {
        /* count payload length */
        clen = end - c;

        /* return payload */
        if( clen > max_len )
          clen = max_len;
        drv->os->memcpy(vbuf, c, clen);

#if 0        
        if( drv->cfg->ctrl & MA_CFG_CTRL_DBG ) {
          drv->log->buf("esp rx pkt", vbuf, clen);
        }
#endif        
        return clen;
      }
    }

  } while(0);

  /* return error */
  return 0;
}

ma_drv_export_t g_export = {
  .version = DRV_NET_VERSION(MA_REPORT_MQTT_WIFI, DRV_ESP_VERSION),
  .name = DRV_ESP_NAME,
  .u.net={
    drv_esp_init,
    drv_esp_run_set,
    drv_esp_net_ready,
    drv_esp_sock_open,
    drv_esp_sock_close,
    drv_esp_connect,
    drv_esp_sendto,
    drv_esp_recvfrom,
  },
};

