/**
 * @file  drv_cnsl.c
 * @brief NET Native Console driver
 * @author Runby F.
 * @date 2022-3-10
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * 版本修订历史
v1.0 @ 2022-02-02
  1) 完成第一版驱动  

v1.1 @ 2023-10-13
  1) Support Binary report dump

v1.2 @ 2024-05-04
  1) Support JC valve dbg on every #7 loop

v1.3 @ 2022-02-23
  1) Remove dummy report in JSON when set ctrl 0x4001

*/

/**
 * @brief Driver version
 */
#define DRV_CNSL_VERSION 0x0103

/**
 * @brief Driver name
 */
#define DRV_CNSL_NAME  "CONSOLE"



/**
* @brief Native Console module init
*/
pos_status_t drv_cnsl_init(void) {
  return POS_STATUS_OK;
}

/**
* @brief Native Console module running state set
*/
pos_status_t drv_cnsl_run_set(pos_run_level_t level) {
  /* Native Console does NOT need to do anything, as it's auto sleep by default and auto wakeup by serial access */
  return POS_STATUS_OK;
}

/**
* @brief Native Console modle link ready check
* @note Always return OK in transparent mode
*/
pos_status_t drv_cnsl_net_ready(pos_u32_t timeout_ms) {
  return POS_STATUS_OK; /* trans mode no way to know ready, just return ok */
}

/**
* @brief Native Console module socket open (simulating)
* @note Always return OK
*/
pos_status_t drv_cnsl_sock_open(pos_u32_t *p_sock, pos_net_trans_t tcp, pos_u16_t port) {
  return POS_STATUS_OK;
}

/**
* @brief Native Console module socket close (simulating)
* @note Always return OK
*/
pos_status_t drv_cnsl_sock_close(pos_u32_t sock) {  
  return POS_STATUS_OK;
}

/**
* @brief Native Console module socket connect (simulating)
* @note Always return OK
*/
pos_status_t drv_cnsl_connect(pos_u32_t sn, pos_u8_t * addr, pos_u16_t port) {
  return POS_STATUS_OK;
}

/**
* @brief Native Console module sendto noneblocking mode
*/
pos_i32_t drv_cnsl_sendto(pos_u32_t sn, void * vbuf, pos_size_t len, pos_u8_t * addr, pos_u16_t port) {
  char *buf = (char*)vbuf;
  drv_api_t *drv = g_drv;
  if( buf[0] == '{' ) {
    if( (drv->cfg->ctrl & MA_CFG_CTRL_CONSOLE_REPORT) == 0 )
      drv->log->data(buf, len);
  } else
    drv->log->buf("tx", buf, len);
  drv->os->flush();
  return len;
}

/**
* @brief Native Console module receive
*/
pos_i32_t drv_cnsl_recvfrom(pos_u32_t sn, void * vbuf, pos_size_t max_len, pos_u8_t * addr, pos_u16_t *port, pos_u32_t timeout_ms) {
  pos_u8_t *buf = (pos_u8_t*)vbuf;
  if( (g_drv->duty_loop & 7) == 7 ) {
    /* simulating a JC cb for valve = 0x99 */
    *buf++ = 32; /* MA CB */
    *buf++ = 3;
    *buf++ = 0;
    *buf++ = 0x4b; /* jc */
    *buf++ = 4; /* valve control */
    *buf++ = 0x99;
    return 6;
  }
  buf[0] = 0xff;
  return 1;
}

/*
 * Driver export
 */
ma_drv_export_t g_export = {
  .version = DRV_NET_VERSION(MA_REPORT_NATIVE_CONSOLE, DRV_CNSL_VERSION),
  .name = DRV_CNSL_NAME,
  .u.net={
    drv_cnsl_init,
    drv_cnsl_run_set,
    drv_cnsl_net_ready,
    drv_cnsl_sock_open,
    drv_cnsl_sock_close,
    drv_cnsl_connect,
    drv_cnsl_sendto,
    drv_cnsl_recvfrom,
  },
};

