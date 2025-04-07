/**
 * @file  drv_rscoil.c
 * @brief SENSOR RSCOIL driver
 * @author Runby F.
 * @date 2022-12-19
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * 版本修订历史
v1.0 @ 2024-01-31
  1) Support RSCOIL sensor

v1.1 @ 2024-02-04
  1) Fix data parsing alignment issue

*/

/**
 * @brief Driver version
 */
#define DRV_RSCOIL_VERSION 0x0101

/**
 * @brief Driver name
 */
#define DRV_RSCOIL_NAME  "RSCOIL"

/**
 * @brief Operating timeout
 */
#define DRV_UART_IO_TIMEOUT  10000

/** 
* Power control
* @param[in]  on   0:Power off, 1:Power on
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
* @note If sensor does NOT require any power control, this fucntion can be NULL
*/ 
pos_status_t sensor_rscoil_set(pos_u32_t on) {  
  pos_gpio_pin_t pwr;
  drv_api_t *drv = g_drv;
  ma_board_t *b = drv->board;
  pwr = (drv->s->slot->io >> 8) & 0x7f;  
  if( !pwr )
    pwr = MB_PWR_ID_VCC_PWRH;

  b->pwr_set(pwr, on);
  return POS_STATUS_OK;
}

/** 
* Sensor collecting
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_rscoil_collect(void){
  pos_io_handle_t *io;
  pos_status_t ret = POS_STATUS_E_TIMEOUT;
  pos_u32_t v, report, t, b;
  drv_api_t *drv = g_drv;
  pos_u8_t response[32];
  
  /* 初始化串口 */
  t = drv->s->slot->io;
  b = drv->s->slot->rsvd32;
  
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("rscoil io", t);        
    drv->log->data("rscoil rsvd", b);    
  }

  t &= 0xff; /* keep io for uart only */
  b &= 0xff;
  if( !b )
    b = 1;
  io = POS_NULL;
  if( POS_IO_IS_SERIAL_PORT(t) ) {
    io = drv->os->io->init(t, 9600, 0); /* 默认用115200 */
  }

  /* refresh to normal by default */
  drv->s->ctrl |= MA_SENSOR_CTRL_STAT_NORMAL;
  drv->s->ctrl &= ~MA_SENSOR_CTRL_STAT_ERROR;
  report = 0;  
  
  do {
    if( !io )
      break;
    
    v = drv->data->modbus_read(io, response, b, 3, 0, 2);
    if( v < 9 ) 
      break;

    report = 1;
    v = drv->os->util->get_u32(&response[3], 4); 
    v = POS_NTOHL(v);
    v = drv->os->util->ufloat_to_u32_precision(v, 10);
    ret = drv->data->plss_json_put(v,
      DRV_MA_PLSS_TRANS(PLSS_PL_EXT_CURRENT_UA, 2, 0, 0, 1), 
      "current" );
  } while(0);

  if( !report ) { 
    /* refresh to ERROR stat */
    drv->s->ctrl |= MA_SENSOR_CTRL_STAT_ERROR;
    drv->log->data("rscoil err", drv->s->slot->io);
    /* 错误的Sensor, 上报出错 */
    drv->data->put("\"%s\":\"no rscoil#0x%x\",", "err", drv->s->slot->io, 0, 0, 0);
  }

  if( io ) {      
    /* shutdown IO after read */
    io->release(io);
  }
  return ret;
}


/*
 * Driver export
 */
ma_drv_export_t g_export = {
  .version = DRV_SENSOR_VERSION(MA_SENSOR_RSCOIL, DRV_RSCOIL_VERSION),
  .name = DRV_RSCOIL_NAME,
  .u.sensor={
    .power_set = sensor_rscoil_set,
    .collect = sensor_rscoil_collect,
  },
};

