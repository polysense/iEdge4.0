/**
 * @file  drv_ultra.c
 * @brief SENSOR ULTRA SONIC driver
 * @author Runby F.
 * @date 2022-3-23
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * 版本修订历史
v1.0 @ 2022-04-03
  1) First version

v1.1 @ 2022-05-24
  1) Use 115200 instead of 9600

v1.2 @ 2022-05-31
  1) Enhance JSON/PLSS data put for smaller footprint
  2) Support configurable UART baudrate
  
*/

/**
 * @brief Driver version
 */
#define DRV_ULTRA_VERSION 0x0102

/**
 * @brief Driver name
 */
#define DRV_ULTRA_NAME  "ULTRA"

/**
 * @brief Retry times
 */
#define DRV_ULTRA_RETRY_NUM  3

/**
 * @brief Operating timeout
 */
#define DRV_ULTRA_IO_TIMEOUT  3000

/** 
* Power control
* @param[in]  on   0:Power off, 1:Power on
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
* @note If sensor does NOT require any power control, this fucntion can be NULL
*/ 
pos_status_t sensor_ultra_set(pos_u32_t on) {  
  pos_gpio_pin_t pwr;
  ma_board_t *b = g_drv->board;
  pwr = (g_drv->s->slot->io >> 8) & 0x7f;  
  if( !pwr )
    pwr = MB_PWR_ID_VCCN_PWR3;

  b->pwr_set(pwr, on);
  
  return POS_STATUS_OK;
}

/** 
* Sensor collecting
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_ultra_collect(void){
  pos_io_handle_t *io;
  pos_status_t ret = POS_STATUS_E_TIMEOUT;
  pos_u32_t v;
  pos_u16_t t, i;
  drv_api_t *drv = g_drv;
  pos_u8_t response[8], c;
  
  /* 初始化串口 */
  i = drv->s->slot->io;

  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("ultra io", i);        
  }  
  i &= 0xff; /* keep io for uart only */  
  if( POS_IO_IS_SERIAL_PORT(i) ) {
  } else
    i = 0;
  v = drv->s->slot->rsvd32 >> 8;
  if( !v )
    v = 115200;
  io = drv->os->io->init(i, v, 0);

  
  /* 重试3次 */
  for(t=0; t < DRV_ULTRA_RETRY_NUM; t++ ) {
    /* 清空串口并等待50ms */
    while( io->read(io, POS_NULL, 1024, 50) >= 1024 ); 
    
    /* Send a ZERO (or any) to trigger sampling */
    response[0] = 0xaa;
    io->write(io, response, 1, DRV_ULTRA_IO_TIMEOUT);

    /* 每次等待1000ms响应 */
    v = io->read(io, response, 4, DRV_ULTRA_IO_TIMEOUT);
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      drv->log->buf("rd", response, v);
    }
    
    if( v != 4 || response[0] != 0xff )
      continue;

    c = 0xff + response[1] + response[2];
    if( c != response[3] )
      continue; /* 格式错误 */
    
    v = (response[1]<<8) + response[2];
    //drv->log->data("RX ULTRA", v); 

    ret = drv->data->plss_json_put(v, 
      DRV_MA_PLSS_TRANS(38, 2, 0, 0, 0), /* 38 - PLSS_PL_US_DISTANCE, 2Bytes, 1 mm */ 
      "distance");    
#if 0    
    drv->data->plss_put_u16(38, v); /* PLSS data format put, 38 for PLSS_PL_US_DISTANCE: U16 */
    
    ret = drv->data->put("\"%s\":%u,", "distance", v, 0, 0, 0);
#endif    
    if( ret != POS_STATUS_OK ) {
      drv->s->ctrl |= MA_SENSOR_CTRL_TOO_LONG; /* indicate data buffer put failed */
    }
    break;
  }

  if( t >= DRV_ULTRA_RETRY_NUM ) {
#if 0    
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {    
      drv->log->data("distance err", i);
    }
#endif    
    /* 错误的Sensor, 上报出错 */
    drv->data->put("\"%s\":\"no ultra#0x%x\",", "err", i, 0, 0, 0);
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
  .version = DRV_SENSOR_VERSION(MA_SENSOR_ULTRA, DRV_ULTRA_VERSION),
  .name = DRV_ULTRA_NAME,
  .u.sensor={
    .power_set = sensor_ultra_set,
    .collect = sensor_ultra_collect,
  },
};

