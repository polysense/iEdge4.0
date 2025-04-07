/**
 * @file  drv_fs217.c
 * @brief SENSOR FS217 PM driver
 * @author Runby F.
 * @date 2022-12-19
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * 版本修订历史
v1.0 @ 2024-09-14
  1) First revision
  
*/

/**
 * @brief Driver version
 */
#define DRV_FS217_VERSION 0x0100

/**
 * @brief Driver name
 */
#define DRV_FS217_NAME  "FS217"

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
pos_status_t sensor_fs217_set(pos_u32_t on) {  
  pos_gpio_pin_t pwr;
  ma_board_t *b = g_drv->board;
  pwr = (g_drv->s->slot->io >> 8) & 0x7f;  
  if( !pwr )
    pwr = MB_PWR_ID_PWRH_PWR3;

  b->pwr_set(pwr, on);
  
  return POS_STATUS_OK;
}

/** 
* Sensor collecting
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_fs217_collect(void){
  pos_io_handle_t *io;
  pos_status_t ret = POS_STATUS_E_TIMEOUT;
  pos_u32_t v, t, b, i;
  drv_api_t *drv = g_drv;
  pos_u8_t response[32], *r;
  
  /* 初始化串口 */
  t = drv->s->slot->io;
  
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("fs217 io", t);        
  }

  t &= 0xff; /* keep io for uart only */
  
  io = POS_NULL;
  if( POS_IO_IS_SERIAL_PORT(t) ) {
    io = drv->os->io->init(t, 9600, 0);
  }

  if( io ) {    
    /* 清空串口并等待50ms */
    while( io->read(io, POS_NULL, 1024, 50) >= 1024 ); 
    
    b = drv->os->tick->get();  
    b += DRV_UART_IO_TIMEOUT;
    do {
      /*
       * 42 4d 00 14 
       * PM1.0_H PM1.0_L PM2.5_H PM2.5_L PM10_H PM10_L (Standard)
       * PM1.0_H PM1.0_L PM2.5_H PM2.5_L PM10_H PM10_L (Atmosphere)
       * PM1.0_H PM1.0_L PM2.5_H PM2.5_L PM10_H PM10_L (Not used, all zero)
       * SUM_H SUM_L
       */
      if( (v=io->read(io, response, sizeof(response), 100)) < 5 )
        continue;
      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {      
        drv->log->buf((const char*)"rd", response,v);
      }
      
      if( v != 24 || response[0] != 0x42 || response[1] != 0x4d || response[3] != 0x14 )
        continue;

      v = 0;
      for( i = 0; i < 22; i++ )
        v += response[i];
      if( (v & 0xff) != response[23] || ((v>>8)&0xff) != response[22] ) {
        if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {      
          drv->log->data("crcerr", v);
          continue;
        }
      }

      /* prepare r for Standard */
      r = &response[16];
      r[0] = PLSS_PL_EXT_PM; /* PLSS data format: 43 for PLSS_PL_EXT_PM: U16[3] for 2.5/10/1.0 */
      i = drv->s->slot->rsvd32&1;
      i = i ? 4 : 10;
      drv->os->memcpy(&r[1], &response[i+2], 4);
      r[5] = response[i];
      r[6] = response[i+1];

      drv->data->plss_put_raw(7, r);
      ret = drv->data->put("\"%s\":%u,", "pm2_5", r[1]*256+r[2], 0, 0, 0);
      if( ret == POS_STATUS_OK )
        ret = drv->data->put("\"%s\":%u,", "pm10", r[3]*256+r[4], 0, 0, 0);
      if( ret == POS_STATUS_OK )
        ret = drv->data->put("\"%s\":%u,", "pm1", r[5]*256+r[6], 0, 0, 0);  
      break;
    } while( !drv->os->tick->is_timeout(drv->os->tick->get(), b) );   /* repeat until timeout */     
  } 

  if( ret != POS_STATUS_OK ) {
    drv->log->data("fs217 err", t);
    /* 错误的Sensor, 上报出错 */
    drv->data->put("\"%s\":\"no fs217#0x%x\",", "err", t, 0, 0, 0);
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
  .version = DRV_SENSOR_VERSION(MA_SENSOR_FS217, DRV_FS217_VERSION),
  .name = DRV_FS217_NAME,
  .u.sensor={
    .power_set = sensor_fs217_set,
    .collect = sensor_fs217_collect,
  },
};

