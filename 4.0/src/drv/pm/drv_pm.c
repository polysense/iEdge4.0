/**
 * @file  drv_pm.c
 * @brief SENSOR NS PM driver
 * @author Runby F.
 * @date 2022-12-19
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * 版本修订历史
v1.0 @ 2023-3-20
  1) Support ZE03 PM2.5/10/1.0 sensor

v1.1 @ 2023-08-26
  1) Use explict IO init uart ctrl field

v1.2 @ 2023-10-24
  1) Support stat/normal/error

v1.3 @ 2023-11-18
  1) Using new PWR ID solution

*/

/**
 * @brief Driver version
 */
#define DRV_PM_VERSION 0x0103

/**
 * @brief Driver name
 */
#define DRV_PM_NAME  "PM"

/**
 * @brief Operating timeout
 */
#define DRV_UART_IO_TIMEOUT  5000


/** 
* Power control
* @param[in]  on   0:Power off, 1:Power on
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
* @note If sensor does NOT require any power control, this fucntion can be NULL
*/ 
pos_status_t sensor_pm_set(pos_u32_t on) {  
  pos_gpio_pin_t pwr;
  ma_board_t *b = g_drv->board;
  pwr = (g_drv->s->slot->io >> 8) & 0x7f;  
  if( !pwr )
    pwr = MB_PWR_ID_PWRH_PWR3; /* both PWRH/EN and 3.3 should be on */

  b->pwr_set(pwr, on);  
  
  return POS_STATUS_OK;
}


void sensor_pm_io_flush(pos_io_handle_t *io) {
  /* 清空串口并等待20ms */
  while( io->read(io, POS_NULL, 1024, 20) >= 1024 );   
}

pos_u32_t sensor_pm_send_buf(pos_io_handle_t *io, const pos_u8_t *buf, pos_u32_t buf_len, pos_u8_t *response, pos_u32_t res_len) {
  pos_u8_t c;
  pos_u32_t v, i;

  /* 清空串口*/
  sensor_pm_io_flush(io);
    
  io->write(io, (pos_u8_t*)buf, buf_len, 100);
  if( !response )
    return 0;
  v = io->read(io, response, 1, 950);
  if( v ) {
    if( response[0] != 0xff )
      return 0;
    v += io->read(io, &response[1], res_len-1, 50);
    c = 0;
    for( i = 1; i < v; i++ ) {
      c += response[i];
    }
    if( c != 0 )
      v = 0; /* force V zero indicating failed crc */      
  }

#if 1
  if( v && (g_drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG) != 0 ) {      
    g_drv->log->buf("pm rd", response, v);
  }
#endif      
  
  return v;
}


pos_u32_t sensor_pm_send(pos_io_handle_t *io, pos_u8_t cmd, pos_u8_t arg, pos_u8_t *response, pos_u32_t res_len) {
  union {
    pos_u8_t buf[9];
    pos_u32_t u32[3];
  } u; 
    
  u.buf[0] = 0xff;
  u.buf[1] = 0x01;
  u.buf[2] = cmd;
  u.buf[3] = arg;
  u.u32[1] = 0;
  u.buf[8] = 0xff - cmd - arg;
  
  return sensor_pm_send_buf(io, u.buf, 9, response, res_len);
}

/** 
* Sensor collecting
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_pm_collect(void){
  pos_io_handle_t *io;
  pos_status_t ret = POS_STATUS_E_TIMEOUT;
  pos_u32_t v, t;
  drv_api_t *drv = g_drv;
  pos_u8_t response[32];
  
  /* 初始化串口 */
  t = drv->s->slot->io;
  
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("pm io", t);        
  }

  t &= 0xff; /* keep io for uart only */
  
  io = POS_NULL;
  if( POS_IO_IS_SERIAL_PORT(t) ) {
    io = drv->os->io->init(t, 9600, 0); /* 默认用9600 */
  }

  v = 0;

  /* refresh to normal by default */
  drv->s->ctrl |= MA_SENSOR_CTRL_STAT_NORMAL;
  drv->s->ctrl &= ~MA_SENSOR_CTRL_STAT_ERROR;
  
  if( io ) {    
    /* setup timeout using AVG sampling durations */
    t = drv->os->tick->get();  
    t += DRV_UART_IO_TIMEOUT;
    do {        
#if 0      
      /* quit Power Saving */
      v = sensor_pm_send(io, 0xa7, 0x00, response, sizeof(response));       
      if( v != 9 || response[2] != 1 )
        continue;
#endif

      /* polling work mode setup */
      sensor_pm_send(io, 0x78, 0x41, POS_NULL, 8);
      drv->os->tick->sleep(250);

      /* query data */    
      v = sensor_pm_send(io, 0x86, 0x00, response, sizeof(response));
      if( v >= 9 ) {
        response[1] = PLSS_PL_EXT_PM; /* PLSS data format: 43 for PLSS_PL_EXT_PM: U16[3] for 2.5/10/1.0 */        
        drv->data->plss_put_raw(7, &response[1]); /* response[2..7] is 2.5/10/1.0 */
        ret = drv->data->put("\"%s\":%u,", "pm2_5", response[2]*256+response[3], 0, 0, 0);
        if( ret == POS_STATUS_OK )
          ret = drv->data->put("\"%s\":%u,", "pm10", response[4]*256+response[5], 0, 0, 0);
        if( ret == POS_STATUS_OK )
          ret = drv->data->put("\"%s\":%u,", "pm1", response[6]*256+response[7], 0, 0, 0);            
        break;
      }

      drv->os->tick->sleep(250);      
    } while( !drv->os->tick->is_timeout(drv->os->tick->get(), t) );   /* repeat until timeout */     

#if 0
    /* enter Power Saving */
    sensor_pm_send(io, 0xa7, 0x01, response, sizeof(response)); 
#endif
    
  } 

  if( v >= 9 ) {    
  } else {
    drv->log->data("pm err", drv->s->slot->io);
    /* 错误的Sensor, 上报出错 */
    drv->data->put("\"%s\":\"no pm#0x%x\",", "err", drv->s->slot->io, 0, 0, 0);
  }

  if( io ) {      
    /* shutdown IO after read */
    io->release(io);
  }

  /* update to error status */
  if( ret != POS_STATUS_OK )
    drv->s->ctrl |= MA_SENSOR_CTRL_STAT_ERROR;
  
  return ret;
}

/*
 * Driver export
 */
ma_drv_export_t g_export = {
  .version = DRV_SENSOR_VERSION(MA_SENSOR_PM, DRV_PM_VERSION),
  .name = DRV_PM_NAME,
  .u.sensor={
    .power_set = sensor_pm_set,
    .collect = sensor_pm_collect,
  },
};

