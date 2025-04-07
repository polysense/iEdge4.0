/**
 * @file  drv_o2ze03.c
 * @brief SENSOR ZE03 O2 driver
 * @author Runby F.
 * @date 2023-01-28
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * 版本修订历史
v1.0 @ 2023-02-02
  1) Support ZE03 O2 sensor

v1.1 @ 2023-08-26
  1) Use explict IO init uart ctrl field

v1.2 @ 2023-11-18
  1) Using new PWR ID solution

*/

/**
 * @brief Driver version
 */
#define DRV_O2ZE03_VERSION 0x0102

/**
 * @brief Driver name
 */
#define DRV_O2ZE03_NAME  "O2ZE03"

/**
 * @brief Operating timeout
 */
#define DRV_UART_IO_TIMEOUT  100

/**
 * @brief Operating timeout retry times
 */
#define DRV_UART_RETRY_NUM  120


/** 
* Power control
* @param[in]  on   0:Power off, 1:Power on
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
* @note If sensor does NOT require any power control, this fucntion can be NULL
*/ 
pos_status_t sensor_o2ze03_set(pos_u32_t on) {  
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
pos_status_t sensor_o2ze03_collect(void){
  pos_io_handle_t *io;
  pos_status_t ret = POS_STATUS_E_TIMEOUT;
  pos_u32_t v, sum, cnt, max, min;
  pos_u16_t t;
  drv_api_t *drv = g_drv;
  pos_u8_t response[12];
  
  /* 初始化串口 */
  t = drv->s->slot->io;

  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("o2ze03 io", t);        
  }

  t &= 0xff; /* keep io for uart only */
  
  io = POS_NULL;
  if( POS_IO_IS_SERIAL_PORT(t) ) {
    v = (drv->s->slot->rsvd32 >> 8)&0xffffff;
    if( !v )
      v = 9600; /* 默认用9600 */
    io = drv->os->io->init(t, v, 0);
  }
  if( !io ) {  
    t = DRV_UART_RETRY_NUM; /* end of loop */
  } else
    t = 0;

  /* 清空串口并等待50ms */
  while( io->read(io, POS_NULL, 1024, 50) >= 1024 );   
  
  /* 重试N次 */
  sum = 0;
  cnt = 0;
  max = 0;
  min = 0xffffffff;
  for(; t < DRV_UART_RETRY_NUM; t++ ) {
    const pos_u8_t cmd_read[9] = {0xff, 0x20, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5a};
    /* 清空串口并等待50ms */
    while( io->read(io, POS_NULL, 1024, 50) >= 1024 ); 

    /* pull command */
    io->write(io, (pos_u8_t*)cmd_read, 9, DRV_UART_IO_TIMEOUT);    
    
    /* 每次等待1000ms响应 */
    v = io->read(io, response, 9, DRV_UART_IO_TIMEOUT);
    //drv->log->buf("RX", response, v); 
    if( v != 9 )
      continue;

    if( response[0] != 0xff && response[1] != 0x86 )
      continue; /* 格式错误 */
    
    /* crc*/
    { 
      pos_u32_t i;
      pos_u8_t c;
      c = response[0];
      for( i = 1; i < v; i++ )
        c += response[i];      
      if( c != 0xff ) /* crc check */
       continue; /* CRC错误 */
    }
    
    v = (response[2]<<8) + response[3];
    cnt++;
    sum += v;
    
    if( cnt >= 1 )
      break;
    
    drv->os->tick->sleep(1000);
    
  }

  if( !cnt ) {
    /* 错误的Sensor, 上报出错 */
    drv->data->put("\"%s\":\"no o2ze03\",", "err", 0, 0, 0, 0);
  } else {
    if( cnt > 2 ) {
      sum = sum - min - max;
      cnt -= 2;
    }
    v = sum / cnt;
    drv->data->plss_put_u16(61, v); /* PLSS data format put, 61 for PLSS_PL_EXT_O2: U16 in  unit of 0.1% */
    
    ret = drv->data->put_raw("\"o2\":", 5);
    if( ret == POS_STATUS_OK )
      ret = drv->data->put_ufloat(v, 10);
    if( ret != POS_STATUS_OK ) {
      drv->s->ctrl |= MA_SENSOR_CTRL_TOO_LONG; /* indicate data buffer put failed */
    }  
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
  .version = DRV_SENSOR_VERSION(MA_SENSOR_O2ZE03, DRV_O2ZE03_VERSION),
  .name = DRV_O2ZE03_NAME,
  .u.sensor={
    .power_set = sensor_o2ze03_set,
    .collect = sensor_o2ze03_collect,
  },
};

