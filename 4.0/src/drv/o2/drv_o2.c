/**
 * @file  drv_o2.c
 * @brief SENSOR OSM-DMH O2 driver
 * @author Runby F.
 * @date 2023-01-28
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * 版本修订历史
v1.0 @ 2023-01-28
  1) Support OSM-DMH O2 sensor by default

v1.1 @ 2023-08-26
  1) Use explict IO init uart ctrl field

v1.2 @ 2023-11-18
  1) Using new PWR ID solution

*/

/**
 * @brief Driver version
 */
#define DRV_O2_VERSION 0x0102

/**
 * @brief Driver name
 */
#define DRV_O2_NAME  "O2"

/**
 * @brief Operating timeout
 */
#define DRV_UART_IO_TIMEOUT  1000

/**
 * @brief Operating timeout retry times
 */
#define DRV_UART_RETRY_NUM  12


/** 
* Power control
* @param[in]  on   0:Power off, 1:Power on
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
* @note If sensor does NOT require any power control, this fucntion can be NULL
*/ 
pos_status_t sensor_o2_set(pos_u32_t on) {  
  pos_gpio_pin_t pwr;
  ma_board_t *b = g_drv->board;
  pwr = (g_drv->s->slot->io >> 8) & 0x7f;  
  if( !pwr )
    pwr = MB_PWR_ID_VCCN_PWR3;

  b->pwr_set(pwr, on);
  
  return POS_STATUS_OK;
}

pos_status_t sensor_o2_mbus_write(pos_io_handle_t *io, drv_api_t *drv, pos_u8_t addr, pos_u8_t cmd, pos_u16_t reg, pos_u16_t value) {
  pos_u8_t response[8];
  pos_u16_t crc16;
  response[0] = addr;
  response[1] = cmd;
  response[2] = reg >> 8;
  response[3] = reg & 0xff;
  response[4] = value >> 8;;
  response[5] = value & 0xff;
  crc16 = drv->os->crc(POS_CRC16_MBUS, response, 6);
  response[6] = crc16&0xff;
  response[7] = (crc16>>8)&0xff;
  io->write(io, response, 8, DRV_UART_IO_TIMEOUT);
  return POS_STATUS_OK;
}

/** 
* Sensor collecting
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_o2_collect(void){
  pos_io_handle_t *io;
  pos_status_t ret = POS_STATUS_E_TIMEOUT;
  pos_u32_t v, modbus_addr, sum, cnt, max, min;
  pos_u16_t t, crc16;
  drv_api_t *drv = g_drv;
  pos_u8_t response[8];
  
  /* 初始化串口 */
  t = drv->s->slot->io;
  modbus_addr = drv->s->slot->rsvd32;

  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("o2 io", t);        
    drv->log->data("o2 addr", modbus_addr);    
  }

  t &= 0xff; /* keep io for uart only */
  
  io = POS_NULL;
  if( POS_IO_IS_SERIAL_PORT(t) ) {
    v = (modbus_addr >> 8)&0xffffff;
    if( !v )
      v = 9600; /* 默认用9600 */
    io = drv->os->io->init(t, v, 0);
  }
  if( !io ) {  
    t = DRV_UART_RETRY_NUM; /* end of loop */
  } else
    t = 0;

  modbus_addr &= 0xff;
  if( !modbus_addr )
    modbus_addr = 0xff; /* use 0xff by default */

  /* 清空串口并等待50ms */
  while( io->read(io, POS_NULL, 1024, 50) >= 1024 );   
  /* Start to work*/
  sensor_o2_mbus_write(io, drv, modbus_addr, 1, 0, 0);

  /* Delay 300s */
  drv->os->tick->sleep(60000); 
  
  /* 重试N次 */
  sum = 0;
  cnt = 0;
  max = 0;
  min = 0xffffffff;
  for(; t < DRV_UART_RETRY_NUM; t++ ) {
    /* 清空串口并等待50ms */
    while( io->read(io, POS_NULL, 1024, 50) >= 1024 ); 
    
    /* 构造Modbus报文并发送 */
    if( sensor_o2_mbus_write(io, drv, modbus_addr, 2, 0, 0) != POS_STATUS_OK ) /* read AD */
      continue;

    /* 每次等待1000ms响应 */
    v = io->read(io, response, 8, DRV_UART_IO_TIMEOUT);
    //drv->log->buf("RX", response, v); 
    if( v != 8 )
      continue;

    if( response[1] != 2 )
      continue; /* 格式错误 */
    
    /* crc16-modbus */
    crc16 = drv->os->crc(POS_CRC16_MBUS, response, 6);
    if( crc16 != (response[7]<<8)+response[6] ) /* crc16 用LE节序存放 */
      continue; /* CRC错误 */

    v = (response[4]<<8) + response[5];
    //drv->log->data("RX PH", v); 
    v = v * 2095 / 1310; /* translate from AD to O2 unit:0.01% */
    cnt++;
    sum += v;
    if( v > max )
      max = v;
    if( v < min )
      min = v;
    if( cnt >= 10 )
      break;
    
    drv->os->tick->sleep(1000);
    
  }

  /* Stop work*/
  sensor_o2_mbus_write(io, drv, modbus_addr, 1, 1, 0);

  if( !cnt ) {
    drv->log->data("o2 err", modbus_addr);
    /* 错误的Sensor, 上报出错 */
    drv->data->put("\"%s\":\"no o2#0x%x\",", "err", modbus_addr, 0, 0, 0);
  } else {
    if( cnt > 2 ) {
      sum = sum - min - max;
      cnt -= 2;
    }
    v = sum / cnt;
    drv->data->plss_put_u16(61, v/10); /* PLSS data format put, 61 for PLSS_PL_EXT_O2ZE03: U16 in  unit of 0.1% */
    
    ret = drv->data->put_raw("\"o2\":", 5);
    if( ret == POS_STATUS_OK )
      ret = drv->data->put_ufloat(v, 100);
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
  .version = DRV_SENSOR_VERSION(MA_SENSOR_O2, DRV_O2_VERSION),
  .name = DRV_O2_NAME,
  .u.sensor={
    .power_set = sensor_o2_set,
    .collect = sensor_o2_collect,
  },
};

