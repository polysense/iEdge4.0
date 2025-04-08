/**
 * @file  drv_mymbus.c
 * @brief SENSOR driver for MODBUS example
 * @author Runby F.
 * @date 2025-4-8
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * Revision history
v1.0 @ 2025-04-08
  1) First revision

*/

/**
 * @brief Driver version
 */
#define DRV_MYMBUS_VERSION 0x0100

/**
 * @brief Driver name
 */
#define DRV_MYMBUS_NAME  "MYMBUS"


/** 
* Power control
* @param[in]  on   0:Power off, 1:Power on
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
* @note If sensor does NOT require any power control, this fucntion can be NULL
*/ 
pos_status_t sensor_mymbus_set(pos_u32_t on) {
  ma_board_t *b = g_drv->board;
  b->pwr_set(MB_PWR_ID_VCC_PWRH, on);
  
  return POS_STATUS_OK;
}

/** 
* Sensor collecting
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_mymbus_collect(void){
  pos_io_handle_t *io;
  pos_status_t ret = POS_STATUS_E_TIMEOUT;
  pos_u32_t addr, v;
  pos_u16_t i;
  drv_api_t *drv = g_drv;
  pos_u8_t buf[16];
  
  i = POS_IO_UART3;
  addr = 1;
  
  /* init io with 9600 baudrate */
  io = drv->os->io->init(i, 9600, 0);

  /* refresh to normal by default */
  drv->s->ctrl |= MA_SENSOR_CTRL_STAT_NORMAL;
  drv->s->ctrl &= ~MA_SENSOR_CTRL_STAT_ERROR;
  
  drv->log->data("mymbus io", i);
  drv->log->data("mymbus addr", addr);

  do {
    v = drv->data->modbus_read(io, buf, addr, 3, 0, 1); /* read(03) 1 modbus register */
    if( v != 7 ) {
      ret = POS_STATUS_E_RESOURCE;
      break;
    }

    v = buf[3]*256+buf[4];
    drv->data->plss_json_put(v,
        DRV_MA_PLSS_TRANS(PLSS_PL_RADIATION, 2, 0, 0, 0),
        "ghi" );
 
  } while(0);

  if( ret != POS_STATUS_OK ) {   
    /* set error flag */
    drv->s->ctrl |= MA_SENSOR_CTRL_STAT_ERROR;
    /* sensor error */
    drv->data->put("\"%s\":\"no sensor#0x%x/0x%x\",", "err", i, addr, 0, 0);
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
  .version = DRV_SENSOR_VERSION(161, DRV_MYMBUS_VERSION), /* 161 is registered sensor type */
  .name = DRV_MYMBUS_NAME,
  .u.sensor={
    .power_set = sensor_mymbus_set,
    .collect = sensor_mymbus_collect,
  },
};

