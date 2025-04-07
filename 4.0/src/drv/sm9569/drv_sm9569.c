/**
 * @file  drv_sm9569.c
 * @brief SENSOR sm9569 driver
 * @author Runby F.
 * @date 2022-11-15
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * Revision history
v1.0 @ 2025-03-24
  1) First revision

*/

/**
 * @brief Driver version
 */
#define DRV_SM9569_VERSION 0x0100

/**
 * @brief Driver name
 */
#define DRV_SM9569_NAME  "SM9569"


/** 
* Power control
* @param[in]  on   0:Power off, 1:Power on
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
* @note If sensor does NOT require any power control, this fucntion can be NULL
*/ 
pos_status_t sensor_sm9569_set(pos_u32_t on) { 
  pos_u16_t pwr;
  ma_board_t *b = g_drv->board;
  pwr = (g_drv->s->slot->io >> 8) & 0x7f;  
  if( !pwr ) {
      pwr = MB_PWR_ID_VCC_PWRH;
  }

  b->pwr_set(pwr, on);
  
  return POS_STATUS_OK;
}

/** 
* Sensor collecting
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_sm9569_collect(void){
  pos_io_handle_t *io;
  pos_status_t ret = POS_STATUS_E_TIMEOUT;
  pos_u32_t addr, v;
  pos_u16_t i;
  drv_api_t *drv = g_drv;
  pos_u8_t buf[16];
  
  /* 初始化串口 */
  i = drv->s->slot->io;
  addr = drv->s->slot->rsvd32;
  
  i &= 0xff; /* keep io for uart only */  
  if( POS_IO_IS_SERIAL_PORT(i) ) {
  } else
    i = 0;
  v = (addr >> 8)&0xffffff;
  if( !v ) {    
    v = 9600; /* 默认用9600 */  
  }
  io = drv->os->io->init(i, v, 0);

  /* refresh to normal by default */
  drv->s->ctrl |= MA_SENSOR_CTRL_STAT_NORMAL;
  drv->s->ctrl &= ~MA_SENSOR_CTRL_STAT_ERROR;
  
  addr &= 0xff; /* keep addr for mbus addr only */
  if( !addr ) {
    addr = 1; /* when 0, use 1 by default */
  }

  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("sm9569 io", i);
    drv->log->data("sm9569 addr", addr);
  }  

  do {
    v = drv->data->modbus_read(io, buf, addr, 3, 0, 1); /* read tmp/rfu/sm9569 */
    if( v != 7 ) {
      ret = POS_STATUS_E_RESOURCE;
      break;
    }

    v = buf[3]*256+buf[4];
    ret = drv->data->plss_json_put(v,
        DRV_MA_PLSS_TRANS(PLSS_PL_RADIATION, 2, 0, 0, 0),
        "ghi" );
 
  } while(0);

  if( ret != POS_STATUS_OK ) {   
    /* set error flag */
    drv->s->ctrl |= MA_SENSOR_CTRL_STAT_ERROR;
    /* 错误的Sensor, 上报出错 */
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
  .version = DRV_SENSOR_VERSION(MA_SENSOR_SM9569, DRV_SM9569_VERSION),
  .name = DRV_SM9569_NAME,
  .u.sensor={
    .power_set = sensor_sm9569_set,
    .collect = sensor_sm9569_collect,
  },
};

