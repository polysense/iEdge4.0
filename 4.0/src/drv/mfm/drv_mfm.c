/**
 * @file  drv_mfm.c
 * @brief SENSOR PH/ORP driver
 * @author Runby F.
 * @date 2022-11-15
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * Revision history
v1.0 @ 2025-02-17
  1) First revision

v1.1 @ 2025-02-19
  1) Fix byte order issue

v1.2 @ 2025-02-20
  1) Fix flow byte order issue for ntohs

*/

/**
 * @brief Driver version
 */
#define DRV_MFM_VERSION 0x0102

/**
 * @brief Driver name
 */
#define DRV_MFM_NAME  "MFM"


/** 
* Power control
* @param[in]  on   0:Power off, 1:Power on
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
* @note If sensor does NOT require any power control, this fucntion can be NULL
*/ 
pos_status_t sensor_mfm_set(pos_u32_t on) {  
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
pos_status_t sensor_mfm_collect(void){
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
    drv->log->data("mfm io", i);        
    drv->log->data("mfm addr", addr);            
  }  

  do {
    v = drv->data->modbus_read(io, buf, addr, 3, 0x000c, 1); /* read flow */
    if( v != 7 ) {
      ret = POS_STATUS_E_RESOURCE;
      break;
    }
    v = drv->os->util->get_u32(&buf[3], 2); /* 0.01 L/min */
    v = POS_NTOHS(v);
    v = v*10; /* 0.01 L/min == > 0.001 L/min */
    drv->data->plss_json_put(v,
      DRV_MA_PLSS_TRANS(PLSS_PL40_VELOCITY_MFLOW, 4, 0, 0, 3),
      "mflow" );

    v = drv->data->modbus_read(io, buf, addr, 3, 0x000a, 2); /* read volume */
    if( v != 9 ) {
      ret = POS_STATUS_E_RESOURCE;
      break;
    }
    v = drv->os->util->get_u32(&buf[3], 4); /* 0.001 m3 */
    v = POS_NTOHL(v);
    ret = drv->data->plss_json_put(v,
      DRV_MA_PLSS_TRANS(PLSS_PL40_VELOCITY_VOLUME, 4, 0, 0, 3),
      "vol" );

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
  .version = DRV_SENSOR_VERSION(MA_SENSOR_MFM, DRV_MFM_VERSION),
  .name = DRV_MFM_NAME,
  .u.sensor={
    .power_set = sensor_mfm_set,
    .collect = sensor_mfm_collect,
  },
};

