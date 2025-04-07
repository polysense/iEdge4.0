/**
 * @file  drv_puf.c
 * @brief SENSOR PUF driver
 * @author Runby F.
 * @date 2022-3-09
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * Revision History
v1.0 @ 2024-12-30
  1) Support first version

*/

/**
 * @brief Driver version
 */
#define DRV_PUF_VERSION 0x0100

/**
 * @brief Driver name
 */
#define DRV_PUF_NAME  "PUF"

/**
* Power control
* @param[in]  on   0:Power off, 1:Power on
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
* @note If sensor does NOT require any power control, this fucntion can be NULL
*/ 
pos_status_t sensor_puf_set(pos_u32_t on) {  
  pos_u16_t pwr, w1;
  drv_api_t *drv = g_drv;  
  ma_board_t *b = drv->board;
  w1 = drv->s->slot->io;
  pwr = (w1 >> 8) & 0x7f;
  if( !pwr )
    pwr = MB_PWR_ID_VCC_PWRH;
  
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data(on ? "puf on" : "puf off", pwr);
  }  

  /* Power ON/OFF
   */
  b->pwr_set(pwr, on);

  return POS_STATUS_OK;
}

/** 
* Sensor Detecting
*/
pos_i16_t sensor_puf_on_off(pos_io_handle_t *io, pos_u8_t state) {
  pos_i16_t t, wr;
  pos_u8_t response[2];
  response[0] = 0;
  response[1] = state ? 0x20 : 0x10; /* 0x20 - ON, 0x10 - OFF */
  for( t = 0; t < 2; t++ ) {
    wr = io->write(io, response, 2, 300);
    if( wr >= 2 )
      break;
  }

  return wr;
}


/** 
 * Sleep without system deep sleep so that I2C can still work
 */
void sensor_puf_i2c_sleep(pos_u32_t ticks) {
  const pos_lib_tick_t *t;  
  t = g_drv->os->tick;
  ticks = t->get() + ticks;
  while( !t->is_timeout(t->get(), ticks) ) {
    t->sleep(5);
  }
}

/**
* Sensor collecting I2C Gas
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_puf_challenge(pos_u16_t io_id, pos_u32_t baud) {
  pos_io_handle_t *io;
  pos_status_t ret = POS_STATUS_E_TIMEOUT;
  pos_u32_t v, i;
  pos_u16_t loop;
  drv_api_t *drv = g_drv;
  pos_u8_t response[64];


  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("puf i2c", io_id);        
  }

  io = POS_NULL;
  v = baud;
  if( !v )
    v = 8192; /* Ä¬ÈÏÓÃ8KHz */  
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("i2c bd", v);
  }    
  io = drv->os->io->init(io_id, v, POS_IO_CTRL_I2C_MASTER); /* MASTER */
  if( !io ) {  
    return ret;
  }

  /* PUF/ON */
  sensor_puf_on_off(io, 1);
  sensor_puf_i2c_sleep(100);
  
  /* I2C Reading */
  drv->os->call(POS_CALL_FUNC_ID_SET_I2C_ADDR, io, 0x00C8); /* ADDR8=0xC8(0x64*2), REGSIZE=0 (0Bits) */    
  for(loop = 0; loop < 3; loop++ ) {
    response[0] = 0;
    response[1] = 0x50;
    for( v = 0; v < 4; v++ ) {
      drv->os->memcpy(&response[2+v*8], drv->cfg->eui, 8);
    } 
    v = io->write(io, response, 34, 300);
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      drv->log->buf("puf chan", response, v);
      drv->os->flush();
    }    
    sensor_puf_i2c_sleep(50);

    v = io->read(io, response, 32, 300);
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      drv->log->buf("puf rsp", response, v);
      i = drv->os->crc(POS_CRC8, response, v);
      drv->log->data("puf crc8", i);
    }
    if( v >= 32 )
      break;

    sensor_puf_i2c_sleep(1000); /* needs sleep before next read, or else read will be failed */
  }

  /* PUF/OFF */
  sensor_puf_on_off(io, 0);

  /* shutdown IO after read */
  io->release(io);

  return ret;
}

/** 
* Sensor collecting
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_puf_collect(void){
  pos_status_t ret = POS_STATUS_OK, r;
  drv_api_t *drv = g_drv;
  pos_u32_t baud, i;

  baud = (drv->s->slot->rsvd32>>16)&0xffff;
  if( baud )
    baud *= 1000;
  
  /* refresh to normal by default */
  drv->s->ctrl |= MA_SENSOR_CTRL_STAT_NORMAL;
  drv->s->ctrl &= ~MA_SENSOR_CTRL_STAT_ERROR;

  i = (drv->s->slot->io&0xf)+POS_IO_I2C0;
  r = sensor_puf_challenge(i, baud);
  if( r != POS_STATUS_OK )
      ret = r;    
 
  /* update to error status */
  if( ret != POS_STATUS_OK )
    drv->s->ctrl |= MA_SENSOR_CTRL_STAT_ERROR;
  
  return ret;  
}


/*
 * Driver export
 */
ma_drv_export_t g_export = {
  .version = DRV_SENSOR_VERSION(MA_SENSOR_PUF, DRV_PUF_VERSION),
  .name = DRV_PUF_NAME,
  .u.sensor={
    .power_set = sensor_puf_set, 
    .collect = sensor_puf_collect,
  },
};

