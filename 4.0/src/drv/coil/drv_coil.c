/**
 * @file  drv_coil.c
 * @brief SENSOR coil driver
 * @author Runby F.
 * @date 2022-11-15
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * Revision history
v1.0 @ 2025-02-24
  1) First revision

v1.1 @ 2025-02-25
  1) support thw0 for multiple report bitmap control

*/

/**
 * @brief Driver version
 */
#define DRV_COIL_VERSION 0x0101

/**
 * @brief Driver name
 */
#define DRV_COIL_NAME  "COIL"


/** 
* Power control
* @param[in]  on   0:Power off, 1:Power on
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
* @note If sensor does NOT require any power control, this fucntion can be NULL
*/ 
pos_status_t sensor_coil_set(pos_u32_t on) {  
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
pos_status_t sensor_coil_collect(void){
  pos_io_handle_t *io;
  pos_status_t ret = POS_STATUS_E_TIMEOUT;
  pos_u32_t addr, v, t, b, s;
  pos_u16_t i;
  drv_api_t *drv = g_drv;
  pos_u8_t buf[32];
  
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
    drv->log->data("coil io", i);        
    drv->log->data("coil addr", addr);            
  }  

  do {
    /* write current transformer value when not zero */
    t = drv->s->slot->arg.u16[0];
    if( t ) {
      v = drv->data->modbus_read(io, buf, addr, 4, 0x0006, 1);
      if( v != 7 ) {
        ret = POS_STATUS_E_RESOURCE;
        break;
      }
      v = buf[3] * 256 + buf[4];
      if( v != t ) {
        buf[0] = addr;
        buf[1] = 0x10;
        buf[2] = 0;
        buf[3] = 6;
        buf[4] = 0;
        buf[5] = 1;
        buf[6] = 2;
        buf[7] = (t>>8)&0xff;
        buf[8] = t&0xff;
        v = drv->os->crc(POS_CRC16_MBUS, buf, 9);
        buf[9] = v & 0xff;
        buf[10] = (v>>8) & 0xff;
        if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
          drv->log->buf("coil wr", buf, 11);     
          drv->os->flush(); /* flush debug before read/write */
        }
        io->write(io, buf, 11, 100);
        v = io->read(io, buf, sizeof(buf), 950);
        if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
          drv->log->buf("coil rd", buf, v);
          drv->os->flush();
        }
      }
    }
    v = drv->data->modbus_read(io, buf, addr, 4, 0x001c, 8); /* read current */
    if( v != 21 ) {
      ret = POS_STATUS_E_RESOURCE;
      break;
    }
    ret = POS_STATUS_OK;
    
    /* multiple report */
    t = drv->s->slot->thld.u16[0]&0xff;
    if( !t )
      t = 0xf;
    s = 0;
    for( b = 0; b < 4; b++ ) {
      if( ((1 << b) & t) == 0 )
        continue;
      v = drv->os->util->get_u32(&buf[3+b*4], 4);
      v = POS_NTOHL(v);
      drv->data->plss_put_u32(PLSS_PL_EM_AAC, v); /* report float32 directly binary */
      v = drv->os->util->ufloat_to_u32_precision(v, 1000);
      if( s == 0 ) {
        drv->data->put("\"%s\":[%u.%03u", "amp", v/1000, v%1000, 0, 0);
      } else {
        drv->data->put(",%u.%03u", POS_NULL, v/1000, v%1000, 0, 0);
      }
      s++;
    }
    drv->data->put_raw("],",2);
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
  .version = DRV_SENSOR_VERSION(MA_SENSOR_COIL, DRV_COIL_VERSION),
  .name = DRV_COIL_NAME,
  .u.sensor={
    .power_set = sensor_coil_set,
    .collect = sensor_coil_collect,
  },
};

