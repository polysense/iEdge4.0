/**
 * @file  drv_oiw.c
 * @brief SENSOR oiw driver
 * @author Runby F.
 * @date 2022-11-15
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * Revision history
v1.0 @ 2025-02-24
  1) First revision

*/

/**
 * @brief Driver version
 */
#define DRV_OIW_VERSION 0x0100

/**
 * @brief Driver name
 */
#define DRV_OIW_NAME  "OIW"


/** 
* Power control
* @param[in]  on   0:Power off, 1:Power on
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
* @note If sensor does NOT require any power control, this fucntion can be NULL
*/ 
pos_status_t sensor_oiw_set(pos_u32_t on) { 
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
pos_status_t sensor_oiw_collect(void){
  pos_io_handle_t *io;
  pos_status_t ret = POS_STATUS_E_TIMEOUT;
  pos_u32_t addr, v, b, bmp;
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
    drv->log->data("oiw io", i);
    drv->log->data("oiw addr", addr);
  }  

  do {
#if 0
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
          drv->log->buf("oiw wr", buf, 11);
          drv->os->flush(); /* flush debug before read/write */
        }
        io->write(io, buf, 11, 100);
        v = io->read(io, buf, sizeof(buf), 950);
        if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
          drv->log->buf("oiw rd", buf, v);
          drv->os->flush();
        }
      }
    }
#endif
    bmp = drv->s->slot->arg.u8[0];
    if( !bmp )
      bmp = 7;

    v = drv->data->modbus_read(io, buf, addr, 3, 0x2600, 6); /* read tmp/rfu/oiw */
    if( v != 17 ) {
      ret = POS_STATUS_E_RESOURCE;
      break;
    }
    ret = POS_STATUS_OK;

    for( b = 0; b < 3; b++ ) {
      if( ((1<<b)&bmp) == 0 )
        continue;
      v = drv->os->util->get_u32(&buf[3+b*4], 4);
      switch(b) {
      case 0: /* tmp */
        v = drv->os->util->ufloat_to_u32_precision(v, 10);
        drv->data->plss_json_puti((pos_i32_t)v,
          DRV_MA_PLSS_TRANS(PLSS_PL_TEMPERATURE, 2, 0, 0, 1),
          "tmp");
        break;

      case 1: /* rfu */
        v = drv->os->util->ufloat_to_u32_precision(v, 1000);
        drv->data->put("\"%s\":%u.%03u,", "rfu", v/1000, v%1000, 0, 0);
        v = ((v+5)/10)&0xffff; /* roud up to 0.01 unit */
        v += 0xc0120000; /* 0xC0:RFU, 1:VOL%, 2:0.01 unit */
        drv->data->plss_put_u32(PLSS_PL_MULTI1, v);
        break;

      case 2: /* oiw */
      default:
        v = drv->os->util->ufloat_to_u32_precision(v, 1000);
        drv->data->put("\"%s\":%u.%03u,", "oiw", v/1000, v%1000, 0, 0);
        if( v > 65535 ) /* truncate to 65535 at most */
          v = 65535;
        v += 0xc1230000; /* 0xC1:oiw, 2:PPM, 0:0.001 unit */
        drv->data->plss_put_u32(PLSS_PL_MULTI1, v);

      }
    }
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
  .version = DRV_SENSOR_VERSION(MA_SENSOR_OIW, DRV_OIW_VERSION),
  .name = DRV_OIW_NAME,
  .u.sensor={
    .power_set = sensor_oiw_set,
    .collect = sensor_oiw_collect,
  },
};

