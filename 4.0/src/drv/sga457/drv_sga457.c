/**
 * @file  drv_sga457.c
 * @brief SENSOR sga457 driver for Temp/Hum/EC/N/P/K/PH
 * @author Runby F.
 * @date 2022-10-30
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * Revision history
v1.0 @ 2024-08-08
  1) First draft

*/

/**
 * @brief Driver version
 */
#define DRV_SGA457_VERSION 0x0100

/**
 * @brief Driver name
 */
#define DRV_SGA457_NAME  "SGA457"


/** 
* Power control
* @param[in]  on   0:Power off, 1:Power on
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
* @note If sensor does NOT require any power control, this fucntion can be NULL
*/ 
pos_status_t sensor_sga457_set(pos_u32_t on) {  
  pos_u16_t pwr;
  ma_board_t *b = g_drv->board;
  pwr = (g_drv->s->slot->io >> 8) & 0x7f;  
  if( !pwr ) {
    pwr = MB_PWR_ID_PWRH_PWR3;
  }

  b->pwr_set(pwr, on);
  
  return POS_STATUS_OK;
}


/** 
* Sensor caliration
*/ 
pos_status_t sensor_sga457_calib(pos_io_handle_t *io, pos_u8_t addr, pos_u16_t calib_v, pos_u8_t *response) {
  pos_u8_t cmd[8], v;
  pos_u32_t t, crc16;
  drv_api_t *drv = g_drv;
  pos_status_t ret = POS_STATUS_ERROR;
  t = drv->os->tick->get() + 10000; /* 10s at most */
  do {
    cmd[0] = addr;
    cmd[1] = 6;
    if( calib_v == 0xfffe ) {
      cmd[2] = 0x01;
      cmd[3] = 0x10;
      cmd[4] = 0x00;
      cmd[5] = 0xaa;
    } else {
      cmd[2] = 0x01;
      cmd[3] = 0x11;
      cmd[4] = (calib_v >> 8)&0xff;
      cmd[5] = calib_v & 0xff;
    }
    crc16 = drv->os->crc(POS_CRC16_MBUS, cmd, 6);
    cmd[6] = crc16&0xff;
    cmd[7] = (crc16>>8)&0xff;
    io->write(io, cmd, 8, 1000);
    v = io->read(io, response, 8, 1000);
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      drv->log->buf("sga ctx", cmd, 8);
      drv->log->buf("sga crx", response, v);
    }
    if( v == 8 ) {
      ret = POS_STATUS_OK;
      break; /* Bit[2]: Factory calibration restored flag */
    }
    drv->os->tick->sleep(1000);
  } while( !drv->os->tick->is_timeout(drv->os->tick->get(), t));
  return ret;
}

/** 
* Sensor collecting
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_sga457_collect(void){
  pos_io_handle_t *io;
  pos_status_t ret = POS_STATUS_E_TIMEOUT;
  pos_u32_t addr, v;
  pos_u16_t i;
  drv_api_t *drv = g_drv;
  
  /* 初始化串口 */
  i = drv->s->slot->io;
  addr = drv->s->slot->rsvd32;
  
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("sga io", i);        
    drv->log->data("sga addr", addr);            
  }  
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

  do {
    pos_u8_t buf[32];
    pos_u16_t t, p, u; /* type precision, unit */
    pos_u32_t div, l;
    char *js;
    static const struct {
      const char json[6];
      pos_u8_t type;
    } sga457[] = {
      {"ch4",   0x05,},
      {"freon", 0x48,},
    };

    t = drv->s->slot->arg.u16[0];
    if( t ) {
      /* calib if b is set */
      if( t == 0xffff )
        t = 0; /* when 0xffff use zero; when 0xfffe calib will triger ZADJ */
      sensor_sga457_calib(io, addr, t, buf);
      v = (pos_u32_t)&drv->s->slot->arg.u32[0] - MA_EEPROM_ADDR;
      buf[0] = 0;
      buf[1] = 0;
      drv->eeprom->update(v, buf, 2); /* clear after calib */
    }

    l = drv->data->modbus_read(io, buf, addr, 3, 0x100, 5); /* read STATUS, VALUE, PRECISION, TYPE, UNIT */
    if( l < 5 + 5 * 2 ) 
      break; /* wrong read */
    v = (buf[5] << 8) + buf[6];
    p = buf[8];
    t = (buf[9] << 8) + buf[10];    
    u = buf[12];
    if( u == 0 )
      u = PLSS_MULTI_UNIT_LEL;
    else if( u == 4 )
      u = PLSS_MULTI_UNIT_NONE;
    js = "gas"; /* default to unknown gas */
    for( l = 0; l < POS_ARRAY_CNT(sga457); l++ ) {
      if( sga457[l].type == t ) {
        js = (char*)sga457[l].json;
        break;
      }
    }
    /* plss put */
    buf[0] = PLSS_PL_MULTI1;
    buf[1] = t;
    buf[2] = ((u&0xf)<<4)+(p&0xf);
    buf[3] = buf[5];
    buf[4] = buf[6];
    drv->data->plss_put_raw(5, buf);

    /* json put */
    div = 1;
    if( p && p < 6 ) {
      for( l = 0; l < p; l++ )
        div *= 10;
    }
    ret = drv->data->put("\"%s\":", js, 0, 0, 0, 0);
    if( ret == POS_STATUS_OK )
      drv->data->put_ufloat(v, div);
  }while(0);
  
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
  .version = DRV_SENSOR_VERSION(MA_SENSOR_SGA457, DRV_SGA457_VERSION),
  .name = DRV_SGA457_NAME,
  .u.sensor={
    .power_set = sensor_sga457_set,
    .collect = sensor_sga457_collect,
  },
};

