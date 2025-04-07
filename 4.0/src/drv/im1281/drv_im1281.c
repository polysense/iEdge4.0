/**
 * @file  drv_im1281.c
 * @brief SENSOR RS-IM1281 driver
 * @author Runby F.
 * @date 2024-2-26
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * Revision history
v1.0 @ 2024-7-14
  1) First revision

*/

/**
 * @brief Driver version
 */
#define DRV_IM1281_VERSION 0x0100

/**
 * @brief Driver name
 */
#define DRV_IM1281_NAME  "IM1281"


/** 
* Power control
* @param[in]  on   0:Power off, 1:Power on
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
* @note If sensor does NOT require any power control, this fucntion can be NULL
*/ 
pos_status_t sensor_im1281_set(pos_u32_t on) {  
  pos_u16_t pwr;
  ma_board_t *b = g_drv->board;
  pwr = (g_drv->s->slot->io >> 8) & 0x7f;  
  /* default it does NOT require any PWR operation, but if it's not zero pwr, we can control it here */
  if( !pwr ) {
    pwr = MB_PWR_ID_VCCN_PWR3;
  }
  b->pwr_set(pwr, on);
  return POS_STATUS_OK;
}


pos_size_t sensor_im1281_modbus_read(drv_api_t *drv, pos_io_handle_t *io, void *buf, pos_u8_t addr, pos_u8_t func, pos_u16_t reg, pos_u16_t reg_cnt) {
  pos_u8_t cmd[8], *response;
  pos_u16_t crc16, i;
  pos_size_t v, exp_len;
  cmd[0] = addr;
  cmd[1] = func;
  cmd[2] = (reg >> 8)&0xff;
  cmd[3] = reg & 0xff;
  cmd[4] = (reg_cnt >> 8)&0xff;
  cmd[5] = reg_cnt & 0xff;
  crc16 = drv->os->crc(POS_CRC16_MBUS, cmd, 6);
  cmd[6] = crc16&0xff;
  cmd[7] = (crc16>>8)&0xff;

  exp_len = 5 + reg_cnt *4; /* im1281 uses 4 bytes per reg lenth */
  response = (pos_u8_t*)buf;
  for( i = 0; i < 3; i++ ) {
    while( io->read(io, POS_NULL, 1024, 50) >= 1024 ); 
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG) {
      drv->log->buf("imbus wr", cmd, 8);     
      drv->os->flush(); /* flush debug before read/write */
    }
    io->write(io, cmd, 8, 1000);
    v = io->read(io, response , exp_len, 1000);
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG )
      drv->log->buf("imbus rd", response, v);     
    if( v != exp_len )
      continue;      

    /* crc16-modbus */
    crc16 = drv->os->crc(POS_CRC16_MBUS, response , v-2);
    if( crc16 != (response[v-1]<<8)+response[v-2] ) /* crc16 用LE节序存放 */
        continue; /* CRC错误 */

    return v;      
  }
  return 0;
}

/** 
* Sensor collecting
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_im1281_collect(void){
  pos_io_handle_t *io;
  pos_status_t ret = POS_STATUS_E_TIMEOUT;
  pos_u32_t addr, v, id;
  pos_u16_t i, bmp;
  drv_api_t *drv = g_drv;
  pos_u8_t buf[48];
  
  /* 初始化串口 */
  i = drv->s->slot->io;
  addr = drv->s->slot->rsvd32;
  bmp = drv->s->slot->arg.u8[0];
  if( !bmp )
    bmp = 0x9f; /* no carbon (co2:0.0001kg), no temp (0.01) */
  i &= 0xff; /* keep io for uart only */  
  if( POS_IO_IS_SERIAL_PORT(i) ) {
  } else
    i = 0;
  v = (addr >> 8)&0xffffff;
  if( !v ) {    
    v = 4800; /* 默认用4800 */  
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
    drv->log->data("im1281 io", i);        
    drv->log->data("im1281 addr", addr);            
  }

#if 0
/* float dbg */
{
  pos_u32_t f, f2;
  POS_CALL_OP_FLOAT_LOAD_U32(drv->os, f, 10);
  drv->log->data("f", drv->os->util->ufloat_to_u32_precision(f, 10));
  POS_CALL_OP_FLOAT_LOAD_U32(drv->os, f2, 22);
  drv->log->data("f2", drv->os->util->ufloat_to_u32_precision(f2, 10));
  POS_CALL_OP_FLOAT_SUB_FLOAT(drv->os, f, f2);
  drv->log->data("f_sub", drv->os->util->ufloat_to_u32_precision(f, 10));
  POS_CALL_OP_FLOAT_ADD_FLOAT(drv->os, f, f2);
  drv->log->data("f_add", drv->os->util->ufloat_to_u32_precision(f, 10));
  POS_CALL_OP_FLOAT_DIV_FLOAT(drv->os, f, f2);
  drv->log->data("f_DIV", drv->os->util->ufloat_to_u32_precision(f, 10));
  POS_CALL_OP_FLOAT_MUL_FLOAT(drv->os, f, f2);
  drv->log->data("f_mul", drv->os->util->ufloat_to_u32_precision(f, 10));
}
#endif

  do {
    union {
      pos_u32_t u32;
      pos_i32_t i32;
    } u;
    pos_u32_t f_v;
    pos_u32_t divide;
    const struct {
      pos_u8_t  pl_ind;
      char      pl_str[7];
    } vs[] = {
      {PLSS_PL_EM_VAC, "vac",},
      {PLSS_PL_EM_AAC, "amp",},
      {PLSS_PL_EM_W,   "watt",},
      {PLSS_PL_EM_KWH, "kwh",},
      {PLSS_PL_EM_PF,  "pf",},
      {PLSS_PL_EXT_CNTR,  "carbon",},
      {PLSS_PL_TEMPERATURE,  "tmp",},
      {PLSS_PL_EM_HZ,  "hz",},
    };
    v = sensor_im1281_modbus_read(drv, io, buf, addr, 3, 0x0048, 8); /* read */
    if( v != 37 ) {
      ret = POS_STATUS_E_RESOURCE;
#if 1      
      break;
#else 
      /* insert dummy */
      for( v = 3; v < 32; v +=4 ) {
        buf[v] = 0;
        buf[v+1] = 0;
        buf[v+2] = 10;
        buf[v+3] = v;
      }
      buf[27] = 0xff;
      buf[28] = 0xff;
      buf[29] = 0xf0;
      buf[30] = 0x12;
      drv->log->buf("buf",buf, 37);      
#endif      
    }

    for( id = 0; id < 8; id++ ) {
      pos_u8_t *d;
      if( ( (1 << id) & bmp ) == 0 )
        continue;
      d = &buf[3+id*4];
      v = 0;
      for( u.u32 = 0; u.u32 < 4; u.u32++ ) {
        v = (v << 8) + *d++;
      }
      switch (vs[id].pl_ind) {
      case PLSS_PL_TEMPERATURE:
        /* put temp */
        u.u32 = v;
        u.i32 = (u.i32 + 5) / 10;
        drv->data->plss_json_puti(u.i32,
           DRV_MA_PLSS_TRANS(vs[id].pl_ind, 2, 0, 0, 1), 
           vs[id].pl_str);
        break;

      case PLSS_PL_EXT_CNTR:
        /* put carbon (co2 0.0001kg ) as raw value */
        drv->data->plss_json_put(v,
           DRV_MA_PLSS_TRANS(vs[id].pl_ind, 4, 0, 0, 4), 
           vs[id].pl_str);
        break;

      default:
        POS_CALL_OP_FLOAT_LOAD_U32(drv->os, f_v, v);
        if( vs[id].pl_ind == PLSS_PL_EM_HZ ) {
          divide = 100;
        } else {
          divide = 10000;
        }
        POS_CALL_OP_FLOAT_DIV_U32(drv->os, f_v, divide);
        drv->data->plss_put_u32(vs[id].pl_ind, f_v); /* report float32 as u32 binary */
        if( divide == 100 ) {
          drv->data->put("\"%s\":%u.%02u,", vs[id].pl_str, v/100, v%100, 0, 0);
        } else {
          drv->data->put("\"%s\":%u.%04u,", vs[id].pl_str, v/10000, v%10000, 0, 0);
        }
        break;
      }
    }

    ret = POS_STATUS_OK;
  } while(0);
  
  if( ret != POS_STATUS_OK ) {   
    /* set error flag */
    drv->s->ctrl |= MA_SENSOR_CTRL_STAT_ERROR;
    /* 错误的Sensor, 上报出错 */
    drv->data->put("\"%s\":\"no im1281#0x%x/0x%x\",", "err", i, addr, 0, 0);
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
  .version = DRV_SENSOR_VERSION(MA_SENSOR_IM1281, DRV_IM1281_VERSION),
  .name = DRV_IM1281_NAME,
  .u.sensor={
    .power_set = sensor_im1281_set,
    .collect = sensor_im1281_collect,
  },
};

