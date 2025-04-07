/**
 * @file  drv_bmp390.c
 * @brief SENSOR BMP390 driver
 * @author Runby F.
 * @date 2022-3-09
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * Revision History
v1.0 @ 2025-03-09
  1) Support first version

  
*/

/**
 * @brief Driver version
 */
#define DRV_BMP390_VERSION 0x0100

/**
 * @brief Driver name
 */
#define DRV_BMP390_NAME  "BMP390"

#define DRV_BMP390_REG_STATUS   0x03
#define DRV_BMP390_REG_DATA0    0x04
#define DRV_BMP390_REG_PWR_CTRL 0x1b
#define DRV_BMP390_REG_OSR      0x1c
#define DRV_BMP390_REG_ODR      0x1d
#define DRV_BMP390_REG_NVM_PAR  0x31

/** 
 * Sleep without system deep sleep so that I2C can still work
 */
void sensor_bmp390_sleep(drv_api_t *drv, pos_u32_t ticks) {
  const pos_lib_tick_t *t;  
  t = drv->os->tick;
  ticks = t->get() + ticks;
  while( !t->is_timeout(t->get(), ticks) ) {
    t->sleep(20);
  }
}

/**
* Power control
* @param[in]  on   0:Power off, 1:Power on
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
* @note If sensor does NOT require any power control, this fucntion can be NULL
*/ 
pos_status_t sensor_bmp390_set(pos_u32_t on) {  
  pos_u16_t pwr, w1;
  drv_api_t *drv = g_drv;  
  ma_board_t *b = drv->board;
  w1 = drv->s->slot->io;
  pwr = (w1 >> 8) & 0x7f;
  if( !pwr )
    pwr = MB_PWR_ID_VCCN_PWR3;

  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data(on ? "bmp390 on" : "bmp390 off", pwr);
  }

  /* Power ON/OFF
   */
  b->pwr_set(pwr, on);

  return POS_STATUS_OK;
}

/** 
* Sensor Detecting
*/
pos_i16_t sensor_i2c_detect(pos_io_handle_t *io) {
  pos_i16_t t, wr;
  for( t = 0 ; t < 3; t++ ) {
    wr = io->write(io, POS_NULL, 1, 100);
    if( wr > 0 )
      return wr;
  }
  return 0;
}


pos_u32_t sensor_bmp390_read(pos_io_handle_t *io, pos_u8_t reg, pos_u8_t *buf, pos_u32_t exp_len) {
  pos_u32_t i;
  for( i = 0; i < 3; i++ ) {
    if( io->read(io, buf, exp_len, POS_IO_WAIT_I2C_READ8(reg, 1000)) == exp_len ) {
      return exp_len;
    }
  }
  return 0;
}

pos_u8_t sensor_bmp390_read8(pos_io_handle_t *io, pos_u8_t reg) {
  pos_u8_t buf[1];
  if( sensor_bmp390_read(io, reg, buf, 1) == 1 )
    return buf[0];
  return 0;
}

pos_u32_t sensor_bmp390_read24(pos_io_handle_t *io, pos_u8_t reg, pos_u32_t *v) {
  pos_u8_t buf[3];
  pos_u32_t i;
  i = sensor_bmp390_read(io, reg, buf, 3);
  if( i == 3 ) {
    *v = buf[0] + (buf[1]<<8) + (buf[2]<<16);
    return 3;
  }
  return i;
}

pos_u32_t sensor_bmp390_write8(pos_io_handle_t *io, pos_u8_t reg, pos_u8_t value) {
  pos_u8_t buf[2];
  pos_u32_t i;
  buf[0] = reg;
  buf[1] = value;
  for(i = 0; i < 3; i++ ) {
    if( io->write(io, buf, 2, 500) == 2 ) {
      return 1;
    }
  }
  return 0;
}

pos_u32_t sensor_bmp390_write24(pos_io_handle_t *io, pos_u8_t reg, pos_u32_t value) {
  pos_u8_t buf[4];
  pos_u32_t i;
  buf[0] = reg;
  buf[1] = value&0xff;
  buf[2] = (value>>8)&0xff;;
  buf[3] = (value>>16)&0xff;;  
  for(i = 0; i < 3; i++ ) {
    if( io->write(io, buf, 4, 500) == 4 ) {
      return 3;
    }
  }
  return 0;
}

pos_u8_t sensor_bmp390_wait_rdy(drv_api_t *drv, pos_io_handle_t *io, pos_u32_t timeout) {
  pos_u8_t s;
  timeout = drv->os->tick->get() + timeout;
  do {
    s = sensor_bmp390_read8(io, DRV_BMP390_REG_STATUS);
    if( (s & 0x60) == 0x60 ) 
      return s;
    sensor_bmp390_sleep(drv, 100);
  } while(!drv->os->tick->is_timeout(drv->os->tick->get(), timeout));
  return s;
}

/** 
* Sensor collecting
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_bmp390_collect(void){
  pos_status_t ret = POS_STATUS_ERROR;
  drv_api_t *drv = g_drv;

  pos_u16_t bmp;
  pos_i16_t t16;
  pos_u32_t v, t, i;
  pos_u8_t buf[8];
  pos_call_calc_bmp690_t s;
  pos_io_handle_t *io;

  /* init I2C */
  t = drv->s->slot->io;
  bmp = drv->s->slot->rsvd32&0xff;
  if( !bmp )
    bmp = 0x02; /* report hpa by default */

  v = (drv->s->slot->rsvd32>>16)&0xffff;
  if( !v )
    v = 100000; /* 100K by default */
  else
    v *= 1000;

  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("bmp390 io", t);
  }

  t = (t&0xf) + POS_IO_I2C0; /* keep io for i2c only */

  io = drv->os->io->init(t, v, POS_IO_CTRL_I2C_MASTER); /* MASTER */

  /* refresh to normal by default */
  drv->s->ctrl |= MA_SENSOR_CTRL_STAT_NORMAL;
  drv->s->ctrl &= ~MA_SENSOR_CTRL_STAT_ERROR;

  do {
    drv->os->call(POS_CALL_FUNC_ID_SET_I2C_ADDR, io, 0x8ee); /* ADDR8=0xee, REGSIZE=8 (8Bits) */    
    v = sensor_i2c_detect(io);
    if( v ) {
      v += 0xee00;
    } else {
      drv->os->call(POS_CALL_FUNC_ID_SET_I2C_ADDR, io, 0x8ec); /* ADDR8=0xec, REGSIZE=8 (8Bits) */    
      v = sensor_i2c_detect(io);
      v += 0xec00;
    }
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {  
      drv->log->data("bmp390 detect", v);
    }
    if( (v&0xff) == 0 )
      break;

#if 0
    for( i = 0; i < 2; i++ ) {
      v = 0xffffffff;
      sensor_bmp390_read24(io, DRV_BMP390_REG_PWR_CTRL, &v);
      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
        drv->log->data("bmp390 odr_osr_pwr", v);
      }
      if( i )
        break;

      sensor_bmp390_write24(io, DRV_BMP390_REG_PWR_CTRL, 0x040233); /* ODR=0x04, OSR=0x02, PWR_CTRL=0x33 */
    }
#else
    v = drv->s->slot->arg.u16[0];
    if( !v )
      v = 0x0415; /* ODR=0x04 OSR=0x15(OSR_T=2, OSR_P=5) */
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      drv->log->data("bmp390 ODR_OSR", v);
      drv->os->flush();
    }
    sensor_bmp390_write8(io, DRV_BMP390_REG_ODR, (v>>8)&0xff);
    sensor_bmp390_write8(io, DRV_BMP390_REG_OSR, 0x15);
    sensor_bmp390_write8(io, DRV_BMP390_REG_PWR_CTRL, 0x33);
#endif

    v = sensor_bmp390_wait_rdy(drv, io, 3000); /* at most 3s waiting */
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      drv->log->data("bmp390 status", v);
      drv->os->flush();
    }
    if( (v&0x60) != 0x60 )
      break;

    v = sensor_bmp390_read(io, DRV_BMP390_REG_DATA0, buf, 6);
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      drv->log->buf("bmp390 rslt", buf, v);
    }
    if( v != 6 )
      break;


    v = sensor_bmp390_read(io, DRV_BMP390_REG_NVM_PAR, s.nvm, 21);
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      drv->log->buf("bmp390 nvm", s.nvm, v);
    }
    if( v != 21 )
      break;
#if 0
    /* T=15.990181, P=99895.781250 TEST data */
    buf[0]=0x90; buf[1]=0xae;buf[2]=0x68;
    buf[3]=0xc0; buf[4]=0x2b;buf[5]=0x7a;
#endif
    s.vrslt[1] = drv->os->util->get_u32(buf, 3); /* #1 is for Buf[0..2] Press */
    s.vrslt[0] = drv->os->util->get_u32(&buf[3], 3); /* #2 is for Buf[3..5] HPA */

    drv->os->call(POS_CALL_FUNC_ID_CALC_BMP690, &s);

    /* report hpa */
    if( bmp & 2 ) {
      v = drv->os->util->ufloat_to_u32_precision(s.vrslt[1], 100);

      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
        drv->log->data("bmp390 hpa", v);
      }
      v=(v+50)/100; /* round up to 1 PA unit */
      drv->history->mv[5] = (v) & 0xffff; 
      drv->history->mv[6] = (v >> 16) & 0xffff;

      /* Report */
      drv->data->plss_json_put(v, 
        DRV_MA_PLSS_TRANS(PLSS_PL_PRESSURE, 4, 0, 0, 2),
        "hpa");
    }

    /* report tmp */
    if( bmp & 1 ) {
      
      v = s.vrslt[0];
      i = v & 0x80000000; /* store flag */
      v &= 0x7fffffff;
      v = drv->os->util->ufloat_to_u32_precision(v,1000);
      if( i ) {
        v = (~v)+1; /* when negative, transalte v */
      }
      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
        drv->log->data("bmp390 tmp", v);
      }
      v=(v+50)/100; /* round up to 0.1 unit */
      t16 = (pos_i32_t)v;
      drv->history->temp[0] = t16;
      drv->history->temp_num = 1;

      /* Report */
      drv->data->put_raw("\"tmp\":", 6);
      drv->data->put_float(v, 10);
      drv->data->plss_put_u16(PLSS_PL_EXT_TEMPERATURE, (pos_u16_t)t16); /* 14 - PLSS_PL_EXT_TEMPERATURE, 2Bytes, 0.1degree */
    }

    ret = POS_STATUS_OK;

  }while(0);

  /* release */
  if( io )
    io->release(io);

  /* update to error status */
  if( ret != POS_STATUS_OK ) {
    /* mark hpa as error */
    if( bmp & 2 ) {
      drv->history->mv[5] = 0;
      drv->history->mv[6] = 0;
    }
    /* mark tmp as error */
    if( bmp & 1 ) {
      drv->history->temp[0] = -9990; /* init -999.0 for default error */
    }
    drv->s->ctrl |= MA_SENSOR_CTRL_STAT_ERROR;
  }

  return ret;

}

/*
 * Driver export
 */
ma_drv_export_t g_export = {
  .version = DRV_SENSOR_VERSION(MA_SENSOR_BMP390, DRV_BMP390_VERSION),
  .name = DRV_BMP390_NAME,
  .u.sensor={
    .power_set = sensor_bmp390_set,
    .collect = sensor_bmp390_collect,
  },
};

