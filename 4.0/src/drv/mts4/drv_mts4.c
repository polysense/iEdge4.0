/**
 * @file  drv_mts4.c
 * @brief SENSOR MTS4 driver
 * @author Runby F.
 * @date 2022-3-09
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * Revision History
v1.0 @ 2024-08-26
  1) Support first version

v1.1 @ 2024-09-14
  1) Fix negative calc issue

v1.2 @ 2025-02-19
  1) Use default error temp as -999.0
  
*/

/**
 * @brief Driver version
 */
#define DRV_MTS4_VERSION 0x0102

/**
 * @brief Driver name
 */
#define DRV_MTS4_NAME  "MTS4"

/**
 * @brief MAX MTS4 history data num
 */
#define DRV_MTS4_HISTORY_MAX 32

/**
 * @brief IRQ控制结构
 */
typedef struct {
  drv_api_t     drv;      /**< 驱动克隆，指向固定SLOT的驱动指针 */
  pos_u16_t     temp_num; /**< temp num */
  pos_u16_t     rsvd;     /**< rsvd */
  pos_i16_t     temp[DRV_MTS4_HISTORY_MAX]; /**< temp history */
} mts4_param_t;

/** 
* Power control
* @param[in]  on   0:Power off, 1:Power on
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
* @note If sensor does NOT require any power control, this fucntion can be NULL
*/ 
pos_status_t sensor_mts4_set(pos_u32_t on) {  
  pos_u16_t pwr, w1;
  drv_api_t *drv = g_drv;  
  ma_board_t *b = drv->board;
  w1 = drv->s->slot->io;
  pwr = (w1 >> 8) & 0x7f;
  if( !pwr )
    pwr = MB_PWR_ID_VCCN_PWR3;

  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data(on ? "mts4 on" : "mts4 off", pwr);
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

/** 
* Sensor slot lkup
*/
pos_status_t sensor_mts4_slot_id_get(pos_u8_t *id, pos_u8_t *last) {
  pos_u8_t i, sum_slot, my_id;
  drv_api_t *drv = g_drv;
  sum_slot = 0;
  my_id = 0;
  for( i = 0; i < MA_SENSOR_SLOT_NUM; i++ ) {
    if( drv->cfg->slot[i].sensor_type == drv->s->slot->sensor_type 
      &&
      drv->cfg->slot[i].cycle == drv->s->slot->cycle
      &&
      (drv->cfg->slot[i].rsvd32&0xffff) == (drv->s->slot->rsvd32&0xffff)
      ) {
      if( drv->s->slot == &drv->cfg->slot[i] )
        my_id = sum_slot; /* record my id */
      ++sum_slot;
    } else {
    }
  }
  if( !sum_slot )
    return POS_STATUS_E_NOT_FOUND;
  if( my_id + 1 >= sum_slot )
    *last = 1; /* set final flag */
  else
    *last = 0; /* clear final flag */
  *id = my_id;
  return POS_STATUS_OK;
}

/**
* Sensor data push to last (skip earliest one)
*/
void sensor_mts4_push(pos_i16_t *buf, pos_i16_t d, pos_u16_t max) {
  pos_u16_t i;
  for( i = 1; i < max; i++ )
    buf[i-1] = buf[i]; /* #0..#N-1 ==> #1..#N-1 #N-1 */
  buf[max-1] = d; /* store to last */
}

pos_u8_t sensor_mts4_crc8(pos_u8_t *serial, pos_u8_t length) {
  pos_u8_t result = 0x00;
  pos_u8_t pDataBuf;
  pos_u8_t i;

  while(length--) {
    pDataBuf = *serial++;
    for(i=0; i<8; i++) {
      if((result^(pDataBuf))&0x01) {
        result ^= 0x18;
        result >>= 1;
        result |= 0x80;
      }
      else {
        result >>= 1;
      }
      pDataBuf >>= 1;
    }
  }
		
  return result;
}

pos_u32_t sensor_mts4_read(pos_io_handle_t *io, pos_u8_t reg, pos_u8_t *buf, pos_u32_t exp_len) {
  pos_u32_t i;
  for( i = 0; i < exp_len; i++ ) {
    if( io->read(io, buf, 1, POS_IO_WAIT_I2C_READ8(reg, 1000)) != 1 ) {
      break;
    }
    buf++;
    reg++;
  }
  return i;
}

/** 
* Sensor collecting
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_mts4_collect(void){
  pos_status_t ret = POS_STATUS_ERROR;
  drv_api_t *drv = g_drv;
  pos_i32_t t;
  pos_u8_t bmp;
  pos_i16_t t16;
  pos_u32_t baud;
  pos_u8_t buf[8], id, last, i;
  pos_io_handle_t *io;

  /* init I2C */
  t = drv->s->slot->io;
  bmp = drv->s->slot->rsvd32&0xffff;
  if( !bmp )
    bmp = 0x0f; /* skip light by default */

  baud = (drv->s->slot->rsvd32>>16)&0xffff;
  if( !baud )
    baud = 100000; /* 100K by default */
  else
    baud *= 1000;
  
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("mts4 io", t);
  }

  t = (t&0xf) + POS_IO_I2C0; /* keep io for i2c only */
  
  io = drv->os->io->init(t, baud, POS_IO_CTRL_I2C_MASTER); /* MASTER */

  /* refresh to normal by default */
  drv->s->ctrl |= MA_SENSOR_CTRL_STAT_NORMAL;
  drv->s->ctrl &= ~MA_SENSOR_CTRL_STAT_ERROR;

  /* Read temperature */  
  id = 0;
  last = 0;
  sensor_mts4_slot_id_get(&id, &last);
  
  drv->os->call(POS_CALL_FUNC_ID_SET_I2C_ADDR, io, 0x0882); /* ADDR8=0x82 (0x41*2), REGSIZE=0x08 (8Bits) */    
  // t16 = sensor_i2c_detect(io);
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {  
    // drv->log->data("mts4 detect", t16);
    drv->log->data("id_last", (last<<8)+id);
  }

  drv->history->temp_num = id + 1;
  drv->history->temp[id] = -9990; /* init -999.0 for default error */

  for( pos_i32_t loop = 0; loop < 3; loop++ ) {
    /* Start convert */
    buf[0] = 0x17; /* EE Cmd */
    buf[1] = 0xb8; /* revert false */
    io->write(io, buf, 2, 1000);
    drv->os->tick->sleep(3); /* delay 3ms */

    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      io->read(io, &buf[0], 1, POS_IO_WAIT_I2C_READ8(3, 1000)); /* status */
      io->read(io, &buf[1], 1, POS_IO_WAIT_I2C_READ8(5, 1000)); /* Temp_Cfg */
      drv->log->buf("st", buf, 2);
    }

    /* Start convert */
    buf[0] = 0x04; /* Temp Cmd */
    buf[1] = 0xc0; /* single convert */
    io->write(io, buf, 2, 1000);
    drv->os->tick->sleep(18); /* delay 17ms */
    i = sensor_mts4_read(io, 0, buf, 3); /* read temp_l/m/crc */
    t = sensor_mts4_crc8(buf, 2);
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      drv->log->buf("mts4 read", buf, i);
//      drv->log->data("p8", drv->os->crc(POS_CRC8, buf, 2));
      drv->log->data("crc8", t);
    }
    if( i != 3 )
      continue;

    if( buf[2] != t ) {
/*          drv->log->buf("rd err1", buf, 6); */
      continue; /* skip CRC8 error for TEMP */
    }

    /* calc temp */
    t = buf[1] * 256 + buf[0];
    if( buf[1] & 0x80 )
      t |= 0xffff0000;
    t = t * 100 / 256 + 2500;

    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {  
      drv->log->data("mts4 temp", t);
    }

    /* re-adjust when t <= -40.0 */
    if( t <= -4000 ) {
      pos_i32_t ta, tb;
      i = sensor_mts4_read(io, 0x0c, buf, 3); /* read coefA/coefB_m/coefB_l */
      if( i != 3 )
        continue;
      ta = buf[0];
      if( buf[0] & 0x80 )
        ta |= 0xffffff80;
      tb = buf[1]*256 + buf[2];
      if( buf[1] & 0x80 )
        tb |= 0xffff8000;

      /* adjust t by T=T*(ta+1.0)+tb */
      t = (t * (ta+10000) + tb) / 10000;
      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
        drv->log->buf("mts4 adjb", buf, i);   
        drv->log->data("mts4 adjv", t);
      }
    }

    t16 = (t+5)/10 + drv->s->slot->thld.i16[0]; /* thw[0] for calibration */
    
    /* Update history data */
    drv->history->temp[id] = t16;
    ret = POS_STATUS_OK;

    break; /* quit loop/retry */
  }

  /* Report when last entry */
  if( last ) {
    pos_u16_t ii;
    mts4_param_t *p = (mts4_param_t*)drv->s->drv_buf;
    /* store and accumulate when single mts4 */
    pos_u8_t num = drv->history->temp_num;
    pos_u8_t a_max = drv->s->slot->arg.u8[0];
    pos_i16_t *a = drv->history->temp;
    if( p && a_max > 1 ) {
      if( drv->ext->flag & MA_EXT_FLAG_MODULE_RX_OK ) {
        p->temp_num = 0; /* clear when ack received */
      }
      if( a_max > DRV_MTS4_HISTORY_MAX )
        a_max = DRV_MTS4_HISTORY_MAX;
      /* put/store all */
      for( ii = 0; ii <= id; ii++ ) {
        if( p->temp_num < a_max )
          p->temp[p->temp_num++] = drv->history->temp[ii];
        else
          sensor_mts4_push(p->temp, drv->history->temp[ii], a_max);   
      }
      num = p->temp_num;
      if( num != 1 )
        a = p->temp; /* replace a to temp accumulated */
    }
    if( num == 1 ) {
      ret = drv->data->plss_json_puti(drv->history->temp[id],
        DRV_MA_PLSS_TRANS(PLSS_PL_TEMPERATURE, 2, 0, 0, 1),
        "etmp");
    } else {
      ii = (1 << (id+1)) - 1; /* get bitmap */
      drv->data->plss_json_put(ii,
        DRV_MA_PLSS_TRANS(PLSS_PL_POS_BMP8, 1, 0, 0, 0),
        "pos");
      ret = drv->data->put_raw("\"etmp\":[", 8);
      for( i = 0; i < num; i++ ) {
        drv->data->put_float(a[i], 10);
        drv->data->plss_put_u16(PLSS_PL_EXT_TEMPERATURE, (pos_u16_t)a[i]); /* 14 - PLSS_PL_EXT_TEMPERATURE, 2Bytes, 0.1degree */
      }
      drv->data->put_array_stop();
    }
    if( ret != POS_STATUS_OK ) {
      drv->s->ctrl |= MA_SENSOR_CTRL_TOO_LONG; /* indicate data buffer put failed */      
    }
  }
  
  /* release */
  if( io )
    io->release(io);

  /* update to error status */
  if( ret != POS_STATUS_OK )
    drv->s->ctrl |= MA_SENSOR_CTRL_STAT_ERROR;
  
  return ret;
  
}


/**
 * Driver init
 * @param[in]  load   0:Unload driver, 1:Load driver
 * @return     0: Successful\n
             !=0: Failed, refer to @ref pos_status_t
 * @note This function can be NULL if it does NOT require any init operatoin
 */
pos_status_t sensor_mts4_init(pos_u32_t load) {
  drv_api_t *drv = g_drv;
  mts4_param_t *param;
  ma_sensor_ctrl_t *s = g_drv->s;
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("mts4 init", load);
  }

  if( load ) {
    param = drv->os->malloc(sizeof(*param));
    if( !param )
      return POS_STATUS_E_MEM;
    drv->os->memset(param, 0, sizeof(*param));

    drv->os->memcpy(&param->drv, drv, sizeof(*drv)); /* clone a g_drv */
    s->drv_buf = param;  /* record to drv_buf */

  } else {
    if( s->drv_buf ) {
      drv->os->free(drv->s->drv_buf);
      s->drv_buf = POS_NULL;
    }
  }
  return POS_STATUS_OK;
}

/*
 * Driver export
 */
ma_drv_export_t g_export = {
  .version = DRV_SENSOR_VERSION(MA_SENSOR_MTS4, DRV_MTS4_VERSION),
  .name = DRV_MTS4_NAME,
  .u.sensor={
    .init = sensor_mts4_init,
    .power_set = sensor_mts4_set, 
    .collect = sensor_mts4_collect,
  },
};

