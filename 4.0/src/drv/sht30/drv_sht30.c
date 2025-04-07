/**
 * @file  drv_sht30.c
 * @brief SENSOR SHT30 driver
 * @author Runby F.
 * @date 2022-3-09
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * Revision History
v1.0 @ 2023-02-03
  1) Support first version

v1.1 @ 2023-02-08
  1) Use internal temp/hum report instead of ext temp report

v1.2 @ 2023-02-21
  1) Add CRC8 check for temp/hum 
  2) Fix hum read result issue

v1.3 @ 2023-03-27
  1) Fix PWR/IO change not work issue

v1.4 @ 2023-03-29
  1) Support VOC

v1.5 @ 2023-04-24
  1) Change default PWR pin to PWR3 (accoding to Jason's suggestion)

v1.6 @ 2023-04-24
  1) Change VOC from ug/m3 to ppb based

v1.7 @ 2023-05-16
  1) Support light through (srsv[n] == 0x10)

v1.8 @ 2023-05-22
  1) Support I2C baud rate change through rsvd (bit16~bit31)

v1.9 @ 2023-06-09
  1) Support HP2 multiple I2C address selctiong through rsvs.bit[31] (0=0xEE, 1=0xEC)

v1.10 @ 2023-06-09
  1) Obsolte the srsv.bit[31] for I2C/HPA selecting as it's conflicting with I2C baudrate
  1) Support HPA 0xee/0xec I2C address auto detection

v1.11 @ 2023-07-11
  1) Unify SHT30/485_TEMP TEMP/HUM location

v1.12 @ 2023-07-15
  1) Support ADXL35x mems sensor

v1.13 @ 2023-07-21
  1) Support ADXL35x for incline change
  2) Support ADXL35x thld control

v1.14 @ 2023-07-22
  1) Use d0 to control adxl355 delay

v1.15 @ 2023-07-22
  1) Support generic json put for onenet support

v1.16 @ 2023-08-10
  1) Support SHT30 update multi temp/hum by I2C ID
  2) Support MULTI SHT30 report
  3) Support SHT30 THW[0]/[1] for temp/hum calibration

v1.17 @ 2023-08-14
  1) Fix last sht30 unable to report issue
  2) Support ADXL355 angle report by srsv:0x40

v1.18 @ 2023-08-15
  1) Support temp/hum=999.9/0% when sht30 not ready

v1.19 @ 2023-08-25
  1) Fix temp/hum not report issue

v1.20 @ 2023-08-25
  1) Fix "put" dbg not removing issue

v1.21 @ 2023-10-24
  1) Support stat/normal/error

v1.22 @ 2023-11-18
  1) Using new PWR ID solution

v1.23 @ 2023-11-23
  1) Fix JSON temp displaying error issue

v1.24 @ 2024-03-26
  1) Change default I2C speed to 100K to support SHT31 new board

v1.25 @ 2024-06-27
  1) Support SHT30 accumulated number set by set d0[n] xx (32 max)

v1.26 @ 2024-07-25
  1) Support SHT30 accumulated for multiple SHT30 sensors with POS indicator
  2) Fix SHT30 accumulating number all zero issue

v1.27 @ 2024-07-26
  1) Fix INFO SHT30 displaying ERROR issue when multiple SHT30 setup

v1.28 @ 2024-10-20
  1) Enhance SHT30 communication baudrate by multiple retry and reset

v1.29 @ 2025-02-19
  1) Use default error temp as -999.0

v1.30 @ 2025-03-02
  1) Report error hpa when not detected
  2) Enhance HPA read robustness

v1.31 @ 2025-04-02
  1) Enhance HPA read robustness by rst during failures
  2) Use safe sleep without i2c bus shutdown

*/

/**
 * @brief Driver version
 */
#define DRV_SHT30_VERSION 0x011f

/**
 * @brief Driver name
 */
#define DRV_SHT30_NAME  "SHT30"

/**
 * @brief MAX SHT30 history data num
 */
#define DRV_SHT30_HISTORY_MAX 32

/**
 * @brief IRQ控制结构
 */
typedef struct {
  drv_api_t     drv;      /**< 驱动克隆，指向固定SLOT的驱动指针 */
  pos_u16_t     temp_num; /**< temp num */
  pos_u16_t     hum_num; /**< temp num */
  pos_i16_t     temp[DRV_SHT30_HISTORY_MAX]; /**< temp history */
  pos_i16_t     hum[DRV_SHT30_HISTORY_MAX]; /**< humidity history */
} sht30_param_t;

/**
 * Sleep without system deep sleep so that I2C can still work
 */
void sensor_sht30_i2c_sleep(pos_u32_t ticks) {
  const pos_lib_tick_t *t;
  t = g_drv->os->tick;
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
pos_status_t sensor_sht30_set(pos_u32_t on) {  
  pos_u16_t pwr, w1;
  drv_api_t *drv = g_drv;  
  ma_board_t *b = drv->board;
  w1 = drv->s->slot->io;
  pwr = (w1 >> 8) & 0x7f;
  if( !pwr )
    pwr = MB_PWR_ID_VCCN_PWR3;
  
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data(on ? "i2c on" : "i2c off", pwr);
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
 * @brief ADXL355 Max sampling buffer cnt
 */
#define DRV_ADXL355_BUF_CNT 1024

/**
 * @brief ADXL355 Status REG
 */
#define DRV_ADXL355_REG_STATUS 0x04

/**
 * @brief ADXL355 XYZ data REG
 */
#define DRV_ADXL355_REG_XYZ 0x08

/**
 * @brief ADXL355 XYZ data REG
 */
#define DRV_ADXL355_REG_PWR_CTL 0x2d

/** 
* Sensor ADXL355 data read
*/
pos_status_t sensor_adxl355_read(drv_api_t *drv, pos_io_handle_t *io, pos_i16_t *x, pos_i16_t *y, pos_i16_t *z, pos_u32_t cnt) {
  pos_u8_t buf[16];
  pos_u32_t i, n;
  pos_i16_t *pp[3];
  pp[0] = (pos_i16_t*)x;
  pp[1] = (pos_i16_t*)y;
  pp[2] = (pos_i16_t*)z;
  for( n = 0; n < cnt; n++ ) {
    i = drv->os->tick->get() + 2000; /* 2000ms for sampling wait, 1s max for Low-Pass Filter */
    if( !i )
      i = 1; /* never be zero, at least 1 */
    do {
      buf[0] = 0;
      io->read(io, buf, 1, POS_IO_WAIT_I2C_READ8(DRV_ADXL355_REG_STATUS, 100));
      if( buf[0] & 0x1 ) {
        if( io->read(io, buf, 9, POS_IO_WAIT_I2C_READ8(DRV_ADXL355_REG_XYZ, 100)) >= 9 ) {
          i = 0; /* clear timeout flag */
/*          drv->log->buf("adxl1355 b", buf, 9); */
          break;
        }
      }
    } while( !drv->os->tick->is_timeout(drv->os->tick->get(),i) );
    if( i ) { /* timeout detected */
      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {  
        drv->log->data("adxl355 err", n);      
      }
      return POS_STATUS_E_RESOURCE; /* HW errror */
    }
    
    /* data ready */
    for( i = 0; i < 3; i++ ) {
      pos_i32_t v;
      v = (buf[i*3]<<12) + (buf[i*3+1]<<4) + ((buf[i*3+2]>>4)&0xf);
      if( v & 0x80000 )
        v |= 0xfff00000;
      v = (v * 39 + 5000) / 10000; /* translate by 3.906ug/LSB into unit mg */
      pp[i][n] = v;
#if 0      
      if( n < 10 && i == 0 && drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {  
        drv->log->data("adxl355 x0", v);      
      }
#endif      
    }
  }  
  return POS_STATUS_OK;
}

/** 
* Sensor ADXL355 power control
*/
void sensor_adxl355_pwr_sleep(pos_io_handle_t *io, pos_u8_t sleep) {
  pos_u8_t buf[8], n;
  io->read(io, buf, 1, POS_IO_WAIT_I2C_READ8(DRV_ADXL355_REG_PWR_CTL, 100));
  if( sleep )
    n = buf[0] | 1;
  else
    n = buf[0] & 0xfe;
  /* write to hw */
  if( n != buf[0] ) {
    buf[0] = DRV_ADXL355_REG_PWR_CTL;
    buf[1] = n;
    io->write(io, buf, 2, 100);
    io->read(io, buf, 1, POS_IO_WAIT_I2C_READ8(DRV_ADXL355_REG_PWR_CTL, 100));    
    if( buf[0] != n ) {
      if( g_drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
        g_drv->log->data("err pwr", buf[0]); 
      }
    }
  }
}

/** 
* Sensor ADXL355 collect
*/
pos_status_t sensor_adxl355_collect(drv_api_t *drv, pos_io_handle_t *io, pos_u8_t angle) {
  pos_status_t ret=POS_STATUS_ERROR;
  pos_i16_t *d[3], v;
  union {
    pos_i16_t acc[3];
    pos_u16_t accu[3];
  } u;
#if 0  
  pos_u16_t final_diff[3];
#endif
  pos_u32_t to, i;
  const pos_lib_util_t *util = drv->os->util;
  
  /* allocate buffer */
  d[0] = (pos_i16_t*)drv->os->malloc(DRV_ADXL355_BUF_CNT*2*3);
  if( !d[0] )
    return POS_STATUS_E_MEM;
  drv->os->memset(d[0], 0, DRV_ADXL355_BUF_CNT*2*3);
  d[1] = &d[0][DRV_ADXL355_BUF_CNT];
  d[2] = &d[1][DRV_ADXL355_BUF_CNT];
  
  do {
    drv->os->call(POS_CALL_FUNC_ID_SET_I2C_ADDR, io, 0x83a); /* ADDR8=0x3a, REGSIZE=8 (8Bits) */    
    v = sensor_i2c_detect(io);
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {  
      drv->log->data("adxl355 detect", v);
    }
    if( !v )
      break;

    /* normal working mode */
    sensor_adxl355_pwr_sleep(io, 0);
    
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {  
      pos_u8_t buf[8];
      io->read(io, buf, 8, POS_IO_WAIT_I2C_READ8(0, 100));
      drv->log->buf("hdr", buf, 8);
    }

    ret = POS_STATUS_OK;
#if 0    
    /* clear diff by default */
    final_diff[0] = 0;
    final_diff[1] = 0;
    final_diff[2] = 0;
    pir = 0;
#endif

    /* timeout setup */
    to = drv->os->tick->get() + drv->s->slot->arg.u16[0]*1000;

    /* giveup first batch of data, as it's too old */
    sensor_adxl355_read(drv, io, d[0], d[1], d[2], 1);
    
    /* scanning for setting time */
    v = 0; /* v for raise pir flag */
    do {
      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {  
        drv->log->data("rd", drv->os->tick->get());
      }      
      ret = sensor_adxl355_read(drv, io, d[0], d[1], d[2], DRV_ADXL355_BUF_CNT);
      
      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {  
        drv->log->data("status", ret);
      }
      if( ret != POS_STATUS_OK )
        break;   
#if 0              
      for( i = 0; i < 3; i++ ) {
        pos_u64_t s;
        pos_u32_t n, diff;
        v = util->sort_i16(d[i], DRV_ADXL355_BUF_CNT, 0);
        s = 0;
        for( n = 0; n < DRV_ADXL355_BUF_CNT; n++ ) {
          diff = util->abs(d[i][n] - v);
          s += diff*diff;
        }
        diff = util->sqrt(s/n);
        if( diff > 32766 )
          diff = 32766;
        if( diff > final_diff[i] )
          final_diff[i] = diff; /* always record the biggest diff in multiple runs */
        if( diff >= drv->s->slot->thld.u16[i] ) {
          drv->s->ctrl |= MA_SENSOR_CTRL_REPORT;
          pir = 1;
        }          
      }        
#endif        

      /* middle filter */
      for( i = 0; i < 3; i++ ) {
        pos_i16_t v_max, v_min;
        u.acc[i] = util->sort_i16(d[i], DRV_ADXL355_BUF_CNT, 0);
        if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {  
          drv->log->data("adxl355 accm", u.acc[i]);
          drv->log->data("adxl355 acc0", d[i][0]);          
          drv->log->data("adxl355 accN", d[i][DRV_ADXL355_BUF_CNT-1]);          
        }
        if( angle ) {
          if( u.acc[i] < -1000 )
            u.acc[i] = -1000;
          else if( u.acc[i] > 1000 )
            u.acc[i] = 1000;
          u.acc[i] *= 10; /* translate from 1000mg to 10000 (1.0000) */
          u.acc[i] = drv->os->util->asin(u.acc[i]); /* translate from 1.0000 to angle */
        }
         
        /* threshold check */
        v_min = drv->s->slot->thld.i16[i*2];
        v_max = drv->s->slot->thld.i16[i*2+1];
        if( v_max || v_min ) {
          v |= 1; /* threshold set, raise pir flag */
          if( u.acc[i] >= v_max
            || u.acc[i] <= v_min ) {
            drv->s->ctrl |= MA_SENSOR_CTRL_REPORT;
            v |= 2; /* indicate warning detected */
          }
        }
      }
      /* when v==0, no thld is defined. 
       * in this case, always report */
      if( !v ) {
        drv->s->ctrl |= MA_SENSOR_CTRL_REPORT ;
        break;
      } else {
        if( drv->s->ctrl & MA_SENSOR_CTRL_REPORT ) {
          break; /* threshold crossed, break immediately */
        }
      }
    } while(!drv->os->tick->is_timeout(drv->os->tick->get(), to));

    if( ret != POS_STATUS_OK )
      break;

    /* raise a pir warning flag when thld defined */
    if( v ) {
      drv->data->plss_json_put(v>>1, /* got pir state */
        DRV_MA_PLSS_TRANS(PLSS_PL_PIR, 1, 0, 0, 0),
        "pir");
#if 0
      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {  
        drv->log->data("adxl355 pir", v);
      }
#endif      
    }

    /* periodic age/check */
    drv->thld->periodic_age(drv->s);

    /* clear report if not set report flag */
    if( (drv->s->ctrl & MA_SENSOR_CTRL_REPORT) == 0 ) {
      drv->call(DRV_CALL_FUNC_ID_SET_DATA_CLEAR, 0);
      break;
    }
#if 0
    /* use long sleep if threshold exceeding */
    if( pir ) {
      to = drv->s->slot->arg.u16[1];
      if( to ) {
        drv->call(DRV_CALL_FUNC_ID_SET_DUTY_MS, to*1000); /* set special duty sleep ms */
      }
    }
    
    /* Report */
    drv->data->plss_json_put(pir, DRV_MA_PLSS_TRANS(PLSS_PL_PIR,1,0,0,0), "pir");
    drv->data->plss_put_u16(PLSS_PL_ACCELERATION, final_diff[0]);
    for( i = 1; i < 3; i++ ) {
      drv->data->plss_put_u8(final_diff[i]>>8, final_diff[i]&0xff);    
    }
    ret = drv->data->put("\"%s\":[%u,%u,%u],", "acc3", final_diff[0], final_diff[1], final_diff[2], 0);  
#endif    
    /* Report */
    drv->data->plss_put_u16(angle ? PLSS_PL_INCLINE: PLSS_PL_ACCELERATION, u.accu[0]);
    for( i = 1; i < 3; i++ ) {
      drv->data->plss_put_u8(u.accu[i]>>8, u.accu[i]&0xff );
    }
    if( angle ) {
      const static char fa[3][5] = {"angx", "angy", "angz"};      
      for( i = 0; i < 3; i++ ) {
        ret = drv->data->plss_json_puti(u.acc[i],
          DRV_MA_PLSS_TRANS(0,0,0,0,2), fa[i]);
        if( ret != POS_STATUS_OK )
          break;        
      }
    } else {
      const static char fm[3][5] = {"accx", "accy", "accz"};      
      for( i = 0; i < 3; i++ ) {
        ret = drv->data->plss_json_puti(u.acc[i],
          DRV_MA_PLSS_TRANS(0,0,0,0,0), fm[i]);
        if( ret != POS_STATUS_OK )
          break;        
      }    
    }
    if( ret != POS_STATUS_OK ) {    
      drv->s->ctrl |= MA_SENSOR_CTRL_TOO_LONG; /* indicate data buffer put failed */      
    }

  } while(0);

  /* free memory */
  drv->os->free(d[0]);

  /* sleep mode */
  sensor_adxl355_pwr_sleep(io, 1);
  
  return ret;
}

/** 
* Sensor slot lkup
*/
pos_status_t sensor_sht30_slot_id_get(pos_u8_t *id, pos_u8_t *last) {
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
void sensor_sht30_push(pos_i16_t *buf, pos_i16_t d, pos_u16_t max) {
  pos_u16_t i;
  for( i = 1; i < max; i++ )
    buf[i-1] = buf[i]; /* #0..#N-1 ==> #1..#N-1 #N-1 */
  buf[max-1] = d; /* store to last */
}

pos_io_handle_t *sensor_sht30_reset(drv_api_t *drv, pos_io_handle_t *io, pos_u8_t i2c_id, pos_u32_t baud) {
  drv->os->flush();
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("i2c rst", i2c_id);
  }
  io->release(io);
  sensor_sht30_set(0);
  drv->os->tick->sleep(500);

  sensor_sht30_set(1);
  drv->os->tick->sleep(500);
  io = drv->os->io->init(i2c_id, baud, POS_IO_CTRL_I2C_MASTER); /* MASTER */
  return io;
}

/** 
* Sensor collecting
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_sht30_collect(void){
  pos_status_t ret = POS_STATUS_ERROR;
  drv_api_t *drv = g_drv;
  pos_i32_t t;
  pos_u8_t bmp, robustness = 0;
  pos_i16_t t16, tmp, hpa;
  pos_u32_t baud;
  pos_u8_t buf[8], id, last, i, i2c_id;
  pos_io_handle_t *io;

  /* init I2C */
  t = drv->s->slot->io;
  bmp = drv->s->slot->rsvd32&0xffff;
  if( !bmp ) {
    bmp = 0x0f; /* skip light by default */
  } else {
    robustness = 1; /* when BMP is set, use robustness mode */
  }

  baud = (drv->s->slot->rsvd32>>16)&0xffff;
  if( !baud )
    baud = 100000; /* 100K by default */
  else
    baud *= 1000;
  
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("i2c io", t);
    drv->log->data("i2c spd", baud);
  }

  i2c_id = (t&0xf) + POS_IO_I2C0; /* keep io for i2c only */
  
  io = drv->os->io->init(i2c_id, baud, POS_IO_CTRL_I2C_MASTER); /* MASTER */

  /* refresh to normal by default */
  drv->s->ctrl |= MA_SENSOR_CTRL_STAT_NORMAL;
  drv->s->ctrl &= ~MA_SENSOR_CTRL_STAT_ERROR;

#if 0
  drv->os->call(POS_CALL_FUNC_ID_SET_I2C_ADDR, io, 0x10ee); /* ADDR8=0xee for HP203b, REGSIZE=0x10 (16Bits) */
  t16 = io->write(io, POS_NULL, 1, 1000);
  drv->log->data("hp203 detect", t16);
  drv->os->tick->sleep(1);
#endif
  /* default value for VOC calc */
  tmp = 240; /* default temp = 24.0 */
  hpa = 1013; /* default hpa = 1013 hpa */
  
  /* Read temperature/hum */  
  if( (bmp & 3) != 0 ) {
    id = 0;
    last = 0;
    sensor_sht30_slot_id_get(&id, &last);
    
    drv->os->call(POS_CALL_FUNC_ID_SET_I2C_ADDR, io, 0x1088); /* ADDR8=0x88, REGSIZE=0x10 (16Bits) */    
    t16 = sensor_i2c_detect(io);
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {  
      drv->log->data("sht30 detect", t16);
      drv->log->data("id_last", (last<<8)+id);
    }

    drv->history->temp_num = id + 1;
    drv->history->temp[id] = -9990; /* init -999.0 for default error */
    drv->history->temp[id+8] = 0; /* init 0 for default error */
    
    if( t16 ) {
      pos_u32_t loop_num, timeout;
      loop_num = drv->s->slot->arg.u16[1];
      if( !loop_num )
        loop_num = 512;
      timeout = drv->os->tick->get() + 60000;
      for( pos_u32_t loop = 0; loop < loop_num; loop++ ) {
        /* retry until timeout */
        if( drv->os->tick->is_timeout(drv->os->tick->get(), timeout) )
          break;
        /* pwr reset after each 16 retry */
        if( loop && (loop &0xf) == 0 ) {
          io = sensor_sht30_reset(drv, io, i2c_id, baud);
        }
        if( io->read(io, buf, 6, POS_IO_WAIT_I2C_READ16(0x2c06, 500)) != 6 ) {
/*          drv->log->data("rd err", 0);  */
          continue;
        }

        if( buf[2] != drv->os->crc(POS_CRC8, buf, 2) ) {
/*          drv->log->buf("rd err1", buf, 6); */
          continue; /* skip CRC8 error for TEMP */
        }

        if( buf[5] != drv->os->crc(POS_CRC8, &buf[3], 2) ) {
/*          drv->log->buf("rd err2", buf, 6); */
          continue; /* skip CRC8 error for HUM */
        }

        if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {  
          drv->log->data("rd loop", loop);
          drv->log->buf("sht30 read", buf, 6);
        }

#if 0
        if( loop )
          drv->log->data("rd loop", loop);
#endif

        if( bmp & 1 ) {
          /* calc temp */
          t = buf[0] * 256 + buf[1];
          t = t * 17500 / 65535 - 4500;

          t16 = t/10 + drv->s->slot->thld.i16[0]; /* thw[0] for calibration */
          tmp =  t16; /* record tmp for VOC */
          
          /* Update history data */
          drv->history->temp[id] = t16;
          ret = POS_STATUS_OK;
        }

        if( bmp & 2 ) {
          /* calc humidity */
          t = buf[3] * 256 + buf[4];
          t = t * 10000 / 65535 + drv->s->slot->thld.i16[1]; /* thw[1] for calibration */
            
          /* Update history data */
          drv->history->temp[id+8] = t/10; /* use 0.1 unit */
          ret = POS_STATUS_OK;          
        }

        break; /* quit loop/retry */
      }
      
    }

    /* Report when last entry */
    if( last ) {
      pos_u16_t ii;
      sht30_param_t *p = (sht30_param_t*)drv->s->drv_buf;
      if( bmp & 0x1 ) {
        /* store and accumulate when single sht30 */
        pos_u8_t num = drv->history->temp_num;
        pos_u8_t a_max = drv->s->slot->arg.u8[0];
        pos_i16_t *a = drv->history->temp;
        if( p && a_max > 1 ) {
          if( drv->ext->flag & MA_EXT_FLAG_MODULE_RX_OK ) {
            p->temp_num = 0; /* clear when ack received */
          }
          if( a_max > DRV_SHT30_HISTORY_MAX )
            a_max = DRV_SHT30_HISTORY_MAX;
          /* put/store all */
          for( ii = 0; ii <= id; ii++ ) {
            if( p->temp_num < a_max )
              p->temp[p->temp_num++] = drv->history->temp[ii];
            else
              sensor_sht30_push(p->temp, drv->history->temp[ii], a_max);   
          }
          num = p->temp_num;
          if( num != 1 )
            a = p->temp; /* replace a to temp accumulated */
        }
        if( num == 1 ) {
          ret = drv->data->plss_json_puti(drv->history->temp[id],
            DRV_MA_PLSS_TRANS(PLSS_PL_TEMPERATURE, 2, 0, 0, 1),
            "tmp");
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

      if( bmp & 0x2 ) {
        /* store and accumulate when single sht30 */
        pos_u8_t num = drv->history->temp_num;
        pos_u8_t a_max = drv->s->slot->arg.u8[0];
        pos_i16_t *a = &drv->history->temp[8];
        if( p && a_max > 1 ) {
          if( drv->ext->flag & MA_EXT_FLAG_MODULE_RX_OK ) {
            p->hum_num = 0; /* clear when ack received */
          }
          if( a_max > DRV_SHT30_HISTORY_MAX )
            a_max = DRV_SHT30_HISTORY_MAX;
          /* put/store all */
          for( ii = 0; ii <= id; ii++ ) {
            if( p->hum_num < a_max )
              p->hum[p->hum_num++] = drv->history->temp[ii+8];
            else
              sensor_sht30_push(p->hum, drv->history->temp[ii+8], a_max);
          }
          num = p->hum_num;
          if( num != 1 )
            a = p->hum; /* replace a to hum accumulated */
        }

        if( num == 1 ) {
          ret = drv->data->plss_json_put((pos_u16_t)drv->history->temp[id+8],
            DRV_MA_PLSS_TRANS(PLSS_PL_HUMIDITY, 1, 1, 0, 1),
            "hum");
        } else {
          drv->data->put_raw("\"ehum\":[", 8);
          for( i = 0; i < num; i++ ) {
            drv->data->put_float(a[i], 10);
            drv->data->plss_put_u16(PLSS_PL_HUMIDITY_HR, (pos_u16_t)a[i]); /* 80 - PLSS_PL_HUMIDITY_HR, 2Bytes, 0.1% */
          }
          drv->data->put_array_stop();
        }
        if( ret != POS_STATUS_OK ) {
          drv->s->ctrl |= MA_SENSOR_CTRL_TOO_LONG; /* indicate data buffer put failed */
        }
      }
    }
  }

  /* Read hp */  
  if( (bmp & 4) != 0 ) {
    pos_u16_t addr, ii, retry, loop;
    t16 = 0;
    /* default HPA to 0 indicating error */
    drv->history->mv[5] = 0;
    drv->history->mv[6] = 0;
    retry = robustness ? 5 : 1;
    for( loop = 0; loop < retry; loop++ ) {
      if( loop ) {
        /* reset i2c when failed */
        io = sensor_sht30_reset(drv, io, i2c_id, baud);
      }
      for( addr = 0x8ee; addr >= 0x8ec; addr -= 2) {
        buf[0] = 0x40; /* detect&convert */
        drv->os->call(POS_CALL_FUNC_ID_SET_I2C_ADDR, io, addr); /* ADDR8=0xee/0xec, REGSIZE=8 (8Bits) */

        for( ii = 0; ii < 3; ii++ ) {
          t16 = io->write(io, buf, 1, 500);
          if( t16 )
            break;
        }
        if( t16 ) {
          if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
            drv->log->data("hp203 addr", addr);
          }
          break;
        }
      }
      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
        drv->log->data("hp203 detect", t16);
      }

      if( t16 ) {
#if 0      
        /* reset delay */
        sensor_sht30_i2c_sleep(150);

        /* start 4096 OSR converting */
        buf[0] = 0x40;
        t16 = io->write(io, buf, 1, 2000);
        if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
          drv->log->data("hp203 conv", t16);
        }
#endif
        /* wait 150ms for result done */
        sensor_sht30_i2c_sleep(150);

        /* read result */
        t16 = io->read(io, buf, 6, POS_IO_WAIT_I2C_READ8(0x10, 3000));
        if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
          drv->log->buf("hp203 read", buf, t16);
        }
      }

      /* read result */
      if( t16 == 6 )  {
        pos_u32_t u;
        /* calc hp */
        u = buf[3] * 65536 + buf[4] * 256 + buf[5];

        /* Update history data */
        drv->history->mv[5] = (u) & 0xffff;        
        drv->history->mv[6] = (u >> 16) & 0xffff;
        hpa =  u/100; /* record hpa for VOC */     
        
        /* Report */
        ret = drv->data->plss_json_put(u, 
          DRV_MA_PLSS_TRANS(PLSS_PL_PRESSURE, 4, 0, 0, 2),
          "hpa");         
        if( ret != POS_STATUS_OK ) {    
          drv->s->ctrl |= MA_SENSOR_CTRL_TOO_LONG; /* indicate data buffer put failed */      
        }
        break;
      }
    }
  }
  /* Read VOC */  
  if( (bmp & 8) != 0 ) {
    drv->os->call(POS_CALL_FUNC_ID_SET_I2C_ADDR, io, 0x854); /* ADDR8=0xee, REGSIZE=8 (8Bits) */    
    t16 = sensor_i2c_detect(io);
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {  
      drv->log->data("voc detect", t16);
    }

    if( t16 ) {        
      /* wait 1ms for result done */
      sensor_sht30_i2c_sleep(150);

      /* read result */
      t16 = io->read(io, buf, 1, POS_IO_WAIT_I2C_READ8(0xaa, 3000));
      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {  
        drv->log->buf("voc read", buf, t16);
        drv->log->data("voc tmp", tmp);        
        drv->log->data("voc hpa", hpa);        
      }      

      /* read result */
      if( t16 > 0 )  {
        pos_u32_t u;
        /* calc voc */
        u = buf[0];
        u = u * 5 * 1000 / 200; /* unit of ppb */
#if 0                
        u = u * 460 / 224; /* unit of ug/m3 (alcohol=46)*/
        u = u * 2730 / (2730+tmp); /* temperature re-calc */
        u = u * hpa / 1013; /* hpa re-calc */
        
        /* Report */
        drv->data->plss_put_u16(59, u);
        ret = drv->data->put("\"%s\":%u,", "voc", u, 0, 0, 0);  
#else
        ret = drv->data->plss_json_put(u, 
          DRV_MA_PLSS_TRANS(PLSS_PL40_VOC_PPB, 2, 0, 0, 0), /* 107 - PLSS_PL_VOC_PPB, 2Bytes, 1 PPB */ 
          "vocb");
#endif
        if( ret != POS_STATUS_OK ) {    
          drv->s->ctrl |= MA_SENSOR_CTRL_TOO_LONG; /* indicate data buffer put failed */      
        }
      }
    }
  } 
  /* Read Light */  
  if( (bmp & 0x10) != 0 ) {
#if 0    
    sensor_sht30_set(0);
    drv->os->tick->sleep(10);
    sensor_sht30_set(1);
#endif
    drv->os->call(POS_CALL_FUNC_ID_SET_I2C_ADDR, io, 0x846); /* ADDR8=0x46, REGSIZE=8 (8Bits) */    
    t16 = sensor_i2c_detect(io);

    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {  
      drv->log->data("light detect", t16);
    }
    if( t16 ) {        
      /* power down */
      buf[0] = 0;
      io->write(io, buf, 1, 100); /* use write pwr down as detect command */

      /* power on */
      buf[0] = 1;
      io->write(io, buf, 1, 100);      
#if 0      
      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {        
        drv->log->data("light on", t16); 
      }
#endif      
      
      /* read result */
      io->read(io, buf, 2, POS_IO_WAIT_I2C_READ8(0x10, 3000)); /* trigger continuously H command */
      sensor_sht30_i2c_sleep(140);      /* 120ms is enough, 140ms is safe */
      t16 = io->read(io, buf, 2, POS_IO_WAIT_I2C_READ8(0x10, 3000));
      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {  
        drv->log->buf("light read", buf, t16);       
      }      

      /* read result */
      if( t16 > 1 )  {
        pos_u32_t u;
        /* calc voc */
        u = buf[0] * 256 + buf[1];
        u = (u * 100 + 60) / 120; /* u/1.2 = unit of lx */
#if 0                
        u = u * 460 / 224; /* unit of ug/m3 (alcohol=46)*/
        u = u * 2730 / (2730+tmp); /* temperature re-calc */
        u = u * hpa / 1013; /* hpa re-calc */
        
        /* Report */
        drv->data->plss_put_u16(59, u);
        ret = drv->data->put("\"%s\":%u,", "voc", u, 0, 0, 0);  
#else
        ret = drv->data->plss_json_put(u, 
          DRV_MA_PLSS_TRANS(6, 2, 0, 0, 0), /* 6 - PLSS_PL_LIGHT, 2Bytes, 1 lux */ 
          "light");
#endif
        if( ret != POS_STATUS_OK ) {    
          drv->s->ctrl |= MA_SENSOR_CTRL_TOO_LONG; /* indicate data buffer put failed */      
        }
      }
    }
  } 

  /* ADXL35x MEMS */  
  if( (bmp & 0x20) != 0 ) {
    ret = sensor_adxl355_collect(drv, io, 0); /* mg */
  } 
  if( (bmp & 0x40) != 0 ) {
    ret = sensor_adxl355_collect(drv, io, 1); /* angle */
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
pos_status_t sensor_sht30_init(pos_u32_t load) {
  drv_api_t *drv = g_drv;
  sht30_param_t *param;
  ma_sensor_ctrl_t *s = g_drv->s;
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("sht30 init", load);
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
  .version = DRV_SENSOR_VERSION(MA_SENSOR_SHT30, DRV_SHT30_VERSION),
  .name = DRV_SHT30_NAME,
  .u.sensor={
    .init = sensor_sht30_init,
    .power_set = sensor_sht30_set, 
    .collect = sensor_sht30_collect,
  },
};

