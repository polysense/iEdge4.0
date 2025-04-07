/**
 * @file  drv_fsxcs.c
 * @brief SENSOR RS-FSXCS driver
 * @author Runby F.
 * @date 2024-2-26
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * Revision history
v1.0 @ 2024-2-26
  1) First revision

v1.1 @ 2024-2-27
  1) Fix wind speed error

v1.2 @ 2025-1-22
  1) Support new FSXCS sensors with 9600 default baudrate

v1.3 @ 2025-1-23
  1) Fix WIND speed unit wrong issue (vendor has updated its unit from cm to 0.1m)

v1.4 @ 2025-3-25
  1) Support new WIND SPEED+DIR only sensors (fxscs_w2 with new 0 based reg)

v1.5 @ 2025-4-02
  1) Change wind speed unit from 0.1m/s to 0.01m/s
  2) Fix wind dir mistake when bmp=3

*/

/**
 * @brief Driver version
 */
#define DRV_FSXCS_VERSION 0x0105

/**
 * @brief Driver name
 */
#define DRV_FSXCS_NAME  "FSXCS"


/** 
* Power control
* @param[in]  on   0:Power off, 1:Power on
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
* @note If sensor does NOT require any power control, this fucntion can be NULL
*/ 
pos_status_t sensor_fsxcs_set(pos_u32_t on) {  
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
pos_status_t sensor_fsxcs_collect(void){
  pos_io_handle_t *io;
  pos_status_t ret = POS_STATUS_E_TIMEOUT;
  pos_u32_t addr, v, bmp, r, reg_base;
  pos_u16_t i, loop;
  drv_api_t *drv = g_drv;
  pos_u8_t buf[48];
  
  /* 初始化串口 */
  i = drv->s->slot->io;
  addr = drv->s->slot->rsvd32;
  bmp = drv->s->slot->arg.u16[0];
  if( !bmp )
    bmp = 0x3ff;
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

  if( bmp <= 3 )
    reg_base = 0; /* 0 based using drv_fsxcs_w2.pdf */
  else
    reg_base = 0x1f4; /* 500 based using drv_fsxcs.pdf */
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("fsxcs io", i);        
    drv->log->data("fsxcs addr", addr);            
  }  

  do {
    if( bmp & 0xffe0 )
      r = 16;
    else if( bmp <= 3 )
      r = 2;
    else
      r = 7;
    for( loop = 0; loop < 5; loop++) {
      v = drv->data->modbus_read(io, buf, addr, 3, reg_base, r); /* read */
      if( v == r*2+5 ) {
        if( (bmp & 4) == 0 || buf[11]*256+buf[12] != 0 ) /* if no hum is required or hum is valid data, quit loop */
          break;
      }
      drv->os->tick->sleep(2000); /* wait 2s if data is invalid */
    }
    if( v != r*2+5 ) {
      ret = POS_STATUS_E_RESOURCE;
      break;
    }
    /* wind speed */
    if( bmp & 1 ) {
      v = buf[3]*256+buf[4];
      if( v > 655 )
        v = 655;
      v = v * 10; /* from 0.01m/s to 1mm/s ??? when reg=0x1f4, it's unsure if its unit is 0.1m/s */
      ret = drv->data->plss_json_put(v,
            DRV_MA_PLSS_TRANS(PLSS_PL_WIND_SPEED, 2, 0, 0, 3),
            "wind"
      );
    }

    /* wind level: skip */
    
    /* wind dir 0~7: skip */
    
    /* wind dir 0~360 degree */
    if( bmp & 2 ) {
      if( r == 2 )
        v = buf[5]*256+buf[6];
      else
        v = buf[9]*256+buf[10];
      if( v > 359 )
        v = 359;
      ret = drv->data->plss_json_put(v,
        DRV_MA_PLSS_TRANS(PLSS_PL_EXT_CNTR, 4, 0, 0, 0),
        "dir" );
    }
    
    /* humidity */
    if( bmp & 4 ) {
      v = buf[11]*256+buf[12];
      ret = drv->data->plss_json_put(v,
        DRV_MA_PLSS_TRANS(PLSS_PL_HUMIDITY_HR, 2, 0, 0, 1),
        "hum" );
    }
    
    /* temperature */
    if( bmp & 8 ) {
      v = buf[13]*256+buf[14];
      if( v & 0x8000 )
        v |= 0xffff0000;
      ret = drv->data->plss_json_puti((pos_i32_t)v,
        DRV_MA_PLSS_TRANS(PLSS_PL_TEMPERATURE, 2, 0, 0, 1),
        "tmp" );    
    }
    
    /* sound */
    if( bmp & 0x10 ) {
      v = buf[15]*256+buf[16];
      ret = drv->data->plss_json_put(v,
        DRV_MA_PLSS_TRANS(PLSS_PL_SOUND, 2, 0, 0, 1),
        "sound" );  
    }

    /* pm2.5/pm10 */
    if( bmp & 0x20 ) {
      /* buf[17..20] */
      buf[16] = PLSS_PL_EXT_PM; /* PLSS data format: 43 for PLSS_PL_EXT_PM: U16[3] for 2.5/10/1.0 */        
      drv->data->plss_put_raw(5, &buf[16]);
      buf[0] = 0; buf[1] = 0;
      drv->data->plss_put_raw(2, &buf[0]); /* use 0/0 data for pm1.0 (Sensor only supports PM2.5/10 */
      ret = drv->data->put("\"%s\":%u,", "pm2_5", buf[17]*256+buf[18], 0, 0, 0);
      if( ret == POS_STATUS_OK ) {
        ret = drv->data->put("\"%s\":%u,", "pm10", buf[19]*256+buf[20], 0, 0, 0);
      }
    }

    /* hpa */
    if( bmp & 0x40 ) {
      v = buf[21]*256+buf[22];
      v *= 100;
      ret = drv->data->plss_json_put(v,
        DRV_MA_PLSS_TRANS(PLSS_PL_PRESSURE, 4, 0, 0, 2),
        "hpa" );  
    } 

    /* light */
    if( bmp & 0x80 ) {
      v = (buf[23]<<24)+(buf[24]<<16)+(buf[25]<<8)+buf[26];
      ret = drv->data->plss_json_put(v,
        DRV_MA_PLSS_TRANS(PLSS_PL_LIGHT, 2, 0, 0, 0),
        "light" );
    }

    /* rainfall */
    if( bmp & 0x100 ) {
      v = buf[29]*256+buf[30];
      ret = drv->data->plss_json_put(v,
        DRV_MA_PLSS_TRANS(PLSS_PL_RAINFALL, 2, 0, 0, 1),
        "rain" );
    }

    /* ghi radiation */
    if( bmp & 0x200 ) {
      v = buf[33]*256+buf[34];
      ret = drv->data->plss_json_put(v,
        DRV_MA_PLSS_TRANS(PLSS_PL_RADIATION, 2, 0, 0, 0),
        "ghi" );
    }

  } while(0);
  
  if( ret != POS_STATUS_OK ) {   
    /* set error flag */
    drv->s->ctrl |= MA_SENSOR_CTRL_STAT_ERROR;
    /* 错误的Sensor, 上报出错 */
    drv->data->put("\"%s\":\"no fsxcs#0x%x/0x%x\",", "err", i, addr, 0, 0);
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
  .version = DRV_SENSOR_VERSION(MA_SENSOR_FSXCS, DRV_FSXCS_VERSION),
  .name = DRV_FSXCS_NAME,
  .u.sensor={
    .power_set = sensor_fsxcs_set,
    .collect = sensor_fsxcs_collect,
  },
};

