/**
 * @file  drv_soil2.c
 * @brief SENSOR soil2 driver for Temp/Hum/EC/N/P/K/PH
 * @author Runby F.
 * @date 2022-10-30
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * Revision history
v1.0 @ 2022-10-30
  1) First draft

v1.1 @ 2023-11-18
  1) Using new PWR ID solution  
  
*/

/**
 * @brief Driver version
 */
#define DRV_SOIL2_VERSION 0x0101

/**
 * @brief Driver name
 */
#define DRV_SOIL2_NAME  "SOIL2"


/** 
* Power control
* @param[in]  on   0:Power off, 1:Power on
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
* @note If sensor does NOT require any power control, this fucntion can be NULL
*/ 
pos_status_t sensor_soil2_set(pos_u32_t on) {  
  pos_u16_t pwr;
  ma_board_t *b = g_drv->board;
  pwr = (g_drv->s->slot->io >> 8) & 0x7f;  
  if( !pwr ) {
    pwr = MB_PWR_ID_VCCN_PWR3;
  }

  b->pwr_set(pwr, on);
  
  return POS_STATUS_OK;
}

/** 
* Sensor collecting
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_soil2_collect(void){
  pos_io_handle_t *io;
  pos_status_t ret = POS_STATUS_E_TIMEOUT;
  pos_u32_t addr, v;
  pos_u16_t i;
  drv_api_t *drv = g_drv;
  
  /* 初始化串口 */
  i = drv->s->slot->io;
  addr = drv->s->slot->rsvd32;
  
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("mbus io", i);        
    drv->log->data("mbus addr", addr);            
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
    pos_u32_t d0, l;
    static const struct {
      pos_u32_t trans;
      const char json[12];
    } soil[7] = {
        {
            DRV_MA_PLSS_TRANS(PLSS_PL_EXT_TEMPERATURE, 2, 0, 0, 1), /* 14 - PLSS_PL_EXT_TEMPERATURE, 2Bytes, 0.1degree */
            "tmp",
        }, 
        {
            DRV_MA_PLSS_TRANS(PLSS_PL_HUMIDITY_HR, 2, 0, 0, 1), /* 80 - PLSS_PL_HUMIDITY_HR, 2Bytes, 0.1% */
            "hum",
        },
        {
            DRV_MA_PLSS_TRANS(PLSS_PL_EXT_EC, 2, 0, 0, 0), /* 103 - PLSS_PL_EXT_EC, 2Bytes, 1us/cm */
            "ec",
        }, 
        {
            DRV_MA_PLSS_TRANS(PLSS_PL_EXT_N, 2, 0, 0, 0), /* 104 - PLSS_PL_EXT_N, 2Bytes, 1 index */
            "n",
        },
        {
            DRV_MA_PLSS_TRANS(PLSS_PL_EXT_P, 2, 0, 0, 0), /* 105 - PLSS_PL_EXT_P, 2Bytes, 1 index */
            "p",
        },
        {
            DRV_MA_PLSS_TRANS(PLSS_PL_EXT_K, 2, 0, 0, 0), /* 106 - PLSS_PL_EXT_K, 2Bytes, 1 index */
            "k",
        },  
        {
            DRV_MA_PLSS_TRANS(PLSS_PL_EXT_PH, 1, 0, 0, 1), /* 60 - PLSS_PL_EXT_PH, 1Bytes, 0.1 index */
            "ph",
        }, 
    };

    d0 = drv->s->slot->arg.u32[0];    
    if( (d0 & 0xff) == 0 )
      d0 |= 0x07; /* report hum/temp/ec by default */    
    l = drv->data->modbus_read(io, buf, addr, 3, 0x00, 6);
    if( l < 5 + 6 * 2 ) 
      break; /* wrong read */
    for( l = 0; l < POS_ARRAY_CNT(soil); l++ ) {
      if( (d0 & (1<<l)) == 0 )
        continue;
      if( l >= 6 )
        v = (buf[3+3*2]<<8) + (buf[4+3*2]); /* ph uses #3 value */
      else
        v = (buf[3+l*2]<<8) + (buf[4+l*2]);
      if( v & 0x8000 )
        v |= 0xffff0000; /* expand from i16 to i32 for negative values */
      ret = drv->data->plss_json_put(v, soil[l].trans, soil[l].json);
      if( ret != POS_STATUS_OK ) {
        break;
      }
    }
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
  .version = DRV_SENSOR_VERSION(MA_SENSOR_SOIL2, DRV_SOIL2_VERSION),
  .name = DRV_SOIL2_NAME,
  .u.sensor={
    .power_set = sensor_soil2_set,
    .collect = sensor_soil2_collect,
  },
};

