/**
 * @file  drv_tb600.c
 * @brief SENSOR ZE03 O2 driver
 * @author Runby F.
 * @date 2023-01-28
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * 版本修订历史
v1.0 @ 2023-11-23
  1) Support First revision

v1.1 @ 2023-11-23
  2) Support gas + temp + hum

v1.2 @ 2023-11-23
  1) Correct version labebl

v1.3 @ 2023-11-28
  1) Correct module name to upper case

v1.4 @ 2024-05-19
  1) Support TB600 I2C by io 0x20~0x23
  2) Enhance sleep to none-hw sleep for better I2C opr

v1.5 @ 2024-06-22
  1) Fix TB600 I2C D1 dbg print issue
  
*/

/**
 * @brief Driver version
 */
#define DRV_TB600_VERSION 0x0105

/**
 * @brief Driver name
 */
#define DRV_TB600_NAME  "TB600"

/**
 * @brief Operating timeout
 */
#define DRV_UART_IO_TIMEOUT  500

/**
 * @brief Operating timeout retry times
 */
#define DRV_UART_RETRY_NUM  10

/**
 * @brief Operating timeout retry times
 */
#define DRV_I2C_RETRY_NUM  3


/** 
* Power control
* @param[in]  on   0:Power off, 1:Power on
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
* @note If sensor does NOT require any power control, this fucntion can be NULL
*/ 
pos_status_t sensor_tb600_set(pos_u32_t on) {  
  pos_gpio_pin_t pwr;
  ma_board_t *b = g_drv->board;
  pwr = (g_drv->s->slot->io >> 8) & 0x7f;  
  if( !pwr ) 
    pwr = POS_IO_IS_SERIAL_PORT(g_drv->s->slot->io) ? MB_PWR_ID_VCCN_PWR3 : MB_PWR_ID_VCC_PWRH;

  b->pwr_set(pwr, on);
  
  return POS_STATUS_OK;
}

/** 
 * Sleep without system deep sleep so that I2C can still work
 */
void sensor_tb600_sleep(pos_u32_t ticks) {
  const pos_lib_tick_t *t;  
  t = g_drv->os->tick;
  ticks = t->get() + ticks;
  while( !t->is_timeout(t->get(), ticks) ) {
    t->sleep(20);
  }
}

/** 
* Fifo flush
*/ 
void sensor_tb600_flush(pos_io_handle_t *io) {
  /* 清空串口并等待50ms */
  while( io->read(io, POS_NULL, 1024, 50) >= 1024 ); 
}

/** 
* Send command and read response
*/ 
pos_u32_t sensor_tb600_cmd_send(pos_io_handle_t *io, pos_u8_t *buf, pos_u32_t buf_len, pos_u8_t *response, pos_u32_t response_len) {
  pos_u32_t ret = 0;
  drv_api_t *drv = g_drv;

  sensor_tb600_flush(io);

  io->write(io, buf, buf_len, DRV_UART_IO_TIMEOUT);    
    
    /* 每次等待specified响应time */
  ret = io->read(io, response, response_len, DRV_UART_IO_TIMEOUT);

  sensor_tb600_flush(io);

  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->buf("tb600 wr", buf, buf_len);    
    drv->log->buf("tb600 rd", response, response_len);
    drv->os->flush();
  }

  //drv->log->buf("RX", response, v); 
  if( ret != response_len || response[0] != 0xff )
    ret = 0; /* indicate error */

  /* crc*/
  if( ret ) { 
    pos_u32_t i;
    pos_u8_t c;
    c = response[0];
    for( i = 1; i < ret; i++ )
      c += response[i];      
    if( c != 0xff ) /* crc check */
      ret = 0; /* CRC错误 */
  }
  return ret;
}

pos_status_t sensor_tb600_add8_chk(pos_u8_t *data, pos_u8_t len) {
  pos_u8_t c;
  c = 0;
  while( len-- > 1 )
    c += *data++;
  if( c == *data )
    return POS_STATUS_OK;
  return POS_STATUS_ERROR;
}

/** 
* Sensor Detecting
*/
pos_i16_t sensor_tb600_i2c_detect(pos_io_handle_t *io) {
  pos_i16_t t, wr;
  for( t = 0 ; t < 3; t++ ) {
    wr = io->write(io, POS_NULL, 1, 100);
    if( wr > 0 )
      return wr;
  }
  return 0;
}

/** 
* Sensor collecting
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_tb600_collect(void){
  pos_io_handle_t *io;
  pos_status_t ret = POS_STATUS_E_TIMEOUT;
  pos_u32_t v, sum, cnt;
  pos_u8_t tb600_type, tb600_precision, ctrl_id = 255, bmp;
  pos_u16_t t, loop;
  drv_api_t *drv = g_drv;
  pos_u8_t response[16];
  static const struct {
    const char json[6];
    pos_u8_t sensor_pl;
    pos_u8_t plss_pl;
  } ctrl[] = {
    {"co",  0x19, PLSS_PL_EXT_CO,},   /* 76 - PLSS_PL_EXT_CO, 2Bytes, 1 PPM */       
    {"cl2", 0x1a, PLSS_PL_EXT_CL2, }, /* 24 - PLSS_PL_EXT_CL2, 2Bytes, 1 PPM */      
    {"h2",  0x1b, PLSS_PL_EXT_H2, },  /* 25 - PLSS_PL_EXT_H2, 2Bytes, 1 PPM */                            
    {"h2s", 0x1c, PLSS_PL_EXT_H2S, }, /* 26 - PLSS_PL_EXT_H2S, 2Bytes, 1 PPM */         
    {"hcl", 0x1d, PLSS_PL_EXT_HCL, }, /* 27 - PLSS_PL_EXT_HCL, 2Bytes, 1 PPM */                            
    {"hcn", 0x1e, PLSS_PL_EXT_HCN, }, /* 28 - PLSS_PL_EXT_HCN, 2Bytes, 1 PPM */
    {"hf",  0x1f, PLSS_PL_EXT_HF, },  /* 29 - PLSS_PL_EXT_HF, 2Bytes, 1 PPM */
    {"nh3", 0x20, PLSS_PL_EXT_NH3, }, /* 21 - PLSS_PL_EXT_NH3, 2Bytes, 1 PPM */
    {"no2", 0x21|0x80, PLSS_PL_EXT_NO2, }, /* 30 - PLSS_PL_EXT_NO2, 2Bytes, 1 ug/m3 */
    {"o2",  0x23, PLSS_PL_EXT_O2,  }, /* 61 - PLSS_PL_EXT_O2, 2Bytes, 0.1% */  
    {"o3",  0x23, PLSS_PL_EXT_O3,  }, /* 31 - PLSS_PL_EXT_O3, 2Bytes, 1 ug/m3 */
    {"so2", 0x24, PLSS_PL_EXT_SO2, }, /* 33 - PLSS_PL_EXT_SO2, 2Bytes, 1 ug/m3 */          
    
  };  
  const pos_u8_t cmd_query_only[9] = {0xff, 0x01, 0x78, 0x41, 0x00, 0x00, 0x00, 0x00, 0x46};
  const pos_u8_t cmd_read[9] = {0xff, 0x01, 0x87, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78};
  const pos_u8_t cmd_type[1] = {0xd7};

  bmp = drv->s->slot->arg.u8[0];
  if( !bmp )
    bmp = 0xff; /* report all by default */
  
  /* 初始化串口 */
  t = drv->s->slot->io;

  /* refresh to normal by default */
  drv->s->ctrl |= MA_SENSOR_CTRL_STAT_NORMAL;
  drv->s->ctrl &= ~MA_SENSOR_CTRL_STAT_ERROR;

  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("tb600 io", t);        
  }

  t &= 0xff; /* keep io for uart only */
  
  io = POS_NULL;
  if( POS_IO_IS_SERIAL_PORT(t) ) {
    v = (drv->s->slot->rsvd32 >> 8)&0xffffff;
    if( !v )
      v = 9600; /* 默认用9600 */
    io = drv->os->io->init(t, v, 0);
    if( io ) {      
      /* 清空串口并等待50ms */
      while( io->read(io, POS_NULL, 1024, 50) >= 1024 );
    }
  } else {
    v = (drv->s->slot->rsvd32 >> 8)&0xffffff;
    if( !v )
      v = 8192; /* 默认用8KHz */  
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      drv->log->data("tb600 bd", v);
    }    
    io = drv->os->io->init((t&0xf) + POS_IO_I2C0, v, POS_IO_CTRL_I2C_MASTER); /* MASTER */
  }
  if( !io ) {  
    loop = DRV_UART_RETRY_NUM + DRV_I2C_RETRY_NUM; /* end of loop */
  } else
    loop = 0;

  /* 重试N次 */
  sum = 0;
  cnt = 0;
  tb600_type = 0;
  tb600_precision = 0;

  if( POS_IO_IS_SERIAL_PORT(t) ) {
    /* UART Reading */
    for(; loop < DRV_UART_RETRY_NUM; loop++ ) {

      /* sleep for not first loop */
      if( loop )
        sensor_tb600_sleep(1000);       

      /* pull type */
      if( !tb600_type ) {
        /* stop report mode using polling mode */
        sensor_tb600_cmd_send(io, (pos_u8_t*)cmd_query_only, 9, response, 9);
        
        v = sensor_tb600_cmd_send(io, (pos_u8_t*)cmd_type, 1, response, 9);
        /* 
        Byte[0]
        0xFF
        Byte[1]
        0xD7
        Byte[2]
        Sensor_type
        Byte[3] Byte[4]
        Range 
        Byte[5] 
        Unit_type 
        Byte[6] [bit7:4]
        Decimal_places 
        Byte[6] [bit3:0]
        & Data_sign
        Byte[7] 
        Reserved
        Byte[8]
        Checksum
        */
        if( !v || response[1] != 0xd7 )
          continue;
        tb600_type = response[2];
        tb600_precision = (response[6]>>4)&0xf;
        drv->os->tick->sleep(1000); /* sleep before next cmd */
      }
      
      /* poll data */
      v = sensor_tb600_cmd_send(io, (pos_u8_t*)cmd_read, 9, response, 13);
      /* 
      Byte[0]
      0xFF
      Byte[1]
      0x86
      Byte[2] Byte[3] 
      Concentration 2
      Byte[4] Byte[5] 
      Range
      Byte[6] Byte[7] 
      Concentration 1
      Byte[8]
      Checksum
      */
      if( v != 13 || response[1] != 0x87 )
        continue;

      if( ctrl[ctrl_id].sensor_pl & 0x80 )
        /* load c2 for mg/ug/10g */
        v = (response[2]<<8) + response[3];
      else
        /* load c1 for ppm/ppb/% */      
        v = (response[6]<<8) + response[7];

      cnt++;
      sum += v;
      
      if( cnt >= 1 )
        break;

    }
  } else {
    /* I2C Reading */
    drv->os->call(POS_CALL_FUNC_ID_SET_I2C_ADDR, io, 0x08FC); /* ADDR8=0xFC(0x7e*2), REGSIZE=0x08 (8Bits) */    
    for(; loop < DRV_I2C_RETRY_NUM; loop++ ) {
      /* sleep for not first loop */
      if( loop )    
        sensor_tb600_sleep(1000);
#if 0
      v = sensor_tb600_i2c_detect(io);
      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {  
        drv->log->data("tb600 detect", v);
      }
      if( !v )
        continue;
#endif
#if 1
      v = io->read(io, response, 6, POS_IO_WAIT_I2C_READ8(0xd2, 3000));
      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
        drv->log->buf("tb600 d2", response, v);
      }
      if( v != 6 )
        continue; /* wrong length */

      sensor_tb600_sleep(1000); /* needs sleep before next read, or else read will be failed */
      
      /* cksm8 check */
      if( sensor_tb600_add8_chk(response, 6) != POS_STATUS_OK )
        continue; /* wrong cksum */
      tb600_type = response[1];
      tb600_precision = response[0];
#else
      tb600_type = 0x21;
      tb600_precision = 3;
#endif
      v = io->read(io, &response[6], 7, POS_IO_WAIT_I2C_READ8(0xd1, 3000));
      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
        drv->log->buf("tb600 d1", &response[6], v);
      }
      if( v != 7 )
        continue; /* wrong length */

      /* cksm8 check */
      if( sensor_tb600_add8_chk(&response[6], 7) != POS_STATUS_OK )
        continue; /* wrong cksum */

      v = (response[6]<<8) + response[7];
      cnt++;
      sum += v;      
      if( cnt >= 1 )
        break;      
    }
  }

  /* reset cnt flag to zero when type is NOT supported */
  if( cnt ) {
    for( ctrl_id = 0; ctrl_id < POS_ARRAY_CNT(ctrl); ctrl_id++ ) {
      if( (ctrl[ctrl_id].sensor_pl&0x7f) == tb600_type ) {
        break;
      }
    }
    if( ctrl_id >= POS_ARRAY_CNT(ctrl) ) {
      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
        drv->log->data("tb600 unk", tb600_type);        
      }          
      cnt = 0; /* not support type, break immediately */        
    }
  }
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("tb600 t_p", (tb600_type<<8)+tb600_precision);        
  }      

  /* report error when cnt flag zero */
  if( !cnt ) {
    /* set error flag */
    drv->s->ctrl |= MA_SENSOR_CTRL_STAT_ERROR;    
    /* 错误的Sensor, 上报出错 */
    drv->data->put("\"%s\":\"no tb600\",", "err", 0, 0, 0, 0);
  } else {
    /* major report */
    if( bmp & 1 ) {
      v = sum / cnt;

      drv->data->plss_json_put(v, 
        DRV_MA_PLSS_TRANS(ctrl[ctrl_id].plss_pl, 2, 0 /* plss use raw precision */, 0, tb600_precision), 
        ctrl[ctrl_id].json);    
    }

    /* temp report */
    if( bmp & 2 ) {
      v = (response[8]<<8) + response[9];
      if( v & 0x8000 )
        v |= 0xffff0000;
      drv->data->plss_json_puti(v,
        DRV_MA_PLSS_TRANS(PLSS_PL_EXT_TEMPERATURE, 2, 1, 0, 2),
        "etmp");
    }

    /* hum report */
    if( bmp & 4 ) {
      v = (response[10]<<8) + response[11];
      drv->data->plss_json_put(v,
        DRV_MA_PLSS_TRANS(PLSS_PL_HUMIDITY_HR, 2, 1, 0, 2),
        "hum");
    }
    
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
  .version = DRV_SENSOR_VERSION(MA_SENSOR_TB600, DRV_TB600_VERSION),
  .name = DRV_TB600_NAME,
  .u.sensor={
    .power_set = sensor_tb600_set,
    .collect = sensor_tb600_collect,
  },
};

