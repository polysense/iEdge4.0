/**
 * @file  drv_mix.c
 * @brief SENSOR MIX driver
 * @author Runby F.
 * @date 2022-3-09
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * Revision History
v1.0 @ 2024-06-21
  1) Support first version

v1.1 @ 2024-06-24
  1) Support CO2 in unit of 100 PPM (0.01%)
  2) Support H2S report in unit of 0.01 PPM

v1.2 @ 2024-06-25
  1) Support CO+H2S two in one DGM10 sensor

v1.3 @ 2024-07-13
  1) Support to record history data as 0xffff even when not ready
  2) Flush console during log printing
  3) Replace tb600 from log by internal sensor ids
  4) Support UART0 for Grp3 CO2

v1.4 @ 2024-07-25
  1) Fix report pl for CO2 wrong (should be CO2_PPM)
  2) Support thw0 for O2 check/sleep/repeat

v1.5 @ 2024-08-09
  1) Fix tb200 not existing read issue
  2) When TB200 not existing report h2s=0xffff
  3) When TB600 not existing report co=0xffff

*/

/**
 * @brief Driver version
 */
#define DRV_MIX_VERSION 0x0105

/**
 * @brief Driver name
 */
#define DRV_MIX_NAME  "MIX"

static const struct {
  const char json[6];
  pos_u8_t sensor_pl;
  pos_u8_t plss_pl;
} mix_ctrl[] = {
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

/**
* Power control
* @param[in]  on   0:Power off, 1:Power on
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
* @note If sensor does NOT require any power control, this fucntion can be NULL
*/ 
pos_status_t sensor_mix_set(pos_u32_t on) {  
  pos_u16_t pwr, w1;
  drv_api_t *drv = g_drv;  
  ma_board_t *b = drv->board;
  w1 = drv->s->slot->io;
  pwr = (w1 >> 8) & 0x7f;
  if( !pwr )
    pwr = MB_PWR_ID_VCC_CTRL1+(w1&3)-1;
  
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data(on ? "mix on" : "mix off", pwr);
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
* Sensor collecting O2/CO2
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_mix_collect_o2_co2(pos_u16_t io_id, pos_u8_t d_type, pos_u8_t grp) {
  drv_api_t *drv = g_drv;
  pos_io_handle_t *io;
  pos_u32_t i, v, to, o2_chk;
  pos_u8_t buf[9], c, j;
  io = drv->os->io->init(io_id, 9600, 0);
  if( !io ) {
    return POS_STATUS_E_RESOURCE;
  }

  to = drv->os->tick->get() + 600000; /* at most 10min timeout for o2 repeating */
  o2_chk = drv->s->slot->thld.u16[0];
  
  /* flush */
  while( io->read(io, POS_NULL, 1024, 50) >= 1024 ); 

  v = 0xffff;
  for( i = 0; i < 10; i++ ) {
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      drv->log->data("mix oc2 io", io_id);
      drv->log->data("mix oc2 dt", d_type);
    }

    /* flush */
    while( io->read(io, POS_NULL, 1024, 50) >= 1024 ); 

    drv->os->memset(buf, 0, 9);
    buf[0] = 0xff;
    buf[1] = 1;
    buf[2] = 0x86;
    buf[8] = 0x79;
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      drv->log->buf("mix oc2/wr", buf, 9);
      drv->os->flush();
    }

    /* Request */    
    io->write(io, buf, 9, 100);
    
    /* pull: 1000ms */
    j = io->read(io, buf, 9, 1000);
    if( j && (drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG) != 0 ) {
      drv->log->buf("mix oc2/rd", buf, j);
      drv->os->flush();
    }
    if( j < 9 || buf[1] != 0x86 ) {
#if 0      
      if( (i & 3) == 3 ) {
        drv->os->memset(buf, 0, 9);
        buf[0] = 0xff;
        buf[1] = 1;
        buf[2] = 0x78;
        buf[3] = 0x40;
        buf[8] = 0x47;
        io->write(io, buf, 9, 100);
        if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
          drv->log->buf("mix o2/wr", buf, 9);
          drv->os->flush();
        }
      }
#endif      
      continue;
    }
    c = 0;
    for( j = 0; j < 9; j++ )
      c += buf[j];
    if( c == 0xff ) {
      if( !d_type ) {
        v = buf[6]*256 + buf[7]; /* o2 */
        v = (v + 5)/10; /* translate to 0.1%VOL */
        if( o2_chk ) {
          /* not reach o2_chk level and timeout is also not reached */
          if( v < o2_chk && !drv->os->tick->is_timeout(drv->os->tick->get(), to) ) {
            /* sleep 30s and re-read o2 */
            if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
              drv->log->data("mix o2 chk", o2_chk);
            }
            drv->os->tick->sleep(30000);
            i = 0;
            continue;
          }
        }
      } else {
        v = buf[2]*256 + buf[3]; /* co2 */
        v *= 100;
        if( v > 65500 )
          v = 65500;
      }
      break;
    }
  }

  if( !d_type ) {
    drv->data->plss_json_put(v, DRV_MA_PLSS_TRANS(PLSS_PL_EXT_O2,2,0,0,1), "o2");
    i = 0; 
  } else {
    drv->data->plss_json_put(v, DRV_MA_PLSS_TRANS(PLSS_PL_EXT_CO2_PPM,2,0,0,0), "co2");
    i = 4;
  }

  /* record o2/co2 */
  DRV_SET_HISTORY_EXT_U16(drv, grp-1, i, v);

  /* shutdown IO after read */
  io->release(io);

  return POS_STATUS_OK;
}

/** 
 * CRC8 chk for uart
 */
pos_status_t sensor_mix_uart_crc8_chk(pos_u8_t *data, pos_u8_t len) {
  pos_u8_t c, i;
  c = 0;
  for( i = 0; i < len; i++ )
    c += data[i];
  if( c == data[0] )
    return POS_STATUS_OK;
  return POS_STATUS_ERROR;
}

/**
* Sensor collecting TB200 UART
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_mix_collect_uart(pos_u16_t io_id, pos_u8_t bmp, pos_u8_t grp) {
  pos_io_handle_t *io;
  pos_status_t ret = POS_STATUS_E_TIMEOUT;
  pos_u32_t v, sum, cnt;
  pos_u8_t tb600_type, tb600_precision, ctrl_id = 255;
  pos_u16_t loop;
  drv_api_t *drv = g_drv;
  pos_u8_t response[13];
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("mix uart", io_id);        
  }

  io = drv->os->io->init(io_id, 9600, 0);
  if( !io ) {  
    return ret;
  }
  
  /* 重试N次 */
  sum = 0;
  cnt = 0;
  tb600_type = 0;
  tb600_precision = 0;

  /* UART Reading */
  for(loop = 0; loop < 5; loop++ ) {
#if 0   
    /* quit sleeping */
    {
      const pos_u8_t cmd[5] = {0xae, 0x45, 0x78, 0x69, 0x74,};
//      const pos_u8_t cmd[6] = {0xa2, 0x45, 0x78, 0x69, 0x74, 0x32};      
      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
        drv->log->buf("tb200 wr", (pos_u8_t*)cmd, POS_ARRAY_CNT(cmd));
        drv->os->flush();
      }
      io->write(io, (pos_u8_t*)cmd, POS_ARRAY_CNT(cmd), 100);
      v = io->read(io, response, 9, 1000);
      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
        drv->log->buf("tb200 rd", response, v);
      }
    }
#endif
    /* flush */
    while( io->read(io, POS_NULL, 1024, 50) >= 1024 ); 
    /* query data type&precision */
    response[0] = 0xd1;
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      drv->log->buf("tb200 wrt", response, 1);
      drv->os->flush();
    }
    io->write(io, response, 1, 100);
    v = io->read(io, response, 9, 1000);
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      drv->log->buf("tb200 rdt", response, v);
    }
    if( v != 9 )
      continue; /* wrong length */
    
    /* cksm8 check */
    if( sensor_mix_uart_crc8_chk(response, v) != POS_STATUS_OK )
      continue; /* wrong cksum */
    tb600_type = response[0];
    tb600_precision = response[7]>>4;

    /* flush */
    while( io->read(io, POS_NULL, 1024, 50) >= 1024 ); 
    /* query data */
    drv->os->memset(response, 0, 9);
    response[0] = 0xff;
    response[1] = 1;
    response[2] = 0x87;
    response[8] = 0x78;
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      drv->log->buf("tb200 wr", response, 9);
      drv->os->flush();
    }
    io->write(io, response, 9, 100);
    v = io->read(io, response, 13, 1000);
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      drv->log->buf("tb200 rd", response, v);
    }
    if( v != 13 || response[1] != 0x87 )
      continue; /* wrong length */

    /* cksm8 check */
    if( sensor_mix_uart_crc8_chk(response, 13) != POS_STATUS_OK )
      continue; /* wrong cksum */

    v = (response[6]<<8) + response[7];
    cnt++;
    sum += v;      
    if( cnt >= 1 )
      break;
  }

  /* reset cnt flag to zero when type is NOT supported */
  if( cnt ) {
    for( ctrl_id = 0; ctrl_id < POS_ARRAY_CNT(mix_ctrl); ctrl_id++ ) {
      if( (mix_ctrl[ctrl_id].sensor_pl&0x7f) == tb600_type ) {
        break;
      }
    }
    if( ctrl_id >= POS_ARRAY_CNT(mix_ctrl) ) {
      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
        drv->log->data("tb200 unk", tb600_type);        
      }          
      cnt = 0; /* not support type, break immediately */        
    }
  }
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("tb200 t_p", (cnt << 16) + (tb600_type<<8)+tb600_precision);        
  }      

  /* major report */
  if( cnt )
    v = sum / cnt;
  else {
    v = 0xffff;
    ctrl_id = 3; /* when not read, always report as h2s */
  }
  drv->data->plss_json_put(v, 
    DRV_MA_PLSS_TRANS(mix_ctrl[ctrl_id].plss_pl, 2, tb600_precision, 0, tb600_precision), 
    mix_ctrl[ctrl_id].json);    

  /* record h2s */
  DRV_SET_HISTORY_EXT_U16(drv, grp-1, 1, v);/* using raw 0.01 PPM unit */

  /* report error when cnt flag zero */
  if( cnt ) {
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

    ret = POS_STATUS_OK;
  }

  /* shutdown IO after read */
  io->release(io);
  
  return ret;
}


/**
* Sensor collecting CH4
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_mix_collect_ch4(pos_u16_t io_id, pos_u8_t grp) {
  drv_api_t *drv = g_drv;
  pos_io_handle_t *io;
  pos_u32_t i, v, r, k;
  pos_u8_t buf[32], j, k1, *p;
  io = drv->os->io->init(io_id, 9600, 0);
  if( !io ) {
    return POS_STATUS_E_RESOURCE;
  }

  /* flush */
  while( io->read(io, POS_NULL, 1024, 50) >= 1024 ); 

  r = 0xffff;
  for( i = 0; i < 10; i++ ) {
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      drv->log->data("mix ch4", io_id);
    }

    /* flush */
    while( io->read(io, POS_NULL, 1024, 50) >= 1024 ); 

    buf[0] = 0x41;
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      drv->log->buf("mix ch4/wr", buf, 1);
      drv->os->flush();
    }

    /* Request */    
    io->write(io, buf, 1, 100);
    
    /* pull: 1000ms */
    j = io->read(io, buf, 1, 1000);
    if( j )
      j += io->read(io, &buf[1], POS_ARRAY_CNT(buf)-2, 100);
    buf[j] = 0;
    p = buf;
    if( p[0] != ':' && j ) {
      p++; /* point to next */
      v = j-1;
    } else {
      v = j;
    }
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      drv->log->buf("mix ch4/rd", p, v);
      drv->os->printf("%s\r\n",p);
      drv->os->flush();
    }
    /* buf="A:LEL,0.0%,10258\r\n" 
     * p=":LEL,0.0%,10258\r\n"
     * crc=modbus_crc(":LEL,0.0%,")
     */    
    if( v < 7 || p[0] != ':' || p[v-2] != 0x0d || p[v-1] != 0x0a) {
      continue;
    }
    k = 0;
    k1 = 0;
    for( j = 0; j < v; j++ ) {
      if( p[j] == ',' ) {
        k++;
        if( k >= 2 ) {          
          k = drv->os->crc(POS_CRC16_MBUS, p, j+1); /* crc including the second ',' */
          k = POS_NTOHS(k); /* it's using network order */
          if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
            drv->log->data("mix ch4 crc", k);
          }
          if( drv->os->util->str2u32((char*)&p[j+1]) == k )
            break;
          j = v;
          break;
        } else
          k1 = j;
      }
    }
    if( j >= v )
      continue; /* wrong crc */

    k = 0;
#if 0 /* DBG */   
    drv->os->memcpy(&p[k1+1], "20.45%VOL", 9); 
#endif
    v = v - k1 - 1;
    drv->os->str2bin((char*)&p[k1+1], (pos_u8_t*)&k, &v, DECODE_UFLOAT1000);
    r = k / 2; /* translate from LEL to PPM, CH4 LEL=5%=50000PPM, k * 50000 / 100000 = k / 2 */;
    if( r > 65534 )
      r = 65534;
    break;
  }

  drv->data->plss_json_put(r, DRV_MA_PLSS_TRANS(PLSS_PL_EXT_CH4,2,0,0,0), "ch4");

  /* record ch4 */
  DRV_SET_HISTORY_EXT_U16(drv, grp-1, 3, r);

  /* shutdown IO after read */
  io->release(io);

  return POS_STATUS_OK;
}


/** 
 * CRC8 chk for i2c
 */
pos_status_t sensor_mix_i2c_crc8_chk(pos_u8_t *data, pos_u8_t len) {
  pos_u8_t c;
  c = 0;
  while( len-- > 1 )
    c += *data++;
  if( c == *data )
    return POS_STATUS_OK;
  return POS_STATUS_ERROR;
}

/** 
 * Sleep without system deep sleep so that I2C can still work
 */
void sensor_mix_i2c_sleep(pos_u32_t ticks) {
  const pos_lib_tick_t *t;  
  t = g_drv->os->tick;
  ticks = t->get() + ticks;
  while( !t->is_timeout(t->get(), ticks) ) {
    t->sleep(20);
  }
}

/**
* Sensor collecting I2C-1 Gas
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_mix_collect_i2c1(pos_u16_t io_id, pos_u32_t baud, pos_u8_t bmp, pos_u8_t grp) {
  pos_io_handle_t *io;
  pos_status_t ret = POS_STATUS_E_TIMEOUT;
  pos_u32_t v, sum, cnt;
  pos_u8_t tb600_type, tb600_precision, ctrl_id = 255;
  pos_u16_t loop;
  drv_api_t *drv = g_drv;
  pos_u8_t response[16];


  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("mix i2c", io_id);        
  }

  io = POS_NULL;
  v = baud;
  if( !v )
    v = 8192; /* 默认用8KHz */  
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("i2c bd", v);
  }    
  io = drv->os->io->init(io_id, v, POS_IO_CTRL_I2C_MASTER); /* MASTER */
  if( !io ) {  
    return ret;
  }
  
  /* 重试N次 */
  sum = 0;
  cnt = 0;
  tb600_type = 0;
  tb600_precision = 0;

  /* I2C Reading */
  drv->os->call(POS_CALL_FUNC_ID_SET_I2C_ADDR, io, 0x08FC); /* ADDR8=0xFC(0x7e*2), REGSIZE=0x08 (8Bits) */    
  for(loop = 0; loop < 3; loop++ ) {
    /* sleep for not first loop */
    if( loop )    
      sensor_mix_i2c_sleep(1000);
#if 0
    v = sensor_tb600_i2c_detect(io);
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {  
      drv->log->data("t6 detect", v);
    }
    if( !v )
      continue;
#endif
#if 1
    v = io->read(io, response, 6, POS_IO_WAIT_I2C_READ8(0xd2, 3000));
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      drv->log->buf("t6 d2", response, v);
    }
    if( v != 6 )
      continue; /* wrong length */

    sensor_mix_i2c_sleep(1000); /* needs sleep before next read, or else read will be failed */
    
    /* cksm8 check */
    if( sensor_mix_i2c_crc8_chk(response, 6) != POS_STATUS_OK )
      continue; /* wrong cksum */
    tb600_type = response[1];
    tb600_precision = response[0];
#else
    tb600_type = 0x21;
    tb600_precision = 3;
#endif
    v = io->read(io, &response[6], 7, POS_IO_WAIT_I2C_READ8(0xd1, 3000));
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      drv->log->buf("t6 d1", &response[6], v);
    }
    if( v != 7 )
      continue; /* wrong length */

    /* cksm8 check */
    if( sensor_mix_i2c_crc8_chk(&response[6], 7) != POS_STATUS_OK )
      continue; /* wrong cksum */

    v = (response[6]<<8) + response[7];
    cnt++;
    sum += v;      
    if( cnt >= 1 )
      break;      
  }

  /* reset cnt flag to zero when type is NOT supported */
  if( cnt ) {
    for( ctrl_id = 0; ctrl_id < POS_ARRAY_CNT(mix_ctrl); ctrl_id++ ) {
      if( (mix_ctrl[ctrl_id].sensor_pl&0x7f) == tb600_type ) {
        break;
      }
    }
    if( ctrl_id >= POS_ARRAY_CNT(mix_ctrl) ) {
      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
        drv->log->data("t6 unk", tb600_type);
      }          
      cnt = 0; /* not support type, break immediately */        
    }
  }
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("t6 t_p", (tb600_type<<8)+tb600_precision);
  }

  if( cnt )
    v = sum / cnt;
  else {
    v = 0xffff;
    ctrl_id = 0; /* when not read, always report as CO */
  }

  /* major report */
  drv->data->plss_json_put(v,
    DRV_MA_PLSS_TRANS(mix_ctrl[ctrl_id].plss_pl, 2, tb600_precision, 0, tb600_precision),
    mix_ctrl[ctrl_id].json);

  /* record co */
  DRV_SET_HISTORY_EXT_U16(drv, grp-1, 2, v);/* using raw 0.1 PPM unit */

  /* report error when cnt flag zero */
  if( cnt ) {
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

    ret = POS_STATUS_OK;
  }

  /* shutdown IO after read */
  io->release(io);

  return ret;
}

/**
* Sensor collecting I2C-2 Gas (2in1/DGM10)
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/
pos_status_t sensor_mix_collect_i2c2(pos_u16_t io_id, pos_u32_t baud, pos_u8_t bmp, pos_u8_t grp) {
  pos_io_handle_t *io;
  pos_status_t ret = POS_STATUS_E_TIMEOUT;
  pos_u32_t v, i;
  pos_u8_t tb600_type, tb600_precision, ctrl_id = 255;
  pos_u16_t loop;
  drv_api_t *drv = g_drv;
  pos_u8_t response[32];
  pos_u16_t r_co, r_h2s;

  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("mix i2c", io_id);
  }

  io = POS_NULL;
  v = baud;
  if( !v )
    v = 8192; /* 8K */ //50000; /* 默认用50KHz */
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("i2c bd", v);
  }    
  io = drv->os->io->init(io_id, v, POS_IO_CTRL_I2C_MASTER); /* MASTER */
  if( !io ) {  
    return ret;
  }
  
  /* 重试N次 */
  tb600_type = 0;
  tb600_precision = 0;

  r_co = 65535;
  r_h2s = 65535;
  /* I2C Reading */
  drv->os->call(POS_CALL_FUNC_ID_SET_I2C_ADDR, io, 0x08a0); /* ADDR8=0xA)(0x50*2), REGSIZE=0x08 (8Bits) */    
  for(loop = 0; loop < 3; loop++ ) {
    /* sleep for not first loop */
    if( loop )
      sensor_mix_i2c_sleep(1000);
#if 0
    v = io->write(io, POS_NULL, 1, 100);
    sensor_mix_i2c_sleep(1000);
    drv->log->data("dgm10 ack", v);
#endif
    v = io->read(io, response, 32, POS_IO_WAIT_I2C_READ8(0x00, 3000));
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      drv->log->buf("dgm10 rd", response, v);
    }
    if( v != 32 || response[30] != 'D' || response[31] != 'G' )
      continue; /* wrong length */

    for( i = 0; i < 2; i++ ) {
      tb600_type = response[19+i*10];
      tb600_precision = 3;
      v = drv->os->util->get_u32(&response[10+i*10], 4);
      v = POS_NTOHL(v);
      v = drv->os->util->ufloat_to_u32_precision(v,1000);
      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
        drv->log->data("dgm10 v1k", v);
      }
      for( ctrl_id = 0; ctrl_id < POS_ARRAY_CNT(mix_ctrl); ctrl_id++ ) {
        if( (mix_ctrl[ctrl_id].sensor_pl&0x7f) == tb600_type ) {
          break;
        }
      }
      if( ctrl_id >= POS_ARRAY_CNT(mix_ctrl) ) {
        if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
          drv->log->data("dgm10 unk", tb600_type);
        }
        ret = POS_STATUS_E_NOT_SUPPORT;
      } else {
        if( mix_ctrl[ctrl_id].plss_pl == PLSS_PL_EXT_CO ) {
          /* uniform co to 0.1 PPM */
          tb600_precision = 1;
          v = (v + 50) / 100;
          r_co = v > 65534 ? 65534 : v;
        } else if( mix_ctrl[ctrl_id].plss_pl == PLSS_PL_EXT_H2S ) {
          /* uniform h2s to 0.01 PPM */
          tb600_precision = 2;
          v = (v + 5) / 10;
          r_h2s = v > 65534 ? 65534 : v;          
        }
        ret = drv->data->plss_json_put(v, 
          DRV_MA_PLSS_TRANS(mix_ctrl[ctrl_id].plss_pl, 2, tb600_precision, 0, tb600_precision),
          mix_ctrl[ctrl_id].json);
      }
    }
    break;
  }

  /* record co */
  DRV_SET_HISTORY_EXT_U16(drv, grp-1, 2, r_co);/* using raw 0.1 PPM unit */          
  
  /* record h2s */
  DRV_SET_HISTORY_EXT_U16(drv, grp-1, 1, r_h2s);/* using raw 0.01 PPM unit */          

  /* shutdown IO after read */
  io->release(io);

  return ret;
}

/** 
* Sensor collecting
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_mix_collect(void){
  pos_status_t ret = POS_STATUS_OK, r;
  drv_api_t *drv = g_drv;
  pos_u8_t grp;
  pos_u8_t bmp;
  pos_u32_t baud, i;

  /* init grp */
  grp = drv->s->slot->io&0xff;
  if( !grp || grp > 3 )
    grp = 1;
  bmp = drv->s->slot->rsvd32&0xff;
  if( !bmp )
    bmp = 0x0f; 

  baud = (drv->s->slot->rsvd32>>16)&0xffff;
  if( baud )
    baud *= 1000;
  
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->os->flush();
    drv->log->data("mix grp", grp);
  }

  /* refresh to normal by default */
  drv->s->ctrl |= MA_SENSOR_CTRL_STAT_NORMAL;
  drv->s->ctrl &= ~MA_SENSOR_CTRL_STAT_ERROR;

  for( i = 0; i < 8; i++ ) {
    if( (bmp & (1<<i)) == 0 )
      continue;

    switch(i) {
    case 0: /* O2 */
      r = sensor_mix_collect_o2_co2(grp*4+POS_IO_UART0, 0, grp);
      break;
 
    case 1: /* CH4 */
      r = sensor_mix_collect_ch4(grp*4+POS_IO_UART1, grp);
      break;

    case 2: /* UART/H2S */
      if( grp > 2 )
        r = POS_STATUS_E_NOT_SUPPORT; /* only grp1/2 support UART/h2s */
      else
        r = sensor_mix_collect_uart(POS_IO_UART2+grp-1, 1, grp); /* grp1 uses UART2, grp2 uses UART3 for H2s */
      break;
      
    case 3: /* CO */
      r = sensor_mix_collect_i2c1(grp*4+POS_IO_I2C1, baud, 1, grp);
      break;

    case 4: /* UART/CO2 */
        r = sensor_mix_collect_o2_co2(grp < 3 ? POS_IO_UART2+grp-1 : POS_IO_UART0, 1, grp); /* grp1 uses UART2, grp2 uses UART3 for H2s, grp3 uses UART0 (tmp solution shared with other 485) */
      break;

    case 5: /* DGM10: CO+H2S */
      r = sensor_mix_collect_i2c2(grp*4+POS_IO_I2C1, baud, 1, grp);
      break;

    default:
      r = POS_STATUS_E_NOT_SUPPORT;
      break;
    }
    if( r != POS_STATUS_OK )
      ret = r;    
  }

  /* update to error status */
  if( ret != POS_STATUS_OK )
    drv->s->ctrl |= MA_SENSOR_CTRL_STAT_ERROR;
  
  return ret;  
}


/*
 * Driver export
 */
ma_drv_export_t g_export = {
  .version = DRV_SENSOR_VERSION(MA_SENSOR_MIX, DRV_MIX_VERSION),
  .name = DRV_MIX_NAME,
  .u.sensor={
    .power_set = sensor_mix_set, 
    .collect = sensor_mix_collect,
  },
};

