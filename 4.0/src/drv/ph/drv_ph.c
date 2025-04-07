/**
 * @file  drv_modbus.c
 * @brief SENSOR modbus driver
 * @author Runby F.
 * @date 2022-3-23
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * 版本修订历史
v1.0 @ 2022-03-23
  1) 完成第一版驱动

v1.1 @ 2022-03-25
  1) 支持4.0整体供电策略

v1.2 @ 2022-05-13
  1) 解决供电配置异常

v1.3 @ 2022-12-19
  1) Support JXBS-3001 PH sensor by default

v1.4 @ 2022-12-21
  1) Shutdown IO after reading to save power

v1.5 @ 2023-04-04
  1) Define this driver as base modbus driver for PH/SOUND/...

v1.6 @ 2023-04-06
  1) Support 17 - SOIL sensor

v1.7 @ 2023-04-21
  1) Support 18 - NPK sensor
  2) Fix SOIL tmp unable to display negative values

v1.8 @ 2023-05-08
  1) Support 19 - ZE03 various gas

v1.9 @ 2023-06-01
  1) Support 20 - ULTRA/MBUS
  2) Fix baudrate unable to change issue and use 115200 for ULTRA/MBUS default
  3) Support 20 - Car Detect (d0=0xAABBLLLL, 0xAA is RED pin, 0xBBB is GREEN pin, LLLL is park slot free thold)

v1.10 @ 2023-06-02
  1) Fix ULTRA/MBUS RED/GREEN light issue
  2) Fix ULTRA/MBUS return 0 issue (use 65533 instead)

v1.11 @ 2023-06-14
  1) Support 21 - 485/Temp
  2) Correct HCL type to 27 instead of 37
  3) Change json string from tmp to etmp
  
v1.12 @ 2023-06-15
  1) Support 22 - Laser/Distance

v1.13 @ 2023-06-16
  1) Fix 22 - Laser/Distance sometime can not read data issue

v1.14 @ 2023-06-19
  1) Support 23 - ZPHS/Multi sensor

v1.15 @ 2023-07-07
  1) Support 24 - DGM10/Multi sensor

v1.16 @ 2023-07-10
  1) Fix DGM10 wakeup/sleep mistake

v1.17 @ 2023-07-11
  1) Fix etmp/ehum bracelet issue

v1.18 @ 2023-07-13
  1) Support type-25 wind speed
  2) Support type-26 wind dir

v1.19 @ 2023-07-18
  1) Support type-27 flow

v1.20 @ 2023-08-26
  1) Use explict IO init uart ctrl field

v1.21 @ 2023-09-05
  1) Support type-28 do

v1.22 @ 2023-09-15
  1) Support ze03 legacy co

v1.23 @ 2023-09-26
  1) Support type-29 light

v1.24 @ 2023-10-10
  1) Support ZE03 O3 

v1.25 @ 2023-10-12
  1) Support type-30 zce04

v1.26 @ 2023-10-24
  1) Support normal/error stat
  2) Support dgm10 (type 24) o3/co

v1.27 @ 2023-10-27
  1) Support new Soil humidity (type 17) and osbolete the old soil humidity

v1.28 @ 2023-11-01
  1) Support dgm1 co as PLSS PPB unit

v1.29 @ 2023-11-14
  1) Support ze03 co (type 19)

v1.30 @ 2023-11-18
  1) Using new PWR ID solution

v1.31 @ 2023-11-23
  1) Support ZPHS01 v10 version (PM2.5+PM10+PM1) and keep original v15 version (PM2.5 only)
  2) Fix ZPHS01 temperature below zero displaying issue
  3) Optimize ZPHS01 calibration process

v1.32 @ 2023-11-30
  1) Support WIND DIR in unit of 0.01 degree 

v1.33 @ 2024-1-25
  1) Support Sound history report for min/max/avg

v1.34 @ 2024-4-17
  1) Change to "soil_hum_x" for SOIL_HUM

v1.35 @ 2024-4-23
  1) Support type=19 srsv=1 for new ZE03/CH4 sensor

v1.36 @ 2024-4-24
  1) Fix ZE03 CH4 displaying issue

v1.37 @ 2024-4-25
  1) Fix ZE03 CH4 report issue for wrong payload detecting

v1.38 @ 2024-4-26
  1) Remove ZE03 CH4 precision detecing and always use 0

v1.39 @ 2024-5-16
  1) Support type-16 thw3 thold/history report mode

v1.40 @ 2024-6-07
  1) Support ZC13 (CH4) based on type - 19 (ZE03) driver writh srsv=0xff

v1.41 @ 2024-6-15
  1) Support SC16 (CO) based on type - 19 (ZE03) driver writh srsv=0xff

v1.42 @ 2024-6-17
  1) Fix CH4 not correct issue

v1.43 @ 2024-7-10
  1) Support MAX 8 485 Temp

v1.44 @ 2025-2-17
  1) Support ZE03/H2

v1.45 @ 2025-2-19
  1) Support default 485 temp as -999.0

*/

/**
 * @brief Driver version
 */
#define DRV_MODBUS_VERSION 0x012d

/**
 * @brief Driver name
 */
#define DRV_MODBUS_NAME  "MODBUS"

/**
 * @brief MAX history data num
 */
#define DRV_MODBUS_HISTORY_MAX 32

/*
 * @brief Sensor MODBUS Structure
 */
typedef struct {
  pos_u32_t     u_opr_id;
  pos_u32_t     u_cnt;
  pos_u32_t     u_min;  
  pos_u32_t     u_max;
  pos_u32_t     u_sum;
  pos_u8_t      history_num; /**< number of history */
  pos_u8_t      rsvd[3]; /**< reserved */
  pos_u16_t     history[DRV_MODBUS_HISTORY_MAX]; /**< max history */
} smbus_param_t;


/** 
* Power control
* @param[in]  on   0:Power off, 1:Power on
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
* @note If sensor does NOT require any power control, this fucntion can be NULL
*/ 
pos_status_t sensor_modbus_set(pos_u32_t on) {  
  pos_u16_t pwr;
  ma_board_t *b = g_drv->board;
  pwr = (g_drv->s->slot->io >> 8) & 0x7f;  
  if( !pwr ) {
    switch(g_drv->s->slot->sensor_type) {
    case MA_SENSOR_SOUND:
    case MA_SENSOR_ZE03:
    case MA_SENSOR_ZPHS_MULTI:      
    case MA_SENSOR_WIND_SPEED:
    case MA_SENSOR_WIND_DIR:   
    case MA_SENSOR_ZCE04:
      pwr = MB_PWR_ID_VCC_PWRH;
      break;
      
    default:
      pwr = MB_PWR_ID_VCCN_PWR3;
      break;
    }
  }

  b->pwr_set(pwr, on);
  
  return POS_STATUS_OK;
}

/** 
* Sensor ZE03 family collecting
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_ze03_collect(pos_io_handle_t *io, pos_u8_t addr, pos_u32_t *v){
  drv_api_t *drv = g_drv;
  pos_u32_t i, j;
  pos_u8_t buf[9], c;
  for( i = 0; i < 10; i++ ) {
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      drv->log->data("ze03 addr", addr);
    }

    /* flush */
    while( io->read(io, POS_NULL, 1024, 50) >= 1024 ); 

    /* pull result when addr is not zero or 0xff */
    if( addr && addr != 0xff ) {
      drv->os->memset(buf, 0, 9);
      buf[0] = 0xff;
      buf[1] = addr;
      buf[2] = 0x86;
      buf[8] = 0x7a - addr; /* cksum */
      if( (drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG) != 0 ) {
        drv->log->buf("ze03 wr", buf, 9);
        drv->os->flush();
      }
      io->write(io, buf, 9, 100);
    }

    /* pull: 500ms, report: 1.2s */
    j = io->read(io, buf, 9, addr != 0xff ? 500 : 1200);
    if( j && (drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG) != 0 ) {
      drv->log->buf("ze03 rd", buf, j);
      drv->os->flush();
    }      
    if( j < 9 )
      continue;
    c = 0;
    for( j = 0; j < 9; j++ )
      c += buf[j];
    if( c == 0xff ) {
      /* ZC13 active report payload format when ADDR = 0xff*/
      if( addr == 0xff ) {
        switch(buf[1]) {
        case 1:
          /* ZC13: FF, 01(CH4), 03(PPM), PRECISION, V16_HI, V16_LO, RNG_HI, RNG_LO, CKSUM8 (9Bytes) */
          v[0] = (buf[3] << 24) + (buf[1] << 16) + buf[4] * 256 + buf[5]; /* PRECISION PL V_HI V_LO */
          break;
        case 0x18:
          /* SC16: FF, 18(CO), 04(PPM), XX(NC), V16_HI, V16_LO, ALRM_HI, ALRM_LO, CKSUM8 (9Bytes) */
          /* ex: ff1804000000ffffe6 */
          v[0] = (0 << 24) + (4 << 16) + buf[4] * 256 + buf[5]; /* PRECISION PL(4:CO) V_HI V_LO */
          break;
        default:
          v[0] = 0xffff;
          break;
        }
      } else if( addr == 0x01 ) {
        /* ZE03/ZC101-CH: FF 86 V16_HI V16_LO RSV_HI RSV_LO 00 00 CKSUM8 (9Bytes) */
        v[0] = 0x010000 + buf[2] * 256 + buf[3]; /* PRECISION PL(1:CH4) V_HI V_LO */
      } else {
        /* ZE03: FF, 86, V16_HI, V16_LO, PL(buf4), PRECISION(buf5) ... (9Bytes) */
        v[0] = (buf[5] << 24) + (buf[4] << 16) + buf[2] * 256 + buf[3];
      }
      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
        drv->log->data("ze03 v", v[0]);
      }
      return POS_STATUS_OK;
    }      
  }
  return POS_STATUS_E_RESOURCE;
}


/** 
* Sensor ZCE04 family collecting
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_zce04_collect(pos_io_handle_t *io, pos_u32_t *v){
  drv_api_t *drv = g_drv;
  pos_u32_t i, j;
  pos_u8_t buf[16], c;
  for( i = 0; i < 10; i++ ) {
    /* flush */
    while( io->read(io, POS_NULL, 1024, 50) >= 1024 ); 
        
    /* pull: 500ms */
    j = io->read(io, buf, 11, 500);
    if( j && (drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG) != 0 ) {
      drv->log->buf("zcb04 rd", buf, j);
    }      
    if( j < 11 )
      continue;
    c = 0;
    for( j = 0; j < 11; j++ )
      c += buf[j];
    if( c == 0xff ) {
      /* ff 86 co/16 h2s/16 o2/16 ch4/16 crc */
      for( c = 0; c < 4; c++ ) 
        v[c] = (buf[2+c*2] << 8) +  buf[2+c*2+1];
      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
        drv->log->buf("zcb04 v", v, 16);
      }  
      return POS_STATUS_OK;
    }      
  }
  return POS_STATUS_E_RESOURCE;
}

/**
* Sensor Laser Distance collecting
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_laser_distance_collect(pos_io_handle_t *io, pos_u32_t *v){
  drv_api_t *drv = g_drv;
  pos_u32_t i, j, k;
  pos_u8_t buf[32], *c;
  for( i = 0; i < 20; i++ ) {

    /* flush */
    while( io->read(io, POS_NULL, 1024, 50) >= 1024 ); 

    /* pull: 500ms */
    j = io->read(io, buf, sizeof(buf), 200);
    if( j && (drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG) != 0 ) {
      drv->log->buf("laser rd", buf, j);
    }
    c = buf;
    while( j && *c != 0x0d ) {
      c++;
      j--;
    }
    if( j < 6 || c[1] != 0x0a || c[2] < '0' || c[2] > '9' )
      continue;
    j -= 2;
    c += 2;
    for( k = 0; k < j; k++ )
      if( c[k] < '0' || c[k] > '9' )
        break;
    if( c[k] != 'm' )
      continue; /* must be 0d 0a xx xx xx xx 'm' 'm' format */
    *v = drv->os->util->str2u32((char*)c);
    return POS_STATUS_OK;
  }
  return POS_STATUS_E_RESOURCE;
}


/**
* Sensor ZPHS Multi-data collecting/reprot
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_zphs_multi_report(pos_io_handle_t *io, pos_u32_t bmp) {
  drv_api_t *drv = g_drv;
  pos_u32_t i, j, k;
  pos_u8_t buf[32];

  /* calibration check */
  k = drv->s->slot->arg.u16[1];
  if( k ) {
    pos_u8_t calib_cmd[] = {0x11, 0x03, 0x03, 0x01, 0x90, 0x58,};
    if( k > 1 ) {
      calib_cmd[3] = (k>>8)&0xff;
      calib_cmd[4] = (k)&0xff;
    }
    j = calib_cmd[0];
    for( i = 1; i < 5; i++ )
      j += calib_cmd[i];
    calib_cmd[i] = (~j)+1;
    if( j && (drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG) != 0 ) {
      drv->log->buf("zphs calib", calib_cmd, 6);
    }
    drv->os->tick->sleep(20*60*1000); /* delay 20mins to let data stable */     
    io->write(io, calib_cmd, 6, 100);
    /* remove calibration flag */
    k = 0;
    j = (pos_u32_t)&drv->s->slot->arg.u16[1] - MA_EEPROM_ADDR;
    drv->eeprom->update(j, &k, 2);
  }

  /* data report */
  for( i = 0; i < 12; i++ ) {
    static const pos_u8_t req_cmd[5] = {0x11, 0x02, 0x01, 0x00, 0xec};      
    static const pos_u8_t poll_cmd[5] = {0x11, 0x02, 0x02, 0x00, 0xeb};      
    pos_u32_t resp_len;
    pos_u8_t *cmd;
    
    /* flush */
    while( io->read(io, POS_NULL, 1024, 50) >= 1024 ); 

    if( bmp & 0x4000 ) {
      /* FULL */
      resp_len = 18; /* 16 0F 02   01 9A(CO2) 00 67(VOC/CH2O) 01 EA(HUM 0.1) 03 04(TEMP+500 0.1) 00 36(PM2.5) 00 3C(PM10) 00 20(PM1) 53 (Sum(18)=0) */
      cmd = (pos_u8_t*)poll_cmd; 
    } else {
      /* PM2.5 only */
      resp_len = 14; /* 16 0B 01 01 9A 00 67 01 EA 03 04 00 36 B4 (Only PM2.5) */
      cmd = (pos_u8_t*)req_cmd;
    }

    /* send tx cmd */
    if( (i & 3) == 0 ) {
      io->write(io, cmd, 5, 100); /* if not receive after several times of retry, send request cmd */      
    }
    
    /* read */
    j = io->read(io, buf, resp_len, 600);
    if( (drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG) != 0 ) {
      drv->log->buf("zphs rd", buf, j);
    }    
    
    /* pattern check */
    if( j < resp_len || buf[0] != 0x16 ) {
      continue;
    }
    k = buf[0];
    for( j = 1; j < resp_len; j++ )
      k += buf[j];      
    if( (k & 0xff) != 0 ) 
      continue;

    /* processing data */
    for( j = 0; j < 5; j++ ) {
      static const struct {
        char json[6];
        pos_u8_t plss_pl;
        pos_u8_t rsvd;        
      } pl[5] = {
        {"co2", PLSS_PL_EXT_CO2_PPM,},
        {"voc", PLSS_PL_EXT_VOC,},
        {"hum", PLSS_PL_HUMIDITY_HR,},
        {"tmp", PLSS_PL_EXT_TEMPERATURE,},
        {"pm2_5", PLSS_PL_EXT_PM,},
      };
      pos_u8_t pl_ind;
      const char *js;
      if( (bmp & (1<<j)) == 0 )
        continue;
      k = (buf[3+j*2]<<8) + buf[4+j*2];
      if( j == 3 )
        k -= 500;
      pl_ind = pl[j].plss_pl;      
      js = pl[j].json;
      if( j == 1 && (bmp & 0x8000) != 0 ) {
        pl_ind = PLSS_PL_EXT_CH2O;
        js = "ch2o";
      }
      drv->data->plss_json_puti(k,
        DRV_MA_PLSS_TRANS(pl_ind,2,0,0,0),
        js
      );

      if( pl_ind == PLSS_PL_EXT_PM ) {
        if( resp_len < 18 ) {
          /* PM2.5 only */
          /* PLSS_PL_EXT_PM contains PM2.5/10/1.0.
           * Append 00 00 00 00 here for PM2.5 only
           */
          k = 0;
          drv->data->plss_put_raw(4, &k);
        } else {
        /* FULL PM2.5/10/1.0 payload */
          drv->data->plss_put_raw(4, &buf[13]); /* response[2..7] is 2.5/10/1.0 */
          drv->data->put("\"%s\":%u,", "pm10", buf[13]*256+buf[14], 0, 0, 0);
          drv->data->put("\"%s\":%u,", "pm1", buf[15]*256+buf[16], 0, 0, 0);     
        }
      }
    }    

    return POS_STATUS_OK;
  }
  return POS_STATUS_E_RESOURCE;

}


/**
* Sensor EC DGM10 wake-up
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_dgm10_wakeup(pos_io_handle_t *io, pos_u8_t addr, pos_u8_t *buf) {
  const static pos_u8_t align[2] = {0x7f, 0x7f};
  pos_u32_t old;
  drv_api_t *drv = g_drv;
  old = drv->os->tick->get(); 
  do {
    io->write(io, (pos_u8_t*)align, 2, 100);
    drv->os->tick->sleep(1000);
#if 1
    if( drv->data->modbus_read(io, buf, addr, 6, 0xc000, 0) ) /* 0 - wakeup */
      return POS_STATUS_OK;
#else
    if( drv->data->modbus_read(io, buf, addr, 3, 0xf000, 17) )
      return POS_STATUS_OK;
#endif    
#if 0
    for( i = 2; i < 256; i++ ) {
      if( drv->data->modbus_read(io, buf, i, 6, 0xc000, 1) )
        return POS_STATUS_OK;
    }
#endif    
  } while( drv->os->tick->elaps(old) < 15000 );
  return POS_STATUS_E_RESOURCE;
}

/**
* Sensor EC DGM10 Multi-data collecting/reprot
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_dgm10_multi_report(pos_io_handle_t *io, pos_u8_t addr, pos_u32_t bmp) {
  drv_api_t *drv = g_drv;
  pos_u32_t i, j, k;
  pos_u8_t buf[48];
  pos_status_t ret;

  static const struct {
    pos_u8_t dgm10_fmt;
    pos_u8_t plss_pl;
    pos_u8_t plss_v_d;
    pos_u8_t json_float_d;
    char json[8];
  } dgm10_db[] = {
      {0x00, PLSS_PL_EXT_TEMPERATURE, 2, 3, "tmp",},
      {0x01, PLSS_PL_HUMIDITY_HR, 2, 3, "hum",},        
      {0x19, PLSS_PL_CO, 0, 3, "co",}, 
      {0x1c, PLSS_PL_EXT_H2S, 0, 3, "h2s",},
      {0x20, PLSS_PL_EXT_NH3, 0, 3, "nh3",},        
      {0x23, PLSS_PL_EXT_O3, 0, 0, "o3",},         
  };
  if( !addr )
    addr = 1;
  
  /* detect & wakeup */
  ret = sensor_dgm10_wakeup(io, addr, buf);
  if( (drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG) != 0 ) {
    drv->log->data("dgm10 chk", ret);
  }  
  if( ret != POS_STATUS_OK )
    return ret;
  
  /* calibration check */
  k = drv->s->slot->arg.u16[1];
  if( k ) {
    for( i = 0; i < 4; i++ ) {
      if( (k & 1<<(i+4)) == 0 )
        continue; /* skip invalid bits */
      j = 0xc100 + i;
      if( i >= 2 )
        j += 0x10;
      /* reg (j) = 0xc100, 0xc101, 0xc110, 0xc111 */
      /* write each calib/zero control bit to reg (j) */
      drv->data->modbus_read(io, buf, addr, 6, j, (k>>i)&0x1);
    }
    /* remove calibration flag */
    k = 0;
    j = (pos_u32_t)&drv->s->slot->arg.u16[1] - MA_EEPROM_ADDR;
    drv->eeprom->update(j, &k, 2);
  }

  /* data report */
  do {
    /* read data */
    j = drv->data->modbus_read(io, buf, addr, 3, 0xf000, 17);
#if 0    
    if( (drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG) != 0 ) {
      drv->log->buf("dgm10 rd", buf, j);
    }
#endif    
    if( j < 5 + 17*2 ) {
      ret = POS_STATUS_E_NOT_FOUND;
      break;
    }

    /* turn off led if it's on */
    if( buf[4] )
      drv->data->modbus_read(io, buf, addr, 6, 0xf000, 0);

    /* filter data and report */
    for( i = 0; i < 4; i++ ) {
      pos_u8_t pos;
      if( (bmp & (1<<i)) == 0 )
        continue;
      
      /* pos = 5, 9, 13, 23 for tmp/hum/gas0/gas1 */
      if( i < 3 )
        pos = 5 + i * 4;        
      else
        pos = 23;

      /* k = 0 for tmp, 1 for hum, buf[22] for gas0, buf[32] for gas1 */
      if( i < 2 )
        k = i; 
      else {
        if( i == 2 )
          k = buf[22];
        else
          k = buf[32];
      }
      
      for( j = 0; j < POS_ARRAY_CNT(dgm10_db); j++ ) {
        /* search fmt */
        if( dgm10_db[j].dgm10_fmt != k )
          continue;

        k = drv->os->util->get_u32(&buf[pos], 4);
        k = POS_NTOHL(k);
        k = drv->os->util->ufloat_to_u32_precision(k, 1000);
        drv->data->plss_json_put(k,
          DRV_MA_PLSS_TRANS(dgm10_db[j].plss_pl,2,dgm10_db[j].plss_v_d,0,dgm10_db[j].json_float_d),
          dgm10_db[j].json
        );        
        break;
      }
    }
  } while(0);
  
  /* sleep */
  drv->data->modbus_read(io, buf, addr, 6, 0xc000, 1);  /* 1 - sleep */
  return ret;
  
}

/** 
 * Driver history data store
 * @param[in]  v   Current data
 */ 
void sensor_modbus_data_store_u(smbus_param_t *param, pos_u32_t v) {
  if( v > param->u_max )
    param->u_max = v;
  if( v < param->u_min )
    param->u_min = v;
  param->u_sum += v;  
  param->u_cnt++;
}

/** 
 * Driver history data clear
 * @param[in]  v   Current data
 */ 
void sensor_modbus_data_clear_u(smbus_param_t *param) {
  param->u_opr_id = 0;
  param->u_cnt = 0;
  param->u_sum = 0;
  param->u_min = 0xffffffff;
  param->u_max = 0; 
}

/**
* Threshold check and history report
* @param[in]  mv   Data
* @return     0: Already processed in history mode (either suppressed or reported)\n
            !=0: Still need to process (none history/thld mode)
*/
pos_status_t sensor_modbus_thld_history_report(pos_u16_t mv, pos_u8_t plss_pl, char *json_field, pos_u32_t json_divide) {
  drv_api_t *drv = g_drv;
  pos_u16_t ratio;
  ratio = drv->s->slot->thld.u16[3]; /* u16[0/1/2] is used for zero filter */
  if( ratio ) {
    smbus_param_t *param;
    param = (smbus_param_t*)drv->s->drv_buf;
    /* threshold check and report */
    if( param->history_num >= DRV_MODBUS_HISTORY_MAX )
      param->history_num = 0; /* anti wrong. usually, it should not reach here */
    param->history[param->history_num++] = mv;
    if( mv >= ratio || param->history_num >= DRV_MODBUS_HISTORY_MAX ) {
      drv->s->ctrl |= MA_SENSOR_CTRL_REPORT;
    }
    drv->thld->periodic_age(drv->s);
    if( drv->s->ctrl & MA_SENSOR_CTRL_REPORT ) {
      /* batch report */
      pos_u16_t i;
      pos_status_t ret;
      /* Array start */
      ret = drv->data->put("\"%s\":[", json_field, 0, 0, 0, 0);
      if( ret == POS_STATUS_OK ) {
        for( i = 0; i < param->history_num; i++ ) {
          mv = param->history[i];
          ret = drv->data->plss_put_u16(plss_pl, mv );
          if( json_divide >= 10 ) {
            drv->data->put_ufloat(mv, json_divide);
          } else {
            drv->data->put("%u,", POS_NULL, mv, 0, 0, 0);
          }
          if( ret != POS_STATUS_OK )
            break;
        }
        drv->data->put_array_stop();
      }
      param->history_num = 0; /* flush history after report */
    }
    return POS_STATUS_OK;
  }
  return POS_STATUS_ERROR;
}

/** 
 * Driver history data accum and report
 * @param[in]  v   Current data
 */ 
pos_status_t sensor_modbus_data_store_put_u(pos_io_handle_t *io, pos_u8_t addr, pos_u8_t func, pos_u16_t reg, pos_u16_t reg_cnt, pos_u32_t plss_trans, const char *json_str) {
  pos_u8_t buf[16];
  static const pos_u16_t json_divide[4] = {1, 10, 100, 1000};
  pos_u32_t v;
  pos_status_t ret = POS_STATUS_OK;
  drv_api_t *drv = g_drv;  
  ma_sensor_ctrl_t *s = drv->s;
  smbus_param_t *param;
  param = (smbus_param_t*) s->drv_buf;
  if( reg_cnt > 2 )
    reg_cnt = 2;
#if 1
  v = drv->data->modbus_read(io, buf, addr, func, reg, reg_cnt);
#else
  /* debug code */
  v = 7; 
  buf[3] = drv->duty_loop >> 8;
  buf[4] = drv->duty_loop;
  buf[5] = 0;
  buf[6] = 0;
#endif
  if( v < 5 + reg_cnt*2 ) {
    return POS_STATUS_E_RESOURCE;
  }
  v = (buf[3] << 8) + buf[4];
  if( reg_cnt > 1 ) {
    v <<= 16;
    v += (buf[5] << 8) + buf[6];
  }

  /* quit if thld/history mode */
  if( sensor_modbus_thld_history_report(v, plss_trans&0xff, (char *)json_str, (pos_u32_t)json_divide[(plss_trans>>24)&0x3]) == POS_STATUS_OK )
    return POS_STATUS_OK;
  
  sensor_modbus_data_store_u(param,v);
  if( ++param->u_opr_id >= s->slot->thld.u16[7] ) {
    if( param->u_cnt ) {
      v = param->u_sum / param->u_cnt; /* calc avg */
      /* report min/max/avg */
      if( s->slot->thld.u16[7] > 1 ) {
        drv->data->plss_json_put(param->u_min, plss_trans, json_str);
        drv->data->plss_json_put(param->u_max, plss_trans, json_str);    
      }
      drv->data->plss_json_put(v, plss_trans, json_str);      
    } else
      ret = POS_STATUS_ERROR; /* force error, when no data accumulated */
    sensor_modbus_data_clear_u(param);    
  } else {
    /* give up current round of data */
    drv->s->ctrl |= MA_SENSOR_CTRL_ABANDON;  
  }
  return ret;
}

/** 
* Sensor collecting
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_modbus_collect(void){
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
#if 0 /* according to william @ 20230601, it is 9600 */   
    if( drv->s->slot->sensor_type == MA_SENSOR_ULTRA_MBUS )
      v = 115200;
#endif    
  }
  io = drv->os->io->init(i, v, 0);

  /* refresh to normal by default */
  drv->s->ctrl |= MA_SENSOR_CTRL_STAT_NORMAL;
  drv->s->ctrl &= ~MA_SENSOR_CTRL_STAT_ERROR;
  
  addr &= 0xff; /* keep addr for mbus addr only */
  if( !addr ) {
    if( drv->s->slot->sensor_type == MA_SENSOR_SOIL )
      addr = 2; /* default 2 for Sensor SOIL */
    else if( drv->s->slot->sensor_type == MA_SENSOR_ZE03 )
      ; /* leave it as zero for ZE03 */
    else
      addr = 1; /* when 0, use 1 by default */
  }
  switch(drv->s->slot->sensor_type) {
  case MA_SENSOR_PH:
    ret = drv->data->modbus_put(io, addr, 3, 6, 1, 
      DRV_MA_PLSS_TRANS(PLSS_PL_EXT_PH2, 2, 0, 0, 2), /* 101 - PLSS_PL_EXT_PH2, 2Bytes */
      "ph" );
    break;
    
  case MA_SENSOR_SOUND: {    
    /* data/store (min/max) report*/
    ret = sensor_modbus_data_store_put_u(io, addr, 3, 0, 1, 
      DRV_MA_PLSS_TRANS(PLSS_PL_SOUND, 2, 0, 0, 1), /* 8 - PLSS_PL_SOUND, 2Bytes */
      "sound" );
    break;    
  }
  
#if 1
  case MA_SENSOR_SOIL: {
    char buf[16];
    for( i = 0; i < 8; i++ ) {
      if( drv->s->slot == &drv->cfg->slot[i] )
        break;
    }
    drv->os->sprintf(buf, "soil_hum_%u", i&0x7);
    ret = drv->data->modbus_put(io, addr, 3, 0, 1, 
      DRV_MA_PLSS_TRANS(PLSS_PL_HUMIDITY_HR, 2, 0, 0, 1), /* 80 - PLSS_PL_HUMIDITY_HR, 2Bytes, 0.1% */
      buf );
    break;
  }
#else
  case MA_SENSOR_SOIL: {
    pos_u8_t buf[16];
    pos_u32_t d0, l;
    static const struct {
      pos_u32_t trans;
      const char json[12];
    } soil[4] = {
        {
            DRV_MA_PLSS_TRANS(PLSS_PL_HUMIDITY_HR, 2, 0, 0, 1), /* 80 - PLSS_PL_HUMIDITY_HR, 2Bytes, 0.1% */
            "hum",
        },
        {
            DRV_MA_PLSS_TRANS(PLSS_PL_EXT_TEMPERATURE, 2, 0, 0, 1), /* 14 - PLSS_PL_EXT_TEMPERATURE, 2Bytes, 0.1degree */
            "tmp",
        }, 
        {
            DRV_MA_PLSS_TRANS(PLSS_PL_EXT_SALINITY, 2, 0, 0, 0), /* 102 - PLSS_PL_EXT_SALINITY, 2Bytes, 1mg/l */
            "salinity",
        }, 
        {
            DRV_MA_PLSS_TRANS(PLSS_PL_EXT_EC, 2, 0, 0, 0), /* 103 - PLSS_PL_EXT_EC, 2Bytes, 1us/cm */
            "ec",
        }, 
    };

    d0 = drv->s->slot->arg.u32[0];    
    if( (d0 & 0xff) == 0 )
      d0 |= 0x0b; /* report hum/temp//ec and skip salinity by default */    
    l = drv->data->modbus_read(io, buf, addr, 3, 0x12, 4);
    if( l < 5 + 4 * 2 )
      break; /* wrong read */
    for( l = 0; l < 4; l++ ) {
      if( (d0 & (1<<l)) == 0 )
        continue;

      v = (buf[3+l*2]<<8) + (buf[4+l*2]);
      if( v & 0x8000 )
        v |= 0xffff0000; /* expand from i16 to i32 for negative values */
      ret = drv->data->plss_json_put(v, soil[l].trans, soil[l].json);
      if( ret != POS_STATUS_OK ) {
        break;
      }
    }
    break;
  }
#endif

  case MA_SENSOR_NPK: {
    pos_u8_t buf[16];
    pos_u32_t d0, l;
    static const struct {
      pos_u32_t trans;
      const char json[4];
    } npk[3] = {
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
    };

    d0 = drv->s->slot->arg.u32[0];    
    if( (d0 & 0xff) == 0 )
      d0 |= 0x7; /* report npk by default */    
    l = drv->data->modbus_read(io, buf, addr, 3, 0x1e, 3);
    if( l < 5 + 3 * 2 )
      break; /* wrong read */
    for( l = 0; l < 3; l++ ) {
      if( (d0 & (1<<l)) == 0 )
        continue;

      v = (buf[3+l*2]<<8) + (buf[4+l*2]);
      ret = drv->data->plss_json_put(v, npk[l].trans, npk[l].json);
      if( ret != POS_STATUS_OK ) {
        break;
      }
    }
    break;
  }  

  case MA_SENSOR_ZE03: {
    ret = sensor_ze03_collect(io, addr, &v);
    if( ret == POS_STATUS_OK ) {
      pos_u8_t pl, plss_precision, json_precision,i;
      static const struct {
        const char json[6];
        pos_u8_t ze03_pl;
        pos_u8_t plss_pl;
      } ctrl[] = {
        {"ch4", 0x01, PLSS_PL_EXT_CH4, }, /* 01 - PLSS_PL_EXT_CH4, 2Bytes, 1 PPM, CH4 payload indicator is for ZC13 */
        {"nh3", 0x02, PLSS_PL_EXT_NH3, }, /* 21 - PLSS_PL_EXT_NH3, 2Bytes, 1 PPM */
        {"h2s", 0x03, PLSS_PL_EXT_H2S, }, /* 26 - PLSS_PL_EXT_H2S, 2Bytes, 1 PPM */          
        {"co",  0x04, PLSS_PL_EXT_CO,},   /* 76 - PLSS_PL_EXT_CO, 2Bytes, 1 PPM */        
        {"o2",  0x05, PLSS_PL_EXT_O2,},   /* 61 - PLSS_PL_EXT_O2, 2Bytes, 0.1% */
        {"h2",  0x06, PLSS_PL_EXT_H2,},   /* 25 - PLSS_PL_EXT_H2, 2Bytes, 1 PPM */
        {"so2", 0x2b, PLSS_PL_EXT_SO2, }, /* 33 - PLSS_PL_EXT_SO2, 2Bytes, 1 ug/m3 */
        {"no2", 0x2c, PLSS_PL_EXT_NO2, }, /* 30 - PLSS_PL_EXT_NO2, 2Bytes, 1 ug/m3 */
        {"hcl", 0x2e, PLSS_PL_EXT_HCL, }, /* 27 - PLSS_PL_EXT_HCL, 2Bytes, 1 PPM */                
        {"hcn", 0x2f, PLSS_PL_EXT_HCN, }, /* 28 - PLSS_PL_EXT_HCN, 2Bytes, 1 PPM */
        {"o3",  0x2a, PLSS_PL_EXT_O3,  }, /* 28 - PLSS_PL_EXT_HCN, 2Bytes, 1 ug/m3 */
      };
      pl = (v >> 16) & 0xff;
      plss_precision = (v >> 24) & 0xff;
      json_precision = plss_precision;
      v &= 0xffff;
      ret = POS_STATUS_E_NOT_FOUND;
      for( i = 0; i < POS_ARRAY_CNT(ctrl); i++ ) {
        if( ctrl[i].ze03_pl != pl )
          continue;
        if( ctrl[i].plss_pl == PLSS_PL_EXT_O3 ) {
          /* transate from 100ppb to ug/m3 */
          plss_precision = 0;
          json_precision = 0;
          if( v > 200 )
            v = 200; /* max 20.0ppm */
          v = v * 48000 / 224;
        } else if( ctrl[i].plss_pl == PLSS_PL_EXT_O2 ) {
            /* force PLSS to keep original 0.1% unit */
        }
        ret = drv->data->plss_json_put(v, 
          DRV_MA_PLSS_TRANS(ctrl[i].plss_pl, 2, plss_precision, 0, json_precision), 
          ctrl[i].json);
        break;
      }
    }
    break;
  }

  case MA_SENSOR_ULTRA_MBUS: {
    pos_u32_t l, v2, j;
    pos_u8_t buf[16];
    v2 = drv->history->mv[4]; /* restore from history */
    for( j = 0; j < 3; j++ ) {
      l = drv->data->modbus_read(io, buf, addr, 3, 0x0100, 1);
      if( l < 5 + 2 ) {
        v = 0; /* wrong read, use shortest values */
      } else {
        v = (buf[3]<<8) + (buf[4]);
        if( !v )
          v = 65533; /* sensor will reort 0 if length is too far, in this case, we return 65533 */
      }
      if( v + 5 >= v2 && v <= v2 + 5 ) /* tolerant for 5mm delta */
        break;
      v2 = v;
      drv->os->tick->sleep(120); /* 120ms guard time */
    }
    drv->history->mv[4] = v; /* record v in history */
        
    /* thold check */
    if( v > 65534 )
      v = 65534; /* at most to be 65534, so that 65535 thold can work for periodic report */    
    drv->thld->u16_process(drv->s, v);
    
    /* signal light processing when both d[0] defined */
    if( drv->s->slot->arg.u32[0]) {
      pos_u8_t red, green, res;
      /* LED LIGHT Control */
      if( drv->s->slot->arg.u16[1] ) {
        red = drv->s->slot->arg.u8[3]; /* hi8 is RED */
        green = drv->s->slot->arg.u8[2];; /* lo8 is GREEN */
      } else {
        red = POS_PB3;
        green = POS_PB4;
      }
      drv->os->gpio->mode_set(red, POS_GPIO_MODE_OUTPUT);
      drv->os->gpio->mode_set(green, POS_GPIO_MODE_OUTPUT);
      if( v > drv->s->slot->arg.u16[0] ) {
        /* over distance: no car in slot, green signal */
        drv->os->gpio->set(red, 0);
        drv->os->gpio->set(green, 1);
        res = 2;
      } else {
        /* short distance: car in slot, red signal */      
        drv->os->gpio->set(red, 1);
        drv->os->gpio->set(green, 0);
        res = 3;
      }
      if( res != drv->history->rsvd[0] ) {        
        /* report and renew periodic */
        if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {      
          drv->log->data("um res", res);
          drv->log->data("um dis", v);        
        }
        drv->thld->u16_process(drv->s, 65535); /* force to raise report */
      }
      drv->history->rsvd[0] = res; /* save state */

    }

    if( (drv->s->ctrl & MA_SENSOR_CTRL_REPORT) == 0 ) {
      ret = POS_STATUS_OK;
      break; /* do NOT report if not allowed */    
    }
    
    ret = drv->data->plss_json_put(v, 
      DRV_MA_PLSS_TRANS(PLSS_PL_US_DISTANCE, 2, 0, 0, 0), /* 38 - PLSS_PL_US_DISTANCE, 2Bytes, 1 mm */ 
      "distance" );
    break; 
  }

  case MA_SENSOR_TEMP_485: {
    pos_u8_t buf[16];
    pos_u32_t num, l, i;
    num = drv->s->slot->arg.u8[0];
    if( num > 8 )
      num  = 8;
    for( i = 0; i < num; i++ ) {
      union {
        pos_u16_t u16[2];
        pos_i16_t i16[2];
      } u;
      l = drv->data->modbus_read(io, buf, addr+i, 3, 0, 2);
      if( l < 5 + 2 * 2 ) {
        /* wrong read */
        u.u16[0] = 0; /* hum */
        u.i16[1] = -9990; /* tmp */
      } else {
        u.u16[0] = (buf[3] << 8) + buf[4];  /* hum */
        u.u16[1] = (buf[5] << 8) + buf[6];  /* tmp */
      }
      drv->history->temp[i] = u.i16[1];
      drv->history->temp[i+8] = u.u16[0];
    }
    /* Array start */
    ret = POS_STATUS_OK;
    drv->data->put_raw("\"etmp\":[", 8);
    for( i = 0; i < num; i++ ) {
      drv->data->put_float(drv->history->temp[i], 10);
      drv->data->plss_put_u16(PLSS_PL_EXT_TEMPERATURE, (pos_u16_t)drv->history->temp[i]); /* 14 - PLSS_PL_EXT_TEMPERATURE, 2Bytes, 0.1degree */
    }
    drv->data->put_array_stop();
    drv->data->put_raw("\"ehum\":[", 8);
    for( i = 0; i < num; i++ ) {
      drv->data->put_float(drv->history->temp[i+8], 10);
      drv->data->plss_put_u16(PLSS_PL_HUMIDITY_HR, (pos_u16_t)drv->history->temp[i+8]); /* 80 - PLSS_PL_HUMIDITY_HR, 2Bytes, 0.1% */ 
    }
    drv->data->put_array_stop();
    break;
  }   

  case MA_SENSOR_LASER_DISTANCE: {
    ret = sensor_laser_distance_collect(io, &v);
    if( ret == POS_STATUS_OK ) {
      ret = drv->data->plss_json_put(v, 
          DRV_MA_PLSS_TRANS(PLSS_PL_US_DISTANCE, 2, 0, 0, 0), 
          "distance");
        break;
    }
    break;
  }

  case MA_SENSOR_ZPHS_MULTI: {
    v = drv->s->slot->arg.u32[0] & 0xffff;  
    if( !v )
      v = 0xff;
    ret = sensor_zphs_multi_report(io, v);
    break;
  }

  case MA_SENSOR_DGM10_MULTI: {
    v = drv->s->slot->arg.u32[0] & 0xffff;  
    if( !v )
      v = 0xff;
    ret = sensor_dgm10_multi_report(io, addr, v);
    break;
  }

  case MA_SENSOR_WIND_SPEED: {
    pos_u32_t l;
    pos_u8_t buf[16];
    l = drv->data->modbus_read(io, buf, addr, 3, 0, 2);
    if( l < 5 + 2 * 2 )
      break; /* wrong read */
    v = (buf[3]<<8) + buf[4];
    v *= 100; /* translate from 0.1m to 0.001m */
    if( v > 65535 )
      v = 65535;
    ret = drv->data->plss_json_put(v, 
      DRV_MA_PLSS_TRANS(PLSS_PL_WIND_SPEED, 2, 0, 0, 3), 
      "wind" );
    break;
  }

  case MA_SENSOR_WIND_DIR: {
    pos_u32_t l;
    pos_u8_t buf[16];
    l = drv->data->modbus_read(io, buf, addr, 3, 0, 2);
    if( l < 5 + 2 * 2 )
      break; /* wrong read */
    #if 0
    v = (buf[5]<<8) + buf[6]; /* buf[3/4] is wind angle in unit of 0~3599 in unit 0.1degreee */
                              /* buf[5/6] is wind dir 0~15 */
    if( v > 15 )
      v = 15;
    ret = drv->data->plss_json_put(v,
      DRV_MA_PLSS_TRANS(PLSS_PL_WIND_DIR, 1, 0, 0, 0), 
      "dir" );
    #else
    v = (buf[3]<<8) + buf[4]; /* buf[3/4] is wind angle in unit of 0~3599 in unit 0.1degreee */
    if( v > 3599 )
      v = 3599;
    v *= 10; /* translate to 0.01 degree */
    ret = drv->data->plss_json_put(v,
      DRV_MA_PLSS_TRANS(PLSS_PL_EXT_CNTR, 4, 0, 0, 2),
      "dir" );

    #endif
    
    break;
  }    

  case MA_SENSOR_FLOW: {
    pos_u32_t l;
    pos_u8_t buf[16];
    l = drv->data->modbus_read(io, buf, addr, 3, 1, 2);
    if( l < 5 + 2 * 2 )
      break; /* wrong read */
    v = drv->os->util->get_u32(&buf[3], 4);
    v = POS_NTOHL(v);    
    v = drv->os->util->ufloat_to_u32_precision(v, 1000);
    ret = drv->data->plss_json_put(v,
      DRV_MA_PLSS_TRANS(PLSS_PL40_VELOCITY_FLOW, 4, 0, 0, 3), 
      "flow" );
    break;
  }    

  case MA_SENSOR_DO: {
    pos_u32_t l;
    pos_u8_t buf[16];
    l = drv->data->modbus_read(io, buf, addr, 3, 0x2600, 6);
    if( l < 5 + 6 * 2 )
      break; /* wrong read */
    v = drv->os->util->get_u32(&buf[11], 4);
    /* v = POS_NTOHL(v);   THIS IS ALREADY LITTLE ENDIAN */
    drv->data->plss_put_u32(PLSS_PL_DO_MG_F, v);
    v = drv->os->util->ufloat_to_u32_precision(v, 1000);
    ret = drv->data->plss_json_put(v,
      DRV_MA_PLSS_TRANS(PLSS_PL_DO_MG_F, 0, 0, 0, 3), /* skip plss pl by using plss_len=0, as it's done previously */
      "do" );
    break;
  }  

  case MA_SENSOR_LIGHT: {
    pos_u32_t l;
    pos_u8_t buf[16];
    l = drv->data->modbus_read(io, buf, addr, 3, 2, 2);
    if( l < 5 + 2 * 2 )
      break; /* wrong read */
    v = drv->os->util->get_u32(&buf[3],4);
    v = POS_NTOHL(v);    
    ret = drv->data->plss_json_put(v,
      DRV_MA_PLSS_TRANS(PLSS_PL_LIGHT, 2, 3, 0, 3), 
      "light" );
    break;
  }   

  case MA_SENSOR_ZCE04: {
    pos_u32_t va[4];
    ret = sensor_zce04_collect(io, va);
    if( ret != POS_STATUS_OK )
      break;
    drv->data->plss_json_put(va[0], DRV_MA_PLSS_TRANS(PLSS_PL_CO,2,0,0,0), "co");
    if( va[1] > 65 )
      va[1] = 65530;
    else
      va[1] *= 1000; /* translate from H2S PPM to PPB */
    drv->data->plss_json_put(va[1], DRV_MA_PLSS_TRANS(PLSS_PL_EXT_H2S,2,0,0,0), "h2s");
    drv->data->plss_json_put(va[2], DRV_MA_PLSS_TRANS(PLSS_PL_EXT_O2,2,0,0,1), "o2");
    if( va[3] > 131 )
      va[3] = 65530;
    else
      va[3] *= 500; /* translate from %LEL to PPM (100% LEL of CH4=5%=50000PPM) */    
    drv->data->plss_json_put(va[3], DRV_MA_PLSS_TRANS(PLSS_PL_EXT_CH4,2,0,0,0), "ch4");
    break;
  }   
  
  default:
    break;
  }
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

/** 
 * Driver init
 * @param[in]  load   0:Unload driver, 1:Load driver
 * @return     0: Successful\n
             !=0: Failed, refer to @ref pos_status_t
 * @note This function can be NULL if it does NOT require any init operatoin
 */ 
pos_status_t sensor_modbus_init(pos_u32_t load) {
  drv_api_t *drv = g_drv;
  smbus_param_t *param;
  ma_sensor_ctrl_t *s = g_drv->s;
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("mbus init", load);
  }  
  
  if( load ) {
    param = drv->os->malloc(sizeof(*param));
    if( !param ) {
      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
        drv->log->data("co2ns mem error", sizeof(*param)); 
      }        
      return POS_STATUS_E_MEM;
    }
    drv->os->memset(param, 0, sizeof(*param));
    sensor_modbus_data_clear_u(param);
    s->drv_buf = param;  /* record to drv_buf */    

  } else if( s->drv_buf ) { 
    /* Free mem */
    drv->os->free(drv->s->drv_buf);
    s->drv_buf = POS_NULL;
  }
  return POS_STATUS_OK;
}

/*
 * Driver export
 */
ma_drv_export_t g_export = {
  .version = DRV_SENSOR_VERSION(MA_SENSOR_PH, DRV_MODBUS_VERSION),
  .name = DRV_MODBUS_NAME,
  .u.sensor={
    .init = sensor_modbus_init,
    .power_set = sensor_modbus_set,
    .collect = sensor_modbus_collect,
  },
};

