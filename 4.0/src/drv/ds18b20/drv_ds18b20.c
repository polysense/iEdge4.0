/**
 * @file  drv_ds18b20.c
 * @brief SENSOR DS18B20 driver
 * @author Runby F.
 * @date 2022-3-09
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * 版本修订历史
v1.0 @ 2022-03-10
  1) 完成第一版驱动

v1.1 @ 2022-03-15
  1) 支持RT-Thread

v1.2 @ 2022-03-23
  1) 支持可配置单线映射和供电控制

v1.3 @ 2022-03-25
  1) 支持4.0整体供电策略

v1.4 @ 2022-04-01
  1) 支持自动固化DS18B20 ROM/NUM
  2) 支持温度数据汇总到#0历史条目 

v1.5 @ 2022-04-15
  1) 解决不正常时ROM ID全零问题
  2) 支持上电采集调试打印PIN脚

v1.6 @ 2022-04-18
  1) 支持PULLUP/PULLDN超级单总线

v1.7 @ 2022-04-20
  1) 优化当传感器异常时的多次相同调试打印信息

v1.8 @ 2022-05-14
  1) 支持内核统一管脚策略

v1.9 @ 2022-06-06
  1) 使用非OD模式单线输出优化输出能力

v1.10 @ 2022-11-14
  1) Support PLSS UDP data format

v1.11 @ 2022-12-09
  1) Support set w1 as ANA_INPUT after finishing collecting

v1.12 @ 2023-06-14
  1) Change json tmp to etmp

v1.13 @ 2023-10-25
  1) Support stat normal/error
  2) Print num = 0 whene eorror

v1.14 @ 2023-11-18
  1) Using new PWR ID solution
  
v1.15 @ 2024-05-15
  1) Fix muti ds18b20 json put format issue

v1.16 @ 2024-06-05
  1) Support DBG functions

v1.17 @ 2024-06-13
  1) Support more DBG for original value data

v1.18 @ 2024-06-18
  1) Support DS18B20 OD mode

v1.19 @ 2025-02-19
  1) Use default error temp as -999.0

*/

/**
 * @brief Driver version
 */
#define DRV_DS18B20_VERSION 0x0113

/**
 * @brief Driver name
 */
#define DRV_DS18B20_NAME  "DS18B20"

/**
 * @brief Max DS18B20 number
 */
#define DS18B20_MAX 8


/**
 * @brief Global structure
 * @note Driver must NOT use local static variable. All variables must be declared in sensor control structure.
 */
typedef struct {
  /**
   * @brief DS18B20 ROM ID buffer
   */
  pos_u64_t ds18b20_rom[DS18B20_MAX];

  /**
   * @brief DS18B20 sensor number
   */
  pos_u8_t ds18b20_num;

} ds18b20_data_t;

/**
 * @brief Sort and insert
 * @param[in] array To be inserted array
 * @param[in] array_num Number of U64 data blocks
 * @param[in] v The new U64 data block to be inserteed
 * @return Insert position ID
 */  
pos_u8_t sort_insert_u64(pos_u64_t *array, pos_u8_t array_num, pos_u64_t v) {
  pos_u8_t i,j;
  for( i = 0; i < array_num; i++ ) {
    if( v <= array[i] ) {
      for( j = array_num; j > i; j-- )
        array[j] = array[j-1];
      break;
    }
  }
  array[i] = v;
  return i;
}

/**
 * @brief Search DS18B20 and save ROM ID to buffer
 * @param[in] h Single wire handle
 * @param[in] rom_addr ROM ID buffer
 * @param[in] rom_addr_num Max ROM ID num
 * @return Actual searched DS18B20 number
 */  
pos_u8_t sensor_ds18b20_search(pos_wire1_handle_t h, pos_u64_t *rom_addr, pos_u8_t rom_addr_num) {
  pos_u8_t i, n, k, b, chk;
  pos_u64_t conflict = 0LL, t, v = 0LL;
  const pos_lib_wire1_t *wire1 = g_drv->os->wire1;

  for( n = 0; n < rom_addr_num; n++ ) {
    wire1->reset(h);

    chk = wire1->check(h);
#if 0 /* DBG */
    g_drv->log->data("ds18b20 chk", chk);
    while(1) {
      g_drv->os->tick->sleep(1000);
    }
#endif
#ifdef DS18B20_DBG
    g_drv->log->data("ds18b20 chk", chk);
#endif

    wire1->write_byte(h, 0xf0);
    k = 0;
    for( i = 0; i < 64; i++ ) {
      k = wire1->read_bit(h);
      k |= (wire1->read_bit(h) <<1)&0x2;
      if( k == 3 ) {
        break;
      }
      b = 0;
      t = 1LL << i;
      if( k == 0 ) {
        if( t <= conflict ) {
          if( t + (t-1)  >= conflict ) {
            b = 1;
            conflict &= ~t; /* pop, if equal */
          } else {
            b = (v >> i)&0x1; /* nop, if smaller */
          }
        } else  {
          conflict |= t; /* push, if bigger */
        }
      } else if( k == 1 ) {
        b = 1;
      }
      if( b )
        v |= t;
      else
        v &= ~t;
      wire1->write_bit(h, b);
    }

#ifdef DS18B20_DBG
    g_drv->log->data("ds18b20 k", k);
#endif

    if( k == 3 ) {
      if( g_drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
        g_drv->log->data("ds18b20 chk3", chk);        
      }
      break; /* no device */
    }

    /* quit if search failed */
    if( g_drv->os->crc(POS_CRC8R, (pos_u8_t*)&v, 7) != ((v >>56)&0xff) )
      break;

#ifdef DS18B20_DBG
    g_drv->log->buf("ds18b20 rom", &v, 8); 
#endif

    /* only unzero rom is accepted */
    if( !v )
      break;

    sort_insert_u64(rom_addr, n, v);
    //rom_addr[n] = v;
    if( !conflict ) {
      n++; /* this is still valid and needs to increase the counter */
      break; /* no more device */
    }
  }
  return n;
}

/**
 * @brief Remap DS18B20 ROM position by report ID
 */
pos_u8_t sensor_ds18b20_remap(pos_u8_t report_id) {
  volatile pos_u8_t *remap;
  pos_u8_t i;
  ds18b20_data_t *data = (ds18b20_data_t *)g_drv->s->drv_buf;
  remap = (volatile pos_u8_t*)g_drv->cfg->ds18b20_pos;
  for( i = 0; i < data->ds18b20_num; i++ ) {
    if( remap[i] == report_id ) {
      report_id = i; /* remap to new rom id location */
      break;
    }
  }
  return report_id; /* when search failed,use rom id as report_id */
}

unsigned char sensor_ds18b20_crc8_r2(unsigned char *serial, unsigned char length) 
{
    unsigned char result = 0x00;
    unsigned char pDataBuf;
    unsigned char i;

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
    return result;  //返回校验和
}

/**
 * @brief Read DS18B20 temperature by report ID
 */
pos_i16_t sensor_ds18b20_read(pos_wire1_handle_t h, pos_u8_t report_id) {
  pos_u8_t buf[10],j, i, rom_id, c;
  union {
    pos_i16_t   i16;
    pos_i16_t  u16;
  } u;
//  pos_u64_t *rom = (pos_u64_t*)g_buf.shared.u8;
  pos_u8_t *rom8;
  drv_api_t *drv = g_drv;
  ds18b20_data_t *data = (ds18b20_data_t *)drv->s->drv_buf;
  const pos_lib_wire1_t *wire1 = drv->os->wire1;

  /* remap rom id */
  rom_id = sensor_ds18b20_remap(report_id);
  rom8 = (pos_u8_t*)&data->ds18b20_rom[rom_id];
  u.i16 = -9990; /* report -999.0 degree when reading is wrong */

  /* read temp data */
  for( j = 0; j < 4; j++ ) { /* retry at most 4 */    
    if( j == 0 && (drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG) != 0 ) {
      drv->log->buf("ds18b20 rom", rom8, 8);
    }
    wire1->reset(h);
    i = wire1->check(h);
    if( i == 0 ) {
      wire1->write_byte(h, 0x55);// select rom
      for( i = 0; i < 8; i++ ) {
        wire1->write_byte(h, rom8[i]);// write rom address
      }
      wire1->write_byte(h, 0xbe);// select rom id 

      /* read 9 bytes full content */
      for( i = 0; i < 9; i++ ) {
        buf[i]=wire1->read_byte(h); 
      }
      
      /* check crc8r */
      c = sensor_ds18b20_crc8_r2(buf,8);//drv->os->crc(POS_CRC8R, buf, 8);
      buf[9] = c;
      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
        drv->log->buf("ds18b20 rd", buf, 10);
      }      
      if( buf[8] == c ) {
        u.u16 = ( buf[1] << 8 ) + buf[0]; /* record real data */
        u.i16  = u.i16 * 10 / 16; /* translate unit 0.0625 to 0.1 (10/16=0.625) */
        break; /* quit if CRC is matched */
      }
    } else {
      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
        drv->log->data("ds18b20 chk/rd", i);
      }
    }
  } 
  
  return u.i16;
}

/** 
* Power control
* @param[in]  on   0:Power off, 1:Power on
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
* @note If sensor does NOT require any power control, this fucntion can be NULL
*/ 
pos_status_t sensor_ds18b20_set(pos_u32_t on) {  
  pos_gpio_pin_t pwr, w1;
  drv_api_t *drv = g_drv;  
  const pos_lib_gpio_t *gpio = g_drv->os->gpio;
  ma_board_t *b = drv->board;
  w1 = drv->s->slot->io;
  pwr = (w1 >> 8) & 0x7f;
  w1 &= 0x80ff;
  if( !pwr )
    pwr = MB_PWR_ID_VCCN_WIRE1;
  w1 = b->pin_map(MB_PT_WIRE1, w1);
  
  if( on ) {
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      drv->log->data("ds18b20 on", w1);
    }  
    
    gpio->set(w1, 1);  
    gpio->mode_set(w1, POS_GPIO_MODE_OUTPUT+POS_GPIO_MODE_OD_ENABLE);        
    
  } else {
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      drv->log->data("ds18b20 off", w1);
    }  
  
    gpio->set(w1, 0);  
    gpio->mode_set(w1, POS_GPIO_MODE_ANALOG+POS_GPIO_MODE_INPUT);      
    
  }

  /* No matter what power state is, always set PULLUP/DN = 1/0 for idle state
   * Kernel single wire driver will choose the correct PULLUP/DN state
   */
  /* POS kernel will control PULLUP/DN automatically */ 

  /* Power ON/OFF
   */
  b->pwr_set(pwr, on);

  return POS_STATUS_OK;
}

/** 
* Sensor collecting
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_ds18b20_collect(void){
  pos_u32_t n, i;
  pos_status_t ret;
  pos_wire1_handle_t h;
  drv_api_t *drv = g_drv;
  ds18b20_data_t *data = (ds18b20_data_t *)drv->s->drv_buf;
  const pos_lib_wire1_t *wire1 = drv->os->wire1;
  pos_gpio_pin_t w1;
  w1  = drv->board->pin_map(MB_PT_WIRE1, drv->s->slot->io);    
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("ds18b20 pin", w1);
  }  
  
  /* refresh to normal by default */
  drv->s->ctrl |= MA_SENSOR_CTRL_STAT_NORMAL;
  drv->s->ctrl &= ~MA_SENSOR_CTRL_STAT_ERROR;

  h = wire1->alloc(w1, drv->os->pin->pull[0], drv->os->pin->pull[1] );
  if( !h ) {
    drv->s->ctrl |= MA_SENSOR_CTRL_STAT_ERROR;
    return POS_STATUS_E_NOT_SUPPORT;
  }
  

  /* If ROM ID is saved already, use the saved ROM ID */
  n = drv->cfg->ds18b20_num;
  if( n > DS18B20_MAX )
    n = DS18B20_MAX;
  if( n ) {
    drv->os->memcpy(data->ds18b20_rom, drv->cfg->ds18b20, n*8);
    data->ds18b20_num = n;
  } else {
  /* If none ROM ID saved, search DS18B20 ROM */
    if( !data->ds18b20_num ) {
      for( i = 0; i < 3; i++ ) {
        data->ds18b20_num = sensor_ds18b20_search(h, data->ds18b20_rom, DS18B20_MAX);
        if( data->ds18b20_num ) {
          break;
        }
      }
    }
    n = data->ds18b20_num;
    /* Save ROM ID*/
    if( n ) {
      ma_eeprom_cfg_t *cfg;
      do {
        /* Memory allocation */
        cfg = (ma_eeprom_cfg_t*)drv->os->malloc(sizeof(*cfg)); 

        /* Print error and quit */
        if( !cfg ) {
          drv->log->data("ds18b20 err mem", sizeof(*cfg)); 
          break;
        }

        /* Copy current cfg */
        drv->os->memcpy(cfg, drv->cfg, sizeof(*cfg));

        /* Update new cfg */
        cfg->ds18b20_num = n;
        drv->os->memcpy(&cfg->ds18b20[0].rom_id[0], &data->ds18b20_rom[0], n*sizeof(data->ds18b20_rom[0]));

        /* Program new cfg */
        drv->eeprom->update(0, cfg, sizeof(*cfg));

        /* Memory release */
        drv->os->free(cfg); 
      } while(0);
      
    }
  }

  /* Error return */
  if( !n )  {
    /* Update history data number to zero */
    drv->history->temp_num = 0;
    
    /* Release handle and then quit */    
    wire1->free(h);
    /* Put error info */    
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      drv->log->data("ds18b20 num", 0);
    }      
    drv->data->put("\"%s\":\"no 18b20#%u\",", "err", g_drv->s->slot->io, 0,0,0);
    drv->s->ctrl |= MA_SENSOR_CTRL_STAT_ERROR;
    return POS_STATUS_E_RESOURCE;
  }

  /* Array start */
  ret = drv->data->put_raw("\"etmp\":[", 8);
  if( ret == POS_STATUS_OK ) {
    /* start to convert */
    wire1->reset(h);
    wire1->check(h);
    wire1->write_byte(h, 0xcc);// skip rom
    wire1->write_byte(h, 0x44);// convert
    
    /* delay */
    drv->os->tick->sleep((750+30)); // delay 750ms+30ms tolerance to let convert finished
    
    /* Loop all sensors */
    for( i = 0; i < n; i++ ) {
      pos_i32_t t;
      pos_i16_t t16;
  
      /* Read temperature */
      t = sensor_ds18b20_read(h, i);

      /* Update history data */
      drv->history->temp[i] = t;
      
      /* Enable searching when reading failed (if it's already saved in EEPROM, reload the mapping next time) */
      if( t == 32767 )
        data->ds18b20_num = 0;
      
      /* Report */
      t16 = t;
      drv->data->plss_put_u16(PLSS_PL_EXT_TEMPERATURE, (pos_u16_t)t16);
      ret = drv->data->put_float(t, 10 );
      if( ret != POS_STATUS_OK ) {    
        drv->s->ctrl |= MA_SENSOR_CTRL_TOO_LONG; /* indicate data buffer put failed */      
        break;
      }
    }
    /* Array finished */
    ret = drv->data->put_array_stop();
  }

  /* Update history data num */
  drv->history->temp_num = n;
  
  /* Release handle and return */
  wire1->free(h);  

  /* update error flag */
  if( ret != POS_STATUS_OK )
    drv->s->ctrl |= MA_SENSOR_CTRL_STAT_ERROR;
  return ret;
  
}

/** 
 * Driver init
 * @param[in]  load   0:Unload driver, 1:Load driver
 * @return     0: Successful\n
             !=0: Failed, refer to @ref pos_status_t
 * @note It can be NULL function here, when sensor does NOT require any init operation
 */ 
pos_status_t sensor_ds18b20_init(pos_u32_t load) {
  drv_api_t *drv = g_drv;
  if( load ) {
    drv->s->drv_buf = drv->os->malloc(sizeof(ds18b20_data_t));
    if( !drv->s->drv_buf )
      return POS_STATUS_E_MEM;
    drv->os->memset( g_drv->s->drv_buf, 0, sizeof(ds18b20_data_t));
  } else {
    if( drv->s->drv_buf )
      drv->os->free(g_drv->s->drv_buf);
    drv->s->drv_buf = POS_NULL;
  }
  return POS_STATUS_OK;
}

/*
 * Driver export
 */
ma_drv_export_t g_export = {
  .version = DRV_SENSOR_VERSION(MA_SENSOR_DS18B20, DRV_DS18B20_VERSION),
  .name = DRV_DS18B20_NAME,
  .u.sensor={
    .init = sensor_ds18b20_init,
    .power_set = sensor_ds18b20_set, 
    .collect = sensor_ds18b20_collect,
  },
};

