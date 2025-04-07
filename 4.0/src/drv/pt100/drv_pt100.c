/**
 * @file  drv_pt100.c
 * @brief SENSOR ANALOG Driver
 * @author Runby F.
 * @date 2022-3-22
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * 版本修订历史
v1.0 @ 2022-03-22
  1) 完成第一版驱动

v1.1 @ 2022-03-23
  1) 支持统一IO管脚映射管理
  2) 支持外部供电控制
  3) 使用OS乘除法优化编译大小

v1.2 @ 2022-03-25
  1) 采用4.0的PWR供电策略
  2) 支持采样数字滤波

v1.3 @ 2022-03-29
  1) 去除pt#数字只保留pt名称

v1.4 @ 2023-02-10
  1) Optimize from ANA to ANA_INPUT

v1.5 @ 2023-04-14
  1) Support ANALOG as base for multile analog sensors
  2) Support UV power sensor
  3) Support MV sensor

v1.6 @ 2023-04-17
  1) Support arg.u32[0] (set d0[n] xxx) for multi-ratio (UV: 20151 default, mv: 10000 default)

v1.7 @ 2023-05-06
  1) Support new 3000 bvat, 100ohm pt100 calc
  2) Support zero delay for pt100 power on/off

v1.8 @ 2023-05-23
  1) Support type-33 (mv) srsv[]=1 for current report

v1.9 @ 2023-07-08
  1) Support zero thld for type-32/33 (UV/current(mv))

v1.10 @ 2023-07-14
  1) Support pt100 d0[0..15] value calibration

v1.11 @ 2023-07-17
  1) Support Water pressure sensor

v1.12 @ 2023-07-24
  1) Support to report original mv

v1.13 @ 2023-08-02
  1) Support analog wind speed - 35
  2) Support analog wind speed - 36
  3) Support analog displacement - 37

v1.14 @ 2023-08-07
  1) Change JSON disp format to unit mm
  2) Change JSON wind format to unit m/s
  3) Support ADC max 200ms for current sampling

v1.15 @ 2023-10-24
  1) Support stat/normal/error

v1.16 @ 2023-11-18
  1) Using new PWR ID solution

v1.17 @ 2024-05-16
  1) Support history threshold report for type - 33

v1.18 @ 2024-07-01
  1) Support history threshold report for type - 33 when mv >=thw[3] || mv < thw[4]

v1.19 @ 2024-07-29
  1) Modify history threshold report for type - 33 when mv >=thw[3] || mv <= thw[4]
  
*/

/**
 * @brief Driver version
 */
#define DRV_ANALOG_VERSION 0x0113

/**
 * @brief Driver name
 */
#define DRV_ANALOG_NAME  "ANALOG"

/**
 * @brief MAX history data num
 */
#define DRV_ANALOG_HISTORY_MAX 32

/**
 * @brief IRQ控制结构
 */
typedef struct {
  drv_api_t     drv;      /**< 驱动克隆，指向固定SLOT的驱动指针 */
  pos_u8_t      history_num; /**< number of history */
  pos_u8_t      rsvd[3]; /**< reserved */
  pos_u16_t     history[DRV_ANALOG_HISTORY_MAX]; /**< max history */
} analog_param_t;

pos_u16_t math_calc_array_data(pos_u16_t data_mv, pos_u8_t num, pos_u16_t *v_mv, pos_u16_t *data) {
  pos_u8_t i;
  pos_u32_t data_calc;
  if( data_mv <= v_mv[0] )
    data_calc = data[0];
  else if( data_mv >= v_mv[num-1] )
    data_calc = data[num-1];
  else {
    for( i = 1; i < num-1; i++ ) {
      if( data_mv <= v_mv[i] )
        break;
    }
    data_calc = (data[i] - data[i-1]);
    data_calc = g_drv->os->util->umd(data_calc, data_mv - v_mv[i-1], v_mv[i] - v_mv[i-1]);
    data_calc += data[i-1];
  }
  return data_calc;
}

void _plss_driver_ext_analog_temp_read(pos_u16_t mv,  pos_i32_t *data ) {
  pos_u32_t rt;
/* r_analog = (battery_voltage - analog_mv) / (analog_mv/100 */
#define PT_TEMP(celsius)      (1000+celsius)  
  const static pos_u16_t rt_100[] = {
      6147,          6551,          6953,          7354,  
      7752,          8150,          8546,          8941,
      9334,          9726,          9922,          10117,
      10312,         10507,         10702,         10896,
      11090,         11283,         11477,         11670,
      11863,         12055,         12247,         12439,
      12822,         13204,         13585,         13964,
      14342,         14720,         15470,         16588,
      17695,         19518,         23076,         26518,
    };
  const static pos_u16_t rt_temp[] = {
      PT_TEMP(-1000),PT_TEMP(-900), PT_TEMP(-800), PT_TEMP(-700), 
      PT_TEMP(-600), PT_TEMP(-500), PT_TEMP(-400), PT_TEMP(-300), 
      PT_TEMP(-200), PT_TEMP(-100), PT_TEMP(-50),  PT_TEMP(0),
      PT_TEMP(50),   PT_TEMP(100),  PT_TEMP(150),  PT_TEMP(200),
      PT_TEMP(250),  PT_TEMP(300),  PT_TEMP(350),  PT_TEMP(400),
      PT_TEMP(450),  PT_TEMP(500),  PT_TEMP(550),  PT_TEMP(600),
      PT_TEMP(700),  PT_TEMP(800),  PT_TEMP(900),  PT_TEMP(1000),
      PT_TEMP(1100), PT_TEMP(1200), PT_TEMP(1400), PT_TEMP(1700),
      PT_TEMP(2000), PT_TEMP(2500), PT_TEMP(3500), PT_TEMP(4500),
    };
  rt =  2500;  


/*
 * according to jason@20230506
 * battery_volage = 3000 for 4.0 PT100
 * ==> pt100 * 100 (unit 0.01) = 100 * 100 * (3000-mv) / mv;
 */
  if( mv == 0 )
    mv = 1;
  else if( mv > 3000 )
    mv = 2999;
  rt = 100 * 100 * (3000-mv) / mv;
  if( rt > 65535 )
    rt = 65535;

  if( g_drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    g_drv->log->data("rt", rt); 
  }

  *data = math_calc_array_data(rt, POS_ARRAY_CNT(rt_100), (pos_u16_t*)rt_100, (pos_u16_t*)rt_temp);
  *data -= 1000;
}

/** 
* Power control
* @param[in]  on   0:Power off, 1:Power on
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
* @note If sensor does NOT require any power control, this fucntion can be NULL
*/ 
pos_status_t sensor_analog_set(pos_u32_t on) {  
  pos_gpio_pin_t pwr;
  ma_board_t *b = g_drv->board;
  pos_u16_t io;
  io = g_drv->s->slot->io;
  pwr = (io >> 8) & 0x7f; 
  io &= 0xff;  
  if( !pwr ) {
      pwr = MB_PWR_ID_VCCN_PWR3;
  }

  /* according to jason@20230506: do NOT long power control for PT100 and only turn it on during collecting */
  if( g_drv->s->slot->sensor_type != MA_SENSOR_PT100 )
    b->pwr_set(pwr, on);
  
  /* 需要打开AIN模拟开关 */
  b->pwr_set(MB_PWR_ID_VCCN_AIN, on);  
  
  return POS_STATUS_OK;
}

/**
* Threshold check and history report
* @param[in]  mv   Data
* @return     0: Already processed in history mode (either suppressed or reported)\n
            !=0: Still need to process (none history/thld mode)
*/
pos_status_t sensor_analog_thld_history_report(pos_u16_t mv, pos_u8_t plss_pl, char *json_field, pos_u32_t json_divide) {
  drv_api_t *drv = g_drv;
  pos_u16_t ratio, low;
  ratio = drv->s->slot->thld.u16[3]; /* u16[0/1/2] is used for zero filter, u16[3] for upper thold */
  low = drv->s->slot->thld.u16[4]; /* u16[4] for lower thold, report when mv <= u16[4] || mv >= u16[3] */
  if( ratio || low ) {
    analog_param_t *param;
    param = (analog_param_t*)drv->s->drv_buf;    
    /* threshold check and report */
    if( param->history_num >= DRV_ANALOG_HISTORY_MAX )
      param->history_num = 0; /* anti wrong. usually, it should not reach here */
    param->history[param->history_num++] = mv;
    if( (ratio && mv >= ratio) || (low && mv <= low) || param->history_num >= DRV_ANALOG_HISTORY_MAX ) {
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
* Sensor collecting
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_analog_collect(void){
  pos_u32_t mv, ratio, uv;
  pos_u16_t io, stype;
  pos_status_t ret = POS_STATUS_OK;
  pos_i32_t v;
  drv_api_t *drv = g_drv;
  pos_gpio_pin_t ain;
  pos_u8_t ain_adc;
  io = drv->s->slot->io & 0x80ff;
  stype = drv->s->slot->sensor_type;
  ratio = drv->s->slot->arg.u32[0];
  if( !io ) {
    if( stype == MA_SENSOR_PT100 ) 
      io = MB_AIN_ID_PT100;
    else
      io = MB_AIN_ID_4;
  }

  /*
   * according to jason@20230506
   * pt100 should be power delay 500us + CFG POWER MS 
   */
  if( stype == MA_SENSOR_PT100 ) {    
    g_drv->board->pwr_set(MB_PWR_ID_VCCN_PT100, 1);
    drv->os->util->udelay(500);    

#if 0
    g_drv->board->pwr_set(MB_PWR_ID_VCCN_PT100, 0);
    drv->os->util->udelay(500);    
    g_drv->board->pwr_set(MB_PWR_ID_VCCN_PT100, 1);
    drv->os->util->udelay(500);    
#endif

    if( drv->s->slot->power_ms )
      drv->os->tick->sleep(drv->s->slot->power_ms);
  }
  
  ain = drv->board->pin_map(MB_PT_AIN, io);
  ain_adc = drv->board->pin_map(MB_PT_AIN_ADC, ain);
  drv->os->gpio->mode_set(ain, POS_GPIO_MODE_ANALOG+POS_GPIO_MODE_INPUT);

  /* refresh to normal by default */
  drv->s->ctrl |= MA_SENSOR_CTRL_STAT_NORMAL;
  drv->s->ctrl &= ~MA_SENSOR_CTRL_STAT_ERROR;

  /* currnet will sampling 200ms max */
  if( drv->s->slot->rsvd32 & 0x100 ) {
    pos_u32_t t, max;
    t = drv->s->slot->arg.u32[1];
    if( !t )
      t = 200;
    t += drv->os->tick->get();
    max = 0;
    do {
      mv = drv->os->adc->read_filter(ain_adc, 1, 64, 0); /* max filter */
      if( mv > max )
        max = mv;
    } while(!drv->os->tick->is_timeout(drv->os->tick->get(), t));
    mv = max;    
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      drv->log->data("adc ms", drv->s->slot->arg.u32[1]);
    }
  } else
    mv = drv->os->adc->read_filter(ain_adc, 0, 64, 0);

  /*
   * according to jason@20230506
   * pt100 should be power delay very short
   */
  if( stype == MA_SENSOR_PT100 )    
    g_drv->board->pwr_set(MB_PWR_ID_VCCN_PT100, 0);
  

  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("pin", ain);
    drv->log->data("mv", mv);
  }

  switch( stype ) {
  case MA_SENSOR_PT100:
    _plss_driver_ext_analog_temp_read(mv, &v); 

    v += drv->s->slot->arg.i16[0]; // support pt100 data calibration 
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      drv->log->data("temp", v); 
    }
    /* 抑制后直接退出 */
    drv->thld->i16_process(drv->s, v);
    if( (drv->s->ctrl & MA_SENSOR_CTRL_REPORT) == 0 )
      break;

    /* Report */
    ret = drv->data->plss_json_put(v,  
      DRV_MA_PLSS_TRANS(PLSS_PL_TEMPERATURE, 2, 0, 0, 1), /* 1 - PLSS_PL_TEMPERATURE, 2Bytes, 0.1degree */
      "pt");
#if 0
    drv->data->plss_put_u16(1, v); /* 1: TEMPERATURE */
    ret = drv->data->put("\"%s\":", "pt", 0, 0, 0, 0);  
    if( ret == POS_STATUS_OK )
      ret = drv->data->put_float(v, 10);
    if( ret != POS_STATUS_OK ) {
      drv->s->ctrl |= MA_SENSOR_CTRL_TOO_LONG; /* indicate data buffer put failed */
    }
#endif    
    break;

  case MA_SENSOR_UV:
    if( !ratio )
      ratio = 20151;
    uv = (mv * ratio+5000) / 10000;
    uv = drv->thld->zero_filter(drv->s, uv);
    ret = drv->data->plss_json_put(uv,  
      DRV_MA_PLSS_TRANS(PLSS_PL_UV_PWR, 2, 0, 0, 0), /* 79 - PLSS_PL_UV_PWR, 2Bytes, 1uw/cm2 */
      "uv");
    break;

  case MA_SENSOR_MV:
    uv = mv;
    if( ratio ) {
      mv = (mv * ratio+5000) / 10000;
    }    
    mv = drv->thld->zero_filter(drv->s, mv);

    /* history/threshold check&report */
    if( drv->s->slot->rsvd32 & 1 ) 
      ret = sensor_analog_thld_history_report(mv, PLSS_PL_EXT_CURRENT, "current", 100);
    else
      ret = sensor_analog_thld_history_report(mv, PLSS_PL_EXT_ANALOG, "mv", 1);
    if( ret == POS_STATUS_OK ) 
      break; /* quit if already processed */

    /* report original when 0x80 defined */
    if( drv->s->slot->rsvd32 & 0x80 ) {      
      ret = drv->data->plss_json_put(uv,  
        DRV_MA_PLSS_TRANS(PLSS_PL_EXT_ANALOG, 2, 0, 0, 0), 
        "mv");
    }
    
    if( drv->s->slot->rsvd32 & 1 ) {
      /* report current when rsvd bit[0] is 1 */
      ret = drv->data->plss_json_put(mv,  
        DRV_MA_PLSS_TRANS(PLSS_PL_EXT_CURRENT, 2, 0, 0, 2), /* 41 - PLSS_PL_EXT_CURRENT, 2Bytes, 10ma */
        "current");
      break;
    } 
    ret = drv->data->plss_json_put(mv,  
      DRV_MA_PLSS_TRANS(PLSS_PL_EXT_ANALOG, 2, 0, 0, 0), /* 11 - PLSS_PL_EXT_ANALOG, 2Bytes, 1mv */
      "mv");
    break;

  case MA_SENSOR_WATER:
    uv = (ratio >> 16); /* dead zone */
    if( !uv )
      uv = 100; /* 100mv deadzone */
    ratio &= 0xffff;
    if( !ratio )
      ratio = 1280;
    if( mv <= uv ) 
      mv = 0;
    else
      mv -= uv;
    uv = (mv * ratio + 500)/ 1000; /* round to kpa */
    uv = drv->thld->zero_filter(drv->s, uv);
    ret = drv->data->plss_json_put(uv,  
      DRV_MA_PLSS_TRANS(PLSS_PL_EXT_PSI, 2, 0, 0, 0), 
      "water");
    break;

    case MA_SENSOR_WIND_SPEED_ANALOG:
      uv = (ratio >> 16); /* dead zone */
      ratio &= 0xffff;
      if( !ratio )
        ratio = 15000;
      if( mv <= uv ) 
        mv = 0;
      else
        mv -= uv;
      uv = (mv * ratio + 500) / 1000; /* round to mm */
      uv = drv->thld->zero_filter(drv->s, uv);
      ret = drv->data->plss_json_put(uv,  
        DRV_MA_PLSS_TRANS(PLSS_PL_WIND_SPEED, 2, 0, 0, 3), 
        "wind");
      break;

  case MA_SENSOR_WIND_DIR_ANALOG:
    uv = (ratio >> 16); /* dead zone */
    ratio &= 0xffff;
    if( !ratio )
      ratio = 75;
    if( mv <= uv ) 
      mv = 0;
    else
      mv -= uv;
    uv = (mv * ratio + 5000) / 10000; /* round to kpa */
    uv = drv->thld->zero_filter(drv->s, uv);
    uv &= 0xf; /* mask to 0~15 */
    ret = drv->data->plss_json_put(uv,  
      DRV_MA_PLSS_TRANS(PLSS_PL_WIND_DIR, 1, 0, 0, 0), 
      "dir");
    break;

  case MA_SENSOR_DISP_ANALOG:
    uv = (ratio >> 16); /* dead zone */
    ratio &= 0xffff;    
    if( !ratio )
      ratio = 5285*4;    
    if( mv <= uv ) 
      mv = 0;
    else
      mv -= uv;
    uv=drv->history->mv[0];
    if( uv )
      mv = mv * 3640 / uv; /** william defined disp=(mv*3640)/vbat*5285/2500 */
    uv = (mv * ratio + 5000) / 10000; /* round to kpa */
    uv = drv->thld->zero_filter(drv->s, uv);
    ret = drv->data->plss_json_put(uv,  
      DRV_MA_PLSS_TRANS(PLSS_PL_DISP, 2, 0, 0, 2), 
      "disp");
    break;
    
  default:
    ret = POS_STATUS_E_NOT_FOUND;
    /* update to error status */
    drv->s->ctrl |= MA_SENSOR_CTRL_STAT_ERROR;    
    break;
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
pos_status_t sensor_analog_init(pos_u32_t load) {
  drv_api_t *drv = g_drv;
  analog_param_t *param;
  ma_sensor_ctrl_t *s = g_drv->s;
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("analog init", load);
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
  .version = DRV_SENSOR_VERSION(MA_SENSOR_PT100, DRV_ANALOG_VERSION),
  .name = DRV_ANALOG_NAME,
  .u.sensor={
    .init = sensor_analog_init,
    .power_set = sensor_analog_set,
    .collect = sensor_analog_collect,
  },
};

