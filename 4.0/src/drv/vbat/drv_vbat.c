/**
 * @file  drv_vbat.c
 * @brief SENSOR VBAT driver
 * @author Runby F.
 * @date 2022-3-10
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * 版本修订历史
v1.0 @ 2022-03-10
  1) 完成第一版驱动

v1.1 @ 2022-03-15
  1) 支持RT-Thread

v1.2 @ 2022-03-25
  1) 支持4.0ADC策略
  2) 按照4.0倍率转换电压
  4) 支持采样数字滤波

v1.3 @ 2022-04-01
  1) 支持电压数据汇总到#0历史条目 

v1.4 @ 2022-04-15
  1) 修正供电异常逻辑

v1.5 @ 2022-05-14
  1) 支持内核统一管脚策略

v1.6 @ 2022-05-29
  1) 采样时永远保持VCC_AIN打开
  2) 采样后放大四倍得到最终电压

v1.7 @ 2022-11-14
  1) Support PLSS UDP data format

v1.8 @ 2022-12-20
  1) Support VBAT PWR control

v1.9 @ 2023-02-09
  1) Enhance as ANA_INPUT

v1.10 @ 2023-06-13
  1) Support unified JSON output

v1.11 @ 2023-07-25
  1) Support to report rssi 

v1.12 @ 2023-08-27
  1) Support to trigger LoRa RSSI retrieval when rssi report = on
  
v1.13 @ 2023-10-24
  1) Support stat/normal for vbat

v1.14 @ 2023-11-18
  1) Using new PWR ID solution

v1.15 @ 2023-12-16
  1) Support VBAT mute by rsvd.bit[1]
  2) Support BAT percentage report by rsvd.bit[2]
  3) Support VBAT calculation by v=adc*d0.bit[0..15]/1000+d0.bit[16..31] (when d0 is zero, uses 2000 for x 2.000)
  4) Support BAT percentage by p=v among d1.bit[0..15]~bit[16..31] (when d1 is zero, uses 3300~3600 mv)  

v1.16 @ 2024-1-11
  1) Support VBAT giveup all remaining slots when vbat < thw[2]

v1.17 @ 2024-03-14
  1) Support PLSS_PL_PER (111/0x6f) for vbat percentage

v1.18 @ 2024-03-21
  1) Cancel this duty report when VBAT < 2.8V

v1.19 @ 2024-06-22
  1) Support multiple vbat cell report

v1.20 @ 2024-06-25
  1) Support use vbat max for low power chk and percentage calc
  2) Support to use vbat as zero when < 1.0V

v1.21 @ 2024-07-10
  1) Support use average voltage as percentage calc

*/

/**
 * @brief Driver version
 */
#define DRV_VBAT_VERSION 0x0115

/**
 * @brief Driver name
 */
#define DRV_VBAT_NAME  "VBAT"

/** 
* Power control
* @param[in]  on   0:Power off, 1:Power on
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
* @note If sensor does NOT require any power control, this fucntion can be NULL
*/ 
pos_status_t sensor_vbat_set(pos_u32_t on) {  
  mb_pwr_id_t pwr;
  ma_board_t *b = g_drv->board;
  pos_u16_t io;
  io = g_drv->s->slot->io;
  pwr = (io >> 8) & 0x7f; 
  io &= 0xff;
  if( !io )
    io = MB_AIN_ID_VBAT;

  /* Special power pin control */
  if( pwr ) {
    b->pwr_set(pwr, on);
  }

  /* turn ON VCC/VBAT */
  if( pwr != MB_PWR_ID_VCC_VBAT )
    b->pwr_set(MB_PWR_ID_VCC_VBAT, on);

  /* Always enable AIN */
  if( pwr != MB_PWR_ID_VCCN_AIN )
    b->pwr_set(MB_PWR_ID_VCCN_AIN, on);
  
  return POS_STATUS_OK;
}

/** 
* Sensor ADC voltage to real voltage
* @return : Real voltage
*/ 
pos_u32_t sensor_vbat_adcv_to_v(pos_u32_t v) {
  pos_i32_t rv;
  pos_u32_t m;
  pos_i16_t ofs;
  drv_api_t *drv = g_drv;  
  m = drv->s->slot->arg.u32[0];
  if( !m )
    m = 2000; /* default using adc x 2.000 as final voltage */
  ofs = (m>>16) & 0xffff;
  m &= 0xffff;
  rv = (v * m + 500) / 1000;
  rv += ofs;
  if( rv < 0 ) {
      rv = 0; 
  } else if ( rv > 0xffff ) {
    rv = 0xffff; /* anti overflow */
  }
  return (pos_u32_t) rv;
}

/** 
* Sensor voltage to percentage
* @return : Percentage from 0~100\n
*/ 
pos_u32_t sensor_vbat_to_per(pos_u32_t v) {
#if 1
  pos_u32_t per, hi, lo;
  drv_api_t *drv = g_drv;  
  lo = drv->s->slot->arg.u32[1];
  if( !lo ) {
    lo = (3600 << 16) + 3300; /* default using 3.3~3.6V range */
  }
  hi = (lo >> 16) & 0xffff;
  lo &= 0xffff;
  if( v >= hi )
    per = 100;
  else if( v <= lo || hi <= lo )
    per = 0;
  else {
    per = hi - lo;
    v -= lo;
    per = (v * 100 + per/2) / per;
  }
  return per;
#else
  pos_u32_t per, i, hi, lo, r[6];
  drv_api_t *drv = g_drv;
  lo = drv->s->slot->arg.u32[1];
  r[0] = lo & 0xffff;
  r[5] = (lo >> 16) & 0xffff;
  if( !r[0] )
    r[0] = 3300;
  if( !r[5] )
    r[5] = 3600;
  if( v >= r[5] )
    return 100;
  if( r[0] >= r[5] )
    r[0] = r[5]-1; /* anti wrong */
  if( v <= r[0] )
    return 0;

  for( i = 1; i < 5; i++ ) {
    r[i] = (r[5] - r[0]) * 20 / 100 * i + r[0];
  }
#if 0
  for( i = 0; i < 6; i++ ) {
    drv->os->printf("r[%u]=%u\n", i, r[i]);
  }
#endif
  per = 100;
  hi = r[5];
  for( i = 5; i > 0; i-- ) {
    lo = r[i-1];
    if( lo >= hi )
      break; /* stop when wrong segment value */

    /* check next range if < lo */
    if( v < lo ) {
      hi = lo;
      per -= 20;
      continue;
    }
    /* within range lo..hi */
    i = hi - lo;
    v -= lo;
    i = ( v * 20 + i/2 ) / i;
    if( i > 20 )
      i = 20;
    return per - 20 + i;
  }

  /* sub-range is not defined in d2/d3, use regular d1 based calc */
  lo = r[0];
  i = hi - lo;
  v -= lo;
  i = (v * per + i/2) / i;
  if( i > per )
    i = per;

  return i;
#endif
}

/** 
* Sensor collecting
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_vbat_collect(void){
  pos_u32_t v, v_cell[4], i, per, v_max = 0;
  pos_status_t ret = POS_STATUS_OK;
  drv_api_t *drv = g_drv;
  pos_gpio_pin_t ain;
  pos_u8_t ain_adc, io, cell_num;
  io = drv->s->slot->io & 0x80ff;
  cell_num = (drv->s->slot->rsvd32 >> 8) & 0xff;
  if( cell_num > 4 )
    cell_num = 4;
  else if( !cell_num )
    cell_num = 1;
  if( !io )
    io = MB_AIN_ID_VBAT;
  per = 0;
  for( i = 0; i < cell_num; i++ ) {
    ain = drv->board->pin_map(MB_PT_AIN, i == 0 ? io : MB_AIN_ID_VBAT-i);
    ain_adc = drv->board->pin_map(MB_PT_AIN_ADC, ain);
    drv->os->gpio->mode_set(ain, POS_GPIO_MODE_ANALOG+POS_GPIO_MODE_INPUT);  
    v = drv->os->adc->read_filter(ain_adc, 0, 32, 1000);  /* 32 x 1ms filter */ 
    if( (i & 1) == 0 )
      v = sensor_vbat_adcv_to_v(v); /* cell#0/#2 uses regular cfg based calc */
    else {
      v *= 4; /* cell#1/#3 uses x 4 fixedly calc ratio */
      if( v >= v_cell[i-1] ) /* cell#1 should be v#1-v#0 */
        v -= v_cell[i-1];
      else
        v = 0;
    }
    if( v < 1000 )
      v = 0; /* force 0 when < 1.0v according to Jason's 20240625 */
    v_cell[i] = v;
    if( v_max < v )
      v_max = v; /* record max v */
    per += sensor_vbat_to_per(v);
  }
  v=v_max;

  /* calculate from average percentage */
  per /= cell_num;
  
  /* refresh to normal by default */
  drv->s->ctrl |= MA_SENSOR_CTRL_STAT_NORMAL;
  drv->s->ctrl &= ~MA_SENSOR_CTRL_STAT_ERROR;
  
  /* Update history data */
  drv->history->mv[0] = v; 
  drv->history->mv[1] = per;
  if( drv->history->mv_num < 2 )
    drv->history->mv_num = 2;

  /* when VBAT < 2.8V, cancel this duty report and quit */
  if( v < 2800 ) {
    drv->s->ctrl |= MA_SENSOR_CTRL_ABANDON;
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      drv->log->data("vbat low", v);
    }
    return POS_STATUS_OK;
  }

  /* thold[2] check for abandon remaining slots */
  if( drv->s->slot->thld.u16[2] && v < drv->s->slot->thld.u16[2] )
    drv->s->ctrl |= MA_SENSOR_CTRL_ABANDON;
  
  /* Under threshold quit */
  if( (drv->s->slot->rsvd32 & 0x4) != 0 ) {
    drv->thld->u16_process(g_drv->s, per); /* use per when per is reported */
  } else {
    drv->thld->u16_process(g_drv->s, v); /* or else, use voltage */
  }
  if( (drv->s->ctrl & MA_SENSOR_CTRL_REPORT) == 0 )
    return POS_STATUS_OK;

  /* Report vbat-precentage */
  if( (drv->s->slot->rsvd32 & 0x4) != 0 ) {
    drv->data->plss_json_put(per, 
      DRV_MA_PLSS_TRANS(PLSS_PL_PERCENTAGE, 1, 0, 0, 0), /* 7 - PLSS_PL_BATTERY, 2Bytes, 1 mv */
      "per");
  }
  
  /* Report bat-v when not muted */
  if( (drv->s->slot->rsvd32 & 0x2) == 0 ) {
    for( i = 0; i < cell_num; i++ ) {
      static const char vbs[][6] = {"vbat", "vbat2", "vbat3", "vbat4",};
      ret = drv->data->plss_json_put(v_cell[i&3],
        DRV_MA_PLSS_TRANS(PLSS_PL_BATTERY, 2, 0, 0, 0), /* 7 - PLSS_PL_BATTERY, 2Bytes, 1 mv */
        vbs[i&3]);
      if( ret != POS_STATUS_OK ) {
        drv->s->ctrl |= MA_SENSOR_CTRL_TOO_LONG; /* indicate data buffer put failed */
      }
    }
  }

  /* append rf/signal when required */
  if( drv->s->slot->rsvd32 & 0x1 ) {
    /* trigger a LORA rssi get */
    if( MA_REPORT_TYPE_IS_LORA(drv->ext->report_type) ) {
      drv->ext->net->recvfrom(0xffff0001, POS_NULL, 0, POS_NULL, POS_NULL, 0); /* use RSSI magic number for LoRa RSSI get */
    }
    v = drv->history->signal;
    drv->data->plss_json_put(v,
      DRV_MA_PLSS_TRANS(PLSS_PL_RSSI_SNR, 4, 0, 0, 0),
      "rf");
  }
  return ret;
}

/*
 * Driver export
 */
ma_drv_export_t g_export = {
  .version = DRV_SENSOR_VERSION(MA_SENSOR_VBAT, DRV_VBAT_VERSION),
  .name = DRV_VBAT_NAME,
  .u.sensor={
    .power_set = sensor_vbat_set,
    .collect = sensor_vbat_collect,
  },
};

