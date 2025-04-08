/**
 * @file  drv_myadc.c
 * @brief SENSOR driver for ADC example
 * @author Runby F.
 * @date 2025-4-8
 * @copyright Polysense
 */

#include "drv_api.h"

/*
 * Revision history
v1.0 @ 2025-04-08
  1) First revision

*/

/**
 * @brief Driver version
 */
#define DRV_MYADC_VERSION 0x0100

/**
 * @brief Driver name
 */
#define DRV_MYADC_NAME  "MYADC"

/** 
* Power control
* @param[in]  on   0:Power off, 1:Power on
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
* @note If sensor does NOT require any power control, this fucntion can be NULL
*/ 
pos_status_t sensor_myadc_set(pos_u32_t on) {
  ma_board_t *b = g_drv->board;

  /* turn ON VCC/VBAT */
  b->pwr_set(MB_PWR_ID_VCC_VBAT, on);

  /* Always enable AIN */
  b->pwr_set(MB_PWR_ID_VCCN_AIN, on);
  
  return POS_STATUS_OK;
}

/** 
* Sensor ADC voltage to real voltage
* @return : Real voltage
*/ 
pos_u32_t sensor_myadc_adcv_to_v(pos_u32_t v) {
  pos_u32_t rv;
  pos_u32_t m;
  m = 2000; /* default using adc x 2.000 as final voltage */
  rv = (v * m + 500) / 1000;
  return rv;
}

/** 
* Sensor voltage to percentage
* @return : Percentage from 0~100\n
*/ 
pos_u32_t sensor_myadc_to_per(pos_u32_t v) {
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
}

/** 
* Sensor collecting
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_myadc_collect(void){
  pos_u32_t v,  per;
  pos_status_t ret = POS_STATUS_OK;
  drv_api_t *drv = g_drv;
  pos_gpio_pin_t ain;
  pos_u8_t ain_adc, io;
  io = drv->s->slot->io & 0x80ff;
  if( !io )
    io = MB_AIN_ID_VBAT;
  ain = drv->board->pin_map(MB_PT_AIN, io); /* get AIN pin from board AIN definitions */
  ain_adc = drv->board->pin_map(MB_PT_AIN_ADC, ain); /* get ADC channel from AIN pin */
  drv->os->gpio->mode_set(ain, POS_GPIO_MODE_ANALOG+POS_GPIO_MODE_INPUT); /* set Analog+Input for ADC sampling */
  v = drv->os->adc->read_filter(ain_adc, 0, 32, 1000);  /* 32 x 1ms filter */
  v = sensor_myadc_adcv_to_v(v);
  per = sensor_myadc_to_per(v);
  
  /* refresh to normal by default */
  drv->s->ctrl |= MA_SENSOR_CTRL_STAT_NORMAL;
  drv->s->ctrl &= ~MA_SENSOR_CTRL_STAT_ERROR;
  
  /* Report vbat-precentage */
  drv->data->plss_json_put(per,
    DRV_MA_PLSS_TRANS(PLSS_PL_PERCENTAGE, 1, 0, 0, 0), /* 7 - PLSS_PL_BATTERY, 2Bytes, 1 mv */
    "per");
  
  /* Report vbat */
  drv->data->plss_json_put(v,
      DRV_MA_PLSS_TRANS(PLSS_PL_BATTERY, 2, 0, 0, 0), /* 7 - PLSS_PL_BATTERY, 2Bytes, 1 mv */
      "mv");

  return ret;
}

/*
 * Driver export
 */
ma_drv_export_t g_export = {
  .version = DRV_SENSOR_VERSION(160, DRV_MYADC_VERSION), /* 160 is registered sensor type */
  .name = DRV_MYADC_NAME,
  .u.sensor={
    .power_set = sensor_myadc_set,
    .collect = sensor_myadc_collect,
  },
};

