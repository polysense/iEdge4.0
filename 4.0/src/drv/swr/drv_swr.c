/**
 * @file  drv_swr.c
 * @brief Spray Water driver
 * @author Runby F.
 * @date 2022-3-16
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * Revision History
v1.0 @ 2024-11-10
  1) first draft

v1.1 @ 2024-11-21
  1) Add pullup for pulse pin

v1.2 @ 2024-12-16
  1) Support button control for auto/manual mode 

v1.3 @ 2024-12-20
  1) Fix auto mode not working issue

v1.4 @ 2025-1-14
  1) Support to turn on/off pump according to valve status by pwr[12]
  2) Reduce valve num to hi/lo=4/2 according to wechat agreement on 20250114 by Rock

v1.5 @ 2025-1-15
  1) Support to turn on/off pump by simulating switching button push

v1.6 @ 2025-1-16
  1) Support 1s pump button time
  2) Support configurable min speed check
  3) Support pump on/off delay

*/

/**
 * @brief Driver version
 */
#define DRV_SWR_VERSION 0x0106

/**
 * @brief Driver name
 */
#define DRV_SWR_NAME  "SWR"

/**
 * @brief Flag for ON state
 */
#define DRV_SWR_FLAG_ON   1

/**
 * @brief Flag for PUMP state (1-ON, 0-OFF)
 */
#define DRV_SWR_FLAG_PUMP   2


/**
 * @brief IRQ driver structure
 */
typedef struct {
  drv_api_t     drv;
  pos_u32_t     width; /* unit of 1mm */
  pos_u32_t     water; /* unit of 1ml/m2 */
  pos_u32_t     irq_cnt; /* last read irq_cnt */
  pos_u32_t     tick_read; /* last read tick */
  pos_u32_t     elaps; /* elaps tick from last irq */
  pos_u16_t     bmp_on; /* overall ctrl valve on/off bitmap */
  pos_u16_t     valve_h; /* unit of 1ml/min for valve/high flow speed */
  pos_u16_t     valve_l; /* unit of 1ml/min for valve/low flow speed */
  pos_u8_t      irq_id; /* 0-pulse, 1-btn */
  pos_u8_t      mode; /* 0-auto, 1-btn */  
  pos_u8_t      pin; /* pulse input pin */
  pos_u8_t      pin_btn; /* user btn input pin */
  pos_u8_t      flag;      /* on off control */
  pos_u8_t      vh_num;     /* number of valve/high */
  pos_u8_t      vl_num;     /* number of valve/low */
  pos_u8_t      on_num_h;   /* on number of valve/high */
  pos_u8_t      on_num_l;   /* on number of valve/high */
  pos_u8_t      vh_id;      /* valve/high opr id */
  pos_u8_t      vl_id;      /* valve/low opr id */
  void          *root_param; /* root param pointer */
  pos_u32_t     btn_ticks; /* btn press ticks */
} swr_param_t;


/**
* Driver IRQ ON/OFF set
*/
void sensor_swr_irq_set(swr_param_t *param, pos_u32_t on) {
  pos_u32_t v;
  if( on ) {
    v = POS_GPIO_MODE_INPUT_PU + POS_GPIO_MODE_IRQ_FALLING;

    /*
     * turn on irq
     * warning: as GOT is implemented in DATA and DATA is not supported in mmap()
     * all GOT based pointer can NOT be used. So the IRQ SET should use the mapped
     * function pointer here (the export structure).
     * the orignal sensor_swr_irq() should NOT be used directly.
     */
    param->drv.os->gpio->irq_set(
      param->pin,
      v,
      (pos_func_t)param->drv.s->drv.u.sensor.irq_cb, /* has to be this. DO NOT use sensor_swr_irq */
      param);
  } else {
    /*
     * turn off irq
     */
    param->drv.os->gpio->irq_set(param->pin, POS_GPIO_MODE_INPUT_PU+POS_GPIO_MODE_IRQ_DISABLE, POS_NULL, POS_NULL);
  }
}

/** 
* Driver collecting IRQ counter
*/
pos_u32_t sensor_swr_read_irq_cntr(swr_param_t *param) {
  pos_u32_t v;
  g_drv->os->task->lock_irq();  
  v = param->drv.s->data.u32;
  param->drv.s->data.u32 = 0;
  g_drv->os->task->unlock_irq();
#if 0
  if( !v ) {
    v = param->drv.s->slot->thld.u16[7];
  }
#endif  
  param->tick_read = g_drv->os->tick->get();
  param->irq_cnt = v;
  return v;
}

/** 
* Driver collecting spraying speed (unit: 0.01 m2/minutes)
*/
pos_u32_t sensor_swr_read_sparying_dead_zone_filter(swr_param_t *param, pos_u32_t v) {
  pos_u32_t d, v_d;
  v_d = v * 60000 / param->elaps;
  d = param->drv.s->slot->thld.u16[5];
  if( !d )
    d = 1000;
  if( v_d < d )
    return 0;
  return v;
}

/** 
* Driver collecting spraying speed (unit: 0.01 m2/minutes)
*/
pos_u32_t sensor_swr_read_sparying_speed(swr_param_t *param) {
  pos_u32_t v, elaps;
  elaps = param->tick_read;
  v = sensor_swr_read_irq_cntr(param);  
  elaps = param->drv.os->tick->elaps_calc(param->tick_read, elaps);
  param->elaps = elaps;
  if( elaps < 100 ) 
    return 0; /* anti too short collecion */
  v = sensor_swr_read_sparying_dead_zone_filter(param,v);
  if( !v )
    return 0;
  if( v > 0xffff )
    v = 0xffff;
  v = v * param->width; /* unit 1 mm2 */

  v = v / 10000; /* unit 0.01 m2 */
  if( v > 0xffff )
    v = 0xffff;
  v = v * 60000 / elaps; /* unit 0.01 m2/min */

  return v;
}

/** 
* Driver control valve adjust
*/
void sensor_swr_valve_adj(swr_param_t *param, pos_u32_t cur, pos_u32_t exp, pos_u32_t num,  pos_u8_t *p_id, pos_u32_t io_ctrl_base) {
  pos_u8_t pin, i, v, id;
  id = p_id[0];
  if( exp >= cur ) {
    exp = exp - cur;
    v = 1; /* turn on valves */
  } else {
    exp = cur - exp;
    v = 0; /* shutdown valves */
    id = id + num - cur;
    if( id >= num )
      id -= num;
  }
#if 0
  g_drv->os->printf("id=%u base=%u adj=%u on=%u\n", id, io_ctrl_base, exp, v);
#endif
  for( i = 0; i < exp; i++ ) {
    pin = param->drv.board->pin_map(MB_PT_CTRL, 16+io_ctrl_base+id);
    if( v )
      param->bmp_on |= 1 << (io_ctrl_base+id);
    else
      param->bmp_on &= ~(1 << (io_ctrl_base+id));
    param->drv.os->gpio->set(pin, v);
    id++;
    if( id >= num )
      id = 0;
  }
  /* only refresh p_id when increasing valves */
  if( v )
    p_id[0] = id;
}

/** 
* Driver control valve/high on/off
*/
void sensor_swr_valve_hi_set(swr_param_t *param, pos_u32_t on_num) {
  if( param->on_num_h == on_num )
    return;
  sensor_swr_valve_adj(param, param->on_num_h, on_num, param->vh_num, &param->vh_id, 5);
  param->on_num_h = on_num;
}

/** 
* Driver control valve/lo on/off
*/
void sensor_swr_valve_lo_set(swr_param_t *param, pos_u32_t on_num) {
  if( param->on_num_l == on_num )
    return;
  sensor_swr_valve_adj(param, param->on_num_l, on_num, param->vl_num, &param->vl_id, 0);

  param->on_num_l = on_num;
}

void sensor_swr_pump_set(swr_param_t *param, pos_u32_t on) {
  pos_u32_t cur, v;
  drv_api_t *drv;
  cur = (param->flag & DRV_SWR_FLAG_PUMP) == 0 ? 0 : 1;
  on = on ? 1 : 0;
  if( cur == on )
    return; /* do nothing when same */

  if( on )
    param->flag |= DRV_SWR_FLAG_PUMP;
  else
    param->flag &= ~DRV_SWR_FLAG_PUMP;

  drv = &param->drv;
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("swr pump", on);
  }

  /* calc v on/off delay */
  v = param->drv.s->slot->thld.u16[6];
  if( on )
    v = v & 0xff;
  else {
    v = (v >> 8)&0xff;
    if( !v )
      v = 2;
  }
  v *= 1000;

  /* pushing btn to trigger a pump state change */
  if( on ) {
    /* always push for start */
    drv->board->pwr_set(MB_PWR_ID_VCC_CTRL2, 1);
  } else {
    /* a push/release for turn off */
    drv->board->pwr_set(MB_PWR_ID_VCC_CTRL2, 0);
    drv->os->tick->sleep(v); 
    drv->board->pwr_set(MB_PWR_ID_VCC_CTRL2, 1);
    drv->os->tick->sleep(100); /* wilson requests to be 0.1s here always */
    drv->board->pwr_set(MB_PWR_ID_VCC_CTRL2, 0);
  }

  
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("swr pump delay", v);
    drv->os->tick->sleep(v);
  }
}

/** 
* Driver control valve by setting flow speed (unit: 1 ml/min)
*/
void sensor_swr_valve_set(swr_param_t *param, pos_u32_t flow) {
  pos_u32_t hi_on = 0, lo_on = 0;

  while( flow ) {
    while( flow + (param->valve_h/4) >= param->valve_h && hi_on < param->vh_num ) {
      hi_on++;
      if( flow > param->valve_h )
        flow -= param->valve_h;
      else
        flow = 0;
      continue;
    }

    while( flow >= param->valve_l && lo_on < param->vl_num ) {
      lo_on++;
      flow -= param->valve_l;
      continue;
    }  

    if( flow >= param->valve_h/2 && hi_on < param->vh_num ) {
      hi_on++;
      break;
    }

    if( flow >= param->valve_l/2 && lo_on < param->vl_num ) {
      lo_on++;
      break;
    }
    break;    
  }

#if 0
  g_drv->os->printf("flow=%u hi=%u(@%u) lo=%u(@%u)\n", flow, hi_on, param->vh_id, lo_on, param->vl_id);
#endif

  sensor_swr_valve_hi_set(param, hi_on);
  sensor_swr_valve_lo_set(param, lo_on);

  /* turn on pump when any valve is on 
   * or 
   * turn off pump when all valves are off
   */
  sensor_swr_pump_set(param, hi_on+lo_on);

}

/** 
* Driver collecting (0-auto, 1-spraying)
*/ 
void sensor_swr_mode_set(swr_param_t *param, pos_u8_t mode){
  drv_api_t *drv = &param->drv;
  pos_u32_t v;
  if( mode == 0 ) {
    /* auto */
    MB_PIN_LED_SET_B(drv->os, 1);
    MB_PIN_LED_SET_Y(drv->os, 0);
    v = 0;
  } else {
    /* spraying */
    MB_PIN_LED_SET_B(drv->os, 0);
    MB_PIN_LED_SET_Y(drv->os, 1);
    v = drv->s->slot->arg.u32[0];
    if( !v )
      v = 100000;
    else if( v == 0xffffffff )
      v = 0;
    sensor_swr_valve_set(param, v);
  }

  if( param->mode != mode ) {
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      drv->log->data("swr mode", mode);
      if( mode )
        drv->log->data("swr ml/min", v);
    }

    if( !mode ) {
      /* change from none-auto to auto */
      /* clear irq status */
      param->tick_read = drv->os->tick->get();
      sensor_swr_read_irq_cntr(param);
      drv->os->tick->sleep(500); /* first auto will use 500ms to measure the speed */
    }
    param->mode = mode;
  }
}


/** 
* Driver collecting
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_swr_collect(void){
  pos_status_t ret = POS_STATUS_OK;
  pos_u32_t v, dbg, s;
  swr_param_t *param;
  drv_api_t *drv = g_drv;

  param = (swr_param_t*)drv->s->drv_buf;

  dbg = drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG;
  do {
    /* do nothing when not started */
    if( (param->flag & DRV_SWR_FLAG_ON) == 0 ) {
      if( dbg )
        drv->log->data("swr off", param->flag);
      break;
    }

    /* choose auto/manual mode */
    sensor_swr_mode_set(param, drv->os->gpio->get(param->pin_btn) );

    if( param->mode != 0 )
      break;

    /* auto mode */
    /* get spraying speed */
    s = sensor_swr_read_sparying_speed(param);

    /* calc water speed: ml/min */
    v = s * param->water / 100;

    /* set valve */
    sensor_swr_valve_set(param, v);
    if( dbg ) {
      pos_u32_t i;
      drv->log->data("valve", param->bmp_on);
      drv->os->printf("    speed=%u.%02u pulse=%u width=%u elaps=%u flow=%u.%03u [",
        s/100, s%100,
        param->irq_cnt,
        param->width,
        param->elaps,
        v/1000, v%1000);
      for( i = 0; i < 16; i++ ) {
        if( (1<<i) & param->bmp_on ) {
          drv->os->printf("%x", i);
        } else {
          if( i == param->vl_id || i == (param->vh_id+5) )
            drv->os->printf("_"); 
          else
            drv->os->printf(".");
        }
      }
      drv->os->printf("]\n");
    }
    
  } while(0);

  /* Release and return */
  return ret;  
}

/** 
* Driver polling
*/ 
void sensor_swr_poll(swr_param_t *param){
#if 0
  pos_u32_t log;
  drv_api_t *drv = &param->drv;
  log = drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG;
#endif  

}

/**
 * IRQ interrupt callback handler
 * @note This function can NOT call g_drv. When IRQ happening, g_drv might piont to any sensor drivers. It should call the param stored driver pointer.
 */
void sensor_swr_irq(swr_param_t *param) {
  if( param->irq_id == 0 ) {
    param->drv.s->data.u32++;
    param->drv.os->gpio->irq_clr(param->pin);
  } else {
    pos_u32_t ticks;
    /* always use the root param */
    param = (swr_param_t*)param->root_param;
    ticks = param->drv.os->tick->get();
    if( !ticks )
      ticks = 1;
    param->btn_ticks = ticks;
    param->drv.os->gpio->irq_clr(param->pin_btn);
  }
}

/** 
* Power control
* @param[in]  on   0:Power off, 1:Power on
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
* @note This function can be NULL if it does NOT require any power operatoin
*/ 
pos_status_t sensor_swr_set(pos_u32_t on) {  
  pos_gpio_pin_t pwr;
  ma_board_t *b = g_drv->board;
  pwr = (g_drv->s->slot->io >> 8) & 0x7f;  
  if( pwr ) {
    b->pwr_set(pwr, on);
  }
  return POS_STATUS_OK;
}

/** 
 * Driver init
 * @param[in]  load   0:Unload driver, 1:Load driver
 * @return     0: Successful\n
             !=0: Failed, refer to @ref pos_status_t
 * @note This function can be NULL if it does NOT require any init operatoin
 */ 
pos_status_t sensor_swr_init(pos_u32_t load) {
  drv_api_t *drv = g_drv;
  swr_param_t *param;
  ma_sensor_ctrl_t *s = g_drv->s;
  pos_gpio_pin_t pin;
  pos_u16_t i;
  
  /* do NOTHING for PULL mode */
  if( g_drv->s->slot->sensor_type == MA_SENSOR_IO_PULL ) {
    return POS_STATUS_OK;
  }

  /* valve init */
  for( i = 0; i < 16; i++ ) {
    pin = drv->board->pin_map(MB_PT_CTRL, 16+i);
    drv->os->gpio->mode_set(pin, POS_GPIO_MODE_OUTPUT);
    drv->os->gpio->set(pin, 0); /* turn off by default */
  }

  /* LED init */
  MB_PIN_LED_SET_B(drv->os, 0);
  MB_PIN_LED_SET_Y(drv->os, 0);

  /* IRQ init */
  pin = drv->board->pin_map(MB_PT_DIO, 0);

  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("swr init", load);
    drv->log->data("swr pin", pin);
  }

  /* call power on/off only when PWR is defined */
  sensor_swr_set(load);
  
  if( load ) {
    s->data.u32 = 0; /* clear counter by default */
    param = drv->os->malloc(sizeof(*param)*2);
    if( !param )
      return POS_STATUS_E_MEM;

    drv->os->memset(param, 0, sizeof(*param)*2);
    param->root_param = param;
    param->pin = pin;
    param->pin_btn = drv->board->pin_map(MB_PT_DIO, MB_DIO_ID_BTN);
    drv->os->gpio->mode_set(param->pin_btn, POS_GPIO_MODE_INPUT_PU);
    drv->os->memcpy(&param->drv, drv, sizeof(*drv)); /* clone a g_drv */
    param->width = s->slot->thld.u16[0];
    if( !param->width )
      param->width = 5000; /* default 5000mm */
    param->water = s->slot->thld.u16[1];
    if( !param->water )
      param->water = 2000; /* default 2000ml/m2 */
    param->valve_h = s->slot->thld.u16[2];
    if( !param->valve_h )
      param->valve_h = 10000; /* default 10000ml/min */
    param->valve_l = s->slot->thld.u16[3];
    if( !param->valve_l )
      param->valve_l = 5000; /* default 5000ml/min */
    i = s->slot->thld.u16[4];
    param->vh_num =  (i >> 8)&0xff;
    param->vl_num =  (i)&0xff;
    if( !param->vh_num )
      param->vh_num = 4;
    if( !param->vl_num )
      param->vl_num = 2;
    param->vh_id = drv->os->rand() % param->vh_num; /* use rand start id */
    param->vl_id = drv->os->rand() % param->vl_num; /* use rand start id */
    s->drv_buf = param;  /* record to drv_buf */
    drv->os->memcpy(&param[1], param, sizeof(*param));

    /*
     * turn on irq
     */
    sensor_swr_irq_set(param, 1);

    sensor_swr_read_sparying_speed(param); /* init tick/cnt/elaps */
    param->flag |= DRV_SWR_FLAG_ON; /* turn on by default */
    param->mode = 0xff; /* init an invalid mode and then changed by btn status in collect() */
  } else {
    if( s->drv_buf ) {
      /*
       * turn off irq
       */
      sensor_swr_irq_set((swr_param_t*)s->drv_buf, 0);

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
  .version = DRV_SENSOR_VERSION(MA_SENSOR_SWR, DRV_SWR_VERSION),
  .name = DRV_SWR_NAME,
  .u.sensor={
    .init = sensor_swr_init,
    .power_set = sensor_swr_set, 
    .collect = sensor_swr_collect,
    .irq_cb = (pos_func_t)sensor_swr_irq,
  },
};

