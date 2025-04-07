/**
 * @file  drv_epd.c
 * @brief EPD LCD driver
 * @author Runby F.
 * @date 2023-2-4
 * @copyright Polysense
 */

#include "drv_api.h"
#include "drv_epd.h"

/* 
 * Revision History
v1.0 @ 2023-02-07
  1) Finish first draft

v1.1 @ 2023-02-11
  1) Correct CHARGE IRQ PIN to input state
  2) Adjust 100% VBAT to 4.1V
  3) Fix battery picture mistake

v1.2 @ 2023-02-11
  1) Adjust ICON CELSIUS and '%' bitmap
  2) Remove unexpected debug dump

v1.3 @ 2023-02-13  
  1) Support VBAT (0-3.6 fix, 1-3~4.1V) type
 
v1.4 @ 2023-02-18
  1) Fix VBAT percentage issue when HUM <= 30.00%

v1.5 @ 2023-02-22
  1) New requirement for face good/bad/normal and RH hum unit change
  2) Adjust VBAT bar 2 pixels right
  3) Support face overwriting by rsvd bit[4..5]
  4) Support good face during startup

v1.6 @ 2023-03-01
  1) Fix VBAT 3.6V displaying percentage zero issue

v1.7 @ 2023-03-06
  1) Support EPD good/normal PPB parameters

v1.8 @ 2023-05-08
  1) Adjust dc/charge pin layout
  2) Significantly enhance epd sleeping power cost by turn off BUSY(PA12) PULLUP

v1.9 @ 2023-05-09
  1) Fix negative temperature displaying issue
  2) Support logo displaying instead of CO2 PPM when thw0[x].bit[0] == 1

v1.10 @ 2023-05-17
  1) Remove power on EPD refresh 

v1.11 @ 2023-06-15
  1) Support Multi Temperature/Hum displaying

v1.12 @ 2023-07-10
  1) Support Two 485 Temp/Hum displaying by thw0=0x20

v1.13 @ 2023-07-11
  1) Fix 3 - 485temp displaying issue
  2) Support unified 485temp/hum and sht30 temp/hum

v1.14 @ 2023-07-11
  1) Fix EPD good face hum unit mistake

v1.15 @ 2023-08-14
  1) Fix EPD not update when multiple temp/hum mode and temp/hum#0 is not changed, but temp/hum#N is changed

v1.16 @ 2023-08-14
  1) Fix EPD update two times issue (obsolete the change for v1.15)

v1.17 @ 2023-08-15
  1) Support to display -- when temp is abnormal

v1.18 @ 2023-09-16
  1) Support to display -- when hum is abnormal

v1.19 @ 2023-10-17
  1) Support to giveup refreshing when time is over 6s

v1.20 @ 2023-10-24
  1) Support to normal/error stat

v1.21 @ 2024-02-23
  1) Support new requirement
  2) Support CO2 history bar and LED r/b/y for bad/good/normal when thw0 = 0x04

v1.22 @ 2024-02-29
  1) Fix CO2 history bar displaying issue

v1.23 @ 2024-03-11
  1) Use co2 0~1000 for good range
  2) Support thw[0]/0x4 for history bar control
  3) Support thw[0]/0x8 for LED sync (good/bad)
  4) Support thw[0]/0x100 for history BAR level (0-1600,1-3200)

v1.24 @ 2024-03-12
  1) Support auto scaled history BAR level (0-1600/3500,1-max(1600,ppm))

v1.25 @ 2024-03-13
  1) Support always max(1600,ppm) history top level
  2) Support thw[0]/0x100 for title displaying (0-report time,1-EUI)
  3) Support history bar top/bottom line

v1.26 @ 2024-03-14
  1) Support EUI upper case displaying

v1.27 @ 2024-03-15
  1) Support stop refreshing when vbat < 3.0 and resume after > 3.1

v1.28 @ 2024-03-20
  1) Support displaying EUI for < 3.0 displaying

v1.29 @ 2024-03-21
  1) Adjust < 3.0V displaying format for low voltage

v1.30 @ 2024-03-29
  1) Support SSD1683 through thw1.bit[0]=1
  2) Support thw1.bit[8]/[9] to control x/y reverse

v1.31 @ 2024-04-01
  1) Adjust epd model/reverse into epd_refresh() so as it can be correct even for init/refresh (if it's on)

v1.32 @ 2024-04-10
  1) Support forcedly refreshing for each duty even when no data change

v1.33 @ 2024-05-11
  1) Support old logo and PLSS logo in the same time (thw[0]=1 for old, 2/3 for new PLSS)

v1.34 @ 2024-07-03
  1) Support uc8276cAA by thw[1]=0x2

v1.35 @ 2024-07-31
  1) Use VBAT percentage instead of EPD private calculations

v1.36 @ 2024-08-05
  1) Support thw2/thw3 to control low/high refreshing voltage
  2) Support 0805 revision of UC8276CAA driver

v1.37 @ 2024-09-08
  1) Support thw0=5 for 48 TEMP history displaying

v1.38 @ 2024-09-13
  1) Do NOT record invalid temp into 48 TEMP history 

v1.39 @ 2024-09-18
  1) Support -100 celsius in 48 temp history displaying
  2) Adjust -x.x format (instead of - x.x) in temp show

v1.40 @ 2024-12-09
  1) Fix data report is still valid issue when network is offline

v1.41 @ 2025-03-02
  1) Support thw0=0x201 for Logo+HPA displaying (no face)

v1.42 @ 2025-03-08
  1) Support new HPA logo for thw0=0x201 Logo+HPA displaying (no face)

v1.43 @ 2025-03-12
  1) Use "hPa" instead of "hPA" in HPA displaying

v1.44 @ 2025-03-16
  1) Support 0.01 HPA displaying unit by small font
  2) Correct EPD sensor always stats=INIT issue

*/

/**
 * @brief Driver version
 */
#define DRV_EPD_VERSION 0x012c

/**
 * @brief Driver name
 */
#define DRV_EPD_NAME  "EPD"

const static char g_epd_rev[4][12] = {"uc8276cbb", "ssd1683", "uc8276caa", "epd_err_rev",};


/** 
 * IRQ interrupt callback handler
 * @note This function can NOT call g_drv. When IRQ happening, g_drv might piont to any sensor drivers. It should call the param stored driver pointer.
 */
void sensor_epd_irq(epd_param_t *param) {
  pos_u32_t ticks;
  ticks = param->drv.os->tick->get();
  if( !ticks )
    ticks = 1; /* 至少是1 */
  param->irq_ticks = ticks; /* 记录中断时刻 */
  param->drv.os->gpio->irq_clr((pos_gpio_pin_t)param->pin_btn);
}


/** 
 * Get current charging state
 * @return     0: NOVBAT, 1: CHARGED, 2: CHARGING, 3: VBAT\n
 */
pos_u32_t sensor_epd_recharge_get(epd_param_t *p) {
  return p->drv.os->gpio->get(p->pin[0]) + p->drv.os->gpio->get(p->pin[1])*2;
}

/**
 * @brief EPD value collect
 * @param[in] p 参数控制块
 */
void sensor_epd_value_collect(epd_param_t *p) {
  epd_value_t *v;
  pos_i16_t v16;
  pos_u32_t v32;
  v = &p->value;
  v->flags = 0;
  
  /* batch temp comp */
  if( p->drv.os->memcmp(&TEMP_GET(p), v->temp, sizeof(v->temp)) != 0 ) {
    v->flags |= 1;
    p->drv.os->memcpy(v->temp, &TEMP_GET(p), sizeof(v->temp));
  }
  
  v16 = HUMIDITY_GET(p);
  if( v16 != v->hum ) {
    v->flags |= 2;
    v->hum = v16;
  }    

  v32 = sensor_epd_recharge_get(p);
  if( v32 != v->charge ) {
    v->flags |= 4;
    v->charge = v32;
  }    

  if( v->disp_mode != (p->drv.s->slot->rsvd32 & 0xff) ) {
    v->flags |= 0x80;
    v->disp_mode = (p->drv.s->slot->rsvd32 & 0xff);
  }

  v32 = VBAT_GET(p);
  if( v->vbat != v32 ) {
    v->flags |= 8;
    v->vbat = v32;
  }
#if 0 
  /* obsolete private calc for percentage */
  if( v->disp_mode & 0x4 ) { /* 3~4.1V */
    if( v32 <= 3000 )
      v32 = 0;
    else
      v32 = ((v32 - 3000) * 100 + (4100-3000)/2 ) / (4100 - 3000); /* 4100: 100%, 3000: 0% */
  }  else {
    if( v32 <= 3300 )
      v32 = 0;  
    else
      v32 = ((v32 - 3300) * 100 + (3600-3300)/2 ) / (3600 - 3300); /* 3.600: 100%, 3300: 0% */
  }
  if( v32 > 100 )
    v32 = 100;
#else
  /* using percentage from VBAT */
  v32 = VBAT_PER_GET(p);
#endif
  v->vbat_per = v32;

  v32 = SIGNAL_GET(p);
  if( v->signal_per != v32 ) {
    v->flags |= 0x10;
    v->signal_per = v32;
  }

  v32 = CO2_GET(p);
  if( v->co2 != v32 ) {
    v->flags |= 0x20;
    v->co2 = v32;
  }

  v32 = p->drv.history->time.tick;
  if( v->report_ticks != v32 ) {
    v->flags |= 0x40;
    v->report_ticks = v32;
  }

  /* get hpa */
  v32 = HPA_GET(p);
  v->hpa = v32;

  if( p->rsvd & 1 ) {
    p->rsvd &= ~1; /* clear it */
    v->flags |= 0x40; /* set forced refreshing flag */
  }

#if 0
  if( v->flags ) {
    p->drv.log->data("vbat per", v->vbat_per);
  }
#endif
}

/** 
* Sensor collecting
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_epd_collect(void){
  drv_api_t *drv = g_drv;
  epd_param_t *p;
  pos_i16_t i0, i1;
  pos_u16_t u0, u1;
  pos_i8_t *p8;
  p = (epd_param_t*)drv->s->drv_buf;
  p8 = drv->s->slot->arg.i8;

  /* good hum */
  i0 = p8[0];
  i1 = p8[1];  
  if( i0 < i1 ) {
    i0 *= 10; /* unit 0.1% */
    i1 *= 10;
  } else {
    i0 = 400; 
    i1 = 700;
  }
  p->hum_good[0] = i0;
  p->hum_good[1] = i1;

  /* normal hum */
  i0 = p8[2];
  i1 = p8[3];  
  if( i0 < i1 ) {
    i0 *= 10;
    i1 *= 10;
  } else {
    i0 = 300; 
    i1 = 800;
  }
  p->hum_normal[0] = i0;
  p->hum_normal[1] = i1;

  /* good co2 */
  u0 = p8[4];
  u1 = p8[5];  
  if( u0 < u1 ) {
    u0 *= 100;
    u1 *= 100;
  } else {
    u0 = 0; 
    u1 = 1000;
  }
  p->co2_good[0] = u0;
  p->co2_good[1] = u1;

  /* normal co2 */
  u0 = p8[6];
  u1 = p8[7];  
  if( u0 < u1 ) {
    u0 *= 100;
    u1 *= 100;
  } else {
    u0 = 0;
    u1 = 1600;
  }
  p->co2_normal[0] = u0;
  p->co2_normal[1] = u1;

  /* good temp */
  i0 = p8[8];
  i1 = p8[9];  
  if( i0 < i1 ) {
    i0 *= 10; /* unit 0.1c */
    i1 *= 10;
  } else {
    i0 = 180; 
    i1 = 260;
  }
  p->temp_good[0] = i0;
  p->temp_good[1] = i1;

  /* normal temp */
  i0 = p8[10];
  i1 = p8[11];  
  if( i0 < i1 ) {
    i0 *= 10; /* unit 0.1c */
    i1 *= 10;
  } else {
    i0 = 180; 
    i1 = 280;
  }
  p->temp_normal[0] = i0;
  p->temp_normal[1] = i1;

  p->rsvd |= 1; /* force to refresh in the next refreshing polling */
  
/* no need to call refresh here, refresh will happen during event process after data report */  
#if 0  

  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("epd collect", (pos_u32_t)p);
  } 
  
  /* Value Collect */  
  sensor_epd_value_collect(p);
  
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("epd collect refresh", p->value.flags);
  } 

  /* if no content change do nothing */
  if( p->value.flags ) {
    /* Refresh EPD */  
    epd_refresh(p, 0); 
  }
#endif
#if 0
  /* Force to disable CONSOLE (LPUART1 INPUT) for Power saving */
  {
    pos_u8_t pin_rx;
    pos_u32_t io;
    drv_api_t *drv = g_drv;
    io = drv->board->console_io_get();
    pin_rx = drv->os->pin->io[io*4+1];
    drv->os->gpio->mode_set(pin_rx, POS_GPIO_MODE_ANALOG_INPUT_PD);
  }
#endif

  /* refresh to normal by default */
  drv->s->ctrl |= MA_SENSOR_CTRL_STAT_NORMAL;
  drv->s->ctrl &= ~MA_SENSOR_CTRL_STAT_ERROR;
  
  return POS_STATUS_OK;

}

/** 
 * Sleep without system deep sleep so that PWM can still work
 */
void sensor_epd_sleep(epd_param_t *p, pos_u32_t ticks) {
  const pos_lib_tick_t *t;  
  t = p->drv.os->tick;
  ticks = t->get() + ticks;
  while( !t->is_timeout(t->get(), ticks) ) {
    t->sleep(20);
  }
}


/** 
 * Turn ON LCD screen
 */
void sensor_epd_on(epd_param_t *p) {
  if( (p->flags & DRV_EPD_FLAG_ON) == 0 ) {
    if( p->drv.cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      p->drv.log->data(g_epd_rev[p->hw_rev], p->value.flags);
    }
    /* 点亮屏幕供电 */
    epd_drv_on(p);    
    p->flags |= DRV_EPD_FLAG_ON; /* SET亮屏标志 */
  }
}

/** 
 * Turn OFF LCD screen
 */
void sensor_epd_off(epd_param_t *p) {
  if( p->flags & DRV_EPD_FLAG_ON ) {    
    /* 关闭屏幕供电 */
    epd_drv_off(p);    
    p->flags &= ~DRV_EPD_FLAG_ON; /* 清除亮屏标志 */

    if( p->drv.cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      p->drv.log->data("epd off", 0);
    }    
  }
}

/** 
 * Polling callback function
 * @note This function can NOT call g_drv. When polling, g_drv might piont to any sensor drivers. It should call the param stored driver pointer.
 */
void sensor_epd_poll(epd_param_t *p) {
  pos_u8_t clear_irq = 0;
  /* IRQ response for switching language/unit */
  if( p->irq_ticks ) {
    pos_u32_t v, t;
    if( p->drv.cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      p->drv.log->data("epd irq", p->irq_ticks); 
    }

    /* repeat at most 10s until btn released */
    for( pos_u32_t i = 0; i < 100; i++ ) {
      if( p->drv.os->gpio->get(p->pin_btn) )
        break;
      p->drv.os->tick->sleep(100);
    }
    v = p->drv.s->slot->rsvd32;
    v = (v & 0xfffffffc) + ((v+1)&0x3); // increase mode
    t = (pos_u32_t)&p->drv.s->slot->rsvd32 - MA_EEPROM_ADDR;
    p->drv.eeprom->update(t, &v, 4); // record
    clear_irq = 1;
  }
  
  /* Value Collect */  
  sensor_epd_value_collect(p);
  
  /* if no content change do nothing */
  if( !p->value.flags ) {
    return;
  }

  if( p->drv.cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    p->drv.log->data("epd poll refresh", p->value.flags);
  } 

  /* Refresh EPD */  
  epd_refresh(p, 0);     

  if( clear_irq )
    p->irq_ticks = 0; // clear irq flag at last
}


/** 
 * Driver init
 * @param[in]  load   0:Unload driver, 1:Load driver
 * @return     0: Successful\n
             !=0: Failed, refer to @ref pos_status_t
 * @note This function can be NULL if it does NOT require any init operatoin
 */ 
pos_status_t sensor_epd_init(pos_u32_t load) {
  drv_api_t *drv = g_drv;
  epd_param_t *param;
  ma_sensor_ctrl_t *s = g_drv->s;
  pos_gpio_pin_t pin[2], pin_btn;
#if 0  
  pin[0] = drv->board->pin_map(MB_PT_CTRL,MB_CTRL_ID_UDF3);
  pin[1] = drv->board->pin_map(MB_PT_CTRL,MB_CTRL_ID_UDF4);  
#else
  /* according to jason@20230508. 
   * this funciton is obsolte by hw. so remove these two pins to empty
   */
  pin[0] = POS_PH31;
  pin[1] = POS_PH31;
#endif
  pin_btn = drv->board->pin_map(MB_PT_DIO,MB_DIO_ID_BTN);
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("epd init", load+(pin_btn<<8));
  }  
  
  if( load ) {
    param = drv->os->malloc(sizeof(*param));
    if( !param )
      return POS_STATUS_E_MEM;
    drv->os->memset(param, 0, sizeof(*param));

    param->pin[0] = pin[0];
    param->pin[1] = pin[1];
    param->pin_btn = pin_btn;
    param->pin_busy =  drv->board->pin_map(MB_PT_CTRL,MB_CTRL_ID_UDF1);
    param->hw_rev = s->slot->thld.u16[1] & 0x3;

    drv->os->memcpy(&param->drv, drv, sizeof(*drv)); /* clone a g_drv */
    s->drv_buf = param;  /* record to drv_buf */

    /* init busy as INPUT PULLUP */
    drv->os->gpio->mode_set(param->pin_busy, POS_GPIO_MODE_INPUT); // according to jason@20230508, this has to be no PULLUP for power cost saving (reduce 50ua idle power)
    
    /* init pin as INPUT PULLUP */    
    drv->os->gpio->mode_set(param->pin[0], POS_GPIO_MODE_INPUT_PU);
    drv->os->gpio->mode_set(param->pin[1], POS_GPIO_MODE_INPUT_PU);

    /* init parameters */
    sensor_epd_collect();
    
    /* 
     * turn on irq
     * warning: as GOT is implemented in DATA and DATA is not supported in mmap()
     * all GOT based pointer can NOT be used. So the IRQ SET should use the mapped
     * function pointer here (the export structure). 
     * the orignal sensor_fastcnt_irq() should NOT be used directly.
     */     
    drv->os->gpio->irq_set(
      pin_btn, 
      POS_GPIO_MODE_INPUT+POS_GPIO_MODE_IRQ_FALLING+POS_GPIO_MODE_PU_ENABLE,
      (pos_func_t)drv->s->drv.u.sensor.irq_cb, /* has to be this. DO NOT use sensor_fastcnt_irq */
      param);

    /*
     * turn on poll
     */
    drv->proc->event_register((pos_func_t)drv->s->drv.u.sensor.poll_cb, param);

/*
 * according to alex@20230517 4.0 WeChat Group
 * Disable the EPD power on screen update
 */
#if 0
    if( drv->fsm < MA_FSM_COLLECT ) {
      /* 
       * 开机亮屏
       */      
      epd_refresh(param, 1);
    }
#endif

  } else {      
    /*
     * turn off poll
     */
    drv->proc->event_unregister((pos_func_t)drv->s->drv.u.sensor.poll_cb, s->drv_buf);

    /* 
     * turn off irq
     */
    drv->os->gpio->irq_set(pin_btn, POS_GPIO_MODE_INPUT_PU+POS_GPIO_MODE_IRQ_DISABLE, POS_NULL, POS_NULL);
    if( s->drv_buf ) {
      if( drv->fsm < MA_FSM_COLLECT ) {
      }    
      
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
  .version = DRV_SENSOR_VERSION(MA_SENSOR_EPDLCD, DRV_EPD_VERSION),
  .name = DRV_EPD_NAME,
  .u.sensor={
    .init = sensor_epd_init,
    .collect = sensor_epd_collect,
    .irq_cb = (pos_func_t)sensor_epd_irq,
    .poll_cb = (pos_func_t)sensor_epd_poll,    
  },
};

