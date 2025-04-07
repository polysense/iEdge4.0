/**
 * @file  drv_cntr12.c
 * @brief SENSOR FastCounter IRQ interrupt driver
 * @author Runby F.
 * @date 2022-3-16
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * Revision history
v1.0 @ 2022-06-16
  1) First draft

v1.1 @ 2022-06-19
  1) Fix IRQ ACK issue

v1.2 @ 2023-11-18
  1) Using new PWR ID solution

v1.3 @ 2024-04-10
  1) Support to report accumulating counter when thw0.bit[0]=1
  
*/

/**
 * @brief Driver version
 */
#define DRV_CNTR12_VERSION 0x0103

/**
 * @brief Driver name
 */
#define DRV_CNTR12_NAME  "CNTR12"

/**
 * @brief Driver will count IRQ with single polarity
 */
#define DRV_CNTR12_MODE_SINGLE_POLARITY      0x10000

/**
 * @brief Driver will count IRQ IRQ with RISING (or else, FALLING). This control is only valid when single polarity mode.
 */
#define DRV_CNTR12_MODE_POLARITY_RISING      0x20000


/**
 * @brief IRQ driver structure
 */
typedef struct {
  drv_api_t     *p_drv;
  pos_u32_t     *p_state_tick;
  pos_u32_t     *p_state_exp;
  pos_u32_t     *p_irq_tick;  
  pos_u32_t     cnt;
  pos_u32_t     jitter_delay;
  pos_u32_t     idle_gap;
  pos_u8_t      my_state;
  pos_u8_t      pin;
  pos_u16_t     rsvd;
} c12_param_t;

/**
 * @brief IRQ driver malloc structure
 */
typedef struct {
  drv_api_t     drv;
  pos_u32_t     state_tick;
  pos_u32_t     state_exp;
  pos_u32_t     irq_tick;
  c12_param_t   p[2];
} c12_param_malloc_t;

/** 
* Driver collecting
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_cntr12_collect(void){
  pos_status_t ret;
  c12_param_malloc_t *m;
  pos_u16_t cnt[2], i;
  pos_u32_t v;
  m = (c12_param_malloc_t*)g_drv->s->drv_buf;

  /* read cntr */
  g_drv->os->task->lock_irq();
  for( i = 0; i < 2; i++ ) {
    cnt[i] = m->p[i].cnt;
    m->p[i].cnt = 0;
  }
  g_drv->os->task->unlock_irq();

  v = (cnt[1] << 16) + cnt[0];
  g_drv->data->plss_put_u32(PLSS_PL_EXT_CNTR_12, v);

  ret = g_drv->data->put("\"%s\":[%u,%u],", "cntr12", cnt[1], cnt[0], 0, 0);

  /* accumulating&report by mv[2]/mv[3] */
  if( g_drv->s->slot->thld.u16[0] & 0x01 ) {
    pos_u16_t *sum;
    sum = &g_drv->history->mv[2];
    sum[0] += cnt[0];
    sum[1] += cnt[1];
    v = (sum[1] << 16) + sum[0];
    g_drv->data->plss_put_u32(PLSS_PL_EXT_CNTR_12, v);
    ret = g_drv->data->put("\"%s\":[%u,%u],", "cntr12", sum[1], sum[0], 0, 0);
  }

  /* Release and return */
  return ret;

}


/** 
 * IRQ interrupt callback handler
 * @note This function can NOT call g_drv. When IRQ happening, g_drv might piont to any sensor drivers. It should call the param stored driver pointer.
 */
void sensor_cntr12_irq(c12_param_t *param) {
  const pos_lib_tick_t *tick = param->p_drv->os->tick;
  pos_u32_t now;
  drv_api_t *drv = param->p_drv;
  
  now = tick->get();
#if 0
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("irq", now);
    drv->log->data("my", param->my_state);    
    drv->log->data("exp", *param->p_state_exp);        
    drv->log->data("stick", *param->p_state_tick);    
    drv->log->data("stick", *param->p_irq_tick);        
  }
#endif  
  /* anti irq interrupts jitter (mis-report) */
  if( tick->elaps_calc(now, *param->p_state_tick) >= param->jitter_delay ) {   
    
    /* when irq is idle for more than given time, clear it */
    if( tick->elaps_calc(now, *param->p_irq_tick) >= param->idle_gap )
      *param->p_state_exp = 0; /* clear flag */

    if( *param->p_state_exp == param->my_state ) {
      param->cnt++; /* #1:2->1 or #0:1->2 increasing */
      *param->p_state_tick = now;
      *param->p_state_exp = 0; /* clear flag */
    } else
      *param->p_state_exp = param->my_state == 1 ? 2 : 1;

    /* update irq tick */
    *param->p_irq_tick = now;
  }

  /* IRQ must be ACK after quit */
  drv->os->gpio->irq_clr(param->pin);
}


/** 
 * Driver init
 * @param[in]  load   0:Unload driver, 1:Load driver
 * @return     0: Successful\n
             !=0: Failed, refer to @ref pos_status_t
 * @note This function can be NULL if it does NOT require any init operatoin
 */ 
pos_status_t sensor_cntr12_init(pos_u32_t load) {
  drv_api_t *drv = g_drv;
  c12_param_malloc_t *pm;
  c12_param_t *param;
  ma_sensor_ctrl_t *s = g_drv->s;
  pos_gpio_pin_t pin1, pin2;
  mb_pwr_id_t pwr;  
  pin1 = drv->board->pin_map(MB_PT_DIO, s->slot->io);
  pin2 = drv->board->pin_map(MB_PT_DIO, s->slot->rsvd32&0xffff);
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("c12 init", load);
    drv->log->data("c12 pin1", pin1);
    drv->log->data("c12 pin2", pin2);    
    drv->log->data("c12 srsv", s->slot->rsvd32);
    drv->log->data("c12 d0", s->slot->arg.u32[0]);  
  }  

  /* call power on/off only when PWR is defined */
  pwr = (s->slot->io >> 8) & 0x7f;  
  if( pwr ) {
    drv->board->pwr_set(pwr, load);
  }
  
  if( load ) {
    pos_u32_t v;
    s->data.u32 = 0; /* clear counter by default */
    pm = (c12_param_malloc_t*)drv->os->malloc(sizeof(*pm));
    if( !pm )
      return POS_STATUS_E_MEM;
    drv->os->memset(pm, 0, sizeof(*pm));
    drv->os->memcpy(&pm->drv, drv, sizeof(*drv)); /* clone a g_drv */    
    s->drv_buf = pm;  /* record to drv_buf */

    /* setup p[0] */
    param = &pm->p[0];
    param->p_drv = &pm->drv;
    param->p_state_tick = &pm->state_tick;
    param->p_state_exp = &pm->state_exp;
    param->p_irq_tick = &pm->irq_tick;
    param->my_state = 1;
    param->pin = pin1;
    param->jitter_delay = s->slot->arg.u16[0];
    if( !param->jitter_delay )
      param->jitter_delay = 500;
    param->idle_gap = s->slot->arg.u16[1];
    if( !param->idle_gap )
      param->idle_gap = 2000;

    /* setup p[1] */
    /* clone p[0] to p[1] and change p[1]'s my state */
    drv->os->memcpy(&pm->p[1], &pm->p[0], sizeof(pm->p[1]));
    pm->p[1].my_state = 2;
    pm->p[1].pin = pin2;    

    /* set PD+RISING for Single Polarity, Rising
     * set PU+FALLING for Single Polarity, !Rising
     * set RISING+FALLSING for !Single Polarity
     */
    if( s->slot->rsvd32 & DRV_CNTR12_MODE_SINGLE_POLARITY ) {
      if( s->slot->rsvd32 & DRV_CNTR12_MODE_POLARITY_RISING ) { 
        v = POS_GPIO_MODE_INPUT_PD + POS_GPIO_MODE_IRQ_RISING;
      } else {
        v = POS_GPIO_MODE_INPUT_PU + POS_GPIO_MODE_IRQ_FALLING;
      }
    } else {
      v = POS_GPIO_MODE_INPUT_IRQ_RISING_FALLING;
    }
    
    /* 
     * turn on irq
     * warning: as GOT is implemented in DATA and DATA is not supported in mmap()
     * all GOT based pointer can NOT be used. So the IRQ SET should use the mapped
     * function pointer here (the export structure). 
     * the orignal sensor_cntr12_irq() should NOT be used directly.
     */     
    drv->os->gpio->irq_set(
      pin1, 
      v,
      (pos_func_t)drv->s->drv.u.sensor.irq_cb, /* has to be this. DO NOT use sensor_cntr12_irq */
      &pm->p[0]);
    drv->os->gpio->irq_set(
      pin2, 
      v,
      (pos_func_t)drv->s->drv.u.sensor.irq_cb, /* has to be this. DO NOT use sensor_cntr12_irq */
      &pm->p[1]);

  } else {
    /* 
     * turn off irq
     */
    drv->os->gpio->irq_set(pin1, POS_GPIO_MODE_INPUT_PU+POS_GPIO_MODE_IRQ_DISABLE, POS_NULL, POS_NULL);
    drv->os->gpio->irq_set(pin2, POS_GPIO_MODE_INPUT_PU+POS_GPIO_MODE_IRQ_DISABLE, POS_NULL, POS_NULL);
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
  .version = DRV_SENSOR_VERSION(MA_SENSOR_CNTR12, DRV_CNTR12_VERSION),
  .name = DRV_CNTR12_NAME,
  .u.sensor={
    .init = sensor_cntr12_init,
/*     .power_set = sensor_cntr12_set, */
    .collect = sensor_cntr12_collect,
    .irq_cb = (pos_func_t)sensor_cntr12_irq,
  },
};

