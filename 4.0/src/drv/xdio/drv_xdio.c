/**
 * @file  drv_xdio.c
 * @brief SENSOR FastCounter IRQ interrupt driver
 * @author Runby F.
 * @date 2022-3-16
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * Revision history
v1.0 @ 2023-11-30
  1) First draft

v1.1 @ 2024-02-27
  1) Fix multiple slots causing poll/collect wrong parameter issue

v1.2 @ 2024-04-07
  1) Enhance the setting judgement to decrease the irq loss possibility

*/

/**
 * @brief Driver version
 */
#define DRV_XDIO_VERSION 0x0102

/**
 * @brief Driver name
 */
#define DRV_XDIO_NAME  "XDIO"

/**
 * @brief Max DIO count
 */
#define DRV_XDIO_NUM    0x20


/**
 * @brief IRQ callback structure
 */
typedef struct {
  void          *param;
  pos_u32_t     cnt;
  pos_u8_t      pin;
  pos_u8_t      rsvd;
  pos_u16_t     rsvd2;
} xdio_cb_param_t;

/**
 * @brief IRQ driver structure
 */
typedef struct {
  drv_api_t       drv;
  pos_u32_t       bmp;
  pos_u32_t       sum_cnt; /* all IRQ cnt */
  pos_u32_t       cnt_process; /* IRQ processed cnt */
  pos_u16_t       rsvd;
  pos_u8_t        rsvd2;
  pos_u8_t        cb_cnt;
  xdio_cb_param_t  cb[DRV_XDIO_NUM];
} xdio_param_t;

/** 
* Driver collecting
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_xdio_poll_collect(pos_u8_t report_when_same, drv_api_t *drv){
  pos_status_t ret = POS_STATUS_OK;
  ma_sensor_ctrl_t *s = drv->s;
  xdio_param_t *param = (xdio_param_t*)s->drv_buf;
  pos_u32_t sleep, v, bmp, i, max;
#if 0  
  pos_u32_t i;
#endif
  sleep = s->slot->arg.u8[1];
  max = s->slot->arg.u8[2];
  i = 0;
  if( !sleep )
    sleep = 25;
  sleep *= 100; /* translate from 0.1s to ms */
  do {
    pos_u32_t old_cnt = param->sum_cnt; /* remember old cnt */
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      drv->log->data("xdio wait", sleep);
    }
    param->cnt_process = param->sum_cnt;  /* ack cnt processed before sleep to decrease the possibility of irq lost */
    drv->os->tick->sleep(sleep);
    bmp = 0;
    for( v = 0; v < param->cb_cnt; v++ ) {
      if( drv->os->gpio->get(param->cb[v].pin) )
        bmp |= 1<<v;
    }
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      drv->log->data("xdio bmp", bmp);
    }  
    /* quit settling check when cnt is not changed */
    if( old_cnt == param->sum_cnt )
      break;
    /* or else, retry another settling down */
  } while(i++ < max);

  /* skip same when not report_when_same */
  if( !report_when_same && bmp == param->bmp )
    return ret; /* same */

  /* record new bmp */
  param->bmp = bmp;
  return drv->data->plss_json_put(bmp,
      (param->cb_cnt > 16) ? DRV_MA_PLSS_TRANS(PLSS_PL_EXT_CNTR, 4, 0, 0, 0) : 
                       DRV_MA_PLSS_TRANS(PLSS_PL_POS_BMP16, 2, 0, 0, 0),
      "bmp"); /* report PLSS_CNT when > 16 */
  
}

/** 
* Driver collecting
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_xdio_collect(void){
  /* report even when same */
  return sensor_xdio_poll_collect(1, g_drv);
  
}

/** 
* Driver polling
*/ 
void sensor_xdio_poll(xdio_param_t *param){
  if( param->drv.cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    param->drv.log->data("xdio poll", param->drv.s->data.u32);
  } 

  /* skip when no new irq */
  if( param->cnt_process == param->sum_cnt )
    return;

  /* only report when delta */
  sensor_xdio_poll_collect(0, &param->drv);
}

/** 
 * IRQ interrupt callback handler
 * @note This function can NOT call g_drv. When IRQ happening, g_drv might piont to any sensor drivers. It should call the param stored driver pointer.
 */
void sensor_xdio_irq(xdio_cb_param_t *cb) {
  xdio_param_t *param = (xdio_param_t*)cb->param;
  cb->cnt++;
  param->sum_cnt++;
  param->drv.os->gpio->irq_clr(cb->pin);
}


/** 
 * Driver init
 * @param[in]  load   0:Unload driver, 1:Load driver
 * @return     0: Successful\n
             !=0: Failed, refer to @ref pos_status_t
 * @note This function can be NULL if it does NOT require any init operatoin
 */ 
pos_status_t sensor_xdio_init(pos_u32_t load) {
  drv_api_t *drv = g_drv;
  ma_sensor_ctrl_t *s = g_drv->s;
  xdio_param_t *param = (xdio_param_t*)s->drv_buf;
  pos_u8_t pwr;  
  pos_u32_t v, i;  

  /* IRQ init */
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("xdio init", load);
    drv->log->data("xdio io", s->slot->io);
    drv->log->data("xdio d0", s->slot->arg.u32[0]);
  }  

  /* call power on/off only when PWR is defined */
  pwr = (s->slot->io >> 8) & 0x7f;  
  if( pwr ) {
    drv->board->pwr_set(pwr, load);
  }

  /* refresh to normal by default */
  drv->s->ctrl |= MA_SENSOR_CTRL_STAT_NORMAL;
  drv->s->ctrl &= ~MA_SENSOR_CTRL_STAT_ERROR;
  
  if( load ) {
    s->data.u32 = 0; /* clear counter by default */
    param = drv->os->malloc(sizeof(*param));
    if( !param ) {
      drv->s->ctrl |= MA_SENSOR_CTRL_STAT_ERROR;
      return POS_STATUS_E_MEM;
    }
    
    drv->os->memset(param, 0, sizeof(*param));
    param->cb_cnt = s->slot->arg.u8[0];
    if( !param->cb_cnt )
      param->cb_cnt = 10;
    if( param->cb_cnt > DRV_XDIO_NUM )
      param->cb_cnt = DRV_XDIO_NUM;
    drv->os->memcpy(&param->drv, drv, sizeof(*drv)); /* clone a g_drv */    
    s->drv_buf = param;  /* record to drv_buf */

    /* init each irq */
    for( v = 0; v < param->cb_cnt; v++ ) {
      param->cb[v].param = param;
      if( s->slot->io&0x8000 ) {
        /* use raw PIN name */
        i = v+(s->slot->io&0xff); 
      } else {
        i = v+(s->slot->io&0x3f);
        if(s->slot->rsvd32 & 1) {
          /* use DIO */
          i = drv->os->pin->dio[i&7];
        } else {
          /* use RSVD */
          i = drv->os->pin->rsvd[i];
        }
      }          
      param->cb[v].pin = i; /* save pin */

      /* 
       * turn on irq
       * warning: as GOT is implemented in DATA and DATA is not supported in mmap()
       * all GOT based pointer can NOT be used. So the IRQ SET should use the mapped
       * function pointer here (the export structure). 
       * the orignal sensor_xdio_irq() should NOT be used directly.
       */     
      drv->os->gpio->irq_set(
        param->cb[v].pin, 
        POS_GPIO_MODE_INPUT_IRQ_RISING_FALLING,
        (pos_func_t)drv->s->drv.u.sensor.irq_cb, /* has to be this. DO NOT use sensor_xdio_irq */
        &param->cb[v]);      
      
      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
        drv->log->data("xdio pin", param->cb[v].pin);      
      }
    }
    
    /* turn on poll */
    drv->proc->event_register((pos_func_t)drv->s->drv.u.sensor.poll_cb, param);    

  } else if( param ) {        
    /*
     * turn off poll
     */     
    drv->proc->event_unregister((pos_func_t)drv->s->drv.u.sensor.poll_cb, s->drv_buf);  
    
    /* 
     * turn off irq
     */
    for( v = 0; v < param->cb_cnt; v++ ) {     
      drv->os->gpio->irq_set(param->cb[v].pin, POS_GPIO_MODE_INPUT_PU+POS_GPIO_MODE_IRQ_DISABLE, POS_NULL, POS_NULL);
    }
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
  .version = DRV_SENSOR_VERSION(MA_SENSOR_XDIO, DRV_XDIO_VERSION),
  .name = DRV_XDIO_NAME,
  .u.sensor={
    .init = sensor_xdio_init,
    .collect = sensor_xdio_collect,
    .irq_cb = (pos_func_t)sensor_xdio_irq,
    .poll_cb = (pos_func_t)sensor_xdio_poll,    
  },
};

