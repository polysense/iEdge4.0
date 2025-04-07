/**
 * @file  drv_fastcnt.c
 * @brief SENSOR FastCounter IRQ interrupt driver
 * @author Runby F.
 * @date 2022-3-16
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * 版本修订历史
v1.0 @ 2022-03-16
  1) 完成第一版驱动

v1.1 @ 2022-03-18
  1) 解决中断回调函数指针异常

v1.2 @ 2022-03-22
  1) 支持DIO统一分配

v1.3 @ 2022-03-25
  1) 支持4.0整体供电策略

v1.4 @ 2022-05-14
  1) 支持内核统一管脚策略

v1.5 @ 2022-05-25
  1) Support different mode by srsv[n] filed
  2) Change default mode (srsv[n]=0) as: Read/Keep, State Report, 8B, Polling

v1.6 @ 2022-05-26
  1) Support CNTR increased only when state changed for STATE/REPORT mode (MODE_RAW_CNTR == 0)

v1.7 @ 2022-05-31
  1) Support EVENT/ACT with by default

v1.8 @ 2022-05-31
  1) Support type=48 (IO_GEN) and report PLSS/PL=108 EXT_IO
  2) Change driver name to IO to be the base type of all IO (DIO) sensors

v1.9 @ 2023-07-10
  1) Support turn ON PWRH_EN when PWRH is selected

v1.10 @ 2023-07-18
  1) Support sensor tyype=56 (IO_PULL)

v1.11 @ 2023-07-22
  1) Support sleep/check for type 56 (IO_PULL)

v1.12 @ 2023-07-22
  1) Fix type 56 pin mdoe not input issue
  2) Fix Type 56 not power on/off issue

v1.13 @ 2023-07-25
  1) Support mute time after each collecting to avoid some mis-report

v1.14 @ 2023-07-25
  1) Fix mute time wrong issue

v1.15 @ 2023-11-18
  1) Using new PWR ID solution  

v1.16 @ 2024-08-27
  1) Support mute time in unit of 1 minutes when >= 65001
  2) Support duty trigger
  3) Support set duty trigger mode when sensor type = TRIGGER type

v1.17 @ 2024-08-29
  1) Support cycle based irq on/off according to wechat requirement

v1.18 @ 2024-08-30
  1) Fix too many fc trigger issue

v1.19 @ 2024-09-03
  1) Use d0.bit[0..15] for fc trigger mute time (unit: 1ms, default 60000ms)

*/

/**
 * @brief Driver version
 */
#define DRV_FASTCNT_VERSION 0x0113

/**
 * @brief Driver name
 */
#define DRV_FASTCNT_NAME  "IO"

/**
 * @brief Driver will clear the CNTR after each reading
 */
#define DRV_FASTCNT_MODE_CLR_AFTER_READ       0x01

/**
 * @brief Driver will report the counter (excluding to report IO state in the highest bit)
 */
#define DRV_FASTCNT_MODE_RAW_CNTR             0x02

/**
 * @brief Driver will report using PLSS 32B counter format (or else using 8B counter)
 */
#define DRV_FASTCNT_MODE_PLSS_CNTR32          0x04

/**
 * @brief Driver will report only on each duty (excluding IRQ triggerred event report/polling)
 */
#define DRV_FASTCNT_MODE_MUTE_EVENT           0x08

/**
 * @brief Driver will count IRQ with single polarity
 */
#define DRV_FASTCNT_MODE_SINGLE_POLARITY      0x10

/**
 * @brief Driver will count IRQ IRQ with RISING (or else, FALLING). This control is only valid when single polarity mode.
 */
#define DRV_FASTCNT_MODE_POLARITY_RISING      0x20

/**
 * @brief Driver will support EVENT/ACT triggering for IO control
 */
#define DRV_FASTCNT_MODE_DISABLE_EVENT_ACT    0x40

/**
 * @brief Driver will trigger a next duty report for all configured sensors
 */
#define DRV_FASTCNT_MODE_TRIGGER              0x80

/**
 * @brief IRQ has been acknowledged by event/polling
 */
#define DRV_FASTCNT_FLAG_IRQ_ACK              0x00001

/**
 * @brief Duty has been processed
 */
#define DRV_FASTCNT_FLAG_DUTY_PROCESS         0x00002


/**
 * @brief IRQ driver structure
 */
typedef struct {
  drv_api_t     drv;
  pos_u32_t     last_cnt;
  pos_u8_t      pin;
  pos_u8_t      mode;
  pos_u8_t      state;
  pos_u8_t      act_pwr_id;
  pos_u32_t     mute_to;
  pos_u32_t     flags;
} irq_param_t;


/** 
* Driver event act handling
*/ 
void sensor_fastcnt_event_act(irq_param_t *param){
  drv_api_t *drv = &param->drv;
  pos_u32_t t, dbg, state;

  dbg = param->drv.cfg->ctrl & MA_CFG_CTRL_DRV_LOG;
  
  /* 0: UNLOCK, 1: LOCKED */
  state = drv->os->gpio->get(param->pin);

  do {
    /* return if already UNLOCK */  
    if( !state ) {
      break;
    }

    /* turn ON ACT_PWR_ID for UNLOCKing */
    drv->board->pwr_set(param->act_pwr_id, 1);
    if( dbg )
      param->drv.log->data("fc act on", param->act_pwr_id);  

    t = drv->os->tick->get() + 2000;
    while( !drv->os->tick->is_timeout(drv->os->tick->get(), t) ) {
      state = drv->os->gpio->get(param->pin);
      if( !state )
        break; /* break if UNLOCKed */
      /* wait until state is UNLOCK or TIMEOUT */
      drv->os->tick->sleep(20);
    }
    /* turn OFF ACT_PWR_ID for no action */
    drv->board->pwr_set(param->act_pwr_id, 0);  
    if( dbg )
      param->drv.log->data("fc act off", param->act_pwr_id);      
  } while(0);
  
  if( dbg ) {
    param->drv.log->data("fc ret", state);
  }  
}

/** 
* Driver state & counter report
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_fastcnt_state_report(irq_param_t *param, pos_u8_t skip_same){
  pos_status_t ret;
  drv_api_t *drv = &param->drv;
  ma_sensor_ctrl_t *s;
  pos_u32_t cnt, vp8;

  /* init s */
  s = drv->s;

  /* read cntr and state */
  drv->os->task->lock_irq();
  cnt = s->data.u32;
  if( param->mode & DRV_FASTCNT_MODE_CLR_AFTER_READ) {
   s->data.u32 = 0;
  }
  vp8 = drv->os->gpio->get(param->pin);
  drv->os->task->unlock_irq();

  /* generate cnt as Bit[31]:state + Bit[0..30]:cntr for STATE REPORT mode */
  if( (param->mode & DRV_FASTCNT_MODE_RAW_CNTR) == 0 ) {
    if( param->state == vp8 ) {
      /* skip same state */
      if( skip_same )
        return POS_STATUS_OK;
    } else {    
      /* increase cnt when state changed */    
      cnt = param->last_cnt + 1;
    }
  } else {
    /* skip same */
    if( skip_same ) {
      if( (param->mode & DRV_FASTCNT_MODE_CLR_AFTER_READ) == 0  ) {
        if( param->last_cnt == cnt )
          return POS_STATUS_OK; /* skip cntr same when READ/KEEP */
      } else if( !cnt ) {
        return POS_STATUS_OK; /* skip cntr zero when READ/CLR */
      }
    }
  }

  /* record cnt&state */
  param->last_cnt = cnt;
  param->state = vp8;

  /* Data put */
  ret = drv->data->put("\"%s\":%u,", "cnt", cnt, 0,0,0);
  if( ret != POS_STATUS_OK ) {    
    s->ctrl |= MA_SENSOR_CTRL_TOO_LONG; /* indicate data buffer put failed */      
  }

  /* State put */    
  if( (param->mode & DRV_FASTCNT_MODE_RAW_CNTR) == 0 ) {
    ret = drv->data->put("\"%s\":%u,", "state", vp8, 0,0,0);
    /* re-generate cnt32/8 with state */
    cnt = ((vp8&0x1)<<31) + (cnt&0x7fffffff); /* re-generate new cnt with state */
    vp8 = ((vp8&0x1)<<7) + (cnt&0x7f); /* re-generate new cnt8 with state */
  } else {
    /* keep cnt32 and copy lowest 8b to cnt8 */
    vp8 = cnt & 0xff;
  }
  
  /* PLSS put */
  if( param->mode & DRV_FASTCNT_MODE_PLSS_CNTR32 ) {
    /* 32B */
    drv->data->plss_put_u32(19, cnt); /* PLSS_PL_EXT_CNTR:19, Bit31: IO State, Bit[0..30]: IRQ cntr */
  } else {  
    /* 8B */
    drv->data->plss_put_u8(
      drv->s->slot->sensor_type == MA_SENSOR_FASTCNT ? 56 : 108, 
                                                          /* (type=3) PLSS_PL_EXT_LEAKAGE:56, Bit7: IO State, Bit[0..6]: IRQ cntr */
                                                          /* (type=48) PLSS_PL_EXT_IO:108, Bit7: IO State, Bit[0..6]: IRQ cntr */
      vp8); 
  }
      
  /* Release and return */
  return ret;
  
}

/** 
* Driver collecting
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_fastcnt_io_pull_collect(void){
  pos_status_t ret = POS_STATUS_OK;
  pos_gpio_pin_t pin;
  drv_api_t *drv = g_drv;
  ma_sensor_ctrl_t *s = g_drv->s;
  pos_u8_t v8, exp;
  pos_u32_t sleep, to;
#if 0  
  pos_u32_t i;
#endif
  pin = drv->board->pin_map(MB_PT_DIO, s->slot->io);
  drv->os->gpio->mode_set(pin, POS_GPIO_MODE_INPUT);
  v8 = drv->os->gpio->get(pin);
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("io pin", pin);
    drv->log->data("io srsv", s->slot->rsvd32);
    drv->log->data("io state", v8);
  }  

  /* support expected result wait/sleep */
  sleep = s->slot->rsvd32;
  if( sleep & 0x7fff ) {
    if( sleep & 0x80000000 )
      exp = 0; /* expect to be low, when 0x80000000 defined */
    else
      exp = 1; /* or else, expect to be high */
    to = sleep & 0x7fff;
    if( (sleep & 0x8000) == 0 )
      to *= 1000; /* unit s, needs to be x 1000 */
    sleep = (sleep >> 16) & 0x7fff; /* unit ms */    
    if( !sleep )
      sleep = 100; /* default 100ms sleep */
#if 0    
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      drv->log->data("io to", to);
      drv->log->data("io sleep", sleep);
    }  
    i = 0;
#endif
    to += drv->os->tick->get();     

//    MB_PIN_LED_SET_R(drv->os, MB_LED_ON); 
    while( v8 != exp && !drv->os->tick->is_timeout(drv->os->tick->get(), to) ) {
      drv->os->tick->sleep(sleep);
      v8 = drv->os->gpio->get(pin);
#if 0      
      i++;
      if( v8 )
        i += 0x10000;
#endif      
    }
//    MB_PIN_LED_SET_R(drv->os, MB_LED_OFF); 
#if 0
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      drv->log->data("io state", v8);
      drv->log->data("io cnt", i);
    }       
#endif    
  }


  /* suppress */
  drv->thld->u16_process(s, v8);
  if( s->ctrl & MA_SENSOR_CTRL_REPORT )  {    
    /* report normally */
    ret = drv->data->plss_json_put(v8,
      DRV_MA_PLSS_TRANS(PLSS_PL_EXT_DIGITAL, 1, 0, 0, 0),
      "io");
  } else {    
    /* cancel this round */
    drv->call(DRV_CALL_FUNC_ID_SET_DATA_CLEAR, 0);
  }
  
  /* Release and return */
  return ret;
  
}

/**
* Driver IRQ ON/OFF set
*/
void sensor_fastcnt_irq_set(irq_param_t *param, pos_u32_t on) {
  pos_u32_t v;
  if( on ) {
    /* set PD+RISING for Single Polarity, Rising
     * set PU+FALLING for Single Polarity, !Rising
     * set RISING+FALLSING for !Single Polarity
     */
    if( param->mode & DRV_FASTCNT_MODE_SINGLE_POLARITY ) {
      if( param->mode & DRV_FASTCNT_MODE_POLARITY_RISING ) {
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
     * the orignal sensor_fastcnt_irq() should NOT be used directly.
     */
    param->drv.os->gpio->irq_set(
      param->pin,
      v,
      (pos_func_t)param->drv.s->drv.u.sensor.irq_cb, /* has to be this. DO NOT use sensor_fastcnt_irq */
      param);
  } else {
    /*
     * turn off irq
     */
    param->drv.os->gpio->irq_set(param->pin, POS_GPIO_MODE_INPUT_PU+POS_GPIO_MODE_IRQ_DISABLE, POS_NULL, POS_NULL);
  }
}

/** 
* Driver collecting
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_fastcnt_collect(void){
  pos_status_t ret;
  pos_u32_t i;
  irq_param_t *iq;
  if( g_drv->s->slot->sensor_type >= MA_SENSOR_IO_PULL ) 
    return sensor_fastcnt_io_pull_collect();

  iq = (irq_param_t*)g_drv->s->drv_buf;

  /* do NOT report for TRIGGER mode */
  if( iq->mode & DRV_FASTCNT_MODE_TRIGGER ) {
    iq->last_cnt = iq->drv.s->data.u32;
    iq->flags &= ~DRV_FASTCNT_FLAG_IRQ_ACK; /* clear IRQ ack flag */
    iq->flags |= DRV_FASTCNT_FLAG_DUTY_PROCESS; /* set DUTY PROCESS flag */
    return POS_STATUS_OK;
  }

  ret = sensor_fastcnt_state_report(iq, 0);

  /* set mute time if defined */
  i = g_drv->s->slot->arg.u16[0];
  if( i ) {
    if( i >= 65001 )
      i = (i - 65000) * 60000; /* in unit of 1 minutes when >= 65001 */
    i += g_drv->os->tick->get();
    if( !i )
      i = 1;
    iq->mute_to = i;
  }
  
  /* Release and return */
  return ret;
  
}

/** 
* Driver polling
*/ 
void sensor_fastcnt_poll(irq_param_t *param){
  pos_u32_t log;
  drv_api_t *drv = &param->drv;
  log = drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG;

  /* trigger process */
  if( param->mode & DRV_FASTCNT_MODE_TRIGGER ) {
    do {
      pos_u32_t i;

      /* mute for given time if duty is reported last time */
      if( param->flags & DRV_FASTCNT_FLAG_DUTY_PROCESS ) {
        i = drv->s->slot->arg.u16[0];
        if( i >= 65001 )
          i = (i - 65000) * 60000; /* in unit of 1 minutes when >= 65001 */
        if( !i )
          i = 60000;
        if( log ) {
          drv->log->data("fc trigger mute", i);
        }
        /* sleep for given time with IRQ OFF */
        sensor_fastcnt_irq_set(param, 0); /* off */
        drv->os->tick->sleep(i);
        sensor_fastcnt_irq_set(param, 1); /* on */
        param->last_cnt = drv->s->data.u32; /* flush all IRQ counters */
        param->flags &= ~DRV_FASTCNT_FLAG_DUTY_PROCESS; /* clear DUTY PROCESS */
      }

      if( (param->flags & DRV_FASTCNT_FLAG_IRQ_ACK) != 0 ) {
        param->last_cnt = drv->s->data.u32;
        break;
      }

      /* when not irq detected, do nothing */
      if( param->last_cnt == drv->s->data.u32 )
        break;

      /* duty reset for immediately report */
      if( log ) {
        drv->log->data("fc trigger", drv->s->data.u32);
      }
      drv->call(DRV_CALL_FUNC_ID_SET_NEXT_DUTY, 0); /* use 0ms here */
      param->last_cnt = drv->s->data.u32;
      param->flags |= DRV_FASTCNT_FLAG_IRQ_ACK; /* set IRQ ack flag */
    } while(0);
    return;
  }

  /* mute check */
  if( param->mute_to ) {
    const pos_lib_t *os = drv->os;
    if( !os->tick->is_timeout(os->tick->get(), param->mute_to) ) {
      if( log ) {
        drv->log->data("fc mute", param->mute_to);
      }
      return; /* do NOTHING if mute detected */
    } else {
      param->mute_to = 0;
    }
  }

  if( log ) {
    drv->log->data("fc poll", drv->s->data.u32);
  } 

  /* perform EVENT_ACT when current FSM is EVENT_ACT and MODE is ON */
  if( g_drv->fsm == MA_FSM_COLLECT_EVENT_ACT ) { /* should use global g_drv here */
    if( log ) {
      drv->log->data("fc act", param->mode);
    }
    if( (param->mode & DRV_FASTCNT_MODE_DISABLE_EVENT_ACT) == 0 )
      sensor_fastcnt_event_act(param);
    return;
  }
  
  /* skip when MUTED */
  if( param->mode & DRV_FASTCNT_MODE_MUTE_EVENT )
    return;
  
  sensor_fastcnt_state_report(param, 1);
}

/** 
 * IRQ interrupt callback handler
 * @note This function can NOT call g_drv. When IRQ happening, g_drv might piont to any sensor drivers. It should call the param stored driver pointer.
 */
void sensor_fastcnt_irq(irq_param_t *param) {
  param->drv.s->data.u32++;
  param->drv.os->gpio->irq_clr(param->pin);
}

/** 
* Power control
* @param[in]  on   0:Power off, 1:Power on
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
* @note This function can be NULL if it does NOT require any power operatoin
*/ 
pos_status_t sensor_fastcnt_set(pos_u32_t on) {  
  pos_gpio_pin_t pwr;
  ma_board_t *b = g_drv->board;
  /* only IO PULLING will use power on/off */
  if( g_drv->s->slot->sensor_type >= MA_SENSOR_IO_PULL ) {
    pwr = (g_drv->s->slot->io >> 8) & 0x7f;  
    if( pwr ) {
      b->pwr_set(pwr, on);
    }
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
pos_status_t sensor_fastcnt_init(pos_u32_t load) {
  drv_api_t *drv = g_drv;
  irq_param_t *param;
  ma_sensor_ctrl_t *s = g_drv->s;
  pos_gpio_pin_t pin;
  pos_u8_t pwr;  
  
  /* do NOTHING for PULL mode */
  if( g_drv->s->slot->sensor_type == MA_SENSOR_IO_PULL ) {
    return POS_STATUS_OK;
  }

  /* IRQ init */
  pin = drv->board->pin_map(MB_PT_DIO, s->slot->io);
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("fc init", load);
    drv->log->data("fc pin", pin);
    drv->log->data("fc srsv", s->slot->rsvd32);
  }  

  /* call power on/off only when PWR is defined */
  pwr = (s->slot->io >> 8) & 0x7f;  
  if( pwr ) {
    drv->board->pwr_set(pwr, load);
  }
  
  if( load ) {
    s->data.u32 = 0; /* clear counter by default */
    param = drv->os->malloc(sizeof(*param));
    if( !param )
      return POS_STATUS_E_MEM;

    drv->os->memset(param, 0, sizeof(*param));
    param->pin = pin;
    param->mode = s->slot->rsvd32 & 0xff; /* record mode, when default 0 for: Read/keep, State Report, 8B Cntr, Polling, RISING+FALLSING TRIGGER, NO_EVENT_ACT */    
    if( g_drv->s->slot->sensor_type == MA_SENSOR_IO_TRIGGER )
      param->mode |= DRV_FASTCNT_MODE_TRIGGER;
    param->act_pwr_id = (s->slot->rsvd32 >> 8) & 0x7f; /* record pwr id */
    if( !param->act_pwr_id )
      param->act_pwr_id = MB_PWR_ID_VCC_PWRH; /* default using PWRH */
    drv->os->memcpy(&param->drv, drv, sizeof(*drv)); /* clone a g_drv */    
    s->drv_buf = param;  /* record to drv_buf */

    /*
     * turn on irq
     */
    sensor_fastcnt_irq_set(param, 1);

    /*
     * turn on poll when EVENT/POLL is NOT MUTED
     */
    if( (param->mode & DRV_FASTCNT_MODE_MUTE_EVENT) == 0  ) {   
      drv->proc->event_register((pos_func_t)drv->s->drv.u.sensor.poll_cb, param);    
    }

  } else {
    /*
     * turn off poll
     */
    drv->proc->event_unregister((pos_func_t)drv->s->drv.u.sensor.poll_cb, s->drv_buf);  

    if( s->drv_buf ) {
      /*
       * turn off irq
       */
      sensor_fastcnt_irq_set((irq_param_t*)s->drv_buf, 0);

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
  .version = DRV_SENSOR_VERSION(MA_SENSOR_FASTCNT, DRV_FASTCNT_VERSION),
  .name = DRV_FASTCNT_NAME,
  .u.sensor={
    .init = sensor_fastcnt_init,
    .power_set = sensor_fastcnt_set, 
    .collect = sensor_fastcnt_collect,
    .irq_cb = (pos_func_t)sensor_fastcnt_irq,
    .poll_cb = (pos_func_t)sensor_fastcnt_poll,    
  },
};

