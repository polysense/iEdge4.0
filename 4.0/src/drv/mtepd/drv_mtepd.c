/**
 * @file  drv_mtepd.c
 * @brief MT LCD/RFID driver
 * @author Runby F.
 * @date 2022-4-1
 * @copyright Polysense
 */

#include "drv_api.h"
#include "drv_mtepd.h"

/* 
 * 版本修订历史
v1.0 @ 2024-07-07
  1) First draft

v1.1 @ 2024-07-13
  1) Display MTEPD CO as 0.1 unit
  2) Do NOT process BTN when BTN is not defined (PH31)
  3) Suppress beep/alarm during refresh before and after

v1.2 @ 2024-07-13
  1) Display EUI in top left

v1.3 @ 2024-08-05
  1) Support thw2/thw3 to control low/high refreshing voltage
  2) Support 0805 revision of UC8276CAA driver

v1.4 @ 2024-08-16
  1) Display -- when temp/hum is not ready
  2) Correct grp-3 displaying issue (overlapped with EUI)

v1.5 @ 2024-08-18
  1) Display -- when co/o2/h2s is not ready

v1.6 @ 2024-10-21
  1) Display AW when thw1.bit[10](set thw1[n] 0x400) is defined
  2) Adjust TEMP invalid to -273.0...999.9

*/

/**
 * @brief Driver version
 */
#define DRV_MTEPD_VERSION 0x0106

/**
 * @brief Driver name
 */
#define DRV_MTEPD_NAME  "MTEPD"


/** 
 * IRQ interrupt callback handler
 * @note This function can NOT call g_drv. When IRQ happening, g_drv might piont to any sensor drivers. It should call the param stored driver pointer.
 */
void sensor_mtepd_irq(mtepd_param_t *param) {
  pos_u32_t ticks;
  ticks = param->drv.os->tick->get();
  if( !ticks )
    ticks = 1; /* 至少是1 */
  param->irq_ticks = ticks; /* 记录中断时刻 */
  param->drv.os->gpio->irq_clr(param->pin_busy);
}

/** 
 * RFID command send
 */
void sensor_mtepd_rfid_send(mtepd_param_t *p, const pos_u8_t *buf, pos_u8_t len, pos_u8_t *ubuf) {
  pos_u8_t crc, i;
  crc = len + 1;  
  ubuf[0] = 0x7f;
  ubuf[1] = crc;
  for( i = 0; i < len; i++ ) {
    ubuf[2+i] = buf[i];
    crc ^= buf[i];
  }
  ubuf[len+2] = crc;
  p->rfid_io->write(p->rfid_io, ubuf, len+3, POS_IO_WAIT_FOREVER);
}

/** 
 * Read RFID and update to EEPROM
 */
pos_status_t sensor_mtepd_rfid_read(mtepd_param_t *p, pos_u32_t timeout) {
  pos_status_t ret = POS_STATUS_E_NOT_FOUND;
  const pos_lib_t *os = p->drv.os;
  pos_u8_t buf[16];
  
  /* 打开模块供电并初始化 */
  if( drv_rfid_mtepd_init() != POS_STATUS_OK )
    return POS_STATUS_E_INIT;
  
  /* rfid scan */
  timeout = os->tick->get() + timeout;
  do {
    do {
      /* rfid read */
      os->task->lock();
      ret = drv_rfid_mtepd_read(buf);
      os->task->unlock();
      if( ret != POS_STATUS_OK )
        break;

      /* wrong signature check */
      if( buf[4] != 0x03 ||
          buf[5] != 0x79 ||
          buf[6] != 0x12 ||
          buf[7] != 0x14 ) {
        ret = POS_STATUS_E_RESOURCE;
        break;
      }

      /* translate from BE to LE */
      os->nps->ntohl((pos_u32_t *)buf, 1);
      
      /* get correct rfid */
      p->drv.eeprom->update((pos_u8_t*)&p->drv.s->slot->rsvd32 - (pos_u8_t*)p->drv.cfg, &buf[0], 4);

      ret = POS_STATUS_OK;
      break;
    }while(0);

    if( ret == POS_STATUS_OK )
      break;
    
    os->tick->sleep(10);
  }while( !os->tick->is_timeout(os->tick->get(), timeout) );
  
  /* 关闭RFID */  
  /* 关闭模块供电 */
  drv_rfid_mtepd_done();
  
  return ret;
}

/** 
* Sensor collecting
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_mtepd_collect(void){
  pos_status_t ret;
  drv_api_t *drv = g_drv;
  ma_sensor_ctrl_t *s;
  mtepd_param_t *p;
  
  s = drv->s;
  p = (mtepd_param_t*)s->drv_buf;

#if 0
  drv->log->data("mtepd rfid scan", 0 );
//  do {
    ret = sensor_mtepd_rfid_read(p, 5000);
//  } while( 1 ); //ret != POS_STATUS_OK ); 
  drv->log->data("mtepd rfid scan result", ret );
  drv->log->data("mtepd rfid",  drv->s->slot->rsvd32 );
#endif

  /* 上报rfid */
  ret = drv->data->put("\"%s\":%u,", "rfid", drv->s->slot->rsvd32, 0,0,0);
  if( ret != POS_STATUS_OK ) {    
    s->ctrl |= MA_SENSOR_CTRL_TOO_LONG; /* indicate data buffer put failed */      
  }


  p->flags |= DRV_MTEPD_FLAG_REFRESH; /* require to refresh */

#if 0
  drv->log->data("mtepd refresh start", 0 );
  drv->os->flush();

  /* 点亮屏幕 */
  mtepd_drv_on(p);  
  mtepd_refresh((mtepd_param_t*)s->drv_buf, 0);
  
  drv->log->data("mtepd refresh done", 0 );
  drv->os->flush();
#endif

  return ret;
  
}

/** 
 * Sleep without system deep sleep so that PWM can still work
 */
void sensor_mtepd_sleep(mtepd_param_t *p, pos_u32_t ticks) {
  const pos_lib_tick_t *t;  
  t = p->drv.os->tick;
  ticks = t->get() + ticks;
  while( !t->is_timeout(t->get(), ticks) ) {
    t->sleep(20);
  }
}

/** 
 * Get current charging state
 * @return     0: None charging\n
              !=0: Charging
 */

pos_u32_t sensor_mtepd_recharge_get(mtepd_param_t *p) {
  return 0; /* do NO SUPPORT recharge state */
}

/** 
 * Polling callback function
 * @note This function can NOT call g_drv. When polling, g_drv might piont to any sensor drivers. It should call the param stored driver pointer.
 */
void sensor_mtepd_poll(mtepd_param_t *p) {
  pos_u32_t ticks, v, sflag, *p_rfid;
  const pos_lib_tick_t *t;

  /* DC充电检查 */  
  if( sensor_mtepd_recharge_get(p) ) {
    p->flags |= DRV_MTEPD_FLAG_DC;
  }
  
  /* 若从无中断and NO Duty Refresh则不处理任何内容 */
  if( !p->irq_ticks ) {
    if( p->flags & DRV_MTEPD_FLAG_REFRESH) {
      p->flags &= ~DRV_MTEPD_FLAG_REFRESH; /* clear poll refresh flag if set */
      mtepd_refresh(p, 0);
    }
    return;
  }

  /* 获得最新时长参数 */
  mtepd_time_collect(p);

  /* 亮屏按键时长检查 */  
  v = mtepd_btn_time_get(p, p->erase_time);
  if( v < p->epd_time ) {
  } else {
    p->flags &= ~DRV_MTEPD_FLAG_NO_ALARM; /* each btn will allow a new round of alarm */
  }

  /* DC充电检查 */
  if( sensor_mtepd_recharge_get(p) ) {
    p->flags |= DRV_MTEPD_FLAG_DC;
  } else {
    p->flags &= ~DRV_MTEPD_FLAG_DC;
  }

  if( v < p->epd_time && (p->flags & DRV_MTEPD_FLAG_REFRESH) == 0 ) {
    p->irq_ticks = 0;
    return; /* 忽略时长短于亮屏时间的无效按键 */
  }
  p->flags &= ~DRV_MTEPD_FLAG_REFRESH; /* clear poll refresh flag until next cycle */
  sflag = p->drv.cfg->ctrl & MA_CFG_CTRL_DRV_LOG;

  /* rfid指针位置 */
  p_rfid = &p->drv.s->slot->rsvd32;
  
  t = p->drv.os->tick;
  if( v < p->epd_time )
    v = p->epd_time;

  while( v >= p->epd_time ) {
    if( sflag ) {
      p->drv.os->flush(); /* flush log for more dump */
    }
    
    /* 每次其它动作前都清除以前告警 */
    mtepd_alarm_set(p, 0);

    /* 信息已经更新重新更新亮屏时间 */
    ticks = t->get() + p->keep_time;      

    /* 擦除 */
    if( v >= p->erase_time ) {      
      if( sflag ) {
        p->drv.log->data("erase rfid", *p_rfid);
      }
      /* 擦除EEPROM为0 */
      p->drv.eeprom->update((pos_u8_t*)p_rfid - (pos_u8_t*)p->drv.cfg, POS_NULL, 4);
      
      /* 重新刷新屏幕 */      
      mtepd_refresh(p, 0);  
    } else if( v >= p->rfid_time) {
      if( sflag ) {
        p->drv.log->data("scan rfid", *p_rfid);
      }    
      /* 显示请扫码 */
      mtepd_refresh(p, 2);
      
      /* RFID扫描5s */
      if( sensor_mtepd_rfid_read(p, p->keep_time) == POS_STATUS_OK ) { 
        if( sflag ) {
          p->drv.log->data("scan ok", *p_rfid);
        }    
        
        /* 显示成功 */
        mtepd_refresh(p, 1);
        
        /* 成功重新更新亮屏时间 */
        ticks = t->get() + p->keep_time;   
        
        /* 成功蜂鸣 */
        p->drv.board->beep_set(1350);
        sensor_mtepd_sleep(p, 250);
        p->drv.board->beep_set(2700);
        sensor_mtepd_sleep(p, 250);
        p->drv.board->beep_set(0);
        
      } else {
        if( sflag ) {
          p->drv.log->data("scan bad", *p_rfid);
        }    
    
        /* 失败蜂鸣 */
        p->drv.board->beep_set(2700);
        sensor_mtepd_sleep(p, 250);
        p->drv.board->beep_set(1350);
        sensor_mtepd_sleep(p, 250);
        p->drv.board->beep_set(0);
        
      }
      
    }else {
      if( sflag ) {
        p->drv.log->data("epd refresh", *p_rfid);
      }
      /* 重新刷新屏幕 */      
      mtepd_refresh(p, 0);
    }

    /* 轮询直到超时 */  
    do {
      v = mtepd_btn_time_get(p, p->erase_time);
      if( v < p->epd_time ) {
        if( t->is_timeout(t->get(), ticks) ) 
          break;
      } else {
        p->flags &= ~DRV_MTEPD_FLAG_NO_ALARM; /* each btn will allow a new round of alarm */
      }
      t->sleep(20);
      /* 动画: 用保存模式重新刷新屏幕 */      
      /* EPD DO NOT Support FLASH updateing: mtepd_refresh(p, m); */
    } while( v < p->epd_time );

    /* DC充电检查 */  
    if( sensor_mtepd_recharge_get(p) ) {
      p->flags |= DRV_MTEPD_FLAG_DC;
    } else {
      p->flags &= ~DRV_MTEPD_FLAG_DC;
    }
  }
  

  /* 清除中断标志 */
  p->irq_ticks = 0;

  /* 关闭告警 */
  mtepd_alarm_set(p, 0);

  /* 退出前再次DC充电检查 */  
  if( sensor_mtepd_recharge_get(p) ) {
    p->flags |= DRV_MTEPD_FLAG_DC;
  } 

}


/** 
 * Driver init
 * @param[in]  load   0:Unload driver, 1:Load driver
 * @return     0: Successful\n
             !=0: Failed, refer to @ref pos_status_t
 * @note This function can be NULL if it does NOT require any init operatoin
 */ 
pos_status_t sensor_mtepd_init(pos_u32_t load) {
  drv_api_t *drv = g_drv;
  mtepd_param_t *param;
  ma_sensor_ctrl_t *s = g_drv->s;
  pos_gpio_pin_t pin;
  pin = drv->board->pin_map(MB_PT_DIO,MB_DIO_ID_BTN);
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("mtepd init", load);
    drv->log->data("mtepd btn", pin);
  }  
  
  if( load ) {
    s->data.u32 = 0; /* clear counter by default */
    param = drv->os->malloc(sizeof(*param));
    if( !param )
      return POS_STATUS_E_MEM;
    drv->os->memset(param, 0, sizeof(*param));
    
    param->pin_btn = pin;
    param->pin_busy =  drv->board->pin_map(MB_PT_CTRL,MB_CTRL_ID_UDF1);
    param->hw_rev = s->slot->thld.u16[1] & 0x3;

    /* init busy as INPUT PULLUP */
    drv->os->gpio->mode_set(param->pin_busy, POS_GPIO_MODE_INPUT); // according to jason@20230508, this has to be no PULLUP for power cost saving (reduce 50ua idle power)
    
    drv->os->memcpy(&param->drv, drv, sizeof(*drv)); /* clone a g_drv */    
    s->drv_buf = param;  /* record to drv_buf */
    
    /* 
     * turn on irq
     * warning: as GOT is implemented in DATA and DATA is not supported in mmap()
     * all GOT based pointer can NOT be used. So the IRQ SET should use the mapped
     * function pointer here (the export structure). 
     * the orignal sensor_fastcnt_irq() should NOT be used directly.
     */
    if( pin != POS_PH31 ) {
      drv->os->gpio->irq_set(
        pin, 
        POS_GPIO_MODE_INPUT_PU+POS_GPIO_MODE_IRQ_FALLING,
        (pos_func_t)drv->s->drv.u.sensor.irq_cb, /* has to be this. DO NOT use sensor_fastcnt_irq */
        param);
    }

    /*
     * turn on poll
     */
    drv->proc->event_register((pos_func_t)drv->s->drv.u.sensor.poll_cb, param);

#if 0 /* EPD does NOT require this */
    if( drv->fsm < MA_FSM_COLLECT ) {
      /* 
       * 开机亮屏
       */      
      mtepd_refresh(param, 3);
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
    if( pin != POS_PH31 ) {
      drv->os->gpio->irq_set(pin, POS_GPIO_MODE_INPUT_PU+POS_GPIO_MODE_IRQ_DISABLE, POS_NULL, POS_NULL);
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
  .version = DRV_SENSOR_VERSION(MA_SENSOR_MTEPD, DRV_MTEPD_VERSION),
  .name = DRV_MTEPD_NAME,
  .u.sensor={
    .init = sensor_mtepd_init,
    .collect = sensor_mtepd_collect,
    .irq_cb = (pos_func_t)sensor_mtepd_irq,
    .poll_cb = (pos_func_t)sensor_mtepd_poll,    
  },
};

