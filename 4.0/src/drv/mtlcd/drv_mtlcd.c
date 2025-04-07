/**
 * @file  drv_mtlcd.c
 * @brief MT LCD/RFID driver
 * @author Runby F.
 * @date 2022-4-1
 * @copyright Polysense
 */

#include "drv_api.h"
#include "drv_mtlcd.h"

/* 
 * 版本修订历史
v1.0 @ 2022-04-01
  1) 完成第一版驱动

v1.1 @ 2022-04-03
  1) 支持轮询模式
  2) 支持EEPROM SLOT RSVD32字段储存RFID
  3) 支持联网异常状态显示
  4) 支持RFID获取并上报

v1.2 @ 2022-05-11
  1) 支持RFID/SPI驱动
  2) 根据最新硬件版本调整RFID供电和复位管脚
  3) 解决字节序不正确问题 

v1.3 @ 2022-05-14
  1) 支持从内核获取管脚

v1.4 @ 2022-05-30
  1) 支持最终MT主板并调整相关管脚控制
  2) 解决fbuf初始化问题

v1.5 @ 2022-06-06
  1) 调整RFID RST管脚为OUTPUT非OD模式

v1.6 @ 2022-06-08
  1) 修正RFID没有显示问题
  2) 驱动启动首次总是亮屏
  3) 修正亮屏按钮时长判断问题

v1.7 @ 2022-06-13
  1) 优化亮屏按钮流程，增加扫码BEEP提醒
  2) 解决扫码后蜂鸣异响问题
  3) 解决扫码后屏幕异常问题
  4) 优化按键时长判断避免长期没有操作时的误判亮屏
  5) 扫码成功重新更新亮屏时间
  6) 修正扫码提示音不正确问题(休眠导致PWM不工作)
  7) 支持按钮1s后但未抬起时的提示扫码音

v1.8 @ 2022-06-24
  1) 修改默认亮屏时长为10s

v1.9 @ 2022-08-18
  1) 支持DIO#0输入作为充电检测
  2) 支持充电电量动态显示
  3) 支持充电始终亮屏
  4) 支持开机亮屏简短提示

v1.10 @ 2022-08-19
  1) 解决温度显示错行问题
  2) 当显示未扫码时不再显示背景温度
  
v1.11 @ 2022-08-31
  1) 优化充电断电反应较慢问题
  2) 解决扫码时长和亮屏不同步问题

v1.12 @ 2023-11-18
  1) Using new PWR ID solution

*/

/**
 * @brief Driver version
 */
#define DRV_MTLCD_VERSION 0x010c

/**
 * @brief Driver name
 */
#define DRV_MTLCD_NAME  "MTLCD"


/** 
 * IRQ interrupt callback handler
 * @note This function can NOT call g_drv. When IRQ happening, g_drv might piont to any sensor drivers. It should call the param stored driver pointer.
 */
void sensor_mtlcd_irq(mtlcd_param_t *param) {
  pos_u32_t ticks;
  ticks = param->drv.os->tick->get();
  if( !ticks )
    ticks = 1; /* 至少是1 */
  param->irq_ticks = ticks; /* 记录中断时刻 */
  param->drv.os->gpio->irq_clr(param->pin);
}

/** 
 * RFID command send
 */
void sensor_mtlcd_rfid_send(mtlcd_param_t *p, const pos_u8_t *buf, pos_u8_t len, pos_u8_t *ubuf) {
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
pos_status_t sensor_mtlcd_rfid_read(mtlcd_param_t *p, pos_u32_t timeout) {
  pos_status_t ret = POS_STATUS_E_NOT_FOUND;
  const pos_lib_t *os = p->drv.os;
  pos_u8_t buf[16];
  
  /* 打开模块供电并初始化 */
  if( drv_rfid_init() != POS_STATUS_OK )
    return POS_STATUS_E_INIT;
  
  /* rfid scan */
  timeout = os->tick->get() + timeout;
  do {
    do {
      /* rfid read */
      os->task->lock();
      ret = drv_rfid_read(buf);
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
  drv_rfid_done();
  
  return ret;
}

/** 
* Sensor collecting
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_mtlcd_collect(void){
  pos_status_t ret;
  drv_api_t *drv = g_drv;
  ma_sensor_ctrl_t *s;
//  mtlcd_param_t *p;
  
  s = drv->s;

#if 0
  p = (mtlcd_param_t*)s->drv_buf;
  drv->log->data("mtlcd rfid scan", 0 );
//  do {
    ret = sensor_mtlcd_rfid_read(p, 5000);
//  } while( 1 ); //ret != POS_STATUS_OK ); 
  drv->log->data("mtlcd rfid scan result", ret );
  drv->log->data("mtlcd rfid",  drv->s->slot->rsvd32 );
#endif

  /* 上报rfid */
  ret = drv->data->put("\"%s\":%u,", "rfid", drv->s->slot->rsvd32, 0,0,0);
  if( ret != POS_STATUS_OK ) {    
    s->ctrl |= MA_SENSOR_CTRL_TOO_LONG; /* indicate data buffer put failed */      
  }

#if 0
  drv->log->data("mtlcd refresh start", 0 );
  drv->os->flush();

  /* 点亮屏幕 */
  mtlcd_drv_on(p);  
  mtlcd_refresh((mtlcd_param_t*)s->drv_buf, 0);
  
  drv->log->data("mtlcd refresh done", 0 );
  drv->os->flush();
#endif

  return ret;
  
}

/** 
 * Sleep without system deep sleep so that PWM can still work
 */
void sensor_mtlcd_sleep(mtlcd_param_t *p, pos_u32_t ticks) {
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

pos_u32_t sensor_mtlcd_recharge_get(mtlcd_param_t *p) {
  p = &p[1]; /* 充电用p[1] */
  return p->drv.os->gpio->get(p->pin) != 0;
}

/** 
 * Turn ON LCD screen
 */
void sensor_mtlcd_on(mtlcd_param_t *p) {
  if( (p->flags & DRV_MTLCD_FLAG_ON) == 0 ) {
    if( p->drv.cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {  
      p->drv.log->buf("lcd on", &p->lcd_time, 20);
    }
    /* 点亮屏幕供电 */
    mtlcd_drv_on(p);    
    p->flags |= DRV_MTLCD_FLAG_ON; /* SET亮屏标志 */
  }
}

/** 
 * Turn OFF LCD screen
 */
void sensor_mtlcd_off(mtlcd_param_t *p) {
  if( p->flags & DRV_MTLCD_FLAG_ON ) {    
    /* 关闭屏幕供电 */
    mtlcd_drv_off(p);    
    p->flags &= ~DRV_MTLCD_FLAG_ON; /* 清除亮屏标志 */

    if( p->drv.cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      p->drv.log->data("lcd off", 0);
    }    
  }
}

/** 
 * Polling callback function
 * @note This function can NOT call g_drv. When polling, g_drv might piont to any sensor drivers. It should call the param stored driver pointer.
 */
void sensor_mtlcd_poll(mtlcd_param_t *p) {
  pos_u32_t ticks, v, sflag, *p_rfid, m;
  const pos_lib_tick_t *t;

  /* DC充电检查 */  
  if( sensor_mtlcd_recharge_get(p) ) {
    p->flags |= DRV_MTLCD_FLAG_DC;
  }
  
  /* 若从无中断且已经处理过首次亮屏且灭屏状态则不处理任何内容 */
  if( !p->irq_ticks && 
      (p->flags & DRV_MTLCD_FLAG_FIRST_ON) != 0 && 
      (p->flags & (DRV_MTLCD_FLAG_ON|DRV_MTLCD_FLAG_DC)) == 0 ) {
    return;
  }

  /* 获得最新时长参数 */
  mtlcd_time_collect(p);
#if 0
  /* 亮屏按键时长检查 */
  if( mtlcd_btn_time_get(p, p->lcd_time) < p->lcd_time && (p->flags & 0x80) ) {
    p->irq_ticks = 0;
    return; /* 如果已经亮过则忽略时长短于亮屏时间的无效按键 */
  }
  p->flags |= 0x80; /* 亮屏标志 */
  sflag = p->drv.cfg->ctrl & MA_CFG_CTRL_DRV_LOG;
  p_rfid = &p->drv.s->slot->rsvd32;

  if( sflag ) {  
    p->drv.log->buf("lcd on", &p->lcd_time, 20);
  }

  /* 点亮屏幕供电 */
  mtlcd_drv_on(p);

  /* 亮屏 */
  mtlcd_refresh(p, 0);

  /* 轮询直到超时 */
  t = p->drv.os->tick;
  ticks = t->get() + p->keep_time;  
  while( !t->is_timeout(t->get(), ticks) ) {
    /* 检查按键 */
    v = mtlcd_btn_time_get(p, p->erase_time);

    /* 无效按键短暂休眠后重试 */
    if( v < p->lcd_time ) {
      t->sleep(20);
      continue;
    }

    /* 每次有效按键均增加亮屏时间 */
    ticks = t->get() + p->keep_time;

    /* 纯亮屏, 不进行任何后续动作 */
    if( v < p->rfid_time || v < p->erase_time )
      continue;
    
    /* 每次其它动作前都清除以前告警 */
    mtlcd_alarm_set(p, 0);

    /* 擦除 */
    if( v >= p->erase_time ) {      
      if( sflag ) {
        p->drv.log->data("erase rfid", *p_rfid);
      }
      /* 擦除EEPROM为0 */
      p->drv.eeprom->update((pos_u8_t*)p_rfid - (pos_u8_t*)p->drv.cfg, POS_NULL, 4);
      
      /* 重新刷新屏幕 */      
      mtlcd_refresh(p, 0);  
    } else {
      if( sflag ) {
        p->drv.log->data("scan rfid", *p_rfid);
      }    
      /* 显示请扫码 */
      mtlcd_refresh(p, 2);

      /* RFID扫描5s */
      if( sensor_mtlcd_rfid_read(p, 5000) == POS_STATUS_OK ) { 
        if( sflag ) {
          p->drv.log->data("scan ok", *p_rfid);
        }    
        
        /* 显示成功 */
        mtlcd_refresh(p, 1);

        /* 成功蜂鸣 */
        p->drv.board->beep_set(1350);
        t->sleep(500);
        p->drv.board->beep_set(2700);
        t->sleep(500);
        
        /* 信息已经更新重新更新亮屏时间 */
        ticks = t->get() + p->keep_time;      
      } else {
        if( sflag ) {
          p->drv.log->data("scan bad", *p_rfid);
        }    

        /* 失败蜂鸣 */
        p->drv.board->beep_set(2700);
        t->sleep(500);
        p->drv.board->beep_set(1350);
        t->sleep(500);
 
        break; /* 扫描失败立即退出 */
      }
    }
    
  }
#else

  /* 亮屏按键时长检查 */  
  v = mtlcd_btn_time_get(p, p->erase_time);
  if( v < p->lcd_time ) {
    if( p->flags & DRV_MTLCD_FLAG_DC )
      v = p->lcd_time; /* 当DC充电时或者刚才是DC充电时，总是保持最小lcd_time */
  } else {
    p->flags &= ~DRV_MTLCD_FLAG_NO_ALARM; /* each btn will allow a new round of alarm */
  }

  /* DC充电检查 */  
  if( sensor_mtlcd_recharge_get(p) ) {
    p->flags |= DRV_MTLCD_FLAG_DC;
    if( v < p->lcd_time )
      v = p->lcd_time; /* 确保充电时亮屏 */
  } else {
    p->flags &= ~DRV_MTLCD_FLAG_DC;
  }
  
  if( v < p->lcd_time && (p->flags & DRV_MTLCD_FLAG_FIRST_ON) ) {
    p->irq_ticks = 0;
    return; /* 如果已经亮过则忽略时长短于亮屏时间的无效按键 */
  }
  sflag = p->drv.cfg->ctrl & MA_CFG_CTRL_DRV_LOG;

  p->flags |= DRV_MTLCD_FLAG_FIRST_ON; /* 设置已经处理首次亮屏标志 */

  /* 
   * 点亮屏幕供电
   */
  sensor_mtlcd_on(p);

  /* rfid指针位置 */
  p_rfid = &p->drv.s->slot->rsvd32;
  
  t = p->drv.os->tick;
  if( v < p->lcd_time )
    v = p->lcd_time;
  m = 0;
  while( v >= p->lcd_time ) {
    /* 每次其它动作前都清除以前告警 */
    mtlcd_alarm_set(p, 0);

    /* 信息已经更新重新更新亮屏时间 */
    ticks = t->get() + p->keep_time;      

    /* 模式 */
    m = 0;

    /* 擦除 */
    if( v >= p->erase_time ) {      
      if( sflag ) {
        p->drv.log->data("erase rfid", *p_rfid);
      }
      /* 擦除EEPROM为0 */
      p->drv.eeprom->update((pos_u8_t*)p_rfid - (pos_u8_t*)p->drv.cfg, POS_NULL, 4);
      
      /* 重新刷新屏幕 */      
      mtlcd_refresh(p, 0);  
    } else if( v >= p->rfid_time) {
      if( sflag ) {
        p->drv.log->data("scan rfid", *p_rfid);
      }    
      /* 显示请扫码 */
      mtlcd_refresh(p, 2);
      m = 2;      /* 同步模式 */
      
      /* RFID扫描5s */
      if( sensor_mtlcd_rfid_read(p, p->keep_time) == POS_STATUS_OK ) { 
        if( sflag ) {
          p->drv.log->data("scan ok", *p_rfid);
        }    
        
        /* 显示成功 */
        mtlcd_refresh(p, 1);
        m = 1;
        
        /* 成功重新更新亮屏时间 */
        ticks = t->get() + p->keep_time;   
        
        /* 成功蜂鸣 */
        p->drv.board->beep_set(1350);
        sensor_mtlcd_sleep(p, 250);
        p->drv.board->beep_set(2700);
        sensor_mtlcd_sleep(p, 250);
        p->drv.board->beep_set(0);
        
      } else {
        if( sflag ) {
          p->drv.log->data("scan bad", *p_rfid);
        }    
    
        /* 失败蜂鸣 */
        p->drv.board->beep_set(2700);
        sensor_mtlcd_sleep(p, 250);
        p->drv.board->beep_set(1350);
        sensor_mtlcd_sleep(p, 250);
        p->drv.board->beep_set(0);
        
      }
      
    }else {
      if( sflag ) {
        p->drv.log->data("lcd refresh", *p_rfid);
      }
      /* 重新刷新屏幕 */      
      mtlcd_refresh(p, 0);
    }

    /* 轮询直到超时 */  
    do {
      v = mtlcd_btn_time_get(p, p->erase_time);
      if( v < p->lcd_time ) {
        if( t->is_timeout(t->get(), ticks) ) 
          break;
      } else {
        p->flags &= ~DRV_MTLCD_FLAG_NO_ALARM; /* each btn will allow a new round of alarm */
      }
      t->sleep(20);
      /* 动画: 用保存模式重新刷新屏幕 */      
      mtlcd_refresh(p, m);
      p[1].lcd_time++; /* 递增充电亮屏计数 */
    } while( v < p->lcd_time );

    /* 清除充电亮屏计数 */
    p[1].lcd_time = 0; 

    /* DC充电检查 */  
    if( sensor_mtlcd_recharge_get(p) ) {
      p->flags |= DRV_MTLCD_FLAG_DC;
    } else {
      p->flags &= ~DRV_MTLCD_FLAG_DC;
    }
  }
  
#endif

  /* 清除中断标志 */
  p->irq_ticks = 0;
  p[1].irq_ticks = 0;

  /* 关闭告警 */
  mtlcd_alarm_set(p, 0);

  /* 退出前再次DC充电检查 */  
  if( sensor_mtlcd_recharge_get(p) ) {
    p->flags |= DRV_MTLCD_FLAG_DC;
  } 
  
  /* 如果是充电状态直接退出 (保留当前屏幕状态为默认亮屏状态并禁止休眠)*/
  if( p->flags & DRV_MTLCD_FLAG_DC ) {
    /* 禁止休眠 , 保持时刻刷新*/
    p->drv.os->call(POS_CALL_FUNC_ID_SET_NO_SLEEP, 1);
    return;
  }

  /* 允许休眠 */
  p->drv.os->call(POS_CALL_FUNC_ID_SET_NO_SLEEP, 0);

  /* 关闭屏幕供电 */
  sensor_mtlcd_off(p);        
}


/** 
 * Driver init
 * @param[in]  load   0:Unload driver, 1:Load driver
 * @return     0: Successful\n
             !=0: Failed, refer to @ref pos_status_t
 * @note This function can be NULL if it does NOT require any init operatoin
 */ 
pos_status_t sensor_mtlcd_init(pos_u32_t load) {
  drv_api_t *drv = g_drv;
  mtlcd_param_t *param;
  ma_sensor_ctrl_t *s = g_drv->s;
  pos_gpio_pin_t pin, pin_dc;
  pin = drv->board->pin_map(MB_PT_DIO,MB_DIO_ID_BTN);
  pin_dc = drv->board->pin_map(MB_PT_DIO,MB_DIO_ID_DEFAULT);  
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("mtlcd init", load);
  }  
  
  if( load ) {
    s->data.u32 = 0; /* clear counter by default */
    param = drv->os->malloc(sizeof(*param)*2);
    if( !param )
      return POS_STATUS_E_MEM;
    drv->os->memset(param, 0, sizeof(*param)*2);
    
    param->pin = pin;
    drv->os->memcpy(&param->drv, drv, sizeof(*drv)); /* clone a g_drv */    
    s->drv_buf = param;  /* record to drv_buf */

    /* clone param[0] to param[1] and keep pin as pin_dc */
    drv->os->memcpy(&param[1], &param[0], sizeof(*param));
    param[1].pin = pin_dc;
    param[1].keep_time = 1; /* special flag for param#1 */
    
    /* 
     * turn on irq
     * warning: as GOT is implemented in DATA and DATA is not supported in mmap()
     * all GOT based pointer can NOT be used. So the IRQ SET should use the mapped
     * function pointer here (the export structure). 
     * the orignal sensor_fastcnt_irq() should NOT be used directly.
     */     
    drv->os->gpio->irq_set(
      pin, 
      POS_GPIO_MODE_INPUT_PU+POS_GPIO_MODE_IRQ_FALLING,
      (pos_func_t)drv->s->drv.u.sensor.irq_cb, /* has to be this. DO NOT use sensor_fastcnt_irq */
      param);
    drv->os->gpio->irq_set(
      pin_dc, 
      POS_GPIO_MODE_INPUT_PD+POS_GPIO_MODE_IRQ_RISING,
      (pos_func_t)drv->s->drv.u.sensor.irq_cb, /* has to be this. DO NOT use sensor_fastcnt_irq */
      &param[1]); /* use param1 */

    /*
     * turn on poll
     */
    drv->proc->event_register((pos_func_t)drv->s->drv.u.sensor.poll_cb, param);

    if( drv->fsm < MA_FSM_COLLECT ) {
      /* 
       * 开机亮屏
       */      
      sensor_mtlcd_on(param);    
      mtlcd_refresh(param, 3);
    }
    
  } else {      
    /*
     * turn off poll
     */
    drv->proc->event_unregister((pos_func_t)drv->s->drv.u.sensor.poll_cb, s->drv_buf);

    /* 
     * turn off irq
     */
    drv->os->gpio->irq_set(pin, POS_GPIO_MODE_INPUT_PU+POS_GPIO_MODE_IRQ_DISABLE, POS_NULL, POS_NULL);
    drv->os->gpio->irq_set(pin_dc, POS_GPIO_MODE_INPUT_PU+POS_GPIO_MODE_IRQ_DISABLE, POS_NULL, POS_NULL);    
    if( s->drv_buf ) {
      if( drv->fsm < MA_FSM_COLLECT ) {
        /* 
         * 开机灭屏
         */      
        sensor_mtlcd_off((mtlcd_param_t*)s->drv_buf);    
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
  .version = DRV_SENSOR_VERSION(MA_SENSOR_MTLCD, DRV_MTLCD_VERSION),
  .name = DRV_MTLCD_NAME,
  .u.sensor={
    .init = sensor_mtlcd_init,
    .collect = sensor_mtlcd_collect,
    .irq_cb = (pos_func_t)sensor_mtlcd_irq,
    .poll_cb = (pos_func_t)sensor_mtlcd_poll,    
  },
};

