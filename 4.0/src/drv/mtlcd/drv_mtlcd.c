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
 * �汾�޶���ʷ
v1.0 @ 2022-04-01
  1) ��ɵ�һ������

v1.1 @ 2022-04-03
  1) ֧����ѯģʽ
  2) ֧��EEPROM SLOT RSVD32�ֶδ���RFID
  3) ֧�������쳣״̬��ʾ
  4) ֧��RFID��ȡ���ϱ�

v1.2 @ 2022-05-11
  1) ֧��RFID/SPI����
  2) ��������Ӳ���汾����RFID����͸�λ�ܽ�
  3) ����ֽ�����ȷ���� 

v1.3 @ 2022-05-14
  1) ֧�ִ��ں˻�ȡ�ܽ�

v1.4 @ 2022-05-30
  1) ֧������MT���岢������عܽſ���
  2) ���fbuf��ʼ������

v1.5 @ 2022-06-06
  1) ����RFID RST�ܽ�ΪOUTPUT��ODģʽ

v1.6 @ 2022-06-08
  1) ����RFIDû����ʾ����
  2) ���������״���������
  3) ����������ťʱ���ж�����

v1.7 @ 2022-06-13
  1) �Ż�������ť���̣�����ɨ��BEEP����
  2) ���ɨ��������������
  3) ���ɨ�����Ļ�쳣����
  4) �Ż�����ʱ���жϱ��ⳤ��û�в���ʱ����������
  5) ɨ��ɹ����¸�������ʱ��
  6) ����ɨ����ʾ������ȷ����(���ߵ���PWM������)
  7) ֧�ְ�ť1s��δ̧��ʱ����ʾɨ����

v1.8 @ 2022-06-24
  1) �޸�Ĭ������ʱ��Ϊ10s

v1.9 @ 2022-08-18
  1) ֧��DIO#0������Ϊ�����
  2) ֧�ֳ�������̬��ʾ
  3) ֧�ֳ��ʼ������
  4) ֧�ֿ������������ʾ

v1.10 @ 2022-08-19
  1) ����¶���ʾ��������
  2) ����ʾδɨ��ʱ������ʾ�����¶�
  
v1.11 @ 2022-08-31
  1) �Ż����ϵ練Ӧ��������
  2) ���ɨ��ʱ����������ͬ������

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
    ticks = 1; /* ������1 */
  param->irq_ticks = ticks; /* ��¼�ж�ʱ�� */
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
  
  /* ��ģ�鹩�粢��ʼ�� */
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
  
  /* �ر�RFID */  
  /* �ر�ģ�鹩�� */
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

  /* �ϱ�rfid */
  ret = drv->data->put("\"%s\":%u,", "rfid", drv->s->slot->rsvd32, 0,0,0);
  if( ret != POS_STATUS_OK ) {    
    s->ctrl |= MA_SENSOR_CTRL_TOO_LONG; /* indicate data buffer put failed */      
  }

#if 0
  drv->log->data("mtlcd refresh start", 0 );
  drv->os->flush();

  /* ������Ļ */
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
  p = &p[1]; /* �����p[1] */
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
    /* ������Ļ���� */
    mtlcd_drv_on(p);    
    p->flags |= DRV_MTLCD_FLAG_ON; /* SET������־ */
  }
}

/** 
 * Turn OFF LCD screen
 */
void sensor_mtlcd_off(mtlcd_param_t *p) {
  if( p->flags & DRV_MTLCD_FLAG_ON ) {    
    /* �ر���Ļ���� */
    mtlcd_drv_off(p);    
    p->flags &= ~DRV_MTLCD_FLAG_ON; /* ���������־ */

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

  /* DC����� */  
  if( sensor_mtlcd_recharge_get(p) ) {
    p->flags |= DRV_MTLCD_FLAG_DC;
  }
  
  /* �������ж����Ѿ�������״�����������״̬�򲻴����κ����� */
  if( !p->irq_ticks && 
      (p->flags & DRV_MTLCD_FLAG_FIRST_ON) != 0 && 
      (p->flags & (DRV_MTLCD_FLAG_ON|DRV_MTLCD_FLAG_DC)) == 0 ) {
    return;
  }

  /* �������ʱ������ */
  mtlcd_time_collect(p);
#if 0
  /* ��������ʱ����� */
  if( mtlcd_btn_time_get(p, p->lcd_time) < p->lcd_time && (p->flags & 0x80) ) {
    p->irq_ticks = 0;
    return; /* ����Ѿ����������ʱ����������ʱ�����Ч���� */
  }
  p->flags |= 0x80; /* ������־ */
  sflag = p->drv.cfg->ctrl & MA_CFG_CTRL_DRV_LOG;
  p_rfid = &p->drv.s->slot->rsvd32;

  if( sflag ) {  
    p->drv.log->buf("lcd on", &p->lcd_time, 20);
  }

  /* ������Ļ���� */
  mtlcd_drv_on(p);

  /* ���� */
  mtlcd_refresh(p, 0);

  /* ��ѯֱ����ʱ */
  t = p->drv.os->tick;
  ticks = t->get() + p->keep_time;  
  while( !t->is_timeout(t->get(), ticks) ) {
    /* ��鰴�� */
    v = mtlcd_btn_time_get(p, p->erase_time);

    /* ��Ч�����������ߺ����� */
    if( v < p->lcd_time ) {
      t->sleep(20);
      continue;
    }

    /* ÿ����Ч��������������ʱ�� */
    ticks = t->get() + p->keep_time;

    /* ������, �������κκ������� */
    if( v < p->rfid_time || v < p->erase_time )
      continue;
    
    /* ÿ����������ǰ�������ǰ�澯 */
    mtlcd_alarm_set(p, 0);

    /* ���� */
    if( v >= p->erase_time ) {      
      if( sflag ) {
        p->drv.log->data("erase rfid", *p_rfid);
      }
      /* ����EEPROMΪ0 */
      p->drv.eeprom->update((pos_u8_t*)p_rfid - (pos_u8_t*)p->drv.cfg, POS_NULL, 4);
      
      /* ����ˢ����Ļ */      
      mtlcd_refresh(p, 0);  
    } else {
      if( sflag ) {
        p->drv.log->data("scan rfid", *p_rfid);
      }    
      /* ��ʾ��ɨ�� */
      mtlcd_refresh(p, 2);

      /* RFIDɨ��5s */
      if( sensor_mtlcd_rfid_read(p, 5000) == POS_STATUS_OK ) { 
        if( sflag ) {
          p->drv.log->data("scan ok", *p_rfid);
        }    
        
        /* ��ʾ�ɹ� */
        mtlcd_refresh(p, 1);

        /* �ɹ����� */
        p->drv.board->beep_set(1350);
        t->sleep(500);
        p->drv.board->beep_set(2700);
        t->sleep(500);
        
        /* ��Ϣ�Ѿ��������¸�������ʱ�� */
        ticks = t->get() + p->keep_time;      
      } else {
        if( sflag ) {
          p->drv.log->data("scan bad", *p_rfid);
        }    

        /* ʧ�ܷ��� */
        p->drv.board->beep_set(2700);
        t->sleep(500);
        p->drv.board->beep_set(1350);
        t->sleep(500);
 
        break; /* ɨ��ʧ�������˳� */
      }
    }
    
  }
#else

  /* ��������ʱ����� */  
  v = mtlcd_btn_time_get(p, p->erase_time);
  if( v < p->lcd_time ) {
    if( p->flags & DRV_MTLCD_FLAG_DC )
      v = p->lcd_time; /* ��DC���ʱ���߸ղ���DC���ʱ�����Ǳ�����Сlcd_time */
  } else {
    p->flags &= ~DRV_MTLCD_FLAG_NO_ALARM; /* each btn will allow a new round of alarm */
  }

  /* DC����� */  
  if( sensor_mtlcd_recharge_get(p) ) {
    p->flags |= DRV_MTLCD_FLAG_DC;
    if( v < p->lcd_time )
      v = p->lcd_time; /* ȷ�����ʱ���� */
  } else {
    p->flags &= ~DRV_MTLCD_FLAG_DC;
  }
  
  if( v < p->lcd_time && (p->flags & DRV_MTLCD_FLAG_FIRST_ON) ) {
    p->irq_ticks = 0;
    return; /* ����Ѿ����������ʱ����������ʱ�����Ч���� */
  }
  sflag = p->drv.cfg->ctrl & MA_CFG_CTRL_DRV_LOG;

  p->flags |= DRV_MTLCD_FLAG_FIRST_ON; /* �����Ѿ������״�������־ */

  /* 
   * ������Ļ����
   */
  sensor_mtlcd_on(p);

  /* rfidָ��λ�� */
  p_rfid = &p->drv.s->slot->rsvd32;
  
  t = p->drv.os->tick;
  if( v < p->lcd_time )
    v = p->lcd_time;
  m = 0;
  while( v >= p->lcd_time ) {
    /* ÿ����������ǰ�������ǰ�澯 */
    mtlcd_alarm_set(p, 0);

    /* ��Ϣ�Ѿ��������¸�������ʱ�� */
    ticks = t->get() + p->keep_time;      

    /* ģʽ */
    m = 0;

    /* ���� */
    if( v >= p->erase_time ) {      
      if( sflag ) {
        p->drv.log->data("erase rfid", *p_rfid);
      }
      /* ����EEPROMΪ0 */
      p->drv.eeprom->update((pos_u8_t*)p_rfid - (pos_u8_t*)p->drv.cfg, POS_NULL, 4);
      
      /* ����ˢ����Ļ */      
      mtlcd_refresh(p, 0);  
    } else if( v >= p->rfid_time) {
      if( sflag ) {
        p->drv.log->data("scan rfid", *p_rfid);
      }    
      /* ��ʾ��ɨ�� */
      mtlcd_refresh(p, 2);
      m = 2;      /* ͬ��ģʽ */
      
      /* RFIDɨ��5s */
      if( sensor_mtlcd_rfid_read(p, p->keep_time) == POS_STATUS_OK ) { 
        if( sflag ) {
          p->drv.log->data("scan ok", *p_rfid);
        }    
        
        /* ��ʾ�ɹ� */
        mtlcd_refresh(p, 1);
        m = 1;
        
        /* �ɹ����¸�������ʱ�� */
        ticks = t->get() + p->keep_time;   
        
        /* �ɹ����� */
        p->drv.board->beep_set(1350);
        sensor_mtlcd_sleep(p, 250);
        p->drv.board->beep_set(2700);
        sensor_mtlcd_sleep(p, 250);
        p->drv.board->beep_set(0);
        
      } else {
        if( sflag ) {
          p->drv.log->data("scan bad", *p_rfid);
        }    
    
        /* ʧ�ܷ��� */
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
      /* ����ˢ����Ļ */      
      mtlcd_refresh(p, 0);
    }

    /* ��ѯֱ����ʱ */  
    do {
      v = mtlcd_btn_time_get(p, p->erase_time);
      if( v < p->lcd_time ) {
        if( t->is_timeout(t->get(), ticks) ) 
          break;
      } else {
        p->flags &= ~DRV_MTLCD_FLAG_NO_ALARM; /* each btn will allow a new round of alarm */
      }
      t->sleep(20);
      /* ����: �ñ���ģʽ����ˢ����Ļ */      
      mtlcd_refresh(p, m);
      p[1].lcd_time++; /* ��������������� */
    } while( v < p->lcd_time );

    /* �������������� */
    p[1].lcd_time = 0; 

    /* DC����� */  
    if( sensor_mtlcd_recharge_get(p) ) {
      p->flags |= DRV_MTLCD_FLAG_DC;
    } else {
      p->flags &= ~DRV_MTLCD_FLAG_DC;
    }
  }
  
#endif

  /* ����жϱ�־ */
  p->irq_ticks = 0;
  p[1].irq_ticks = 0;

  /* �رո澯 */
  mtlcd_alarm_set(p, 0);

  /* �˳�ǰ�ٴ�DC����� */  
  if( sensor_mtlcd_recharge_get(p) ) {
    p->flags |= DRV_MTLCD_FLAG_DC;
  } 
  
  /* ����ǳ��״ֱ̬���˳� (������ǰ��Ļ״̬ΪĬ������״̬����ֹ����)*/
  if( p->flags & DRV_MTLCD_FLAG_DC ) {
    /* ��ֹ���� , ����ʱ��ˢ��*/
    p->drv.os->call(POS_CALL_FUNC_ID_SET_NO_SLEEP, 1);
    return;
  }

  /* �������� */
  p->drv.os->call(POS_CALL_FUNC_ID_SET_NO_SLEEP, 0);

  /* �ر���Ļ���� */
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
       * ��������
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
         * ��������
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

