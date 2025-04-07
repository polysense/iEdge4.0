/**
 * @file  drv_ips.c
 * @brief SENSOR ips driver for Temp/Hum/EC/N/P/K/PH
 * @author Runby F.
 * @date 2022-10-30
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * Revision history
v1.0 @ 2024-12-17
  1) First draft
  2) Support user button (io5)
  3) Support power output control (io0)
  4) Support OTA commd (20-03-00-55-aa-00/01 for pwr off/on)
  5) Support OTA commd (20-03-00-55-bb-00 for immediate report of status)

v1.1 @ 2024-12-21
  1) Support I/A/WATT/KWH/HZ
  2) Support updated I/A/WATT/KWH formula
  3) Support IRQ uart mode to fix the reading not enough issue
  4) Support OTA remote clear accumulated KWH

v1.2 @ 2024-12-24
  1) Fix BTN not working issue
  2) Support BTN/WATT auto trigger report

v1.3 @ 2024-12-25
  1) Support default 50ms button jitter judgemet
  2) Support default 500ms power output switching delay
  3) Support dual irq detecing for user btn
*/

/**
 * @brief Driver version
 */
#define DRV_IPS_VERSION 0x0103

/**
 * @brief Driver name
 */
#define DRV_IPS_NAME  "IPS"

/**
 * @brief Driver UART opr gap delay
 */
#define DRV_IPS_DELAY(drv) drv->os->tick->sleep(500)


/**
 * @brief IRQ控制结构
 */
typedef struct {
  drv_api_t     drv;      /**< 驱动克隆，指向固定SLOT的驱动指针 */
  pos_u32_t     effective_thld; /**< minimal btn pressing ticks */
  pos_u32_t     press_ticks; /**< btn release ticks */
  pos_u32_t     irq_ticks; /**< last effective pressing ticks */
  pos_u32_t     release_ticks; /**< btn release ticks */
  pos_u8_t      pin_btn;   /**< button pin */
  pos_u8_t      pin_pwr; /**< power pin */
  pos_u8_t      btn_flag; /**< 1 - btn triggered report */
  pos_u32_t     last_watt; /**< last watt state */
} ips_param_t;


/** 
* Power control
* @param[in]  on   0:Power off, 1:Power on
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
* @note If sensor does NOT require any power control, this fucntion can be NULL
*/ 
pos_status_t sensor_ips_set(pos_u32_t on) {  
  return POS_STATUS_OK;
}

/** 
 * Driver IPS power state load
 * @param[in] param   Driver param structure pointer
 * @return     0: Power output Down\n
               1: Power output ON\n
 */
pos_u32_t sensor_ips_pwr_output_get(ips_param_t *param) {
  return param->drv.s->slot->rsvd32 & 1;
}

/** 
 * Driver IPS power state load
 * @param[in] param   Driver param structure pointer
 * @param[in] state    0: Power output Down\n
               1: Power output ON\n
 */
void sensor_ips_pwr_output_set(ips_param_t *param, pos_u32_t state) {
  drv_api_t *drv = &param->drv;
  pos_u32_t v, t;
  state &= 1;
  /* sync power output state */
  drv->os->gpio->polarity_set(param->pin_pwr, state);

  /* sync led state */
  MB_PIN_LED_SET_B(drv->os, state);

  /* save to flash when changed */
  if( state != sensor_ips_pwr_output_get(param) ) {
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      drv->log->data("ips pwr", state);
    }
    /* save to flash */
    v = drv->s->slot->rsvd32 & 0xfffffffe;
    v += state;
    t = (pos_u32_t)&drv->s->slot->rsvd32 - MA_EEPROM_ADDR;
    drv->eeprom->update(t, &v, 4);

    /* power switching delay */
    v = drv->s->slot->thld.u16[2];
    if( !v )
      v = 500;
    drv->os->tick->sleep(v);
  }

}

/** 
* Sensor read single reg
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/
pos_status_t sensor_ips_reg_get(drv_api_t *drv, pos_io_handle_t *io, pos_u8_t addr, pos_u8_t reg, pos_u32_t *p_v) {
  pos_u8_t buf[6], i, l, c, ci;
  
  for( i = 0; i < 3; i++ ) {
    buf[0] = 0x58+(addr&3);
    buf[1] = reg;

    /* flush dbg before read */
    drv->os->flush();
    io->read(io, POS_NULL, 1024, 10); /* flush */

    io->write(io, buf, 2, 100);
    l = io->read(io, &buf[2], 4, 1000);
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      drv->log->buf("ips reg/get", buf, 2+l);
    }
    if( l < 4 )
      continue;
    c = buf[0] + buf[1];
    for( ci = 0; ci < l; ci++ )
      c += buf[2+ci];
    if( c != 0xff )
      continue;
    
    *p_v = buf[2] + (buf[3] << 8) + (buf[4] << 16);
    return POS_STATUS_OK;
  }
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("ips reg/get failed", reg);
  }
  return POS_STATUS_E_RESOURCE;
}

/** 
* Sensor write single reg
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/
pos_status_t sensor_ips_reg_set(drv_api_t *drv, pos_io_handle_t *io, pos_u8_t addr, pos_u8_t reg, pos_u32_t set_msk, pos_u32_t set_value) {
  pos_status_t ret;
  pos_u8_t buf[6], i, c;
  pos_u32_t old_v;
  set_value &= set_msk;
  if( (set_msk & 0xffffff) != 0xffffff ) {
    ret = sensor_ips_reg_get(drv, io, addr, reg, &old_v);
    if( ret != POS_STATUS_OK )
      return ret;

    /* quit if already same */
    if( (old_v & set_msk) == set_value )
      return POS_STATUS_OK;

    /* delay/gap before next op */
    DRV_IPS_DELAY(drv);    

    old_v &= ~set_msk; /* mask off unset bits */
    set_value = old_v | set_value; /* or new bits */
  }

  buf[0] = 0xA8+(addr&3);
  buf[1] = reg;
  buf[2] = set_value & 0xff;
  buf[3] = (set_value>>8) & 0xff;
  buf[4] = (set_value>>16) & 0xff;
  c = 0;
  for( i = 0; i < 5; i++ )
    c += buf[i];
  buf[5] = ~c;
  io->write(io, buf, 6, 100);
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->buf("ips reg/set", buf, 6);
  }

  /* delay/gap before next op */
  DRV_IPS_DELAY(drv);
  
  return POS_STATUS_OK;
}

/**
* Sensor collecting (local, safe called by poll)
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/
pos_status_t sensor_ips_report_chk(ips_param_t *param, pos_u32_t curr_watt) {
  pos_status_t ret = POS_STATUS_E_NOT_SUPPORT;
  pos_u32_t t_w;
  drv_api_t *drv = &param->drv;
  do {
    if( param->btn_flag ) {
      param->btn_flag = 0;
      ret = POS_STATUS_OK;
    }

    t_w = drv->s->slot->thld.u16[0];
    if( !t_w )
      t_w = 10000; /* 10.000w */
    else
      t_w *= 100; /* from 0.1w to 0.001w unit */
    
    if( 
      ( curr_watt >= t_w && param->last_watt < t_w ) 
      ||
      ( curr_watt < t_w && param->last_watt >= t_w )
      ) {
      ret = POS_STATUS_OK;
    }

    param->last_watt = curr_watt;
  } while(0);

  if( ret == POS_STATUS_OK )
    drv->s->ctrl |= MA_SENSOR_CTRL_REPORT;
  
  /* periodic age/check */
  drv->thld->periodic_age(drv->s);

  /* set report when Periodic reached */
  if( drv->s->ctrl & MA_SENSOR_CTRL_REPORT ) {
    ret = POS_STATUS_OK;
  }

  return ret;
}

pos_i64_t _sensor_ips_watt_calc(pos_u32_t v) {
  pos_i64_t i64;

  if( v & 0x800000 )
    v |= 0xff800000;
  i64 = (pos_i32_t)v;
  i64 = i64*1604122 + 500000;
  i64 /= 1000000;
  return i64;
}

/**
* Sensor collecting (local, safe called by poll)
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/
pos_status_t _sensor_ips_collect(drv_api_t *drv, pos_u8_t read_clear){
  pos_io_handle_t *io;
  pos_status_t ret = POS_STATUS_E_TIMEOUT;
  pos_u32_t addr, v;
  pos_i64_t i64;
  pos_u16_t i, bmp;
  ips_param_t *param = (ips_param_t*)drv->s->drv_buf;

  /* 初始化串口 */
  i = drv->s->slot->io;
  addr = drv->s->slot->rsvd32;
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("ips io", i);        
    drv->log->data("ips rsvd", addr);
  }
  i &= 0xff; /* keep io for uart only */  
  if( POS_IO_IS_SERIAL_PORT(i) ) {
  } else
    i = 0;
  v = (addr >> 8)&0xffffff;
  if( !v ) {    
    v = 9600; /* 默认用9600 */      
  }   
  io = drv->os->io->init(i, v, 0);
  addr = (addr>>4)&3; /* ips only uses addr 0~3 */

  /* refresh to normal by default */
  drv->s->ctrl |= MA_SENSOR_CTRL_STAT_NORMAL;
  drv->s->ctrl &= ~MA_SENSOR_CTRL_STAT_ERROR;

  /* report power state */
  v = sensor_ips_pwr_output_get(param);
  drv->data->plss_json_put(v,
        DRV_MA_PLSS_TRANS(PLSS_PL_EXT_DIGITAL, 1, 0, 0, 0),
        "io");

#if 0 /* manual irq simulating for debug purpose */
  if( (g_drv->duty_loop & 3) == 3 && g_drv->duty_loop != param->release_ticks) {
    /* simulate a btn pressing */
    param->irq_ticks = 100;
    drv->log->data("simulate irq ticks", param->irq_ticks);
    drv->log->data("simulate duty", g_drv->duty_loop);
    param->release_ticks = g_drv->duty_loop;
  }
#endif

  /* report power */
  do {
    pos_u8_t buf[32];
    pos_u32_t r, l, k, r_id;
    const struct {
      pos_u8_t pos;
      pos_u8_t pl_type;
      char pl_name[6];
    }rp[] = {
      {3, PLSS_PL_EM_AAC, "amp",},
      {6, PLSS_PL_EM_VAC, "vac",},
      {12, PLSS_PL_EM_W,  "watt",},
      {15, PLSS_PL_EM_KWH, "kwh",},
      {18, PLSS_PL_EM_HZ, "hz",},
      
    };

    /* set mode/read_clear */
    if( read_clear ) {
      /* RELEASE PROTECTION */
      sensor_ips_reg_set(drv, io, addr, 0x1d, 0xffffff, 0x55);      
      ret = sensor_ips_reg_set(drv, io, addr, 0x19, 0x40, 0x40);
      if( ret != POS_STATUS_OK )
        break;
    }

    for( r_id = 0; r_id < 3; r_id++ ) {
      drv->os->flush();
      io->read(io, POS_NULL, 1024, 10); /* flush */
      buf[0] = 0x58+(addr&0x3);
      buf[1] = 0xaa;
      io->write(io, buf, 2, 100);
      l = 2;
      l += io->read(io, &buf[l], sizeof(buf)-l, 200);
      k = 0;
      for( r = 0; r < l; r++ )
       k += buf[r];
      k &= 0xff;
      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
        drv->log->buf("ips rx", buf, l);
        drv->log->data("ips crc", k);
      }
      /* crc is expected to be 0xff, however for unknown reason it's always 0xa9 here for batch reading */
      if( (k&0xff) == 0xa9 && l == 25 )
        break;
    }
    if( r_id >= 3 )
      break; /* wrong readings */

    /* judge report */
    v = buf[12] + (buf[12+1]<<8) + (buf[12+2]<<16);
    i64 = _sensor_ips_watt_calc(v);
    v = drv->os->util->abs(i64);
    r_id = sensor_ips_report_chk(param, v);
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      drv->log->data("ips io", sensor_ips_pwr_output_get(param));
      drv->log->data(i64 < 0 ? "ips r-watt" : "ips watt", v);
      drv->log->data("ips report", r_id);
    }
    if( r_id != 0 ) {
      drv->data->init(); /* clear data and report nothing if not required to report */
      ret = POS_STATUS_OK;
      break;
    }

#if 0
    /* test data for
     * 1A
     * 220V
     * -220W or 220w
     * 0.224 KWH
     */
    buf[5] = 0x03; buf[4] = 0xd5; buf[3] = 0x4d;
    buf[8] = 0x35; buf[7] = 0x51; buf[6] = 0xc0;
    buf[14] = 0x02; buf[13] = 0x17; buf[12] = 0xba; // THIS IS +220w
    //buf[14] = 0xFD; buf[13] = 0xE8; buf[12] = 0x46; // THIS IS -220w
    buf[17] = 0x00; buf[16] = 0x04; buf[15] = 0xb0;
#endif

    bmp = drv->s->slot->arg.u8[0];
    if( !bmp )
      bmp=0x1f;
    for( r_id = 0; r_id < POS_ARRAY_CNT(rp); r_id++ ) {
      if( (bmp & (1<<r_id)) == 0 )
        continue;
      k = rp[r_id].pos;
      v = buf[k] + (buf[k+1]<<8) + (buf[k+2]<<16);
      i64 = 0;
      switch( rp[r_id].pl_type ) {
      case PLSS_PL_EM_AAC:
        i64 = v;
        i64 = i64*1218 + (305978/2);
        i64 /= 305978;
        v = i64;
        POS_CALL_OP_FLOAT_LOAD_U32(drv->os, r, v);
        break;

      case PLSS_PL_EM_VAC:
        i64 = v;
        i64 = i64 * 62959 + 500000;
        i64 /= 1000000;
        v = i64;
        POS_CALL_OP_FLOAT_LOAD_U32(drv->os, r, v);
        break;

      case PLSS_PL_EM_W: {
        i64 = _sensor_ips_watt_calc(v);
        v = drv->os->util->abs(i64);
        POS_CALL_OP_FLOAT_LOAD_U32(drv->os, r, 0);
        POS_CALL_OP_FLOAT_ADD_I32(drv->os, r, i64);
        break;
      }

      case PLSS_PL_EM_KWH: {
        i64 = v;
        i64 = i64*186894+500000;
        i64 /= 1000000;
        v = i64;
        POS_CALL_OP_FLOAT_LOAD_U32(drv->os, r, v);
        break;
      }

      case PLSS_PL_EM_HZ:
        if( !v )
          v = 50;
        else
          v = 1000000000 / v;
        POS_CALL_OP_FLOAT_LOAD_U32(drv->os, r, v);
        break;
      }
      POS_CALL_OP_FLOAT_DIV_U32(drv->os, r, 1000);
      drv->data->plss_put_u32(rp[r_id].pl_type, r); /* report */
      if( i64 < 0 ) {
        drv->data->put("\"%s\":-%u.%03u,", rp[r_id].pl_name, v/1000, v%1000, 0, 0);
      } else {
        drv->data->put("\"%s\":%u.%03u,", rp[r_id].pl_name, v/1000, v%1000, 0, 0);
      }

      ret = POS_STATUS_OK;
    }
  }while(0);

  /* restore mode/read_cear to normal */
  if( read_clear ) {
    sensor_ips_reg_set(drv, io, addr, 0x19, 0x40, 0x0);
    /* RESUME/LOCK PROTECTION */
    sensor_ips_reg_set(drv, io, addr, 0x1d, 0xffffff, 0);
  }

  if( ret != POS_STATUS_OK ) {   
    /* set error flag */
    drv->s->ctrl |= MA_SENSOR_CTRL_STAT_ERROR;
    /* 错误的Sensor, 上报出错 */
    drv->data->put("\"%s\":\"no sensor#0x%x/0x%x\",", "err", i, addr, 0, 0);
  }

  if( io ) {      
    /* shutdown IO after read */
    io->release(io);
  }
  
  return ret;
}

/** 
* Sensor collecting (unsafe for poll/cb calling)
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/
pos_status_t sensor_ips_collect(void){
  return _sensor_ips_collect(g_drv, 0);
}

/**
 * IRQ interrupt callback handler
 * @note This function can NOT call g_drv. When IRQ happening, g_drv might piont to any sensor drivers. It should call the param stored driver pointer.
 */
void sensor_ips_irq(ips_param_t *param) {
  pos_u32_t ticks, effective_thld;
  ticks = param->drv.os->tick->get();
  if( !ticks )
    ticks = 1; /* 至少是1 */
  if( param->drv.os->gpio->get(param->pin_btn) == 0 )
    param->press_ticks = ticks; /* 记录BtnPress中断时刻 */
  else {
    param->release_ticks = ticks; /* 记录BtnRelease中断时刻 */
    if( param->press_ticks ) {
      effective_thld = param->drv.os->tick->elaps_calc(ticks, param->press_ticks);
      if( effective_thld >= param->effective_thld )
        param->irq_ticks = effective_thld;
      param->press_ticks = 0; /* clear press ticks after reading */
    }
  }
  param->drv.os->gpio->irq_clr((pos_gpio_pin_t)param->pin_btn);
}

/**
 * Polling callback function for IRQ
 * @note This function can NOT call g_drv. When polling, g_drv might piont to any sensor drivers. It should call the param stored driver pointer.
 */
void sensor_ips_poll_irq(ips_param_t *p) {
  drv_api_t *drv;
  pos_u32_t v;
  /* IRQ response power switching */
  if( !p->irq_ticks ) 
    return;

  drv  = &p->drv;

  /* switching on/off --> off/on */
  v = sensor_ips_pwr_output_get(p);
  v =  v ? 0 : 1;
  sensor_ips_pwr_output_set(p, v);

  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("ips button", v);
  }
  
  /* update current time as last ticks */
  p->irq_ticks = 0; // clear irq flag at last

  /* report when btn pressed */
  p->btn_flag = 1;
  sensor_ips_collect();
}

/** 
* CLI callback
* @return     0: Buffer has been processed and no need to be processed further more\n
            !=0: Buffer still has to be processed by regular CLI processings
*/ 
pos_status_t sensor_ips_drv_cb(pos_u32_t src, pos_u8_t *cmd, pos_size_t cmd_len, ips_param_t *p) {

  /* when src is a pointer of param: it's called by main polling. redirect the main polling for irq here
   */
  if( src >= ((pos_u32_t)g_drv & 0xffff0000) ) { 
    sensor_ips_poll_irq((ips_param_t*)src);
    return POS_STATUS_OK;
  }

  /* treate src as dn/cli msg callback */
  
  if( src != DRV_CB_SRC_DN_MSG ) {
    return POS_STATUS_ERROR; /* only support dn msg currently */
  }

  if( p->drv.cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    p->drv.log->buf("ips cb/msg", cmd, cmd_len);
  }
  if( cmd_len < 3 )
    return POS_STATUS_E_RESOURCE;
  if( cmd[0] != (MA_SENSOR_IPS&0xff) )
    return POS_STATUS_E_NOT_FOUND;
  switch( cmd[1] ) {
  case 0xaa:
    /* power control */
    if( p->drv.cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      p->drv.log->data("ips cb/pwr", cmd[2]); 
    }
    p->btn_flag = 1;
    sensor_ips_pwr_output_set(p, cmd[2]&1);
    _sensor_ips_collect(&p->drv, 0);
    return POS_STATUS_OK;
    break;

  case 0xbb:
  case 0xcc:
    /* report current status (ignore cmd[2]) */
    if( p->drv.cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      p->drv.log->data("ips cb/get", cmd[1]); 
    }
    p->btn_flag = 1;
    _sensor_ips_collect(&p->drv, cmd[1] == 0xbb ? 0 : 1);
    return POS_STATUS_OK;
    break;
    
  default:
    break;
  } 

  return POS_STATUS_E_NOT_SUPPORT;
}

/** 
 * Driver init
 * @param[in]  load   0:Unload driver, 1:Load driver
 * @return     0: Successful\n
             !=0: Failed, refer to @ref pos_status_t
 * @note This function can be NULL if it does NOT require any init operatoin
 */ 
pos_status_t sensor_ips_init(pos_u32_t load) {
  drv_api_t *drv = g_drv;
  ips_param_t *param;
  ma_sensor_ctrl_t *s = g_drv->s;
  pos_gpio_pin_t pin_pwr, pin_btn;
  pos_u32_t v;
  drv_call_cb_param_t cb;
  pin_pwr = drv->board->pin_map(MB_PT_DIO,MB_DIO_ID_DEFAULT);
  pin_btn = drv->board->pin_map(MB_PT_DIO,MB_DIO_ID_BTN);

  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("ips init", load+(pin_btn<<8)+(pin_pwr<<16));
  }  
  
  if( load ) {
    param = drv->os->malloc(sizeof(*param));
    if( !param )
      return POS_STATUS_E_MEM;

    drv->os->memset(param, 0, sizeof(*param));

    param->btn_flag = 1;
    param->pin_pwr = pin_pwr;
    param->pin_btn = pin_btn;
    param->effective_thld = s->slot->thld.u16[1];
    if( !param->effective_thld )
      param->effective_thld = 50; /* 50ms time by default */
    drv->os->memcpy(&param->drv, drv, sizeof(*drv)); /* clone a g_drv */
    s->drv_buf = param;  /* record to drv_buf */

    /* LED blinking indicating sys started */
    for( v = 1; v < 7; v++ ) {
      MB_PIN_LED_SET_B(drv->os, v&1);
      drv->os->tick->sleep(250);
    }

    /* init busy as INPUT PULLUP */
    drv->os->gpio->mode_set(param->pin_btn, POS_GPIO_MODE_INPUT_PU);

    /* restore default state */
    v = sensor_ips_pwr_output_get(param);
    sensor_ips_pwr_output_set(param, v);
    drv->os->gpio->mode_set(param->pin_pwr, POS_GPIO_MODE_OUTPUT);
    sensor_ips_pwr_output_set(param, v);

    /*
     * turn on irq
     * warning: as GOT is implemented in DATA and DATA is not supported in mmap()
     * all GOT based pointer can NOT be used. So the IRQ SET should use the mapped
     * function pointer here (the export structure). 
     * the orignal sensor_fastcnt_irq() should NOT be used directly.
     */     
    drv->os->gpio->irq_set(
      pin_btn, 
      POS_GPIO_MODE_INPUT+POS_GPIO_MODE_IRQ_RISING+POS_GPIO_MODE_IRQ_FALLING+POS_GPIO_MODE_PU_ENABLE,
      (pos_func_t)drv->s->drv.u.sensor.irq_cb, /* has to be this. DO NOT use sensor_fastcnt_irq */
      param);

    /*
     * turn on poll
     */
    drv->proc->event_register((pos_func_t)drv->s->drv.u.sensor.poll_cb, param);

    /*
     * turn on CLI CB
     */
    cb.cb = (drv_call_cb_t)drv->s->drv.u.sensor.poll_cb;
    cb.cookie = param;
    drv->call(DRV_CALL_FUNC_ID_SET_CB, (pos_u32_t)&cb);

  } else {      
    /*
     * turn off CLI CB
     */
    cb.cb = POS_NULL;
    cb.cookie = POS_NULL;
    drv->call(DRV_CALL_FUNC_ID_SET_CB, (pos_u32_t)&cb);
    
    /*
     * turn off poll
     */
    drv->proc->event_unregister((pos_func_t)drv->s->drv.u.sensor.poll_cb, s->drv_buf);

    /*
     * turn off irq
     */
    drv->os->gpio->irq_set(pin_btn, POS_GPIO_MODE_INPUT_PU+POS_GPIO_MODE_IRQ_DISABLE, POS_NULL, POS_NULL);
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
  .version = DRV_SENSOR_VERSION(MA_SENSOR_IPS, DRV_IPS_VERSION),
  .name = DRV_IPS_NAME,
  .u.sensor={
    .power_set = sensor_ips_set,
    .collect = sensor_ips_collect,
    .init = sensor_ips_init,
    .irq_cb = (pos_func_t)sensor_ips_irq,
    .poll_cb = (pos_func_t)sensor_ips_drv_cb,

  },
};

