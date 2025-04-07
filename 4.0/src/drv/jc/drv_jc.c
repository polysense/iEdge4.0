/**
 * @file  drv_jc.c
 * @brief SENSOR JC driver
 * @author Runby F.
 * @date 2024-2-26
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * Revision history
v1.0 @ 2024-5-17
  1) First revision
  2) Support time sync from JC sensor when ext module time is not ready

v1.1 @ 2024-5-31
  1) Support PLSS payload report when srsv=1

*/

/**
 * @brief Driver version
 */
#define DRV_JC_VERSION 0x0101

/**
 * @brief Driver name
 */
#define DRV_JC_NAME  "JC"

#define JC_HDR_SYNC 0xfe
#define JC_HDR_PREFIX 0x68
#define JC_HDR_TYPE 0x10
#define JC_HDR_SUFFIX 0x16
#define JC_CMD_ID_READ_DATA_REQ   0x011f90 /* lora read from meter for data querying */
#define JC_CMD_ID_READ_DATA_RSP   0x811f90
#define JC_CMD_ID_READ_TIME_REQ   0x784da1 /* lora read from meter for time querying */
#define JC_CMD_ID_READ_TIME_RSP   0xe04da1
#define JC_CMD_ID_VALVE_CTRL_REQ  0x0417a0 /* lora set to meter for valve control */
#define JC_CMD_ID_VALVE_CTRL_RSP  0x8417a0
#define JC_CMD_ID_LORA_SET_REQ    0xa72131 /* meter/rf set lora params */
#define JC_CMD_ID_LORA_SET_RSP    0x292131
#define JC_CMD_ID_LORA_GET_REQ    0xa82232 /* meter/rf get lora params */
#define JC_CMD_ID_LORA_GET_RSP    0x2c2232
#define JC_CMD_ID_REPORT_SET_REQ  0x7b50a1 /* meter/rf set report params */
#define JC_CMD_ID_REPORT_SET_RSP  0xe350a1
#define JC_CMD_ID_REPORT_GET_REQ  0x7c51a1 /* meter/rf get lora params */
#define JC_CMD_ID_REPORT_GET_RSP  0xe451a1
#define JC_CMD_ID_REPORT_FORCE_REQ    0xcb15cc /* report forcedly immediately */
#define JC_CMD_ID_REPORT_FORCE_RSP    0xdb15cc

#define JC_CMD_REQ_TIMEOUT 1000

/*
 * @brief JC Structure
 */
typedef struct {
  drv_api_t     drv;      /**< 驱动克隆，指向固定SLOT的驱动指针 */
  pos_u8_t      buf[128];
  pos_u8_t      buf_len;
  pos_u8_t      rsvd;
  pos_u16_t     seq;
} jc_param_t;

pos_u32_t app_jc_cmd_id_get(pos_u8_t *buf) {
  pos_u32_t v;
  v = (buf[9] << 16) + (buf[12] << 8) + buf[13];
  return v;
}

pos_u8_t app_jc_cksum(pos_size_t len, pos_u8_t *buf) {
  pos_u8_t cksm = 0;
  while(len--) {
    cksm += *buf++;
  }
  return cksm;
}

pos_u32_t app_jc_cksm_check(pos_size_t len, pos_u8_t *buf) {
  if( buf[len-2] == app_jc_cksum(len-2, buf) )
    return 0;
  return 1;
}

void app_jc_cmd_send(jc_param_t *p, pos_u32_t jc_cmd_id, pos_u8_t param_len, pos_u8_t *param, pos_u8_t *original_req) {
  const pos_lib_t *os = p->drv.os;
  pos_u8_t buf[96], *c, len;   
  if( param_len > 96 - 18 - 1 )
    return; /* wrong length */

  /* flush and stop a very small cycle */
  os->flush(); 
  os->tick->sleep(50);
  
  buf[0] = JC_HDR_SYNC; /* #0 for 0xfe sync */
  
  c = &buf[1]; /* payload starts from buf #1 */
  c[0] = JC_HDR_PREFIX;
  c[1] = JC_HDR_TYPE;

  /* set meter id from #2..#8 */
  if( original_req )
    os->memcpy(&c[2], &original_req[2], 7);
  else
    os->memset(&c[2], 0xaa, 7);
  
  c[9] = (jc_cmd_id >> 16)&0xff;
  c[10] = 0; /* stat */
  c[11] = param_len+4;
  c[12] = (jc_cmd_id >> 8)&0xff;
  c[13] = jc_cmd_id&0xff;

  /* seq seq */
  if( original_req ) {
    c[14] = original_req[14];
    c[15] = original_req[15]; 
  } else {
    ++p->seq;
    if( p->seq == 0 )
      p->seq = 1; /* avoid to use zero */
    c[14] = p->seq&0xff;
    c[15] = (p->seq>>8)&0xff;
  }

  if( param_len )
    os->memcpy(&c[16], param, param_len);

  len = 18 + param_len;
  c[len-2] = app_jc_cksum(len-2, c);
  c[len-1] = JC_HDR_SUFFIX;
  os->util->console_write(buf, len+1); /* 0xfe/SYNC must be added to len */

  /* let console flush out */
  os->flush();
}

/* processing jc query commands to our side */
pos_u8_t app_jc_req(jc_param_t *p, pos_u32_t cmd_id, pos_u32_t rsp_id,  pos_u8_t param_len, pos_u8_t *param, pos_u8_t exp_len, pos_u8_t *exp_buf) {
  const pos_lib_t *os = p->drv.os;
  pos_u8_t *b;
  pos_size_t len, i, j;
  for( i = 0; i < 3; i++ ) {
    p->buf_len = 0;
    app_jc_cmd_send(p, cmd_id, param_len, param, POS_NULL);
    for( j = 0; j < JC_CMD_REQ_TIMEOUT; j += 100 ) {
      os->tick->sleep(100);
      if( p->buf_len >= 18 )
        break;
    }
    len = p->buf_len;
    if( len < 18 )
      continue;
    b = p->buf;
    /* eat the first 0xfe */
    while(len && *b == JC_HDR_SYNC) {
      len--;
      b++;
    }
    if( len < 18 )
      continue;

    /* cksm check */
    if( app_jc_cksm_check(len, b) != 0 ) {
      continue;
    }

    /* exp ID check */
    if( b[9] != ((rsp_id >> 16)&0xff)) {
      continue;
    }

    if( len < exp_len )
      exp_len = len;
    os->memcpy(exp_buf, b, exp_len);
    return exp_len;
  }
  return 0;
}

/** 
* CLI callback
* @return     0: Buffer has been processed and no need to be processed further more\n
            !=0: Buffer still has to be processed by regular CLI processings
*/ 
pos_status_t sensor_jc_drv_cb_dn_msg(pos_u8_t *cmd, pos_size_t cmd_len, jc_param_t *p) {
  pos_status_t ret = POS_STATUS_OK;
  if( p->drv.cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    p->drv.log->buf("jc cb", cmd, cmd_len);
  }  
  if( cmd_len < 3 )
    return POS_STATUS_E_RESOURCE;
  if( cmd[0] != 0x4b )
    return POS_STATUS_E_NOT_FOUND;
  if( cmd[1] == (JC_CMD_ID_VALVE_CTRL_REQ>>16) ) {
    pos_u8_t buf[26], v;
    /* valve control */
    /* cmd[2] =- 0x55/0x99/0xaa for ON/OFF/HALF */
    v = app_jc_req(p, JC_CMD_ID_VALVE_CTRL_REQ, JC_CMD_ID_VALVE_CTRL_RSP, 1, &cmd[2], 26, buf);
    if( v != 20 ) {
      /* valve control wrong */
      if( p->drv.cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
        p->drv.log->buf("jc valve", buf, v); 
      }
    }
  } else
    return POS_STATUS_E_NOT_SUPPORT;
  return ret;
}

/** 
* CLI callback
* @return     0: Buffer has been processed and no need to be processed further more\n
            !=0: Buffer still has to be processed by regular CLI processings
*/ 
pos_status_t sensor_jc_drv_cb(drv_cb_src_t src, pos_u8_t *cmd, pos_size_t cmd_len, jc_param_t *p) {
  pos_status_t ret = POS_STATUS_OK;
  const pos_lib_t *os = p->drv.os;

  if( src == DRV_CB_SRC_DN_MSG ) {
    return sensor_jc_drv_cb_dn_msg(cmd, cmd_len, p);
  }

  if( src != DRV_CB_SRC_CLI )
    return POS_STATUS_ERROR;

  /* eat the first 0xfe */
  while(cmd_len && *cmd == JC_HDR_SYNC) {
    cmd_len--;
    cmd++;
  }
  if( cmd_len < 18 ) /* minimal length check */
    return 7;
  
  do {    
    /* hdr check */
    if( cmd[0] != JC_HDR_PREFIX || cmd[1] != JC_HDR_TYPE || cmd[cmd_len-1] != JC_HDR_SUFFIX ) {
      ret = 1; /* wrong hdr */
      break;
    }

    /* cksm check */
    if( app_jc_cksm_check(cmd_len, cmd) != 0 ) {
      ret = 2; /* cksm error */
      break;
    }

    switch(app_jc_cmd_id_get(cmd)) {
    case JC_CMD_ID_LORA_SET_REQ: {
      ma_lora_cfg_t *cfg;
      if( cmd_len != 87 ) {
        ret = 4; /* wrong payload */
        break;
      }
      cfg = (ma_lora_cfg_t*)os->malloc(sizeof(*cfg));
      if( !cfg ) {
        ret = 5; /* mem err */
        break;
      }
      os->memcpy((pos_u8_t*)cfg, (pos_u8_t*)&p->drv.cfg->lora, sizeof(*cfg));
      os->memcpy((pos_u8_t*)&cfg->lora_dev_addr, &cmd[16], 4);
      os->memcpy((pos_u8_t*)cfg->lora_dev_eui, &cmd[20], 64); /* eui+app+key+skey+nwk skey */
      if( cmd[84] ) {
        cfg->lora_ctrl &= ~0x10; /* Unset OTA flag for ABR mode */
      } else {
        cfg->lora_ctrl |= 0x10; /* Set OTA flag for OTA mode */
      }
      app_jc_cmd_send(p, JC_CMD_ID_LORA_SET_RSP, 0, POS_NULL, cmd); /* rsp before RESET */
      if( os->memcmp((pos_u8_t*)cfg, (pos_u8_t*)&p->drv.cfg->lora, sizeof(*cfg)) != 0 ) {
        /* program and reset when cfg changed */
        p->drv.eeprom->update((pos_u8_t*)&p->drv.cfg->lora - (pos_u8_t*)p->drv.cfg, cfg,sizeof(*cfg));
        os->tick->sleep(100);
        os->reset();
      }
      os->free(cfg);
      break;
    }

    case JC_CMD_ID_LORA_GET_REQ: {
      pos_u8_t *d;
      ma_lora_cfg_t *cfg;      
      d = os->malloc(96);
      if( !d ) {
        ret = 5; /* mem err */
        break;
      }
      cfg = (ma_lora_cfg_t*)&p->drv.cfg->lora;
      os->memcpy(&d[0], (pos_u8_t*)&cfg->lora_dev_addr, 4);
      os->memcpy(&d[4], (pos_u8_t*)&cfg->lora_dev_eui, 64);
      d[68] = (cfg->lora_ctrl & 0x10) != 0 ? 0 : 1; /* OTA:ret 0, ABP: ret 1 */
      d[69] = p->drv.cfg->report_type & 0xff; /* lora type */
      app_jc_cmd_send(p, JC_CMD_ID_LORA_GET_RSP, 70, d, cmd);
      os->free(d);
      break;
    }

    case JC_CMD_ID_REPORT_SET_REQ:
      if( cmd_len != 23 ) {
        ret = 4; /* wrong payload */
        break;
      }
      p->drv.eeprom->update(MA_CFG_OFS_TIME_CTRL, &cmd[16], 5);
      app_jc_cmd_send(p, JC_CMD_ID_REPORT_SET_RSP, 0, POS_NULL, cmd);
      break;

    case JC_CMD_ID_REPORT_GET_REQ:
      app_jc_cmd_send(p, JC_CMD_ID_REPORT_GET_RSP, 5, (pos_u8_t*)&p->drv.cfg->time_ctrl, cmd);
      break;

    case JC_CMD_ID_REPORT_FORCE_REQ:
      app_jc_cmd_send(p, JC_CMD_ID_REPORT_FORCE_RSP, 0, POS_NULL, cmd);
      p->drv.call(DRV_CALL_FUNC_ID_SET_NEXT_DUTY, 0);
      break;

    default:
      /* save */
      if( cmd_len <= sizeof(p->buf) ) {
        os->memcpy(p->buf, cmd, cmd_len);
        p->buf_len = cmd_len;
      }
      /* return zero to eat others */
      break;
    }
  } while(0);
  return ret;
}

/** 
* Power control
* @param[in]  on   0:Power off, 1:Power on
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
* @note If sensor does NOT require any power control, this fucntion can be POS_NULL
*/ 
pos_status_t sensor_jc_set(pos_u32_t on) {  
  pos_u16_t pwr;
  ma_board_t *b = g_drv->board;
  pwr = (g_drv->s->slot->io >> 8) & 0x7f;  
  if( !pwr ) {
      b->pwr_set(pwr, on);
  }
  
  return POS_STATUS_OK;
}

/** 
 * Driver init
 * @param[in]  load   0:Unload driver, 1:Load driver
 * @return     0: Successful\n
             !=0: Failed, refer to @ref pos_status_t
 * @note This function can be POS_NULL if it does NOT require any init operatoin
 */ 
pos_status_t sensor_jc_init(pos_u32_t load) {
  drv_api_t *drv = g_drv;
  ma_sensor_ctrl_t *s = g_drv->s;
  jc_param_t *param;
  drv_call_cb_param_t cb;
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("jc init", load);
  }
  if( load ) {
    s->data.u32 = 0; /* clear counter by default */
    param = drv->os->malloc(sizeof(*param));
    if( !param ) {
      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
        drv->log->data("jc mem error", sizeof(*param)); 
      }        
      return POS_STATUS_E_MEM;
    }
    drv->os->memset(param, 0, sizeof(*param));

    drv->os->memcpy(&param->drv, drv, sizeof(*drv)); /* clone a g_drv */    
    s->drv_buf = param;  /* record to drv_buf */
    
    /*
     * turn on CLI CB
     */
    cb.cb = (drv_call_cb_t)drv->s->drv.u.sensor.poll_cb;
    cb.cookie = param;     
    drv->call(DRV_CALL_FUNC_ID_SET_CB, (pos_u32_t)&cb);
  } else if( s->drv_buf ) { 
    /*
     * turn off CLI CB
     */
    cb.cb = POS_NULL;
    cb.cookie = POS_NULL;
    drv->call(DRV_CALL_FUNC_ID_SET_CB, (pos_u32_t)&cb);

    /* Free mem */
    drv->os->free(drv->s->drv_buf);
    s->drv_buf = POS_NULL;    
  }
  return POS_STATUS_OK;
}


/** 
* Translate from JC bcd to binary (unit: 0.01 m3 / 10L)
*/
pos_u32_t sensor_jc_bcd_to_bin(pos_u8_t *jc_bcd) {
  pos_u8_t i;
  pos_u32_t v;
  v = 0;
  for( i = 4; i > 0; i-- ) {
    v = v * 100 + (jc_bcd[i-1] >> 4)*10 + (jc_bcd[i-1]&0xf);
  }
  return v;
}

/** 
* Sensor collecting
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/
pos_status_t sensor_jc_collect(void){
  pos_status_t ret = POS_STATUS_E_TIMEOUT;
  pos_u32_t v;
  drv_api_t *drv = g_drv;
  pos_u8_t buf[40], i;
  char jbuf[96];
  jc_param_t *param;

  param = (jc_param_t *)drv->s->drv_buf;
  
  /* refresh to normal by default */
  drv->s->ctrl |= MA_SENSOR_CTRL_STAT_NORMAL;
  drv->s->ctrl &= ~MA_SENSOR_CTRL_STAT_ERROR;  

  do {
    /* call timesync from sensor when module time is not ready */
    if( drv->history->time.tick == 0 ) {
      /* timesync */
      i = 1;
      v = app_jc_req(param, JC_CMD_ID_READ_TIME_REQ, JC_CMD_ID_READ_TIME_RSP, 1, &i, 40, buf);
      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
        drv->log->buf("jc rd", buf, v);
      }
      if( v != 25 ) {
        ret = POS_STATUS_E_RESOURCE;
        break;
      }
      for( i = 0; i < 6; i++ ) 
        buf[i] = buf[22-i];
      buf[6] = 7;/* wrong weekdays to trigger auto weekday setup */
      drv->os->rtc->bcd2bin(buf, buf);
      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
        drv->log->buf("jc time", buf, 7);
      }
      drv->os->rtc->time2_set(buf, drv->os->tick->get()); /* sync to os */
    }

    v = app_jc_req(param, JC_CMD_ID_READ_DATA_REQ, JC_CMD_ID_READ_DATA_RSP, 0, POS_NULL, 40, buf);
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      drv->log->buf("jc rd", buf, v);
    }
    if( v != 37 ) {
      ret = POS_STATUS_E_RESOURCE;
      break;
    }

    /* report PLSS format when srsv=1 */
    if( (drv->s->slot->rsvd32 & 0xff) == 1 ) {
#if 0 
      /* DBG code */
      buf[16] = 0x10;
      buf[17] = 0x23;
      buf[18] = 0x01;
      buf[21] = 0x40;
      buf[22] = 0x23;
      buf[23] = 0x00;
#endif      
      v = sensor_jc_bcd_to_bin(&buf[16]);
      v *= 10; /* translate from 0.01 m3 to 0.001 m3 */
      drv->data->plss_json_put(v, DRV_MA_PLSS_TRANS(PLSS_PL40_VELOCITY_VOLUME, 4, 0, 0, 3), "vol");
      v = sensor_jc_bcd_to_bin(&buf[21]);
      v *= 10; /* translate from 0.01 m3 to 0.001 m3 */
      drv->data->plss_json_put(v, DRV_MA_PLSS_TRANS(PLSS_PL40_VELOCITY_VOLUME_R, 4, 0, 0, 3), "rvol");
      ret = POS_STATUS_OK;
      break;
    }

    /* data put */
    jbuf[0] = PLSS_PL_RTU_PAYLOAD;
    jbuf[1] = 37;
    drv->os->memcpy(&jbuf[2], buf, 37); 
    drv->data->plss_put_raw(37+2,jbuf);
    drv->os->sprintf(jbuf, "\"rtu\":\"%37B\",", buf);
    drv->data->put_raw(jbuf, drv->os->strlen(jbuf));
    ret = POS_STATUS_OK;    
  } while(0);
  
  if( ret != POS_STATUS_OK ) {
    /* set error flag */
    drv->s->ctrl |= MA_SENSOR_CTRL_STAT_ERROR|MA_SENSOR_CTRL_ABANDON; /* do NOT report when error JC */
#if 0    
    /* 错误的Sensor, 上报出错 */
    drv->data->put("\"%s\":\"no jc\",", "err", 0,0,0,0);
#endif
  }
  
  return ret;
}

/*
 * Driver export
 */
ma_drv_export_t g_export = {
  .version = DRV_SENSOR_VERSION(MA_SENSOR_JC, DRV_JC_VERSION),
  .name = DRV_JC_NAME,
  .u.sensor={
    .power_set = sensor_jc_set,
    .collect = sensor_jc_collect,
    .init = sensor_jc_init,
    .poll_cb = (pos_func_t)sensor_jc_drv_cb,
  }
};

