/**
 * @file  drv_pack.c
 * @brief Packer pseudo sensor driver to pack multiple report datas and report it later
 * @author Runby F.
 * @date 2023-11-04
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * Revision history
v1.0 @ 2023-11-09
  1) First draft

v1.1 @ 2023-11-28
  1) Support disable timestamp recording by srsv.bit[0] == 0
  2) Support thold flag by srsv.bit[1] == 1

v1.2 @ 2023-11-29
  1) Support to skip time and GPS when pack/thold 
  
*/

/**
 * @brief Driver version
 */
#define DRV_PACK_VERSION 0x0102

/**
 * @brief Driver name
 */
#define DRV_PACK_NAME  "PACK"

/**
 * @brief Max packet
 */
#define DRV_PACK_MAX_PACKET  8

/**
 * @brief Max threshold check data types
 */
#define DRV_PACK_MAX_THOLD_FLAG  8

/**
 * @brief Pack Control Strucutre
 */
typedef struct {
  drv_api_t     drv;      /**< Clone driver, pointing to fixed slot driver pointer */
  pos_u16_t     packet_size;  /**< Each packet buffer size */
  pos_u8_t      packet_num;   /**< Actual number of packet buffer */
  pos_u8_t      store_id;     /**< Current store of packet ID */
  pos_u8_t      report_id;    /**< Current report of packet ID */
  pos_u8_t      duty_cnt;     /**< Accumulated stored duty/data count */
  pos_u16_t     rsvd;         /**< Reserved for future */
  pos_u8_t      *pack[DRV_PACK_MAX_PACKET]; /**< Packet buffer pool */
  pos_u16_t     pack_len[DRV_PACK_MAX_PACKET]; /**< Current stored size for each packet */
  pos_u8_t      buf_plain[0]; /**< Plain buffer pool (do NOT use it. use pack[n]. */
} pack_param_t;


/**
 * @brief PLSS pl length
 */
static const pos_u8_t g_pack_plss_len[] = {PLSS_SENSOR_PL_LEN_DEFINE};

/** 
 * Driver init
 * @param[in]  load   0:Unload driver, 1:Load driver
 * @return     0: Successful\n
             !=0: Failed, refer to @ref pos_status_t
 * @note This function can be NULL if it does NOT require any init operatoin
 */ 
pos_status_t sensor_pack_init(pos_u32_t load) {
  drv_api_t *drv = g_drv;
  pack_param_t *param;
  ma_sensor_ctrl_t *s = g_drv->s;
  
  if( load ) {
    pos_u16_t size, num, i;
    /* refresh to normal by default */
    drv->s->ctrl |= MA_SENSOR_CTRL_STAT_NORMAL;
    drv->s->ctrl &= ~MA_SENSOR_CTRL_STAT_ERROR;
    size = drv->ext->mtu - 16; /* reserve header fields */
    if( size <= 256 )
      num = 8;
    else if( size <= 512 )
      num = 4;
    else
      num = 2;
    param = drv->os->malloc(sizeof(*param)+size*num);
    if( !param ) {
      drv->s->ctrl |= MA_SENSOR_CTRL_STAT_ERROR;
      return POS_STATUS_E_MEM;
    }
    drv->os->memset(param, 0, sizeof(*param)+size*num);
    /* init default variable */
    param->packet_size = size;
    param->packet_num = num;
    param->report_id = num; /* do NOT report by setting its id >= num */
    for( i = 0; i < num; i++ )
      param->pack[i] = &param->buf_plain[i*num]; 
    drv->os->memcpy(&param->drv, drv, sizeof(*drv)); /* clone a g_drv */
    s->drv_buf = param;  /* record to drv_buf */

    /*
     * turn on poll
     */
    drv->proc->event_register((pos_func_t)drv->s->drv.u.sensor.poll_cb, param);


  } else {      
    /*
     * turn off poll
     */
    drv->proc->event_unregister((pos_func_t)drv->s->drv.u.sensor.poll_cb, s->drv_buf);

    if( s->drv_buf ) {
      drv->os->free(drv->s->drv_buf);
      s->drv_buf = POS_NULL;
    }

  }
  return POS_STATUS_OK;
}

/** 
 * Polling thld flag check
 */
void sensor_pack_poll_thld_flag(pack_param_t *p, pos_u32_t len, pos_u8_t *pl) {
  pos_u32_t data_cnt, i, l, v;
  pos_u32_t flag[DRV_PACK_MAX_THOLD_FLAG];
  pos_u8_t pl_type[DRV_PACK_MAX_THOLD_FLAG];
  pos_u8_t pl_loop[DRV_PACK_MAX_THOLD_FLAG];
  data_cnt = 0;

  while(len) {
    for( i = 0; i < data_cnt; i++ ) {
      if( pl[0] == pl_type[i] )
        break;
    }
    if( pl[0] >= POS_ARRAY_CNT(g_pack_plss_len) )
      break; /* unable to parse */
    l = g_pack_plss_len[pl[0]];
    if( !l )
      break; /* payload end */
    if( len < l + 1 )
      break; /* wrong payload */
    do {
      /* skip gps and time */
      if( pl[0] == PLSS_PL_GPS || pl[0] == PLSS_PL_TIME )
        break;
      if( i >= data_cnt ) {
        if( data_cnt >= DRV_PACK_MAX_THOLD_FLAG )
          break; /* exceed */
        /* add new */
        pl_type[i] = pl[0];
        pl_loop[i] = 0;
        flag[i] = 0;
        
        data_cnt++;
      }

      /* check thold */
      v = pl[1];
      if( l > 1 )
        v = (v << 8) + pl[2];
      if( v > p->drv.s->slot->thld.u16[i&0x7] )
        flag[i] |= 1 << pl_loop[i]; /* set flag when cross thold */
      
      /* increase loop */
      pl_loop[i]++;
    } while(0);

    len -= l + 1;
    pl += l + 1;
  }
  
  /* append threshold flag */
  for( i = 0; i < data_cnt; i++ )
    p->drv.data->plss_put_u16(PLSS_PL_POS_BMP16, flag[i]);
  
}

/** 
 * Polling callback function
 * @note This function can NOT call g_drv. When polling, g_drv might piont to any sensor drivers. It should call the param stored driver pointer.
 */
void sensor_pack_poll(pack_param_t *p) {
  pos_u16_t len;
  
  /* do nothing and return when report_id is disabled */
  if( p->report_id >= p->packet_num )
    return;

  /* retrieve report length */
  if( p->report_id <= p->store_id ) {
    len = p->pack_len[p->report_id];
  } else {
    len = 0;
  }
 
  if( p->drv.cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    p->drv.log->data("pack report id", p->report_id);    
    p->drv.log->data("pack report len", len);
  }

  if( len ) {
#if 0    
    /* init data to empty */
    p->drv.call(DRV_CALL_FUNC_ID_SET_DATA_CLEAR, 0);
#endif

    /* report data */
    p->drv.data->plss_put_raw(len, p->pack[p->report_id]);
    /* report thold flag */
    if( p->drv.s->slot->rsvd32 & 2 )
      sensor_pack_poll_thld_flag(p, len, p->pack[p->report_id]);
    p->drv.data->put("\"%s\":%u,\"size\":%u,","report_id",p->report_id, len, 0, 0); /* json information only. not support json payload */
    /* clear store length after report */
    p->pack_len[p->report_id] = 0;
  }
  
  /* proceed to next */
  p->report_id++;

  /* set finish flag and reset store_id to zero when all reported */
  if( !len || p->report_id > p->store_id ) {
    p->report_id = p->packet_num; /* finish report */
    p->store_id = 0; /* reset store id */
    p->duty_cnt = 0; /* reset duty cnt */
    /* report immediately */
    if( p->drv.cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      p->drv.log->data("pack report done", p->report_id);      
    }
    
  }
}

/** 
* Sensor collecting
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_pack_collect(void){
  pos_status_t ret = POS_STATUS_OK;
  drv_api_t *drv = g_drv;
  pack_param_t *param = (pack_param_t*)drv->s->drv_buf;
  pos_u8_t *plss_data;
  pos_u32_t plss_data_len, step_len;

  /* reset to ERROR by default */
  drv->s->ctrl |= MA_SENSOR_CTRL_STAT_NORMAL|MA_SENSOR_CTRL_STAT_ERROR;

  do {  
    /* only support pack for LoRa and PLSS Report */
    if( MA_REPORT_TYPE_IS_MQTT(drv->ext->report_type) ) {
      ret = POS_STATUS_E_NOT_SUPPORT;
      break;
    }

    /* do Nothing when param is not init */
    if( !param ) {
      ret = POS_STATUS_E_INIT;
      break;
    }

    /* do Nothing when param parameter wrong */
    if( !param->packet_num || !param->packet_size || param->packet_num > DRV_PACK_MAX_PACKET ) {
      ret = POS_STATUS_E_PARAMETER;
      break;
    }

  } while(0);

  if( ret != POS_STATUS_OK ) {
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      drv->log->data("pack err", ret);
    }
  return ret;
  }

  /* clear ERROR flag, indicating OKAY */
  drv->s->ctrl &= ~MA_SENSOR_CTRL_STAT_ERROR;    

  /* length check */
  plss_data_len = 0;
  plss_data = POS_NULL;
  drv->call(DRV_CALL_FUNC_ID_GET_PLSS_DATA_BUF, (pos_u32_t)&plss_data);
  drv->call(DRV_CALL_FUNC_ID_GET_PLSS_DATA_LEN, (pos_u32_t)&plss_data_len);  

  /* no valid data and ignore */
  if( plss_data_len < 14 ) {    
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      drv->log->data("pack skip null", plss_data_len);
    }    
    return POS_STATUS_OK;
  }
  step_len = plss_data_len - 13 + 7; /*-13 for remove header, +7 for timestamp */  

  do {  
    /* increase duty cnt */
    param->duty_cnt++;

    /* check too big */
    if( step_len > param->packet_size ) {
      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
        drv->log->data("pack skip big", step_len);
      }
      return POS_STATUS_E_RESOURCE; /* skip too big packets (do NOTHING, let it reported directly)*/
    }

    if( param->pack_len[param->store_id] + step_len > param->packet_size ) {
      if( param->store_id+1 >= param->packet_num ) {
        break;
      }
      /* point to next */
      param->store_id++;
      param->pack_len[param->store_id] = 0;
    }
    
    /* copy to current */
    {
      pos_u8_t *b;
      drv_data_time_t dt;
      b = &param->pack[param->store_id][param->pack_len[param->store_id]];

      /* Add timestamp */
      if( (drv->s->slot->rsvd32 & 1) == 0 ) {
        drv->call(DRV_CALL_FUNC_ID_GET_DATE_TIME, (pos_u32_t)&dt);
        *b++ = PLSS_PL_TIME;
        *b++ = dt.year % 100;
        *b++ = dt.month;
        *b++ = dt.day;
        *b++ = dt.hour;
        *b++ = dt.minute;
        *b++ = dt.second;
        param->pack_len[param->store_id] += step_len; 
      } else 
        param->pack_len[param->store_id] += step_len - 7; /* remove timestamp */
      
      /* Add payload */
      drv->os->memcpy(b, &plss_data[13], plss_data_len-13);
    };

  } while(0);  

  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    pos_u8_t i;
    drv->log->data("pack ok", step_len);
    drv->os->printf("    duty(%u) report(%u) buf(%ubx%u):", param->duty_cnt, param->report_id, param->packet_size, param->packet_num);
    for( i = 0; i < param->packet_num; i++ ) {
      drv->os->printf(" %s%u", i == param->store_id ? "*" : "", param->pack_len[i]);
    }
    drv->os->printf("\r\n");
  }

  /* flush data */
  drv->call(DRV_CALL_FUNC_ID_SET_DATA_CLEAR, 0);

  /* report when duty/threshold reached or table is full */
  if( param->duty_cnt >= drv->s->slot->arg.u16[0] || /* thold reached */
      ( 
        param->store_id + 1 >= param->packet_num && /* last entry */
        param->pack_len[param->store_id] + step_len > param->packet_size  /* AND entry can NOT contain one more duty data */
      )
  ) {
    /* report immediately */
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      drv->log->data("pack report start", 0);      
    }
    param->report_id = 0; /* kick start flag */
    sensor_pack_poll(param); /* report data */
  } else {
    /* give up current round of data */
    drv->s->ctrl |= MA_SENSOR_CTRL_ABANDON;
  }
  
  return ret;
}

/*
 * Driver export
 */
ma_drv_export_t g_export = {
  .version = DRV_SENSOR_VERSION(MA_SENSOR_PACK, DRV_PACK_VERSION),
  .name = DRV_PACK_NAME,
  .u.sensor={
    .init = sensor_pack_init,
    .collect = sensor_pack_collect,
    .poll_cb = (pos_func_t)sensor_pack_poll, 
  },
};


