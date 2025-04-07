/**
 * @file  drv_lora.c
 * @brief NET LoRa driver
 * @author Runby F.
 * @date 2022-3-10
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * 版本修订历史
v1.0 @ 2022-03-10
  1) 完成第一版驱动

v1.1 @ 2022-03-14
  1) 优化drv指针提高效率
  2) 打印初始化日志

v1.2 @ 2022-03-15
  1) 支持RT-Thread

v1.3 @ 2022-03-18
  1) 优化初始化时间, 缩短等待时间

v1.4 @ 2022-04-18
  1) 修复初始化参数配置异常
  2) 优化透传发送规避OS返回异常问题

v1.5 @ 2022-04-18
  1) 解决透传模式下调试打印较多时上报不完整问题
  2) 优化透传模式接收数据等待时间

v1.6 @ 2022-04-27
  1) 支持mtu初始化

v1.7 @ 2023-03-27
  1) Add INIT delay for JOIN so that the first TX won't be missed

v1.8 @ 2023-04-07
  1) Support lora/flags for unconfirm/confirm set

v1.9 @ 2023-04-21
  1) Fix lora/flags sytax mistake
  2) Remove lora/flags sync as DTU_LORA/FLAGS is only for debug purpose and different than 4.0/PPB lora flags

v1.10 @ 2023-08-26
  1) Use explict IO init uart ctrl field

v1.11 @ 2023-08-27
  1) Support RSSI retrieve

v1.12 @ 2023-10-19
  1) Support to hold lora init until register to resolve first report missing issue in lora

v1.13 @ 2023-12-14
  1) Support CONFIRM mode 

v1.14 @ 2023-12-26
  1) Support to use 120s to wait join at most during lora init
  2) Return 0% signal when not joined

v1.15 @ 2024-03-27
  1) Always reset lora module for each bootup init according to Rock's decision in wechat

v1.16 @ 2024-07-08
  1) Change to none-transparent mode to work-around the field issue causing by unepxected lora module reset

v1.17 @ 2024-12-06
  1) Support P2P send and receive
  2) Support any io as lora io

v1.18 @ 2025-02-07
  1) Support STM32 new LoRa module
  2) Support set loractrl 0x20 for state saving (ABP always enabled state saving)
  3) Support per hour state saving control for 10 years flash healthy

v1.19 @ 2025-03-06
  1) Support module calib

v1.20 @ 2025-03-31
  1) Support new NS1.0.4 and devnonce sync
  2) Support only state saving when lora_ctrl&0x20 for JOIN and 1H timer
  3) Support new PLSS WL55 module

*/

/**
 * @brief Driver version
 */
#define DRV_LORA_VERSION 0x0114

/**
 * @brief Driver name
 */
#define DRV_LORA_NAME  "LORA"

/**
 * @brief Use transparent mode
 */
#define LORA_TRANS_MODE_NOT_DEFINE

/**
 * @brief Wait time until JOIN during INIT/CFG in unit of ms
 */
#define LORA_SHORT_WAIT_UNTIL_JOIN 120000 /* 120s by default */

/**
 * @brief Dev Nonce record check&boundary
 */
#define LORA_DEV_NONCE_REC_BOUNDARY 16 /* record every 16 */

typedef struct {
  union {
    struct {
      pos_u32_t type:8; /* 0: unknown or old module, 1: new STM32 module */
      pos_u32_t join_init:1; /* 0: never perform, 1: performed join init */
    } s;
    pos_u32_t u32;
  } u;
  pos_u32_t   save_timeout; /* timeout ticks for state saving */
  pos_u16_t   dev_nonce; /* dev nonce tracing and sync */
  pos_u16_t   rsvd;
} drv_slora_ctrl_t;

/**
 * @brief LoRa STM32 Check join
 */
pos_status_t drv_s32_lora_is_join(drv_api_t *drv) {
  return drv->ext->cmd_query_check((pos_u8_t*)"AT+RSSI", 0, (pos_u8_t*)"+RSSI:1",0);
}

/**
* @brief LoRa module detect (return OK if STM32 LoRa Module
*/
pos_status_t drv_s32_lora_detect(drv_api_t *drv){
  pos_status_t ret = POS_STATUS_OK;
  pos_u32_t i;
  for( i = 0; i < 3; i++ ) {
    ret = drv->ext->cmd_query((pos_u8_t*)"AT", 1000);
    if( ret != POS_STATUS_OK )
      continue;
    ret = drv->ext->cmd_query_check((pos_u8_t*)"ATI", 0, (pos_u8_t*)"+VER:3", 1000);
    if( ret == POS_STATUS_OK )
      break;
  }
  return ret;
}

/**
* @brief LoRa STM32 module init
*/
pos_u32_t drv_s32_lora_band_trans(pos_u32_t v40_region) {
  /* 0:AS923, 1:AU915, 2:CN470, 3:CN779, 4:EU433, 5:EU868, 6:KR920, 7:IN865, 8:US915, 9:RU864 */
  const pos_u8_t s32_bands[] = {
    8, /* #0 US915 */
    2, /* #1 CN470 */
    5, /* #2 EU868 */
    3, /* #3 CN779 */
    4, /* #4 CN433 */
    2, /* #5 EU470: STM32 LORA does NOT support, re-use CN470 */
    0, /* #6 AS923 */
    9, /* #7 RU864 */
    1, /* #8 AU915 */
    1, /* #9 AU915#2: STM32 LORA does NOT support, re-use AU915 */
    6, /* #10 KR920: Only STM32 LORA can support */
    7, /* #11 IN865: Only STM32 LORA can support */
  };
  if( v40_region >= POS_ARRAY_CNT(s32_bands) )
    v40_region = 0;
  return s32_bands[v40_region];
}

/**
* @brief LoRa STM32 module command query
*/
pos_status_t drv_s32_lora_query(drv_api_t *drv, const char *cmd) {
  pos_status_t ret = POS_STATUS_OK;
  pos_u32_t i;
  for( i = 0; i < 3; i++ ) {
    ret = drv->ext->cmd_query((pos_u8_t*)cmd, 1000);
    if( ret == POS_STATUS_OK )
      break;
  }
  return ret;
}

/**
* @brief LoRa STM32 module command set for xx:xx:xx...:xx format
*/
pos_status_t drv_s32_lora_query_bytes(drv_api_t *drv, const char *cmd_field, const pos_u8_t *v, pos_u32_t v_len) {
  pos_u32_t i;
  char _buf[64], *c = _buf;
  drv->os->sprintf(c, "AT+%s=%02X", cmd_field, v[0]);
  for(i = 1; i < v_len; i++ ) {
    c += drv->os->strlen(c);
    drv->os->sprintf(c, ":%02X", v[i]);
  }
  return drv_s32_lora_query(drv, _buf);
}

/**
* @brief LoRa STM32 module command set for AT+XXX=NNN format
*/
pos_status_t drv_s32_lora_query_u32(drv_api_t *drv, const char *cmd_field, pos_u32_t v) {
  char _buf[64], *c = _buf;
  drv->os->sprintf(c, "AT+%s=%u", cmd_field, v);
  return drv_s32_lora_query(drv, c);
}

/**
* @brief LoRa STM32 module state save
*/
pos_status_t drv_s32_lora_state_save(drv_api_t *drv) {
  pos_status_t ret = POS_STATUS_OK;
  drv_slora_ctrl_t *sc = (drv_slora_ctrl_t*)drv->ext->rsvd32;
  pos_u32_t t;
  do {
    /* save when ContextSaving is defined */
    if( (drv->cfg->lora.lora_ctrl & 0x20) == 0 )
      break;

    /* AND save when already joined (join_init > 0) */
    if( sc->u.s.join_init == 0 )
      break;

    /* save only when timeout for flash healthy */
    t = drv->os->tick->get();
    if( !drv->os->tick->is_timeout(t, sc->save_timeout) )
      break;

    sc->save_timeout = t + 3600000; /* 1H could guarantee 10 years for 100K saving times */
    ret = drv_s32_lora_query(drv, "AT+CS");
  } while(0);
  return ret;
}

/**
* @brief LoRa STM32 module join init (only init once after the first join ready)
*/
pos_status_t drv_s32_lora_join_init(drv_api_t *drv) {
  pos_u32_t i;
  pos_status_t ret;
  char *c;

  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG )
    drv->log->data("slora jinit", (pos_u32_t)drv);

  /* Sync all configurations */
  c = (char*)drv->ext->buf;

  do {
    /* CLS */
    i = (drv->cfg->lora.lora_ctrl >> 6) & 3;
    i = i > 2 ? 2 : i;
    drv->os->sprintf(c, "AT+CLASS=%c", 'A'+i);
    ret = drv_s32_lora_query(drv, c);
    if( ret != POS_STATUS_OK )
      break;

    /* DR & ADR */
    i = drv->cfg->module_dr;
    if( i & 0x80 ) {
      /* Always Force ADR=0 before DR chanage */
      ret = drv_s32_lora_query_u32(drv, "ADR", 0);
      if( ret != POS_STATUS_OK )
        break;

      /* Change DR when ADR=0 */
      ret = drv_s32_lora_query_u32(drv, "DR", i&0x7f);
      if( ret != POS_STATUS_OK )
        break;

      /* turn ON ADR if necessary */
      if( drv->cfg->lora.lora_ctrl & 0x2 ) { 
        ret = drv_s32_lora_query_u32(drv, "ADR", 1);
        if( ret != POS_STATUS_OK )
          break;
      }
    } else {
      ret = drv_s32_lora_query_u32(drv, "ADR", (drv->cfg->lora.lora_ctrl >> 1)&0x1);
      if( ret != POS_STATUS_OK )
        break;
    }

  } while( 0 );

  if( ret != POS_STATUS_OK ) {
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG )
      drv->log->data("slora jinit err", ret);
    return POS_STATUS_E_INIT;
  }
  
  return POS_STATUS_OK;  
}

/**
* @brief LoRa STM32 module init
*/
pos_status_t drv_s32_lora_init(drv_api_t *drv) {
  pos_u32_t i, region;
  pos_status_t ret;
  char c[64];

  region = drv_s32_lora_band_trans(drv->cfg->report_type&0xff);

  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG )
    drv->log->data("slora init", region);

  drv->ext->mtu = 220; /* use small mtu */

  drv->ext->tx_rt[0] = 1; /* \r */

#if 0
  drv->os->io->init(drv->ext->module_io, MB_EXT_UART_BAUD, MB_EXT_UART_CTRL);  /* Enable UART after reset */
  drv_s32_lora_query(drv, "AT+CS"); /* store before reset */

  drv->os->io->done(drv->ext->module_io);  /* Disable UART before reset to avoid dummy information reading */
#endif

  /* reset */
  drv->ext->module_reset();

  drv->os->io->init(drv->ext->module_io, MB_EXT_UART_BAUD, MB_EXT_UART_CTRL);  /* Enable UART after reset */

  drv->ext->io_flush(); /* flush IO */

  /* Check&Sync all configurations */
  do {
    /* make sure AT ready */
    ret = drv_s32_lora_query(drv, "AT");
    if( ret != POS_STATUS_OK )
      break;

    /* Read CFG */
    ret = drv_s32_lora_query(drv, "AT+CFG");
    if( ret != POS_STATUS_OK )
      break;

    /* Compare CFG 
    * 72Bytes
    * 4: JOIN(00-none, 01-ABP, 02-OTAA) REGION 00 00
    * 8: EUI
    * 8: APP
    * 16: APP-KEY (NWK-KEY)
    * 16: APP-SKEY
    * 16: NWK-SKEY
    * 4: ADDR
    */
    if(drv->ext->rlen > 144 ) {
      pos_u8_t *r = &drv->ext->buf[256];
      pos_u8_t join;
      i = 72;
      drv->os->str2bin((char*)drv->ext->buf, r, &i,DECODE_HEXSTR);
      do {
        /* CFG wrong */
        if( i != 72 ) 
          break;

        /* Join Type Check */
        join = (drv->cfg->lora.lora_ctrl & 0x10) ? 1 : 2;
        if( r[0] && r[0] != join )
          break; /* mismatch */

        /* Region Check */
        if( r[1] != region )
          break; /* mismatch */

        if( join == 1 ) {
          /* ABP Check */
          if( drv->os->memcmp(drv->cfg->lora.lora_app_skey, &r[36], 36) != 0 ) {
            break; /* APP/NWK SKEY or ADDR mismatch */
          }
        } else {
          /* OTAA Check */
          if( drv->os->memcmp(drv->cfg->lora.lora_dev_eui, &r[4], 32) != 0 ) {
            break; /* EUI or KEY mismatch */
          }
        }
        if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG )
          drv->log->data("slora init identical", 0);

        /* still Break and Re-Init system when normal OTAA */
        if( (drv->cfg->lora.lora_ctrl & 0x30) == 0 )
          break;

        return POS_STATUS_OK;
      } while(0);
    }

    /* Clear before CFG provision */
    ret = drv_s32_lora_query(drv, "AT+RFS");
    if( ret != POS_STATUS_OK )
      break;

    /* force another reboot after RFS */
    drv_s32_lora_query(drv, "ATZ");

    /* Region (must be first setup, as it will init/restore APPKEY/EUI/... */
    ret = drv_s32_lora_query_u32(drv, "BAND", region);
    if( ret != POS_STATUS_OK )
      break;

    if( drv->cfg->lora.lora_ctrl & 0x10 ) {
      /* ABP */

      /* APP SKEY */
      ret = drv_s32_lora_query_bytes(drv, "APPSKEY", drv->cfg->lora.lora_app_skey, 16);
      if( ret != POS_STATUS_OK )
        break;

      /* NWK SKEY */
      ret = drv_s32_lora_query_bytes(drv, "NWKSKEY", drv->cfg->lora.lora_nwk_skey, 16);
      if( ret != POS_STATUS_OK )
        break;

      /* DEV ADDR */
      i = drv->cfg->lora.lora_dev_addr;
      drv->os->sprintf(c, "AT+DADDR=%02X:%02X:%02X:%02X",
        (i>>24)&0xff, (i>>16)&0xff, (i>>8)&0xff, (i)&0xff );
      ret = drv_s32_lora_query(drv, c);
      if( ret != POS_STATUS_OK )
        break;

    } else {
      /* OTA */

      /* DEV EUI */
      ret = drv_s32_lora_query_bytes(drv, "DEUI", drv->cfg->lora.lora_dev_eui, 8);
      if( ret != POS_STATUS_OK )
        break;

      /* APP EUI */
      ret = drv_s32_lora_query_bytes(drv, "APPEUI", drv->cfg->lora.lora_app_eui, 8);
      if( ret != POS_STATUS_OK )
        break;

      /* APP KEY */
      ret = drv_s32_lora_query_bytes(drv, "APPKEY", drv->cfg->lora.lora_app_key, 16);
      if( ret != POS_STATUS_OK )
        break;

      /* NWK KEY */
      ret = drv_s32_lora_query_bytes(drv, "NWKKEY", drv->cfg->lora.lora_app_key, 16); /* sync APP key to NWK KEY */
      if( ret != POS_STATUS_OK )
        break;

      /* DEV NONCE */
      ret = drv_s32_lora_query_u32(drv, "DEVNONCE", drv->cfg->lora.lora_dev_nonce);
      if( ret != POS_STATUS_OK )
        break;

    }

#if 0 /* not work, even after AT+CS, the reboot and join cntr is still zero */
    /* Store into Flash Finally */
    ret = drv_s32_lora_query(drv, "AT+CS");
#endif
  } while( 0 );

  if( ret != POS_STATUS_OK ) {
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG )
      drv->log->data("slora init err", ret);
    return POS_STATUS_E_INIT;
  }
  
  return POS_STATUS_OK;  
}

/**
* @brief LoRa STM32 modle link ready check
* @note Always return OK in transparent mode
*/
pos_status_t drv_s32_lora_net_ready(drv_api_t *drv, pos_u32_t timeout_ms) {
  pos_status_t ret = POS_STATUS_ERROR;
  pos_u32_t save = 0;
  drv_slora_ctrl_t *sc = (drv_slora_ctrl_t*)drv->ext->rsvd32;
  if( !timeout_ms )
    timeout_ms = 30000;
  timeout_ms = drv->os->tick->get() + timeout_ms;
  if( !sc->u.s.join_init && (drv->cfg->lora.lora_ctrl & 0x30) == 0 )  {
    /* when never joined and Join/Session state is not required to save (OTAA without state saving),
     * always ignore previous join status and use ret=ERROR for a Re-Join.
     * This will make sure when mote is joined as OTAA without state saving. A system reboot could rejoin as usually.
     */
    ;
  } else {
    /* when already joined or ABP/State saving is defined,
     * accept previous join staus 
     */
    ret = drv_s32_lora_is_join(drv);
  }
  while(ret != POS_STATUS_OK) {
    if( drv->os->tick->is_timeout(drv->os->tick->get(), timeout_ms) )
      break;
    /* OTA sync DevNonce */
    if( (drv->cfg->lora.lora_ctrl & 0x10) == 0 ) {
      if( (sc->dev_nonce & (LORA_DEV_NONCE_REC_BOUNDARY-1)) == 0 ) {
        pos_u16_t record;
        record = (sc->dev_nonce+LORA_DEV_NONCE_REC_BOUNDARY)&(~(LORA_DEV_NONCE_REC_BOUNDARY-1)); /* round to next boundary */
        drv->eeprom->update((pos_u32_t)&drv->cfg->lora.lora_dev_nonce - MA_EEPROM_ADDR,
          &record,
          2);
      }
      sc->dev_nonce++; /* advance to next */
    }
    drv_s32_lora_query_u32(drv, "JOIN", (drv->cfg->lora.lora_ctrl & 0x10) ? 0 : 1); /* AT+JOIN=1 for OTAA, 0 for ABP */
    drv->ext->cmd_query_check(POS_NULL, 0, (pos_u8_t*)"+EVT:JOIN",10000);
    ret = drv_s32_lora_is_join(drv);
    save = 1; /* when join counter changed sync to flash */
  }
  if( ret == POS_STATUS_OK ) {
    if( !sc->u.s.join_init ) {
      /* init join for the first time */
      drv_s32_lora_join_init(drv);
      drv_s32_lora_state_save(drv); /* save state after first join ready */
      sc->u.s.join_init = 1;
    }
  }

  /* Store to flash when JOIN counter changed */
  if( save ) {
#if 0    
    drv_s32_lora_query(drv, "AT+CS");
#endif
  }

  return ret;
}

/**
* @brief LoRa STM32 module RSSI retrieve
*/
pos_i32_t drv_s32_lora_rssi(drv_api_t *drv, pos_u8_t *rf_percentage) {
  ma_ext_ctrl_t *ext = drv->ext;
  pos_u32_t i;
  pos_i32_t rssi;  

  rssi = 0;
  do {
    /* query RSSI */
    if( ext->cmd_query_check((pos_u8_t*)"AT+RSSI", 0, (pos_u8_t*)"+RSSI:",0) == POS_STATUS_OK ) {
      if( ext->buf[6] == '0' ) /* +RSSI:0:XX:snr for 0-Not JOINED, always return 0 RSSI when not joined */
        break;
      if( ext->buf[8] == '-' || (ext->buf[8] >= '0' && ext->buf[8] <= '9') ) {
        drv->os->sscan( (char*)&ext->buf[8], "%4n,", &rssi );
      }
    }
  } while(0);

  if( rssi ) {
    if( rssi <= -140 )
      i = 1; /* at least 1% */
    else if( rssi >= -40 )
      i = 100;
    else /* rssi -41 .. -139 */
      i = rssi + 140;
  } else 
    i = 0;
  if( rf_percentage )
    *rf_percentage = i;
  return rssi;
}

/**
* @brief LoRa STM32 module sendto noneblocking mode
*/
pos_i32_t drv_s32_lora_sendto(drv_api_t *drv, pos_u8_t *buf, pos_size_t len) {
  char *c, i;
  for( i = 0; i < 3; i ++ ) {
    c = (char*)drv->ext->buf;
    /* LoRaWAN send */
    drv->os->sprintf( c, "AT+SEND=%u:%u:", drv->cfg->lora.lora_fport_tx, drv->cfg->lora.lora_flags & 1 );
    c += drv->os->strlen(c);
    drv->os->bin2str(c, buf, len, MA_EXT_HEX_CASE);
    if( drv->ext->cmd_query(drv->ext->buf, 1000) == POS_STATUS_OK ) {
      /* save state if necessary */
      drv_s32_lora_state_save(drv);
      break;
    }
    drv->os->tick->sleep(3000);
  }
  return len;
}

/**
* @brief LoRa STM32 module receive
*/
pos_i32_t drv_s32_lora_recvfrom(drv_api_t *drv, pos_u8_t *buf, pos_size_t max_len, pos_u32_t timeout_ms) {
  pos_i32_t len;
  pos_u32_t recv_len = 0;
  char *c;
  pos_size_t d_len, r;
  pos_u8_t fport;
  
/*
+EVT:RX_1, PORT 222, DR 0, RSSI -70, SNR 5
+RECV:2:DEFF
*/
  drv->ext->cmd_query_check(POS_NULL, 0, (pos_u8_t*)"+EVT:RX", timeout_ms);

  do {
    if( drv_s32_lora_query(drv, "AT+RECV") != POS_STATUS_OK ) 
      break;
    
    c = (char*)drv->ext->buf;
    len = drv->os->strlen(c);
#if 0
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      drv->log->buf("slora rx1", c, len);
    }
#endif
    if( len < 12 || c[5] != ':' ) 
      break;

    d_len =4;

    c = &c[6];
    c = drv->os->str2bin(c, (pos_u8_t*)&recv_len, &d_len, DECODE_UINT32);
#if 0
      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
        drv->log->data("slora recv_len", recv_len);
        drv->log->data("slora c#", c - (char*)drv->ext->buf);
      }
#endif
    /* no data or data invalid */
    if( d_len != 4 || !c || recv_len == 0 )
      break;    
    if( *c++ != ':' )
      break;

    r = 1;
    drv->os->str2bin(c, &fport, &r, DECODE_HEXSTR);
    if( r != 1 )
      break;

    /* valid fport got */

    /* check fport */
    if( fport != drv->cfg->lora.lora_fport_rx ) {
      /* use a simulated return */
      if( max_len ) {
        buf[0] = 0xfd; /* indicating fport wrong, but data is received */
        recv_len = 1;
      }
      break;
    }

    /* fport match */
    c += 2;
    recv_len -= 1; /* remove the fport byte */
    if( recv_len > max_len )
      recv_len = max_len;
    if( !buf ) 
      break;
    
    r = recv_len;
    drv->os->str2bin(c, buf, &r, DECODE_HEXSTR);
#if 0
          if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
            drv->log->buf("slora r", buf, r);
          }
#endif
    if( r != recv_len ) {
      recv_len = 0; /* wrong payload */
      break;
    }            
  }while(0);

  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    if( MA_LORA_IS_P2P(buf, recv_len) )
      drv->log->buf("slora rx p2p", buf, recv_len);      
    else
      drv->log->buf("slora rx", buf, recv_len);
  }
  return recv_len;
}

/**
 * @brief Check join
 */
pos_status_t drv_lora_is_join(drv_api_t *drv) {
  return drv->ext->cmd_query_check((pos_u8_t*)"AT+STATE", 0, (pos_u8_t*)"+STATE:1",0);
}

/**
 * @brief Sync LORA configuration item
 */
pos_status_t drv_lora_cfg_item(const char *fname, pos_size_t f_pos, pos_size_t f_len) {
  pos_u8_t *dat;
  char *c;
  drv_api_t *drv = g_drv;
  dat = (pos_u8_t*)drv->cfg;
  c = (char*)drv->ext->buf;
  drv->os->sprintf(c, "AT+CFG=%s:", fname);
  c += drv->os->strlen(c);
  if( f_len <= 4 ) {
    drv->os->sprintf(c, "%u", drv->os->util->get_u32(&dat[f_pos], f_len) );
  } else {
    drv->os->bin2str(c, &dat[f_pos], f_len, MA_EXT_HEX_CASE );
  }      
  return drv->ext->cmd_query(drv->ext->buf, 0); /* query cmd */
}

/**
 * @brief Sync all LORA configurations
 */
void drv_lora_cfg(drv_api_t *drv) {
  drv_lora_cfg_item("REGION",   MA_CFG_OFS_REPORT_TYPE,  1);
  drv_lora_cfg_item("MODEL",    MA_CFG_OFS_LORA_MODEL,  1);
  drv_lora_cfg_item("ACKLMT",   MA_CFG_OFS_LORA_ACKLMT,  1);
  drv_lora_cfg_item("ACKDLY",   MA_CFG_OFS_LORA_ACKDLY,  1);
  drv_lora_cfg_item("NBTRANS",  MA_CFG_OFS_LORA_NBTRANS,  1);
  drv_lora_cfg_item("CTRL",     MA_CFG_OFS_LORA_CTRL,  1);
  drv_lora_cfg_item("EUI",      MA_CFG_OFS_LORA_EUI,  8);
  drv_lora_cfg_item("APP",      MA_CFG_OFS_LORA_APP,  8);
  drv_lora_cfg_item("KEY",      MA_CFG_OFS_LORA_KEY, 16);
  drv_lora_cfg_item("NONCE",    MA_CFG_OFS_LORA_NONCE,  2);
  drv_lora_cfg_item("RFPWR",    MA_CFG_OFS_LORA_PWR,  1);
  drv_lora_cfg_item("CHGRP",    MA_CFG_OFS_LORA_GRP,  1);
  drv_lora_cfg_item("ADDR",     MA_CFG_OFS_LORA_ADDR,  4);
  drv_lora_cfg_item("AKEY",     MA_CFG_OFS_LORA_ASKEY, 16);
  drv_lora_cfg_item("NKEY",     MA_CFG_OFS_LORA_NSKEY, 16);
  drv_lora_cfg_item("TXPORT",   MA_CFG_OFS_LORA_TX,  1);
  drv_lora_cfg_item("RXPORT",   MA_CFG_OFS_LORA_RX,  1);
  drv_lora_cfg_item("CALIB",    MA_CFG_OFS_MCALIB,  1);
#if 0
  /* LoRa/FLAG is used for internal debug and not same as 4.0/PPB lora_flags */
  /* DO NOT SYNC from 4.0 to DTU_LORA/FLAG directly */
  drv_lora_cfg_item("FLAG",   MA_CFG_OFS_LORA_FLAGS,  1);  
#endif

#ifdef LORA_TRANS_MODE  
#if 1
  {
    pos_u32_t i;
    /* disable trans by default */
    if( drv->cfg->lora.lora_flags & 1 ) {    
      drv->ext->cmd_query((pos_u8_t*)"AT+CFG=TRANS:1", 0); /* enter trans+confirm mode */
    } else {
      drv->ext->cmd_query((pos_u8_t*)"AT+CFG=TRANS:0", 0);  /* always disable auto-trans mode */
    }
    if( drv->ext->cmd_query((pos_u8_t*)"AT+RESET", 0) != POS_STATUS_OK ) { /* 20240327 according to wechat
    Rock decides to accept Pengpeng's suggestion to always RESET the board during each bootup. Here,
    AT+RESET .... Always RESET
    AT+RESET=2 .... Reset only when CFG change
    */     
      /* board is reboot */
      drv->os->tick->sleep(5000); /* sleep 5s to wait */
     drv->ext->quit_trans_mode();       
    }

    /* check register status */
#ifdef LORA_SHORT_WAIT_UNTIL_JOIN
    /* wait forever */
    i = drv->os->tick->get() + LORA_SHORT_WAIT_UNTIL_JOIN;
    while( !drv->os->tick->is_timeout(drv->os->tick->get(), i) ) {
      /* query STATE */
      if( drv_lora_is_join(drv) == POS_STATUS_OK ) {
          break;
      }
      drv->os->tick->sleep(3000);
    }
#else
    /* wait forever */
    i = 0;
    while( 1 ) {
      i++;
      drv->os->tick->sleep( i < 150 ? 2000 : 30000 ); /* Within 150*2=300s, 2s interval; or else, 30s interval */
      
      /* prevent transparent */
      if( (i & 0xf) == 0 )
        drv->ext->quit_trans_mode(); 

      /* query STATE */
      if( drv_lora_is_join(drv) == POS_STATUS_OK ) {
        break;
      }
    }
#endif
    drv->ext->cmd_query((pos_u8_t*)"AT+TRANS", 1000); /* enter trans mode */
  }
#else
  drv->ext->cmd_query((drv->cfg->lora.lora_flags & 1) == 0 ? (pos_u8_t*)"AT+CFG=TRANS:2" /*unconfirm*/: (pos_u8_t*)"AT+CFG=TRANS:1"/*confirm*/, 1000);  /* trans unconfirm */
  if( drv->ext->cmd_query((pos_u8_t*)"AT+RESET=2", 1000) == POS_STATUS_OK ) { /* reboot if necessary */
    /* when OK returns, it means RESET does not take affect (no change in CFG) */
    /* then, we need to manually enter TRANS mode */
    drv->ext->cmd_query((pos_u8_t*)"AT+TRANS", 1000); /* force reboot to enter trans mode */
  } else {
    /* when RESET failed, it means trans already */
  }
  drv->os->tick->sleep(8200); /* 8.2s should be enough for first join */
#endif  
#else
  /* None Transparent mode init */
  drv->ext->cmd_query((pos_u8_t*)"AT+CFG=TRANS:0", 0);  /* always disable auto-trans mode */
  drv->ext->cmd_query((pos_u8_t*)"AT+RESET=2", 0); /* reboot if necessary */

#ifdef LORA_SHORT_WAIT_UNTIL_JOIN
  {
    pos_u32_t i;
    /* wait forever */
    i = drv->os->tick->get() + LORA_SHORT_WAIT_UNTIL_JOIN;
    while( !drv->os->tick->is_timeout(drv->os->tick->get(), i) ) {
      /* query STATE */
      if( drv_lora_is_join(drv) == POS_STATUS_OK ) {
          break;
      }
      drv->os->tick->sleep(3000);
    }
  }
#endif

#endif
}

/**
* @brief LoRa module init
*/
pos_status_t drv_lora_init(void) {
  drv_api_t *drv = g_drv;

  /* free if previously allocated */
  if( drv->ext->rsvd32 ) {
    drv->os->free((void*)drv->ext->rsvd32);
    drv->ext->rsvd32 = 0;
  }

  /* auto detect lora types */
  drv->ext->module_reset();
  drv->os->io->init(drv->ext->module_io, MB_EXT_UART_BAUD, MB_EXT_UART_CTRL);  /* Enable UART after reset */
  drv->ext->tx_rt[0] = 1; /* \r default for S32 before detect */  
  if( drv_s32_lora_detect(drv) == POS_STATUS_OK ) { 
    /* new STM32 lora module */
    drv_slora_ctrl_t *sc;
    sc = drv->os->malloc(sizeof(*sc));
    if( !sc ) {
      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG )
        drv->log->data("lora init mem err", sizeof(*sc));
      return POS_STATUS_E_MEM;
    }
    drv->os->memset(sc, 0, sizeof(*sc));
    sc->u.s.type = 1; /* set new lora flag */
    sc->dev_nonce = drv->cfg->lora.lora_dev_nonce; /* load dev nonce */
    drv->ext->rsvd32 = (pos_u32_t)sc;
    return drv_s32_lora_init(drv);
  } else 
    drv->ext->rsvd32 = 0; /* regular old lora module */

  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG )
    drv->log->data("lora init", (pos_u32_t)drv);

  drv->ext->mtu = 220; /* use small mtu */

  drv->ext->tx_rt[0] = 2; /* \r\n */

  drv->os->io->done(drv->ext->module_io);  /* Disable UART before reset to avoid dummy information reading */

  /* reset */
  drv->ext->module_reset();

  drv->os->io->init(drv->ext->module_io, MB_EXT_UART_BAUD, MB_EXT_UART_CTRL);  /* Enable UART after reset */

  drv->ext->io_flush(); /* flush IO */

  drv->ext->cmd_query_check(POS_NULL,0, (pos_u8_t*)"OK", 1000);
 
  do {
    pos_u32_t i;
    drv->ext->quit_trans_mode();

    for( i = 5; i > 0; i-- ) {
      if( drv->ext->cmd_query((pos_u8_t*)"AT", 1000) == POS_STATUS_OK )
        break;
      drv->os->tick->sleep(1000); /* check by 1+1=2s inverval */
    }

    if( !i )
      return POS_STATUS_E_INIT; /* Error return */

    drv->os->tick->sleep(10000); /* sleep 10s if not wait AT okay */
  } while(0);

  /* Sync all configurations */
  drv_lora_cfg(drv); 

  return POS_STATUS_OK;
}

/**
* @brief LoRa module running state set
*/
pos_status_t drv_lora_run_set(pos_run_level_t level) {
  /* LoRa does NOT need to do anything, as it's auto sleep by default and auto wakeup by serial access */
  return POS_STATUS_OK;
}

/**
* @brief LoRa modle link ready check
* @note Always return OK in transparent mode
*/
pos_status_t drv_lora_net_ready(pos_u32_t timeout_ms) {
#ifdef LORA_TRANS_MODE  
  return POS_STATUS_OK; /* trans mode no way to know ready, just return ok */
#else
  drv_api_t *drv = g_drv;
  pos_status_t ret;

  /* CALL S32 */
  if( drv->ext->rsvd32 )
    return drv_s32_lora_net_ready(drv, timeout_ms);

  if( !timeout_ms ) {
    /* net ready idle check */
    /* ext module does not require this, 
     * it typically is used for dhcp for ETH mode 
     */
    return POS_STATUS_OK; /* do nothing and return OK */
  }
  timeout_ms = drv->os->tick->get() + timeout_ms;
  while(1) {
    ret = drv_lora_is_join(drv);
    if( ret == POS_STATUS_OK )
      break;
    if( drv->os->tick->is_timeout(drv->os->tick->get(), timeout_ms) )
      break;
    drv->os->tick->sleep(3000);
  }
  return ret;
#endif
}

/**
* @brief LoRa module socket open (simulating)
* @note Always return OK
*/
pos_status_t drv_lora_sock_open(pos_u32_t *p_sock, pos_net_trans_t tcp, pos_u16_t port) {
  return POS_STATUS_OK;
}

/**
* @brief LoRa module socket close (simulating)
* @note Always return OK
*/
pos_status_t drv_lora_sock_close(pos_u32_t sock) {  
  return POS_STATUS_OK;
}

/**
* @brief LoRa module socket connect (simulating)
* @note Always return OK
*/
pos_status_t drv_lora_connect(pos_u32_t sn, pos_u8_t * addr, pos_u16_t port) {
  return POS_STATUS_OK;
}

/**
* @brief LoRa module sendto noneblocking mode
*/
pos_i32_t drv_lora_sendto(pos_u32_t sn, void * vbuf, pos_size_t len, pos_u8_t * addr, pos_u16_t port) {
  pos_u8_t *buf = (pos_u8_t*)vbuf;
  drv_api_t *drv = g_drv;
#ifdef LORA_TRANS_MODE  
  drv->os->task->lock();
  drv->ext->io->write(drv->ext->io, buf, len, 5000); 
  drv->os->task->unlock();  
  return len;
#else
  char *c, i;

  /* CALL S32 */
  if( drv->ext->rsvd32 )
    return drv_s32_lora_sendto(drv, buf, len);

  for( i = 0; i < 2; i++ ) {
    c = (char*)drv->ext->buf;
    if( MA_LORA_IS_P2P(buf,len) ) {
      /* P2P send does Not use fport */
      drv->os->sprintf( c, "AT+SEND=2,%u,", len );
    } else {
      /* LoRaWAN send */
      drv->os->sprintf( c, "AT+SEND=%u,%u,%02x", drv->cfg->lora.lora_flags & 1, len+1, drv->cfg->lora.lora_fport_tx );
    }
    c += drv->os->strlen(c);
    drv->os->bin2str(c, buf, len, MA_EXT_HEX_CASE);
    if( drv->ext->cmd_query(drv->ext->buf, 0) == POS_STATUS_OK )
      return len;
    if( i == 0 )
      drv->os->tick->sleep(3000);
  }
  return -1;
#endif
}

/**
* @brief LoRa module RSSI retrieve
*/
pos_i32_t drv_lora_rssi(drv_api_t *drv, pos_u8_t *rf_percentage) {
  ma_ext_ctrl_t *ext = drv->ext;
  pos_u32_t i;
  pos_i32_t rssi;
  
#ifdef LORA_TRANS_MODE  
  /* quit TRANSPARENT */
  ext->quit_trans_mode();
  rssi = 0;
  do {
    for( i = 0; i < 3; i++ ) {
      if( ext->cmd_query((pos_u8_t*)"AT",1000) == POS_STATUS_OK )
        break;
    }
    if( i >= 10 ) /* AT failed */
      break;

    /* query STATE */
    if( drv_lora_is_join(drv) != POS_STATUS_OK ) {
      return 0; /* return 0% indicating network not ready */
    }

    /* query RSSI */
    if( ext->cmd_query_check((pos_u8_t*)"AT+RSSI", 0, (pos_u8_t*)"+RSSI:",0) == POS_STATUS_OK ) {
      if( ext->buf[8] == '-' || (ext->buf[8] >= '0' && ext->buf[8] <= '9') ) {
        drv->os->sscan( (char*)&ext->buf[8], "%4n,", &rssi );
      }
    }    
  } while(0);
  
  /* re-enter TRANSPARENT */
  ext->cmd_query((pos_u8_t*)"AT+TRANS", 500);
  
  if( rssi ) {
    if( rssi <= -140 )
      i = 1; /* at least 1% */
    else if( rssi >= -40 )
      i = 100;
    else /* rssi -41 .. -139 */
      i = rssi + 140;
  } else 
    i = 0;
  if( rf_percentage )
    *rf_percentage = i;
  return rssi;
#else
  /* CALL S32 */
  if( drv->ext->rsvd32 )
    return drv_s32_lora_rssi(drv, rf_percentage);

  rssi = 0;
  do {
    /* query STATE */
    if( drv_lora_is_join(drv) != POS_STATUS_OK ) {
      return 0; /* return 0% indicating network not ready */
    }

    /* query RSSI */
    if( ext->cmd_query_check((pos_u8_t*)"AT+RSSI", 0, (pos_u8_t*)"+RSSI:",0) == POS_STATUS_OK ) {
      if( ext->buf[8] == '-' || (ext->buf[8] >= '0' && ext->buf[8] <= '9') ) {
        drv->os->sscan( (char*)&ext->buf[8], "%4n,", &rssi );
      }
    }
  } while(0);

  if( rssi ) {
    if( rssi <= -140 )
      i = 1; /* at least 1% */
    else if( rssi >= -40 )
      i = 100;
    else /* rssi -41 .. -139 */
      i = rssi + 140;
  } else 
    i = 0;
  if( rf_percentage )
    *rf_percentage = i;
  return rssi;
#endif
}

/**
* @brief LoRa module receive
*/
pos_i32_t drv_lora_recvfrom(pos_u32_t sn, void * vbuf, pos_size_t max_len, pos_u8_t * addr, pos_u16_t *port, pos_u32_t timeout_ms) {
  pos_u8_t *buf = (pos_u8_t*)vbuf;
  drv_api_t *drv = g_drv;  
#ifdef LORA_TRANS_MODE 
  pos_size_t r, s;

  /* read rssi */
  if( sn == 0xffff0001 ) {
    pos_i32_t rssi;
    rssi = drv_lora_rssi(drv, &drv->history->signal);
    if( vbuf && max_len && max_len <= 4 )
      drv->os->memcpy(vbuf, &rssi, max_len);
    return max_len;
  }
  
  s = 0;
  timeout_ms += drv->os->tick->get();
  do {
    r = drv->ext->io->read(drv->ext->io, buf, max_len, 100);
    if( !r && s ) /* When read but data is emtpy, indicating data is broken. It can quit here */
      break;
    s += r;    
  } while( !drv->os->tick->is_timeout(drv->os->tick->get(), timeout_ms));
  return s;
#else
  pos_i32_t len;
  pos_u32_t recv_len = 0;
  char *c;

  /* read rssi */
  if( sn == 0xffff0001 ) {
    pos_i32_t rssi;
    rssi = drv_lora_rssi(drv, &drv->history->signal);
    if( vbuf && max_len && max_len <= 4 )
      drv->os->memcpy(vbuf, &rssi, max_len);
    return max_len;
  }

  /* CALL S32 */
  if( drv->ext->rsvd32 )
    return drv_s32_lora_recvfrom(drv, buf, max_len, timeout_ms);

  drv->ext->cmd_query_check(POS_NULL, 0, (pos_u8_t*)"+MRECV", timeout_ms);

  if( drv->ext->cmd_query((pos_u8_t*)"AT+RECV", 0) == 0 ) {
    c = (char*)drv->ext->buf;
    len = drv->os->strlen(c);
#if 0
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      drv->log->buf("lora rx1", c, len);
    }
#endif
    if( len > 12 && c[7] == ':' && c[9] == ',' ) {
      pos_size_t d_len = 4;
      c = &c[10];
      c = drv->os->str2bin(c, (pos_u8_t*)&recv_len, &d_len, DECODE_UINT32);
#if 0
      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
        drv->log->data("lora recv_len", recv_len);
      }
#endif
      if( d_len == 4 && c && recv_len && *c++ == ',' ) {
        if( recv_len > max_len )
          recv_len = max_len;
        if( buf ) {
          pos_u32_t r = recv_len;
          drv->os->str2bin(c, buf, &r, DECODE_HEXSTR);
#if 0
          if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
            drv->log->buf("lora r", buf, r);
          }
#endif
          if( r != recv_len )
            recv_len = 0; /* wrong payload or fport is not fport_rx */
          else if( MA_LORA_IS_P2P(buf, recv_len) ) {
            /* p2p recv */
            /* use raw data */
          } else if( buf[0] != drv->cfg->lora.lora_fport_rx ) {
            /* lorawan unknown fport */
            recv_len = 0;
          } else if( recv_len ) {
            /* lorawan receive with correct fport */
            recv_len--; /* remove the first fport */
            drv->os->memcpy(&buf[0], &buf[1], recv_len); /* adjust the buffer */
          }
        }
      }        
    }
  }
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    if( MA_LORA_IS_P2P(buf, recv_len) )
      drv->log->buf("lora rx p2p", buf, recv_len);      
    else      
      drv->log->buf("lora rx", buf, recv_len);
  }
  return recv_len;
#endif
}

/*
 * Driver export
 */
ma_drv_export_t g_export = {
  .version = DRV_NET_LORA_VERSION(DRV_LORA_VERSION),
  .name = DRV_LORA_NAME,
  .u.net={
    drv_lora_init,
    drv_lora_run_set,
    drv_lora_net_ready,
    drv_lora_sock_open,
    drv_lora_sock_close,
    drv_lora_connect,
    drv_lora_sendto,
    drv_lora_recvfrom,
  },
};

