#ifndef __PLSS_MOTE_INCLUDE_H
#define __PLSS_MOTE_INCLUDE_H

#ifdef __cplusplus
 extern "C" {
#endif
                                              
/* Includes ------------------------------------------------------------------*/


#define LORAWAN_PLSS_MGMT_PORT              251
#define LORAWAN_PLSS_MGMT_PORT2             222
#define LORAWAN_PLSS_RFU_TXT                7       /* RFU: 0-normal, 7-Plss plain text format, 6-Plss Simple AES format */
#define LORAWAN_PLSS_RFU_SIMPLE_AES         6
#define BOARD_EEPROM_CFG_SIGNATURE_V0       0xF9A7  /* obsolete signature */
#define BOARD_EEPROM_CFG_SIGNATURE_V1       0xF9A8  /* PPB for LoRa motes */
#define BOARD_GLB_CFG_ADR_ACK_LIMIT(glb)     (((glb).model[1] >> 6) & 0x3)
#define BOARD_GLB_CFG_ADR_ACK_DELAY(glb)     (((glb).model[1] >> 4) & 0x3)
#define BOARD_GLB_CFG_IS_EXT_PWR_LO(glb)     (((glb).model[1] & 0x08) == 0)
#define BOARD_GLB_CFG_IS_CONFIRM_MSG(glb)    (((glb).model[1] & 0x04) != 0)
#define BOARD_GLB_CFG_IS_CNTR_16(glb)        (((glb).model[1] & 0x02) == 0)
#define BOARD_GLB_CFG_IS_RF_PWR_SAVING(glb)  (((glb).model[1] & 0x01) != 0)
#define BOARD_GLB_CFG_IS_TIME_SYNC(glb)      (((glb).cls_join & 0x20) != 0)
#define BOARD_GLB_CFG_IS_JOIN_OTA(glb)      (((glb).cls_join & 0x10) == 0)
#define BOARD_GLB_CFG_IS_PRB_READY(glb)     (((glb).cls_join & 0x08) != 0)
#define BOARD_GLB_CFG_IS_OTA_FIXED_NONCE(glb) (((glb).cls_join & 0x04) != 0)
#define BOARD_GLB_CFG_IS_OTA_NONCE_RETRY(glb) (((glb).cls_join & 0x14) == 0)
#define BOARD_GLB_CFG_IS_ADR_ON(glb)          (((glb).cls_join & 0x02) != 0)
#define BOARD_GLB_CFG_IS_LBT_ON(glb)          (((glb).cls_join & 0x01) != 0)

#define BOARD_SENSOR_CFG_IS_ENABLE(sensor)  (((sensor).ctrl&0x80) == 0x80) /* presence on (only check one bit) */
#define BOARD_SENSOR_CFG_IS_AVG_REPORT(sensor) (((sensor).ctrl&0x01) != 0) /* report average */
#define BOARD_SENSOR_CFG_IS_ADV_THOLD(sensor)  (((sensor).ctrl&0x02) != 0) /* adv thold control */
#define BOARD_SENSOR_CFG_IS_TIME_RANGE(sensor) (((sensor).ctrl&0x04) != 0) /* report only within time range */
#define BOARD_SENSOR_CFG_IS_OPT2(sensor)       (((sensor).ctrl&0x04) != 0) /* OPT2 re-use time range as no free ctrl bit */
#define BOARD_SENSOR_CFG_IS_PERIODIC_REPORT(sensor) (((sensor).ctrl&0x08) != 0) /* peridoic report even when thold check failed */
#define BOARD_SENSOR_CFG_IS_DELTA_REPORT(sensor) (((sensor).ctrl&0x10) != 0) /* delta report:
                                                                              data will report when 
                                                                                1) delta report (last!=current);
                                                                                OR
                                                                                2) thold report
                                                                                OR
                                                                                3) perioric report
                                                                              */
#define BOARD_SENSOR_CFG_DELAY_REPORT_MODE(sensor) ((sensor).ctrl&0x60)      /* delay report mode 
                                                                              * 0x00 - none / normal mdoe
                                                                              * 0x20 - only above thold/delta data will be delay report and report nothing in current round
                                                                              * 0x40 - only under thold/delta data will be delay report and report only above thold data in current round
                                                                              * 0x60 - both above and under thold/delta data will be delay report and report above thold data in current round
                                                                              */
#define BOARD_SENSOR_CFG_IS_DELAY_REPORT(sensor) (BOARD_SENSOR_CFG_DELAY_REPORT_MODE(sensor) != 0) /* delay report:
                                                                                data will report to buffer and when sensor (type=delay_report)
                                                                                is trigger, the payload from buffer will be reported to ns
                                                                                note: delay report and block report (smpl_num>1) are conflicting, each time
                                                                                      only one style could be activated.
                                                                                
                                                                                */
#define BOARD_SENSOR_CFG_IS_DELAY_REPORT_TRADITIONAL(sensor) (BOARD_SENSOR_CFG_DELAY_REPORT_MODE(sensor) == 0x20) 
#define BOARD_SENSOR_CFG_IS_DELAY_REPORT_UNDER_THOLD(sensor) (BOARD_SENSOR_CFG_DELAY_REPORT_MODE(sensor) >= 0x40) 
#define BOARD_SENSOR_CFG_IS_DELAY_REPORT_ABOVE_THOLD(sensor) (BOARD_SENSOR_CFG_DELAY_REPORT_MODE(sensor) != 0x40) 

#define BOARD_SENSOR_CFG_CLR_DELTA_REPORT(sensor) do { (sensor).ctrl &= ~0x10; } while(0) /* clear delta report */

#define BOARD_SENSOR_CFG_IS_ADV_DELTA_REPORT(sensor)  (((sensor).ctrl&0x12) == 0x10 && ((sensor).thld_hi == 0) && ((sensor).thld_lo != 0) && ((sensor).type != SENSOR_TYPE_INCLINE)) 
                                                                                /* adv delta report (excluding inline sensor, as inline use special delta checking criteria):
                                                                                 data will report when
                                                                                 1) delta report if
                                                                                    abs(last-current) > thold_lo
                                                                                 OR
                                                                                 2) periodic report
                                                                                */

#define BOARD_EEPROM_PRB_SIGNATURE_V0       0xE9B8

#define BOARD_EEPROM_UPG_SIGNATURE_V0       0xD9C8

/****************************************************************************
 * OBSOLTE BEGIN *************
 ****************************************************************************/
/* this is for upgrading public definitions shared between mote and ilora */
#define BSP_UPGRADE_BLK_SIZE    16384
#define BSP_UPGRADE_PAGE_SIZE   192
#define BSP_UPGRADE_PAGE_NUM    ((BSP_UPGRADE_BLK_SIZE+BSP_UPGRADE_PAGE_SIZE-1)/BSP_UPGRADE_PAGE_SIZE)
#define BSP_UPGRADE_BLK_NUM     8

typedef enum {
  BSP_BOARD_8800_V10      =0, /* 8800 V1.0 */
  BSP_BOARD_8800_V11      =1, /* 8800 V1.1 */
  BSP_BOARD_8700_V13      =2, /* 8700 V1.3 */
  BSP_BOARD_8800_V12      =3, /* 8800 V1.2 & V1.3 */ 
  BSP_BOARD_9800_V10      =4, /* 9800 V1.0 */ 
  BSP_BOARD_7800_V10      =5, /* 7800 V1.0 */ 
  BSP_BOARD_9810_V10      =6, /* 9810 V1.0 */ 
  BSP_BOARD_TYPE_NUM
} bsp_board_t;

typedef enum {
  BSP_MTYPE_UPG_REQ   =         0xf1,
  BSP_MTYPE_UPG_RES   =         0xf2,
} bsp_upgrade_mtype_t;  

typedef union {
  struct {
    uint8_t           id:7;
    uint8_t           more:1;
  } s;
  uint8_t             u8;
} bsp_upgrade_id_t;

typedef struct {
  uint8_t             mtype;
  bsp_upgrade_id_t    blk_id;
  bsp_upgrade_id_t    page_id;
  uint8_t             pl_len;
  uint16_t            fw_crc;
  uint16_t            nonce;
  uint8_t             pl[BSP_UPGRADE_PAGE_SIZE];
  uint16_t            rsvd;  
  uint16_t            crc_xor;
} __attribute__((packed)) bsp_upgrade_res_t;

typedef struct {
  uint8_t             mtype;
  bsp_upgrade_id_t    blk_id;
  uint8_t             fw_type:4;
  bsp_board_t         board:4;
  uint8_t             page_size;
  uint16_t            fw_crc;
  uint16_t            nonce;    
  uint8_t             eui[8];  
  uint8_t             page_bmp[16];
  uint16_t            rsvd;  
  uint16_t            crc_xor;
} __attribute__((packed)) bsp_upgrade_req_t;
/****************************************************************************
 * OBSOLTE END *************
 ****************************************************************************/

typedef struct {
  uint8_t           model[2];
  uint16_t          periodic_cycle;         /* periodic reprot cycle used when sensor periodic/report is enabled */
  uint8_t           tmr_granularity;        /* in unit of s */
  uint8_t           cls_join;  
}  __attribute__((packed)) board_glb_cfg_t;

typedef struct {
  uint8_t       dev_eui[8];
  uint8_t       app_eui[8];
  uint8_t       app_key[16];
  uint16_t      dev_nonce; /* used when BOARD_GLB_CFG_IS_OTA_FIXED_NONCE() is true */
  uint8_t       rsvd[2];
}  __attribute__((packed)) board_ota_cfg_t;

typedef struct {
  uint32_t      dev_addr;
  uint8_t       app_skey[16];
  uint8_t       nwk_skey[16];
}  __attribute__((packed)) board_pers_cfg_t;

typedef struct {
  uint8_t       dev_eui[8];
  char          ssid[14];
  char          password[14];
}  __attribute__((packed)) board_wifi_cfg_t;

typedef struct {
  uint8_t         type;
  uint8_t         ctrl;
  uint16_t        cycle;            /* final tmr is: cycle * tmr_granularity */
  uint16_t        sample_num;
  uint16_t        sample_ms;        
  uint16_t        thld_hi;          /* data threshold */
  uint16_t        thld_lo;
  union {
    int16_t         calib[3];         /* calibration data */
    uint16_t        ucalib[3];
  } u;
}  __attribute__((packed)) board_sensor_cfg_t;
/* alt mode for each sensor: 
   analog sensors (type 14, 16, 17, 18, 22, 23, 43, 44, 47, 48, 49, 50):
      0-ext/in, 1-ext/in2, 2-ext/in3, 3-ext/in4 
   co and ext/gas (type 12 and 24) sensors:
      0-adc#14(8800)/adc#1(8700), 1-ext/in2, 2-ext/in,
   mc (type 37 and 41) sensors:
      0-adc#11, 1-ext/in2, 2-ext/in,
   internal irq sensors (type 5):
      0-pir_in, 1-ext/in2, 2-ext/in,
   external irq sensors (type 19, 20, 21, 45, 51, 60):
      0-ext/in, 1-ext/in2, 2-ext/in3, 3-ext/in4
   gps sensors (type 9):
      0-uart1, 1-uart1, 2-uart2, 3-uart3
   uart sensors (type 9, 10, 42, 46, 54, 58, 61):
   uart sensors for 8700&8800v1.3 (485 available) boards:
      0-485, 1-232, 2-232 rsvd#2, 3-232 rsvd#3
   uart sensors for other boards:
      0-uart1, 1-uart1, 2-uart2, 3-uart3
   acc int sensors (type 25):
      0-auto, 1-x axis, 2-y axis, 3-z axis for threshold/interrupts
 */
#define BOARD_SENSOR_CFG_ALT_MODE(scfg)         (((scfg).u.calib[1]>>8)&0x3)
/* alt polarity for each irq sensor: 0-default, 1-pol#1(usually RISING), 2-pol#2(usually FALING), 3-pol#3(usually RISING+FALLING) */
/* alt polarity for each analog/input sensor: 0-nopull, 1-pulldown, 2-pullup, 3-nopull */
#define BOARD_SENSOR_CFG_ALT_POLARITY(scfg)     (((scfg).u.calib[1]>>10)&0x3)
/* alt power-delay for each sensor: 0-default, 1-500ms, 2-2s, 3-6s, 4-30s, 5-60s, 6-300s, 7-always on, 8-always off */
#define BOARD_SENSOR_CFG_ALT_PWR_DELAY(scfg)    (((scfg).u.calib[1]>>12)&0xf)
/* alt func for each sesnor: 
   irq sensors:
      0-default (trigger once), 
      1-multiple trigger
      All IRQ sensor could trigger another sensor type through ALT_IS_TRIGGER and ALT_TRIGGER_TYPE
   soil humdity (type 18):
      0~2: different algotithms
   analog sensors (type 14:ext/analog, 24:ext/gas, 37:mc/gas):
      0: report analog mv
      1~0xff: report PL_IND=ALT_FUNC(), VALUE=(mv<<ALT_CALC_POW2)/CALC_DIV 
   co sensor (type 12):
      0: VALUE=mv/1.8669
      1: VALUE=mv/3.6
   485/ttl sensors (type 61):
     0: report analog mv
     1~0xff: report PL_IND=ALT_FUNC(), VALUE=MODBUS_RTU@ADDR=HI,REG=LO of BOARD_SENSOR_CFG_ALT_REF_ADDR()
*/
#define BOARD_SENSOR_CFG_ALT_FUNC(scfg)         ((scfg).u.calib[1]&0xff)
/* sensor triggering setup (only for IRQ sensors, type 5, 19, 20, 21, 45, 51, 59, 60) */
#define BOARD_SENSOR_CFG_ALT_IS_TRIGGER(scfg)   ((scfg).u.calib[2]&0x80)
#define BOARD_SENSOR_CFG_ALT_TRIGGER_TYPE(scfg) ((scfg).u.calib[2]&0x7f)
/* sensor calc factors (only for analog sensors, type 14, 24, 37) */
#define BOARD_SENSOR_CFG_ALT_CALC_POW2(scfg)    (((scfg).u.calib[2]>>12)&0xf)
#define BOARD_SENSOR_CFG_ALT_CALC_DIV(scfg)     ((scfg).u.calib[2]&0xfff)
/* sensor reference reg addr (only for 485/ttl sensors, type 61) */
#define BOARD_SENSOR_CFG_ALT_REF_ADDR(scfg)    ((scfg).u.calib[2])
/* multi data sensor reporting bitmap (only for multi sensors, type 27) */
#define BOARD_SENSOR_CFG_ALT_MULTI_BMP(scfg)    ((scfg).u.calib[0])

/* sensor camera setup (type 58)
g_sensor_cfg.u.ucalib[0]&0x0f,      // calib#0.bit0:3 is resolution: 0-176x120, 1-320x240, 2-640x480, 3-1280x720
g_sensor_cfg.u.ucalib[2]&0xfff,     // calib#2.bit0:11 is stream rate: for example 800 for 800kbps
(g_sensor_cfg.u.ucalib[0]>>15)&0x1, // calib#0.bit15 is x flip: 0-default, 1-reverse flip
(g_sensor_cfg.u.ucalib[2]>>15)&0x1, // calib#2.bit15 is y flip: 0-default, 1-reverse flip
(g_sensor_cfg.u.ucalib[0]>>14)&0x1, // calib#0.bit14 is night filter: : 0-default, 1-night filter
(g_sensor_cfg.u.ucalib[0]>>8)&0x1f, // calib#0.bit8:12 is fps: for example 25 for 25fps
(g_sensor_cfg.u.ucalib[0]>>13)&0x1  // calib#0.bit13 is codec: 0-h264, 1-mjpeg
*/
#define BOARD_SENSOR_CFG_ALT_CAMERA_RES(scfg)   ((scfg).u.calib[0]&0x0f)
#define BOARD_SENSOR_CFG_ALT_CAMERA_SR(scfg)    ((scfg).u.calib[2]&0xfff)
#define BOARD_SENSOR_CFG_ALT_CAMERA_X(scfg)     (((scfg).u.calib[0]>>15)&0x1)
#define BOARD_SENSOR_CFG_ALT_CAMERA_Y(scfg)     (((scfg).u.calib[2]>>15)&0x1)
#define BOARD_SENSOR_CFG_ALT_CAMERA_N(scfg)     (((scfg).u.calib[0]>>14)&0x1)
#define BOARD_SENSOR_CFG_ALT_CAMERA_FPS(scfg)   (((scfg).u.calib[0]>>8)&0x1f)
#define BOARD_SENSOR_CFG_ALT_CAMERA_CODEC(scfg) (((scfg).u.calib[0]>>13)&0x1)


typedef struct {
  uint16_t            signature;  /* 2 bytes */
  board_glb_cfg_t     glb;        /* 6 bytes */
  union {
    board_ota_cfg_t   ota;    /* effective if LoRa OTA mode */
    board_pers_cfg_t  pers;   /* effective if LoRa None-OTA mode */
  } u;                            /* 36 bytes */
  board_sensor_cfg_t  sensor[8];  /* 144 bytes (18x8) */
  uint8_t             chan_grp;   /* 1 bytes, channel grp id range: bit0~3 - from grp id, bit4~7 to grp id */
  uint8_t             time_range[4]; /* 4 bytes, 1/0: hh/mm from, 3/2 hh/mm to */
  uint8_t             mpi_cfg;    /* 1 bytes, mpi cfg bitmap: 
                                  bit[0:1], bit[2:3], bit[4:5]: PULLUP, PULLDN, PWRPULL
                                  for each item: 0 - untouch  1 - analog  2 - output low  3 - output high 
                                  bit[6:7]: 0 - ext_in1, 1 - ext_in2, 2 - ext_in3, 3 - ext_in4
                                  */
  uint8_t             thld_lmt;    /* 1 bytes, mpi cfg bitmap: 
                                  bit[0:3]: 0 - unlimit cross report, 1~15 - limit 2~32768 times of cross thold report 
                                  bit[4:7]: 0 - never report under thold, 1~15 - limit 2~32768 times of under thold report */
  uint8_t             auto_reset:3; /* 0~5 - 2~7 N, 6 - 14 N for Reset duration, 7 - disable auto Reset function */
  uint8_t             reset_level:1; /* 0 - the abvoe N is 1 Day, 1 - the above N is 1 Hour */                                  
  uint8_t             simple_aes:1;/* 0 - normal lora/txt/json payload, 1 - txt with simple aes (only for region=0x11, ignored for other regions) */
  uint8_t             temp_unit:1;/* 0 - Celsius degree, 1 - Fahrenheit degree */
  uint8_t             strict_fmt:1;/* 0 - Loose format:{field:value}, 1 - Strict JSON format: {"field":value} */
  uint8_t             rsvd_flags:1;/* 1 bytes */
  uint8_t             rsvd[2];    /* 2 bytes */
  uint16_t            crc16;      /* 2 bytes */
}  __attribute__((packed)) board_eeprom_cfg_t; /* total 200 bytes */

#define BOARD_EEPROM_OFS(field) ((uint8_t*)(&((board_eeprom_cfg_t*)NULL)->field) - (uint8_t*)NULL )

typedef struct {
  uint8_t             tx_pwr_id;
  uint8_t             dr_id;
  uint8_t             exp_dr_id;
  uint8_t             rx_delay_1;
  uint8_t             rx_delay_2;
  uint8_t             channel_nb_rep;
  uint8_t             rx1_dr_offset;
  uint8_t             rx2_dr_id;
  uint32_t            rx2_frequency;
  uint16_t            channel_msk[6];
}  __attribute__((packed)) board_prb_mac_t; /* total 24 bytes */ 

typedef enum {
  REGION_FLAG_BIT_CHANNEL_VALID = 0x1,
  REGION_FLAG_BIT_DWELL_UP = 0x2,  
} region_flag_bmp_t;
typedef union {
  struct {
    uint32_t            freq_100hz:24;
    uint32_t            dr_id_from:4;
    uint32_t            dr_id_to:4;
  } s;
  uint32_t              val32;
} region_channel_t;
  
typedef struct {
  uint16_t            signature;  /* 2 bytes */
  uint8_t             is_joined;  /* 1 bytes */
  uint8_t             region:4;
  region_flag_bmp_t   region_flag_bmp:4; /* 1 bytes */
  uint32_t            up_cntr;    /* 4 bytes */
  uint32_t            dev_addr;   /* 4 bytes */
  uint32_t            net_id;     /* 4 bytes */
  uint8_t             app_skey[16]; /* 16 bytes */
  uint8_t             nwk_skey[16]; /* 16 bytes */
  board_prb_mac_t     mac_param; /* 24 bytes mac parameters */
  region_channel_t    region_channel[16]; /* 64 bytes */
  uint8_t             rsvd1:4;
  uint8_t             repeat_cmds_len:4;
  uint8_t             repeat_cmds[15]; /* repeat cmds+len: 16 bytes */
  uint16_t            region_di_channel_freq[16]; /* 32 bytes, in unit of 25000 Hz for specified downstream channel, 0 for default */
  uint8_t             max_pwr_db; /* 1 bytes, max tx pwr */
  uint8_t             rsvd2[13];  /* 13 bytes */
  uint16_t            crc16;     /* 2 bytes */
}  __attribute__((packed)) board_eeprom_prb_t; /* total 200 bytes */

typedef struct {
  uint32_t            version;
  uint32_t            sys_tick;
  uint32_t            loop_cntr;
} __attribute__((packed)) board_info_t; /* total 12 bytes */

typedef struct {
  uint16_t            send_err; /* mac layer error */
  uint16_t            recv_slot2;
  uint16_t            recv_mcast; 
  uint16_t            recv_err; /* mac layer error or no response from server */
  uint16_t            recv_crc_err; /* mic  checking error */
  uint16_t            recv_proto_err; /* protocol or not expected messages received */
  uint16_t            recv_pl_err; /* payload length or content error */
  uint32_t            alive_ticks; /* alive ticks(ms) counting from last */
  int16_t             rx_rssi;
  int8_t              rx_snr;
} __attribute__((packed)) board_info_ext_t; /* total 21 bytes */

typedef struct {
  uint8_t             crc8; /* app_bsp_crc8 including from type to dst_addr */
  uint8_t             type:4; /* 0 - main fw, others - reserved */
  uint8_t             force_upg:1; /* 0 - regular crc16 check and do nothing if already same, 1 - always upgrading even it's same crc16 */
  uint8_t             full_power:1; /* 0 - turn off all sensors during upgrading, 1 - keep all sensors as it is */
  uint8_t             rsvd:2;
  uint16_t            crc16; /* fw crc16, use HTONS(modbus rtu crc16) as format: crc16.hi,crc16,lo */
  uint32_t            version;
  uint32_t            len;
  uint32_t            dst_addr; 
} __attribute__((packed)) board_upg_req_t; /* total 16 bytes */

typedef struct {
  uint8_t             crc8; /* crc8 including from version to offset */
  uint8_t             rsvd; /* reserved */
  uint16_t            fw_crc16; /* the value from upg_req */
  uint32_t            version;
  uint32_t            offset;
} __attribute__((packed)) board_upg_cmd_t;

typedef struct {
  uint8_t             crc8; /* crc8 including from status to data */
  uint8_t             status; /* 0 - okay, 1 - failed */
  uint16_t            len;  /* length of data */
  uint32_t            offset; /* must be same as upg_cmd.offset */  
  uint8_t             data[0];
} __attribute__((packed)) board_upg_rsp_t;

typedef struct {
  uint8_t             delay_cnt:3; /* delay_ms = delay_cnt * unit + 500 */
  uint8_t             delay_unit:1; /* 0 - unit=500ms, 1 - unit-5000ms */
  uint8_t             baudrate_id:3; /* 0-9600, 1-19200, 2-38400, 3-57600, 4-115200, 5~7-rsvd */
  uint8_t             module:1; /* 0-485, 1-232 */
  uint8_t             len;  /* length of TX data (can be zero and nothing will be sent)*/  
  uint8_t             data[0]; /* TX data */
} __attribute__((packed)) board_rtu_get_t;

typedef enum {
  PLSS_MGMT_NONE = 0,   /* no command, ignored */
  PLSS_MGMT_SAVE_PPB  = 1, /* save PPB with the given data buffer and reboot immediately */
  PLSS_MGMT_SYS_REBOOT  = 2, /* reboot */
  PLSS_MGMT_LOAD_PPB = 3, /* load PPB from EEPROM in next cycle and report to server side */
  PLSS_MGMT_RESTORE_PPB = 4, /* restore PPB to default and reboot */
  PLSS_MGMT_LOAD_PRB = 5, /* load PRB from EEPROM in next cycle and report to server side */
  PLSS_MGMT_SAVE_PRB  = 6, /* save PRB with the given data buffer */
  PLSS_MGMT_RECORD_PRB  = 7, /* record current running status into PRB */
  PLSS_MGMT_REPORT_INFO = 8, /* Report MGMT Info */
  PLSS_MGMT_RECORD_LOAD_PRB = 9, /* Record current running status into PRB and load it from EEPROM in next cycle */
  PLSS_MGMT_UPDATE_MIN = 10, /* minmal command for update series */
    PLSS_MGMT_UPDATE_PPB_GLB = PLSS_MGMT_UPDATE_MIN, /* Update PPB global configurations: Model, TstCycle, Granu, ClsJoin */
    PLSS_MGMT_UPDATE_PPB_SENSOR = PLSS_MGMT_UPDATE_MIN+1, /* Update (or add) PPB sensor according to sensor type */
    PLSS_MGMT_UPDATE_PPB_SENSOR_CTRL = PLSS_MGMT_UPDATE_MIN+2, 
    PLSS_MGMT_UPDATE_PPB_SENSOR_CYCLE = PLSS_MGMT_UPDATE_MIN+3,
    PLSS_MGMT_UPDATE_PPB_SENSOR_SMPL_NUM = PLSS_MGMT_UPDATE_MIN+4,
    PLSS_MGMT_UPDATE_PPB_SENSOR_SMPL_MS = PLSS_MGMT_UPDATE_MIN+5,
    PLSS_MGMT_UPDATE_PPB_SENSOR_THLD_HI = PLSS_MGMT_UPDATE_MIN+6,
    PLSS_MGMT_UPDATE_PPB_SENSOR_THLD_LO = PLSS_MGMT_UPDATE_MIN+7,
    PLSS_MGMT_UPDATE_PPB_SENSOR_CALIB = PLSS_MGMT_UPDATE_MIN+8,
    PLSS_MGMT_UPDATE_PPB_OFS = PLSS_MGMT_UPDATE_MIN+9, /* update PPB with offset/length */        
    PLSS_MGMT_UPDATE_PRB_OFS = PLSS_MGMT_UPDATE_MIN+10, /* update PRB with offset/length */    
    PLSS_MGMT_UPDATE_CLOCK = PLSS_MGMT_UPDATE_MIN+11,
    PLSS_MGMT_UPDATE_BLK_PAGE = PLSS_MGMT_UPDATE_MIN+12, /* 22 */
  PLSS_MGMT_UPDATE_MAX = PLSS_MGMT_UPDATE_BLK_PAGE, 
  PLSS_MGMT_SET_EXT_CNTR = PLSS_MGMT_UPDATE_MAX+3, /* 25 */
  PLSS_MGMT_GET_PPB_OFS, /* 26 */
  PLSS_MGMT_UPGRADE_FW, /* 27 firmware upgrade request with board_upg_req_t format */
  PLSS_MGMT_UPDATE_VIDEO_OFS, /* 28 update video ofs(32), if OFS(32) & 0x80000000, re-start ofs send from OFS(32)&0x7fffffff; or else, indicating no frame loss */
  PLSS_MGMT_GET_TYPE_OFS, /* 29, follow by: U8-Type(0-EEPROM, 1-FLASH1, 2-FLASH2, 3-RAM), U16-OFS, U8-Length */
  PLSS_MGMT_SET_TYPE_OFS, /* 30, follow by: U8-Type(0-EEPROM, 1-FLASH1, 2-FLASH2, 3-RAM), U16-OFS, U8-Length, ....length of data .... */
  PLSS_MGMT_UPDATE_PPB_SENSOR_CALIB_0, /* 31, update sensor calib#0 according to sensor type */
  PLSS_MGMT_UPDATE_PPB_SENSOR_CALIB_1, /* 32, update sensor calib#1 according to sensor type */
  PLSS_MGMT_UPDATE_PPB_SENSOR_CALIB_2, /* 33, update sensor calib#2 according to sensor type */  
  PLSS_MGMT_LOAD_PPB_INFO, /* 34, report PPB in next cycle and report MGMT Info in the second next cycle */
  PLSS_MGMT_UPGRADE_FW_RSP, /* 35 firmware upgrade response with board_upg_rsp_t format */
  PLSS_MGMT_RTU_GET, /* 36 generic RTU get (will response PLSS_PL_RTU_PAYLOD) */
  PLSS_MGMT_REPORT_ESIGNAL, /* 37 report extended signal */
} plss_mgmt_cmd_t;

typedef enum {
  SENSOR_TYPE_BATTERY = 0,
  SENSOR_TYPE_TEMPERATURE = 1, 
  SENSOR_TYPE_HUMIDITY = 2,
  SENSOR_TYPE_HUMIDITY_TEMPERATURE = 3,
  SENSOR_TYPE_SOUND = 4,
  SENSOR_TYPE_PIR = 5,
  SENSOR_TYPE_LIGHT = 6,
  SENSOR_TYPE_INCLINE = 7,
  SENSOR_TYPE_ACCELERO = 8,
  SENSOR_TYPE_GPS = 9,  
  SENSOR_TYPE_ULTRASONIC = 10,
  SENSOR_TYPE_PRESSURE = 11,
  SENSOR_TYPE_CO = 12,
  SENSOR_TYPE_ACCELERO_2D = 13,
  SENSOR_TYPE_EXT_ANALOG = 14,
  SENSOR_TYPE_EXT_DIGITAL = 15,
  SENSOR_TYPE_EXT_TEMPERATURE = 16,  
  SENSOR_TYPE_DISPLACEMENT = 17,
  SENSOR_TYPE_SOIL_HUMIDITY = 18, 
  SENSOR_TYPE_EXT_PIR = 19,
  SENSOR_TYPE_EXT_CNTR = 20,
  SENSOR_TYPE_EXT_SWITCH = 21,
  SENSOR_TYPE_EXT_ANALOG_TEMP = 22,
  SENSOR_TYPE_EXT_NH3 = 23,
  SENSOR_TYPE_GAS_ANALOG = 24,
  SENSOR_TYPE_ACCELERO_INT = 25, 
  SENSOR_TYPE_ACCELERO_INC = 26,
  SENSOR_TYPE_MULTI_SENSOR = 27,
  SENSOR_TYPE_SGA400 = 28, 
  SENSOR_TYPE_RSVD_29 = 29,
  SENSOR_TYPE_RSVD_30 = 30,
  SENSOR_TYPE_RSVD_31 = 31, 
  SENSOR_TYPE_RSVD_32 = 32,
  SENSOR_TYPE_RSVD_33 = 33,
  SENSOR_TYPE_RSVD_34 = 34, 
  SENSOR_TYPE_RSVD_35 = 35,
  SENSOR_TYPE_RSVD_36 = 36,
  SENSOR_TYPE_MC_ANALOG = 37,
  SENSOR_TYPE_RSVD_38 = 38, 
  SENSOR_TYPE_RSVD_39 = 39,
  SENSOR_TYPE_DELAY_REPORT = 40,
  SENSOR_TYPE_MC_CO2 = 41,
  SENSOR_TYPE_GAS_CO2_PPM = 42,
  SENSOR_TYPE_GAS_MAX = SENSOR_TYPE_GAS_CO2_PPM,
  SENSOR_TYPE_EXT_CURRENT = 43,
  SENSOR_TYPE_EXT_PSI = 44,
  SENSOR_TYPE_EXT_CNTR_RAW = 45,
  SENSOR_TYPE_EXT_PM = 46,
  SENSOR_TYPE_EXT_CURRENT_AVG = 47, 
  SENSOR_TYPE_EXT_UV = 48,
  SENSOR_TYPE_EXT_WIND_DIR = 49, 
  SENSOR_TYPE_EXT_WIND_SPEED = 50,   
  SENSOR_TYPE_EXT_RAINFALL = 51,   
  SENSOR_TYPE_VIBRATION_2 = 52,   
  SENSOR_TYPE_VIBRATION_3 = 53,   
  SENSOR_TYPE_SOIL_HUMIDITY2 = 54, 
  SENSOR_TYPE_MOTION_HZ = 55,
  SENSOR_TYPE_MOTION_V = 56,
  SENSOR_TYPE_MOTION_D = 57,
  SENSOR_TYPE_CAMERA = 58,
  SENSOR_TYPE_EXT_CNTR_12 = 59,
  SENSOR_TYPE_EXT_LEAKAGE = 60,
  SENSOR_TYPE_EXT_485_TTL = 61,
  SENSOR_TYPE_EXT_VOC = 62, /* this sensor is obsoleted by Celiar on 20200910, use sensortype 27, calib2=0x000c */
  SENSOR_TYPE_EXT_PH = 63, 
  
  SENSOR_TYPE_MAX,
} sensor_type_t;


/* Definies the PayLoad format */
#define PLSS_PL_NONE                    0 /* none paylaod, should be ignored */
#define PLSS_PL_TEMPERATURE             1 /* int16 in unit of 0.1degree */
#define PLSS_PL_HUMIDITY                2 /* uint8 in unit of 1% */
#define PLSS_PL_ACCELERATION            3 /* int16[3] x,y,z in unit of 1mg */
#define PLSS_PL_INCLINE                 4 /* int16[3] x,y,r in unit of 0.01degree */
#define PLSS_PL_ACCELERATION_2D         5 /* int16[2] x,y in unit of 0.1mg */
#define PLSS_PL_LIGHT                   6 /* uint16 in unit of lumen */
#define PLSS_PL_BATTERY                 7 /* uint16 in unit of mv */
#define PLSS_PL_SOUND                   8 /* uint16 in unit of 0.1db */
#define PLSS_PL_GPS                     9 /* uint32[2] lati, long in unit of 0.001 second WGS84  */
#define PLSS_PL_CO                      10 /* uint16 in unit of 1ppb*/
#define PLSS_PL_EXT_ANALOG              11 /* uint16 in unit of mv */
#define PLSS_PL_EXT_DIGITAL             12 /* uint8  */
#define PLSS_PL_PRESSURE                13 /* int32 in unit of pa */
#define PLSS_PL_EXT_TEMPERATURE         14 /* int16 in unit of 0.1degree */
#define PLSS_PL_DISP                    15 /* int16 in unit of 0.01mm displacement */
#define PLSS_PL_PIR                     16 /* uint8: 0 -none, 1 -detected */
#define PLSS_PL_SOIL_HUMIDITY           17 /* uint8: in unit of 1% */
#define PLSS_PL_EXT_PIR                 18 /* uint8: 0 -none, 1 -detected */
#define PLSS_PL_EXT_CNTR                19 /* uint32: external counter */
#define PLSS_PL_EXT_SWITCH              20 /* uint8: Bit7-1 for open, 0 for normal, Bit0~6 for open cntr */
#define PLSS_PL_EXT_NH3                 21 /* uint16 in unit of ppb  */
#define PLSS_PL_EXT_ASH3                22 /* uint16 in unit of ppb  */
#define PLSS_PL_EXT_C6H6                23 /* uint16 in unit of ppb  */
#define PLSS_PL_EXT_CL2                 24 /* uint16 in unit of ppb  */
#define PLSS_PL_EXT_H2                  25 /* uint16 in unit of lel%  */
#define PLSS_PL_EXT_H2S                 26 /* uint16 in unit of ppb  */
#define PLSS_PL_EXT_HCL                 27 /* uint16 in unit of ppm  */
#define PLSS_PL_EXT_HCN                 28 /* uint16 in unit of ppb  */
#define PLSS_PL_EXT_HF                  29 /* uint16 in unit of ppb  */
#define PLSS_PL_EXT_NO2                 30 /* uint16 in unit of ug/m3  */
#define PLSS_PL_EXT_O3                  31 /* uint16 in unit of ppb  */
#define PLSS_PL_EXT_PH3                 32 /* uint16 in unit of ppb  */
#define PLSS_PL_EXT_SO2                 33 /* uint16 in unit of ug/m3  */
#define PLSS_PL_EXT_CH4                 34 /* uint16 in unit of ppm (changed from PPB to PPM on 20231012)  */
#define PLSS_PL_EXT_C2H2                35 /* uint16 in unit of ppb  */
#define PLSS_PL_EXT_GASOLINE            36 /* uint16 in unit of ppm  */
#define PLSS_PL_EXT_C2H4O3              37 /* uint16 in unit of ppb  */
#define PLSS_PL_US_DISTANCE             38 /* uint16 in unit of mm, ultrasonic distance */
#define PLSS_PL_EXT_CO2                 39 /* uint16 in unit of 0.1% */
#define PLSS_PL_EXT_CO2_PPM             40 /* uint16 in unit of ppm */
#define PLSS_PL_EXT_CURRENT             41 /* uint16 in unit of 10ma */
#define PLSS_PL_EXT_PSI                 42 /* uint16 in unit of kpa, for Water Pressure (20230717 wechat change from psi to kpa)*/
#define PLSS_PL_EXT_PM                  43 /* uint16[3] in unit of ug/m3 for pm2.5/10/1.0 */
#define PLSS_PL_POS_BMP8                44 /* uint8 of multiple sensor position bitmap */
#define PLSS_PL_POS_BMP16               45 /* uint16 of multiple sensor position bitmap */
#define PLSS_PL_EXT_CURRENT_AVG         46 /* uint16[3] of 10ma, for avg/max/min */
#define PLSS_PL_UV_INDEX                47 /* uint8 of uv index 0~11 */
#define PLSS_PL_WIND_DIR                48 /* uint8 of wind dir 0: north, 1:nen, 2:ne, 3:nee, 4:east, ...., 12: west, 13: nww, 14: nw, 15: nwn */
#define PLSS_PL_WIND_SPEED              49 /* uint16 of mm/s for wind speed */
#define PLSS_PL_RAINFALL                50 /* uint16: of 0.1mm for rainfall  */
#define PLSS_PL_VIBRATION               51 /* uint8: of vibration detection (b[0]: vib. flag, b[1~2]:0 for 2 1 for 3 axis, b[3]: change flag */
#define PLSS_PL_MOTION_HZ               52 /* uint16[3]: of Hz x/y/z */
#define PLSS_PL_MOTION_V                53 /* int16[3]: of 0.1mm/s Velocity x/y/z*/
#define PLSS_PL_MOTION_D                54 /* int16[3]: of 0.1mm Displacement x/y/z */
#define PLSS_PL_EXT_CNTR_12             55 /* uint16[2]: 1->2, 2->1 interrupts counter */
#define PLSS_PL_EXT_LEAKAGE             56 /* uint8: Bit7-1 for leakage, 0 for normal, Bit0~6 for leakage cntr */
#define PLSS_PL_EXT_CH2O                57 /* uint16 in unit of ug/m3  gas-ch2o*/
#define PLSS_PL_EXT_CURRENT_UA          58 /* uint16 of 0.1ma */
#define PLSS_PL_EXT_VOC                 59 /* uint16 of ug/m3 */
#define PLSS_PL_EXT_PH                  60 /* uint8 of 0.1PH (0~160 for PH0.0~16.0) */
#define PLSS_PL_EXT_O2                  61 /* uint16 of 0.1%VOL */
#define PLSS_PL_TIME                    62 /* uint8[6] of YYMMDDHHMMSS for example 0d091d113005 for 2013-9-29 17:48:05*/
#define PLSS_PL_EM_VAC                  63 /* float32 of voltage ac */
#define PLSS_PL_EM_AAC                  64 /* float32 of ampere ac */
#define PLSS_PL_EM_W                    65 /* float32 of w for ac active power */
#define PLSS_PL_EM_PF                   66 /* float32 of ac power factor */
#define PLSS_PL_EM_HZ                   67 /* float32 of hz for ac power */
#define PLSS_PL_EM_KWH                  68 /* float32 of kwh for ac active power */
#define PLSS_PL_EM_VAC_ABC              69 /* UINT16[3] of voltage in unit 0.1v ac */
#define PLSS_PL_EM_AAC_ABC              70 /* UINT16[3] of ampere in unit 0.01a ac */
#define PLSS_PL_EM_AW_SABC              71 /* UINT16[4] of active watt (w) for ac active power (Sum/A/B/C)*/
#define PLSS_PL_EM_RW_SABC              72 /* UINT16[4] of reacive watt (w) for ac reactive power (Sum/A/B/C) */
#define PLSS_PL_EM_PF_ABC               73 /* UINT16[3] of ac power factor in unit 0.0001 (A/B/C) */
#define PLSS_PL_EM_RKWH                 74 /* float32 of kwh for ac reactive power */
#define PLSS_PL_EXT_SIGNAL              75 /* int32[5] of extended NB/RF signal: rsrp/sinr/pci/ecl/cell_id */
#define PLSS_PL_EXT_CO                  76 /* uint16 of any sensor unit for CO */
#define PLSS_PL_RSSI_SNR                77 /* int16[2] rssi,snr */
#define PLSS_PL_DELTA_01                78 /* int16[3] of any delta value in unit of 0.01 */
#define PLSS_PL_UV_PWR                  79 /* uint16 of uw/cm2 for UV power */
#define PLSS_PL_HUMIDITY_HR             80 /* uint16 in unit of 0.1% high resoltion humidity */
#define PLSS_PL_DO                      81 /* uint16 of 0.01%VOL for Dissolved Oxygen */
#define PLSS_PL_NTU                     82 /* uint16 of 0.1NTU */
#define PLSS_PL_EC_F                    83 /* float32 of 1mS/CM (or 0.1S/M) Electrical Conductivity */
#define PLSS_PL_EXT_CH2O_PPB            84 /* uint16 in unit of ppb  gas-ch2o*/
#define PLSS_PL_DO_MG_F                 85 /* float32 of 1mg/L for Dissolved Oxygen */
#define PLSS_PL_CHL_F                   86 /* float32 of 1ug/L for CHL */
#define PLSS_PL_BGA_F                   87 /* float32 of 1ug/L for BGA */
#define PLSS_PL_COD_F                   88 /* float32 of COD */
#define PLSS_PL_TOC_F                   89 /* float32 of TOC */
#define PLSS_PL_NH3_N_F                 90 /* float32 of 1mg/L for NH3_N */
#define PLSS_PL_K_PLUS_F                91 /* float32 of 1mg/L for K+ */
#define PLSS_PL_NH4_F                   92 /* float32 of 1mg/L for NH4 */
#define PLSS_PL_POS_BMP64               93 /* uint64 of multiple sensor position bitmap */
#define PLSS_PL_RADIATION               94 /* uint16 of 1w/m2 for radiation */
#define PLSS_PL_EXT_C2H4                95 /* uint16 in unit of ppb  */
#define PLSS_PL_EM_HZ_ABC               96 /* UINT16[3] of hz for ac power (A/B/C) in unit 0.01 */
#define PLSS_PL_EM_APW_SABC             97 /* UINT16[4] of apparent watt (w) for ac apparent power (Sum/A/B/C) */
#define PLSS_PL_NTU_F                   98 /* float32 of NTU */
#define PLSS_PL_SALINITY_F              99 /* float32 of PPT salinity */
#define PLSS_PL_PH_F                    100 /* float32 of PH */
#define PLSS_PL_EXT_PH2                 101 /* uint16 of 0.01PH */
#define PLSS_PL_EXT_SALINITY            102 /* uint16 of 1mg/L salinity */
#define PLSS_PL_EXT_EC                  103 /* uint16 of 1us/cm EC */
#define PLSS_PL_EXT_N                   104 /* uint16 of N index (soil N/P/K) */
#define PLSS_PL_EXT_P                   105 /* uint16 of P index (soil N/P/K) */
#define PLSS_PL_EXT_K                   106 /* uint16 of K index (soil N/P/K) */

/* folllowing are supported in 4.0 only */
#define PLSS_PL40_VOC_PPB               107 /* uint16 of VOC in unit of PPB */
#define PLSS_PL40_EXT_IO                108 /* uint8: Bit7-1 for IO state, 0 for normal, Bit0~6 for state change cntr */
#define PLSS_PL40_OHM                   109 /* uint32 of Ohm in unit of 0.001 ohm */
#define PLSS_PL40_VELOCITY_FLOW         110 /* uint32 of velocity of flow in unit of 0.001 m3/hour */
#define PLSS_PL_PERCENTAGE              111 /* uint8 of percentage in unit of 1% for VBAT and others */
#define PLSS_PL40_VELOCITY_VOLUME       112 /* uint32 of velocity volume in unit of 0.001 m3 */
#define PLSS_PL40_VELOCITY_VOLUME_R     113 /* uint32 of reversed velocity volume in unit of 0.001 m3 */
#define PLSS_PL_MULTI1                  114 /* uint16[2] of multi unit data:
                                               uint16[0] = 0xTTAB, TT: Type of data, A: Unit, B: Precision
                                               uint16[1]: Data value
                                               */
#define PLSS_PL40_VELOCITY_MFLOW        115 /* uint32 of velocity of flow in unit of 0.001 L/min */
#define PLSS_PL_REGULAR_END             116 /* the end flag for regular pl indicator */

/* reserved for future expansion */
#define PLSS_PL_RTU_PAYLOAD             120 /* len8+data0+data1+...data[len8-1]
                                            * raw 485/uart payload returned for PLSS_MGMT_RTU_GET */

#define PLSS_PL_INDICATOR               0xd7 /* PLSS Payload Indicator */
#define PLSS_PL_IND_V0                  0x7e /* V0 normal sending data format */
#define PLSS_PL_IND_V1                  0x7f /* V1 block sending data format for 3d accelero */
#define PLSS_PL_IND_PPB                 0x80 /* PPB data block */
#define PLSS_PL_IND_V3                  0x81 /* V3 block sending data format for 2d accelero */
#define PLSS_PL_IND_V4                  0x82 /* V4 block sending data format for common datas */
#define PLSS_PL_IND_PRB                 0x83 /* PRB data block */
#define PLSS_PL_IND_INFO                0x84 /* INFO data block */
#define PLSS_PL_IND_CLOCK               0x85 /* Clock data block */
#define PLSS_PL_IND_TXT                 0x86 /* TXT information from UART or 485 */
#define PLSS_PL_IND_INFO_EXT            0x87 /* Extended INFO data block */
#define PLSS_PL_IND_DATA                0x88 /* PPB or other data block: Byte#0 - Offset/Type, Byte#1 - Length, Byte#2..#N (size=Length) - Payload */
#define PLSS_PL_IND_VIDEO               0x89 /* Video stream */
#define PLSS_PL_IND_INFO_CAP            0x8a /* Driver capability list */
#define PLSS_PL_IND_UPG_CMD             0x8b /* Upgrade cmd */

#define PLSS_PL_NONE_INDICATOR          0x00 /* PLSS None-Payload Indicator fport=2, payload=00 indicats nothing */


#define LORA_MAC_DR_LEN_SET_EU(dr_len) do {\
  dr_len[0] = 51; /* Ignore the too large definitions and use the standard default */\
  dr_len[1] = 51; /* Ignore the too large definitions and use the standard default */\
  dr_len[2] = 51;\
  dr_len[3] = 115;\
  dr_len[4] = 222;\
  dr_len[5] = 222;\
  dr_len[6] = 222;\
  dr_len[7] = 222;\
  dr_len[8] = 0;\
  dr_len[9] = 0;\
  dr_len[10] = 0;\
  dr_len[11] = 0;\
  dr_len[12] = 0;\
  dr_len[13] = 0;\
  dr_len[14] = 0;\
  dr_len[15] = 0;\
} while(0)

#define LORA_MAC_DR_LEN_SET_EU_DWELL(dr_len) do {\
  dr_len[0] = 1; /* Ignore the too large definitions and use the standard default */\
  dr_len[1] = 1; /* Ignore the too large definitions and use the standard default */\
  dr_len[2] = 11;\
  dr_len[3] = 53;\
  dr_len[4] = 125;\
  dr_len[5] = 242;\
  dr_len[6] = 242;\
  dr_len[7] = 242;\
  dr_len[8] = 0;\
  dr_len[9] = 0;\
  dr_len[10] = 0;\
  dr_len[11] = 0;\
  dr_len[12] = 0;\
  dr_len[13] = 0;\
  dr_len[14] = 0;\
  dr_len[15] = 0;\
} while(0)

#define LORA_MAC_DR_LEN_SET_CN470(dr_len) do {\
  dr_len[0] = 51; /* Ignore the too large definitions and use the standard default */\
  dr_len[1] = 51; /* Ignore the too large definitions and use the standard default */\
  dr_len[2] = 51;\
  dr_len[3] = 115;\
  dr_len[4] = 222;\
  dr_len[5] = 222;\
  dr_len[6] = 0;\
  dr_len[7] = 0;\
  dr_len[8] = 0;\
  dr_len[9] = 0;\
  dr_len[10] = 0;\
  dr_len[11] = 0;\
  dr_len[12] = 0;\
  dr_len[13] = 0;\
  dr_len[14] = 0;\
  dr_len[15] = 0;\
} while(0)

#define LORA_MAC_DR_LEN_SET_CN470_DWELL(dr_len) do {\
  dr_len[0] = 1; /* Ignore the too large definitions and use the standard default */\
  dr_len[1] = 1; /* Ignore the too large definitions and use the standard default */\
  dr_len[2] = 11;\
  dr_len[3] = 53;\
  dr_len[4] = 125;\
  dr_len[5] = 242;\
  dr_len[6] = 0;\
  dr_len[7] = 0;\
  dr_len[8] = 0;\
  dr_len[9] = 0;\
  dr_len[10] = 0;\
  dr_len[11] = 0;\
  dr_len[12] = 0;\
  dr_len[13] = 0;\
  dr_len[14] = 0;\
  dr_len[15] = 0;\
} while(0)

#define LORA_MAC_DR_LEN_SET_US915(dr_len) do {\
  dr_len[0] = 11; /* note the spec 1.1 is wrong which is too large/long for dr0/dr1 */\
  dr_len[1] = 53; /* note the spec 1.1 is wrong which is too large/long for dr0/dr1 */\
  dr_len[2] = 125;\
  dr_len[3] = 242;\
  dr_len[4] = 242;\
  dr_len[5] = 222;\
  dr_len[6] = 0;\
  dr_len[7] = 0;\
  dr_len[8] = 33;\
  dr_len[9] = 109;\
  dr_len[10] = 222;\
  dr_len[11] = 222;\
  dr_len[12] = 222;\
  dr_len[13] = 222;\
  dr_len[14] = 0;\
  dr_len[15] = 0;\
} while(0)

#define LORA_MAC_DR_LEN_SET_AU915(dr_len) do {\
  dr_len[0] = 11; /* note the spec 1.1 is wrong which is too large/long for dr0/dr1 */\
  dr_len[1] = 51; /* note the spec 1.1 is wrong which is too large/long for dr0/dr1 */\
  dr_len[2] = 51;\
  dr_len[3] = 115;\
  dr_len[4] = 222;\
  dr_len[5] = 222;\
  dr_len[6] = 222;\
  dr_len[7] = 0;\
  dr_len[8] = 33;\
  dr_len[9] = 109;\
  dr_len[10] = 222;\
  dr_len[11] = 222;\
  dr_len[12] = 222;\
  dr_len[13] = 222;\
  dr_len[14] = 0;\
  dr_len[15] = 0;\
} while(0)

#define LORA_MAC_DR_LEN_SET_AU915_DWELL(dr_len) do {\
  dr_len[0] = 1; /* note the spec 1.1 is wrong which is too large/long for dr0/dr1 */\
  dr_len[1] = 1; /* note the spec 1.1 is wrong which is too large/long for dr0/dr1 */\
  dr_len[2] = 11;\
  dr_len[3] = 53;\
  dr_len[4] = 125;\
  dr_len[5] = 242;\
  dr_len[6] = 242;\
  dr_len[7] = 0;\
  dr_len[8] = 33;\
  dr_len[9] = 109;\
  dr_len[10] = 222;\
  dr_len[11] = 222;\
  dr_len[12] = 222;\
  dr_len[13] = 222;\
  dr_len[14] = 0;\
  dr_len[15] = 0;\
} while(0)

#define LORA_MAC_863_FREQ_DEF           868100000 /* the first default channel freq */
#define LORA_MAC_863_FREQ_FROM          863000000
#define LORA_MAC_863_FREQ_TO            870000000
#define LORA_MAC_863_FREQ_RX2           869525000
#define LORA_MAC_779_FREQ_DEF           779500000 /* the first default channel freq */
#define LORA_MAC_779_FREQ_FROM          779000000
#define LORA_MAC_779_FREQ_TO            787000000
#define LORA_MAC_779_FREQ_RX2           786000000
#define LORA_MAC_433_FREQ_DEF           433175000 /* the first default channel freq */
#define LORA_MAC_433_FREQ_FROM          433050000
#define LORA_MAC_433_FREQ_TO            434790000
#define LORA_MAC_433_FREQ_RX2           434665000
#define LORA_MAC_470EU_FREQ_DEF         470300000 /* the first default channel freq */
#define LORA_MAC_470EU_FREQ_FROM        470000000
#define LORA_MAC_470EU_FREQ_TO          510000000
#define LORA_MAC_470EU_FREQ_RX2         471300000
#define LORA_MAC_470EU_DR_RX2           0
#define LORA_MAC_923_FREQ_DEF           923200000 /* the first default channel freq */
#define LORA_MAC_923_FREQ_FROM          920000000
#define LORA_MAC_923_FREQ_TO            928900000
#define LORA_MAC_923_FREQ_RX2           923200000
#define LORA_MAC_923_DR_RX2             2
#define LORA_MAC_864RU_FREQ_DEF         868900000 /* the first default channel freq */
#define LORA_MAC_864RU_FREQ_FROM        864000000
#define LORA_MAC_864RU_FREQ_TO          870000000
#define LORA_MAC_864RU_FREQ_RX2         869100000

/* Multicast Definitions */
#define PLSS_MCAST_ADDR                 0xc0fffe73
#define PLSS_MCAST_DN_COUNTER           0
#define PLSS_MCAST_NWKS_KEY {0x73, 0x74, 0x88, 0x89, 0x03, 0x47, 0x18, 0x98,\
  0x37, 0x26, 0x55, 0x44, 0xaf, 0x37, 0x28, 0x29,}
#define PLSS_MCAST_APPS_KEY {0x63 , 0x64, 0x78, 0x79, 0xf3, 0x37, 0x08, 0x88,\
  0x26, 0x25, 0x54, 0x43, 0xae, 0x36, 0x27, 0x28,}

/* Common String Array */
#define PLSS_SENSOR_TYPE_STR_DEFINE \
  "Battery power", \
  "Temperature",\
  "Humidity",\
  "Temperature+Humidity",\
\
  "Sound",\
  "PIR",\
  "Light",\
  "Tilt / Incline",\
\
  "3-Axis Accelero/Vibration",\
  "GPS",\
  "Ultrasonic Distance",\
  "Atmosphere Pressure",\
\
  "CO",\
  "2-Axis Accelero/Vibration", \
  "External Analog",\
  "External Digital",\
\
  "External Temperature",\
  "Displacement",\
  "Soil Humidity ADC",\
  "External PIR",\
\
  "External Counter",\
  "External Switch",\
  "External Analog temp",\
  "External NH3",\
  \
  "Gas Analog",\
  "Interrupt/3-Axis Accelero",\
  "3-Axis Accelero/Incline",\
  "Multi Sensors",\
  \
  "SGA400",\
  "rsvd29",\
  "rsvd30",\
  "rsvd31",\
\
  "rsvd32",  \
  "rsvd33",\
  "rsvd34",\
  "rsvd35",\
  \
  "rsvd36",\
  "MC Analog",\
  "C2H2",\
  "Gasoline",\
  \
  "Delay Report",\
  "CO2 Percentage",\
  "CO2 PPM",\
  "External Current",\
  \
  "External PSI",\
  "External Counter/raw",\
  "External PM",\
  "External Current/AVG",\
\
  "External UV",\
  "External Wind/DIR",\
  "External Wind/Speed",\
  "External Rainfall",\
\
  "Motion/Vibration 2",\
  "Motion/Vibration 3",\
  "Soil Humidity 485",\
  "Motion HZ",\
\
  "Motion Velocity",\
  "Motion Displacement",\
  "Camera",\
  "External Counter 12",\
\
  "External Leakage", \
  "External 485/TTL(232)", \
  "External VOC", \
  "External PH",

#define PLSS_SENSOR_PL_NAME_DEFINE \
  "zero",\
  "temp",\
  "hum",\
  "acc3",\
  "inc",\
  "acc2",\
  "light",\
  "vbat",\
\
  "snd",\
  "gps",\
  "co_ppm",\
  "ext_mv",\
  "ext_01",\
  "pressure",\
  "ext_temp",\
  "disp",\
\
  "int_pir",\
  "shum",\
  "pir",\
  "cntr",\
  "sw",\
  "nh3",\
  "ash3",\
  "c6h6",\
  \
  "cl2",\
  "h2",\
  "h2s",\
  "hcl",\
  "hcn",\
  "hf",\
  "no2",\
  "o3",\
  \
  "ph3",\
  "so2",\
  "ch4",\
  "c2h2",\
  "gasoline",\
  "c2h4o3",\
  "distance",\
  "co2_kppm",\
\
  "co2",\
  "current",\
  "psi",\
  "pm",\
  "pos8",\
  "pos16",\
  "avg_current",\
  "uv",\
\
  "wind_dir",\
  "wind_speed",\
  "rainfall",\
  "vibration",\
  "m_hz",\
  "m_v",\
  "m_d",\
  "cntr_12",\
\
  "leakage",\
  "ch2o",\
  "current_100ua",\
  "voc",\
  "ext_ph",\
  "o2_kppm",\
  "time",\
  "ac_v",\
\
  "ac_i",\
  "ac_w",\
  "ac_pf",\
  "ac_hz",\
  "ac_kwh",\
  "ac_v3",\
  "ac_i3",\
  "ac_w4",\
\
  "ac_rw4",\
  "ac_pf3",\
  "ac_rkwh",\
  "esignal",\
  "co",\
  "rssi",\
  "delta",\
  "uvp",\
\
  "hum",\
  "o2",\
  "ntu",\
  "ec",\
  "ch2oppb",\
  "do_mg",\
  "chl",\
  "bga",\
\
  "cod",\
  "toc",\
  "nh3_n",\
  "k+",\
  "nh4",\
  "pos64",\
  "rad",\
  "c2h4",\
\
  "ac_hz3",\
  "ac_aw4",\
  "do",\
  "salt",\
  "ph",\
  "ph",\
  "salt",\
  "ec",\
\
  "n",\
  "p",\
  "k",\
  "voc",\
  "io",\
  "ohm",\
  "flow",\
  "per",\
\
  "vol",\
  "rvol",\
  "multi",

#define PLSS_SENSOR_PL_LEN_DEFINE \
  0,2,1,6,6,4,2,2,\
  2,8,2,2,1,4,2,2,\
  1,1,1,4,1,2,2,2,\
  2,2,2,2,2,2,2,2,\
  2,2,2,2,2,2,2,2,\
  2,2,2,6,1,2,6,1,\
  1,2,2,1,6,6,6,4,\
  1,2,2,2,1,2,6,4,\
  4,4,4,4,4,6,6,8,\
  8,6,4,20,2,4,6,2,\
  2,2,2,4,2,4,4,4,\
  4,4,4,4,4,8,2,2,\
  6,8,4,4,4,2,2,2,\
  2,2,2,2,1,4,4,1,\
  4,4,4,

/* JSON Data Unit
 * [unit]             [type]             
 * voltage            voltage/ac_v/ac_v3
 * ppm                gas (co2,co,o2,so2,ch4,...)
 * mm                 distance/displacement/rainfall
 * celsius            temperature
 * mm/s               speed/wind speed/motion_velocity
 * ampere             current/ac_i/ac_i3
 * degree             angle
 * ug/m3              AQI/pm
 * psi                water/psi
 * pa                 pressure
 * mg                 acclerator
 * 1%                 humidity
 * lumen              light
 * 1                  ph/hz/uv/cntr/pir/sw/...
 * watt               ac_w/aw (sum/A/B/C)
 * kw                 ac_kw/rkw
 * kwh                ac_kwh/rkwh
 *
 */
#define PLSS_SENSOR_PL_NAME_DEFINE_JSON \
  "zero",\
  "temp",\
  "hum",\
  "acc3",\
  "inc",\
  "acc2",\
  "light",\
  "vbat",\
\
  "snd",\
  "gps",\
  "co_ppm",\
  "vext",\
  "ext_01",\
  "pressure",\
  "ext_temp",\
  "disp",\
\
  "int_pir",\
  "shum",\
  "pir",\
  "cntr",\
  "sw",\
  "nh3",\
  "ash3",\
  "c6h6",\
  \
  "cl2",\
  "h2",\
  "h2s",\
  "hcl",\
  "hcn",\
  "hf",\
  "no2",\
  "o3",\
  \
  "ph3",\
  "so2",\
  "ch4",\
  "c2h2",\
  "gasoline",\
  "c2h4o3",\
  "distance",\
  "co2",\
\
  "co2",\
  "current",\
  "psi",\
  "pm",\
  "pos",\
  "pos",\
  "avg_current",\
  "uv",\
\
  "wind_dir",\
  "wind_speed",\
  "rainfall",\
  "vibration",\
  "m_hz",\
  "m_v",\
  "m_d",\
  "cntr_12",\
\
  "leakage",\
  "ch2o",\
  "current",\
  "voc",\
  "ph",\
  "o2",\
  "time",\
  "ac_v",\
\
  "ac_i",\
  "ac_w",\
  "ac_pf",\
  "ac_hz",\
  "ac_kwh",\
  "ac_v",\
  "ac_i",\
  "ac_w",\
\
  "ac_rw",\
  "ac_pf",\
  "ac_rkwh",\
  "esignal",\
  "co",\
  "rssi",\
  "delta",\
  "uvp",\
\
  "hum",\
  "o2",\
  "ntu",\
  "ec",\
  "ch2oppb",\
  "do_mg",\
  "chl",\
  "bga",\
\
  "cod",\
  "toc",\
  "nh3_n",\
  "k+",\
  "nh4",\
  "pos",\
  "rad",\
  "c2h4",\
\
  "ac_hz3",\
  "ac_aw",\
  "do",\
  "salt",\
  "ph",\
  "ph",\
  "salt",\
  "ec",\
\
  "n",\
  "p",\
  "k",\
  "voc",\
  "io",\
  "ohm",\
  "flow",\
  "per",\
\
  "vol",\
  "rvol",\
  "multi",

#define PLSS_SENSOR_PL_TYPE_DEFINE_JSON \
  1,0x91,1,0x83,0xa3,0x82,1,0x31,\
  0x11,0x2,1,0x31,1,0x81,0x91,0xa1,\
  1,1,1,1,1,1,1,1,\
  1,1,1,1,1,1,1,1,\
  1,1,1,1,1,1,1,1,\
  1,0x21,1,0x3,1,1,0x23,1,\
  1,1,1,1,0x3,0x93,0x93,0x82,\
  1,1,0x41,1,0x11,1,1,0xff,\
  0xff,0xff,0xff,0xff,0xff,0x13,0x23,0x04,\
  0x04,0x3,0xff,0x85,0x21,0x82,0xa3,1,\
  0x11,0x21,0x11,0xff,1,0xff,0xff,0xff,\
  0xff,0xff,0xff,0xff,0xff,1,1,1,\
  0x23,0x04,0xff,0xff,0xff,0x21,1,1,\
  1,1,1,1,1,1,1,1,\
  1,1
  
#define PLSS_SENSOR_PL_MULTI_1000_DEFINE_JSON \
  PLSS_PL_EXT_CO2,PLSS_PL_EXT_O2,
#define PLSS_PL_JSON_FILED_EUI                "id"
#define PLSS_PL_JSON_FILED_CTRL               "ctrl"
#define PLSS_PL_JSON_FILED_CTRL_B64           "cb64"
#define PLSS_PL_JSON_FIELD_RTU                "rtu"

#define PLSS_SENSOR_PL_TYPE_DEFINE \
  1,0x91,1,0x83,0xa3,0x82,1,1,\
  0x11,0x2,1,1,1,0x81,0x91,0xa1,\
  1,1,1,1,1,1,1,1,\
  1,1,1,1,1,1,1,1,\
  1,1,1,1,1,1,1,0x11,\
  1,1,1,0x3,1,1,0x3,1,\
  1,1,1,1,0x3,0x83,0x83,0x82,\
  1,1,0x11,1,0x11,0x11,1,0xff,\
  0xff,0xff,0xff,0xff,0xff,0x13,0x23,0x04,\
  0x04,0x03,0xff,1,0x21,0x82,0xa3,1,\
  0x11,0x21,0x11,0xff,1,0xff,0xff,0xff,\
  0xff,0xff,0xff,0xff,0xff,1,1,1,\
  0x23,0x04,0xff,0xff,0xff,0x21,1,1,\
  1,1,1,1,1,1,1,1,\
  1,1,1,

#define PLSS_SENSOR_RF_NAME_DEFINE \
"US902",\
"CN470",\
"EU864",\
"CN779",\
"EU433",\
"EU470",\
"AS923",\
"RU864",\
"RSVD8",\
"RSVD9",\
"RSVD10",\
"RSVD11",\
"RSVD12",\
"RSVD13",\
"RSVD14",\
"RSVD15",\
"NB_WIFI",

#define PLSS_BOARD_NAME_DEFINE \
  "WXS8800 V1.0",\
  "WXS8800 V1.1",\
  "WXS8700 V1.3",\
  "WXS8800 V1.3",\
  "WXS9800 V1.0",\
  "WXS7800 V1.0",\
  "WXS9810 V1.0",\
  "UNKNOWN BOARD",

#define PLSS_APP_PROFILE_DEFINE \
  "ALL",\
  "RSVD1",\
  "APP",\
  "SECURITY",\
  "MOTION",\
  "RSVD5",\
  "RSVD6",\
  "RSVD7",

#define PLSS_AUTO_RESET_DEFINE \
  "2",\
  "3",\
  "4",\
  "5",\
  "6",\
  "7",\
  "14",\
  "Disable",

#define PLSS_MULTI_PL_TYPE_DEFINE \
  "gas",\
  "flam",\
  "co",\
  "o2",\
  "h2",\
  "ch4",\
  "c3h8",\
  "co2",\
\
  "o3",\
  "h2s",\
  "so2",\
  "nh3",\
  "cl2",\
  "c2h4o",\
  "hcl",\
  "ph3",\
\
  "hbl",/*0x10*/\
  "hcn",\
  "ash3",\
  "hf",\
  "bl2",\
  "no",\
  "no2",\
  "nox",\
  \
  "clo2",\
  "sih4",\
  "cs2",\
  "f2",\
  "b2h6",\
  "geh4",\
  "n2",\
  "c4h8s",\
  \
  "c2h2",/*0x20*/\
  "c2h4",\
  "ch2o",\
  "lpg",\
  "chx",\
  "c6h6",\
  "h2o2",\
  "voc",\
\
  "sf6",\
  "c7h8",\
  "c4h6",\
  "cos",\
  "n2h4",\
  "h2si",\
  "c8h8",\
  "c4h8",\
\
  "ch2",/*0x30*/\
  "n2o",\
  "ngas",\
  "cocl2",\
  "c2h3cl",\
  "ch4o",\
  "c2h6o",\
  "c3h8o",\
\
  "c3h6o",\
  "c2h4o",\
  "c3h3n",\
  "c2h6s",\
  "c3h5clo",\
  "c4h8o2",\
  "c4h8o",\
  "ch3sh",\
\
  "c2cl4",/*0x40*/\
  "socl2",\
  "c4h6o2",\
  "tbm",\
  "tvoc",\
  "c6h12",\
  "c2hcl3",\
  "c8h10",\
\
  "freon",\
  "ch3cl",\
  "ch2cl2",\
  "chcl3",\
  "ch3nh2",\
  "c5h12",\
  "c6h14",\
  "c7h16",\
\
  "c8h18",/*0x50*/\
  "c2h6",\
  "pe",\
  "c4h10",\
  "he",

typedef enum {
  PLSS_MULTI_UNIT_NONE = 0,
  PLSS_MULTI_UNIT_VOL,
  PLSS_MULTI_UNIT_PPM,
  PLSS_MULTI_UNIT_PPB,
  PLSS_MULTI_UNIT_LEL,
  PLSS_MULTI_UNIT_MG_M3,
  PLSS_MULTI_UNIT_UG_M3,
} plss_multi_unit_t;
 

#ifdef __cplusplus
}
#endif

#endif /* __STM32L1XX_NUCLEO_H */

