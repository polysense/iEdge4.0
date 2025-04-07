/**
 * @file  drv_oven.c
 * @brief SENSOR OVEN driver
 * @author Runby F.
 * @date 2022-4-28
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * 版本修订历史
v1.0 @ 2022-04-28
  1) 完成第一版驱动
  
*/

/**
 * @brief Driver version
 */
#define DRV_OVEN_VERSION 0x0101

/**
 * @brief Driver name
 */
#define DRV_OVEN_NAME  "OVEN"

/**
 * @brief Driver buffer size
 */
#define DRV_OVEN_BUF_SIZE  256

/**
 * @brief CRC check
 */
pos_status_t oven_read_chk(char *buf, pos_size_t len) {
  const drv_api_t *drv = g_drv;
  const pos_lib_t *os = g_drv->os;
  pos_u16_t c16;
  char rsp[8];
  if( len < 7  ) /* C L L H H \r \n*/
    return POS_STATUS_E_PARAMETER;

  if( buf[len-2] != '\r' && buf[len-3] != '\n' )
    return POS_STATUS_E_PARAMETER;
  
  c16 = os->crc(POS_CRC16_MBUS, (pos_u8_t*)buf, len - 6);
  os->sprintf(rsp, "%02X%02X", c16&0xff, (c16>>8)&0xff );
  if( os->memcmp(rsp, &buf[len-6], 4) != 0 ) {
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) 
      drv->log->data("oven exp crc", c16);
    return POS_STATUS_E_NOT_SUPPORT;
  }
  
  return POS_STATUS_OK;
}

/**
 * @brief Finish send command
 */
pos_size_t oven_add_crc(char *buf) {
  const pos_lib_t *os = g_drv->os;
  pos_u16_t c16;
  pos_size_t len;
  len = os->strlen(buf);
  c16 = os->crc(POS_CRC16_MBUS, (pos_u8_t*)buf, len);
  os->sprintf(&buf[len], "%02X%02X\r\n", c16&0xff, (c16>>8)&0xff );
  return len+6;
}


/** 
* Power control
* @param[in]  on   0:Power off, 1:Power on
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
* @note If sensor does NOT require any power control, this fucntion can be NULL
*/ 
pos_status_t sensor_oven_set(pos_u32_t on) {  
  const drv_api_t *drv = g_drv;
  pos_u16_t io;
  io = drv->s->slot->io;

  if( on ) {
   drv->os->io->init(POS_IO_UART0+(io&0xf), 9600, 0);
  } else {
   drv->os->io->done(POS_IO_UART0+(io&0xf));
  }
    
  return POS_STATUS_OK;
}

/** 
* Sensor collecting
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_oven_collect(void){
  const drv_api_t *drv = g_drv;
  const pos_lib_t *os = drv->os;
  pos_io_handle_t *io;
  char *buf;
  pos_size_t len, rsp_len;
  pos_u32_t log_to = 0;
  
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("oven start", DRV_OVEN_VERSION);
  }
  
  io = os->io->get(POS_IO_UART0+(drv->s->slot->io&0x0f));
  if( !io )
    return POS_STATUS_E_NOT_SUPPORT;

  buf = (char*)os->malloc(DRV_OVEN_BUF_SIZE);
  if( !buf )
    return POS_STATUS_E_MEM;

  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("oven loop", 0);
  }

  do {    
    while( io->read(io, (pos_u8_t*)&buf[0], 1, POS_IO_WAIT_FOREVER) == 0);
    len = 1 + io->read(io, (pos_u8_t*)&buf[1], DRV_OVEN_BUF_SIZE-2, 100);
    buf[len] = 0;
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      if( os->tick->is_timeout(os->tick->get(), log_to) ) {      
        drv->log->data("oven rx", len);
        os->puts(buf);
      }
    }
    if( oven_read_chk(buf, len) != POS_STATUS_OK ) {
      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
        if( os->tick->is_timeout(os->tick->get(), log_to) ) {
          drv->log->data("oven rx err", 0);
          log_to = os->tick->get() + 1000;
        }
      }      
      continue;
    }

    rsp_len = 0;
    switch(buf[0]) {
    case 'P': /* PING */
      os->strcpy(buf, "OK");
      rsp_len = os->strlen(buf);
      break;

    case 'S': /* STATUS */
      os->strcpy(buf, "\"C1\":{\"D\":\"01\",\"CS\":\"1234\",\"CA\":\"1234\",\"CM\":\"02\",\"ByT\":\"1234\",\"ByP\":\"1234\",\"OM\":\"01\",\"T\":[\"1234\"],\"PS\":\"1\",\"CO\":\"0060\",\"HO\":\"0030\",\"PO\":[\"0000\"],\"RN\":\"A\",\"ER\":\"FEDCBA9887654321\"}");
      rsp_len = os->strlen(buf);
      break;
      
    default:
      break;
    }    

    if( !rsp_len ) {      
      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
        if( os->tick->is_timeout(os->tick->get(), log_to) ) {
          drv->log->data("oven rx unknown", 0);
          log_to = os->tick->get() + 1000;
        }
      }      
      continue;
    }

    rsp_len = oven_add_crc(buf);
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      drv->log->data("oven tx", rsp_len);
      os->puts(buf);
    }    
    io->write(io, (pos_u8_t*)buf, rsp_len, 5000);
  } while(10);

  os->free(buf);
  return POS_STATUS_OK;
}

#define MA_SENSOR_OVEN 0xf0

/*
 * Driver export
 */
ma_drv_export_t g_export = {
  .version = DRV_SENSOR_VERSION(MA_SENSOR_OVEN, DRV_OVEN_VERSION),
  .name = DRV_OVEN_NAME,
  .u.sensor={
    .power_set = sensor_oven_set,
    .collect = sensor_oven_collect,
  },
};

