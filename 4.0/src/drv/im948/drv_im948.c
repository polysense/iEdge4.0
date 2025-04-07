/**
 * @file  drv_im948.c
 * @brief SENSOR IM948 driver
 * @author Runby F.
 * @date 2022-12-19
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * 版本修订历史
v1.0 @ 2024-01-23
  1) Support IM948 sensor

v1.1 @ 2024-01-26
  1) Support IM948 calibration

v1.2 @ 2024-02-01
  1) Use axis-9 in dbg printing
  2) Use Euler data

v1.3 @ 2024-02-03
  1) Support d0 7:turn on meganetism, 6: turn off meganetism

*/

/**
 * @brief Driver version
 */
#define DRV_IM948_VERSION 0x0103

/**
 * @brief Driver name
 */
#define DRV_IM948_NAME  "AXIS9"

/**
 * @brief Operating timeout
 */
#define DRV_UART_IO_TIMEOUT  10000

/** 
* Power control
* @param[in]  on   0:Power off, 1:Power on
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
* @note If sensor does NOT require any power control, this fucntion can be NULL
*/ 
pos_status_t sensor_im948_set(pos_u32_t on) {  
  pos_gpio_pin_t pwr;
  drv_api_t *drv = g_drv;
  ma_board_t *b = drv->board;
  pwr = (drv->s->slot->io >> 8) & 0x7f;  
  if( !pwr )
    pwr = MB_PWR_ID_VCCN_PWR3;

  b->pwr_set(pwr, on);
#if 0  
  pin = b->pin_map(MB_PT_DIO, drv->s->slot->rsvd32 & 0xffff);
  if( on ) {
    drv->os->gpio->mode_set(pin, POS_GPIO_MODE_OUTPUT);
  }
#endif  
  return POS_STATUS_OK;
}

void sensor_im948_io_flush(pos_io_handle_t *io) {
  /* 清空串口并等待20ms */
  while( io->read(io, POS_NULL, 1024, 20) >= 1024 );   
}

pos_u32_t sensor_im948_send_buf(pos_io_handle_t *io, pos_u8_t *buf, pos_u32_t buf_len, pos_u8_t *response, pos_u32_t res_len) {
  pos_u8_t c;
  pos_u32_t v, i, j, loop_cnt;
  drv_api_t *drv = g_drv;  
  loop_cnt = buf_len ? 4 : 1;
  for( j = 0; j < loop_cnt; j++ ) {
    /* 清空串口*/
    if( buf_len ) {      
      sensor_im948_io_flush(io);
      io->write(io, (pos_u8_t*)buf, buf_len, 100);
    }
    v = io->read(io, response, 1, 450);
    if( v ) {
      v += io->read(io, &response[1], res_len-1, 50);
    }
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {    
      drv->log->buf("axis9 tx", buf, buf_len);
      drv->log->buf("axis9 rx",response, v);
      drv->os->tick->sleep(buf_len+v+100); /* wait until all printed */
    }
    if( v <= 3 || response[0] != 0x49 || response[v-1] != 0x4d )
      continue;
    c = response[1];
    for( i = 2; i < v - 2; i++ ) {
      c += response[i];
    }
    if( c == response[i] )
      return v;
  }
  return 0;
}

pos_u32_t sensor_im948_send_multi(pos_io_handle_t *io, pos_u8_t *cmd, pos_u32_t cmd_len, pos_u8_t *response, pos_u32_t res_len) {
  pos_u8_t buf[72], i, crc;

  if( cmd_len > 16 || !cmd_len )
    return 0;
  
  g_drv->os->memset(buf, 0, sizeof(buf));
  buf[47] = 0xff;
  buf[49] = 0xff;
  buf[50] = 0x49; /* start code */
  buf[51] = 0xff; /* bc address */
  buf[52] = cmd_len; /* length */
  crc = buf[51] + buf[52];
  for( i = 0; i < cmd_len; i++ ) {
    buf[53+i] = cmd[i];
    crc += cmd[i];
  }
  buf[53+cmd_len] =  crc; /* cksum */
  buf[54+cmd_len] = 0x4d;  /* stop code */
  return sensor_im948_send_buf(io, buf, 55+cmd_len, response, res_len);
}

pos_u32_t sensor_im948_send_single(pos_io_handle_t *io, pos_u8_t cmd, pos_u8_t *response, pos_u32_t res_len) {
  return sensor_im948_send_multi(io, &cmd, 1, response, res_len);
}

void sensor_im948_print_len(char *name, pos_u8_t *d, pos_i32_t mul, pos_i32_t div, pos_u8_t d_len) {
  pos_i32_t v;
  char *sign;
  v = d[0] + d[1] * 256;
  if( d_len > 2)
    v += d[2] << 16;
  g_drv->os->printf( "(0x%04x) ", v&0xffff);
  if( v & 0x8000 )
    v |= 0xffff0000;
  v = (v * mul + div/2) / div;
  sign = v < 0 ? "-" : "";
  v = g_drv->os->util->abs(v);
  g_drv->os->printf("%7s: %s%d.%03d\n", name, sign, v/1000, v%1000);  
}

void sensor_im948_print(char *name, pos_u8_t *d, pos_i32_t mul, pos_i32_t div) {
  sensor_im948_print_len(name, d, mul, div, 2);
}
void sensor_im948_dbg(pos_u8_t *d) {
  pos_u32_t i;
  d += 4; /* skip 49004911 */

  i = d[0] + (d[1]<<8);
  g_drv->os->printf("%16s: 0x%x\n", "TAG", i); d+=2;
  i = d[0] + (d[1]<<8) + (d[2]<<16) + (d[3]<<24);
  g_drv->os->printf("%16s: %u\n", "MS", i); d+=4;

  sensor_im948_print("aX", d, 47852, 10000); d+=2;
  sensor_im948_print("aY", d, 47852, 10000); d+=2;
  sensor_im948_print("aZ", d, 47852, 10000); d+=2;  
  sensor_im948_print("AX", d, 47852, 10000); d+=2;
  sensor_im948_print("AY", d, 47852, 10000); d+=2;
  sensor_im948_print("AZ", d, 47852, 10000); d+=2;  
  sensor_im948_print("GX", d, 61035, 1000); d+=2;
  sensor_im948_print("GY", d, 61035, 1000); d+=2;
  sensor_im948_print("GZ", d, 61035, 1000); d+=2;    
  sensor_im948_print("CX", d, 15106, 100); d+=2;
  sensor_im948_print("CY", d, 15106, 100); d+=2;
  sensor_im948_print("CZ", d, 15106, 100); d+=2;
  sensor_im948_print("TEMP", d, 10, 1); d+=2;
  sensor_im948_print_len("HPA", d, 23842, 100000, 3); d+=3;
  sensor_im948_print_len("HEIGHT", d, 10729, 10000, 3); d+=3;
  sensor_im948_print("W", d, 30512, 10000); d+=2;  
  sensor_im948_print("X", d, 30512, 10000); d+=2;  
  sensor_im948_print("Y", d, 30512, 10000); d+=2;  
  sensor_im948_print("Z", d, 30512, 10000); d+=2;    
  g_drv->os->tick->sleep(20);  
  sensor_im948_print("X-EULER", d, 54931, 10000); d+=2;    
  sensor_im948_print("Y-EULER", d, 54931, 10000); d+=2;    
  sensor_im948_print("Z-EULER", d, 54931, 10000); d+=2;      
  sensor_im948_print("X-POS", d, 1, 1); d+=2;    
  sensor_im948_print("Y-POS", d, 1, 1); d+=2;    
  sensor_im948_print("Z-POS", d, 1, 1); d+=2;      
  i = d[0] + (d[1]<<8) + (d[2]<<16) + (d[3]<<24);
  g_drv->os->printf("%16s: %u\n", "STEP", i); d+=4;
  g_drv->os->printf("%16s: 0x%x\n", "STATUS", d[0]); d+=1;
  sensor_im948_print("asX", d, 47852, 10000); d+=2;
  sensor_im948_print("asY", d, 47852, 10000); d+=2;
  sensor_im948_print("asZ", d, 47852, 10000); d+=2;    
  i = d[0] + (d[1]<<8);
  g_drv->os->printf("%16s: 0x%x\n", "ADC", i); d+=2;
  g_drv->os->printf("%16s: 0x%x\n", "GPIO", d[0]);
  g_drv->os->tick->sleep(20);
}

/** 
* Sensor collecting
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_im948_collect(void){
  pos_io_handle_t *io;
  pos_status_t ret = POS_STATUS_E_TIMEOUT;
  pos_u32_t v, t, b, report, last;
  drv_api_t *drv = g_drv;
  pos_u8_t response[96];
  pos_i16_t data[3];
  
  /* 初始化串口 */
  t = drv->s->slot->io;
  b = drv->s->slot->rsvd32;
  
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("axis9 io", t);        
    drv->log->data("axis9 rsvd", b);    
  }

  t &= 0xff; /* keep io for uart only */
  
  io = POS_NULL;
  if( POS_IO_IS_SERIAL_PORT(t) ) {
    io = drv->os->io->init(t, 115200, 0); /* 默认用115200 */
  }

  /* refresh to normal by default */
  drv->s->ctrl |= MA_SENSOR_CTRL_STAT_NORMAL;
  drv->s->ctrl &= ~MA_SENSOR_CTRL_STAT_ERROR;
  report = 0;  
  
  do {
    const pos_u8_t param[] = {0x12, 0x05, 0xff, 0x00, 0x07, 0x02, 0x02, 0x04, 0x09, 0xff, 0xff};
    if( !io )
      break;
    
    /* work mode setup */
    v = sensor_im948_send_single(io, 0x03, response, sizeof(response)); /* start cmd */
    if( v < 6 ) {
      break;
    }

    /* calib if b is set */
    if( (b & 0xffff) ) {
      pos_u16_t i;
      const pos_u8_t calib_cmd[] = {
        0x05, /* z angle zero */
        0x06, /* x/y/z axis zero */
        0x32, /* CX/CY/CZ calib */
        0x07, /* acc X/Y/Z calib */
        0x14, /* factory default */
        0x13, /* 3d zero */
        0x16, /* step zero */
        0x08, /* z angle/axis default */
      };
      v = (b >> 16) & 0xff;
      if( !v )
        v = 10000;
      else
        v *= 1000;
      for( i = 0; i < POS_ARRAY_CNT(calib_cmd); i++ ) {
        if( (b & (1<<i)) == 0 ) 
          continue;
        sensor_im948_send_single(io, calib_cmd[i], response, sizeof(response)); /* start calib */
        if( calib_cmd[i] == 0x32 ) {
          drv->os->tick->sleep(v);          
          sensor_im948_send_single(io, 0x04, response, sizeof(response)); /* stop calib */
        }
      }
      v = b & 0xffff0000;
      t = (pos_u32_t)&drv->s->slot->rsvd32 - MA_EEPROM_ADDR;
      drv->eeprom->update(t, &v, 4);
      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {      
        drv->log->data("axis9 calib", b);
        drv->log->data("axis9 supp",POS_ARRAY_CNT(calib_cmd) );
      }
    }

    /* parameter setup */
    drv->os->memcpy(response, param, 11);
    v = drv->s->slot->arg.u8[0];
    if( v )
      response[4] = v;    
    v = sensor_im948_send_multi(io, response, 11, response, sizeof(response)); 
    if( v < 6 ) {
      break;
    }
    
    /* set auto report */
    v = sensor_im948_send_single(io, 0x19, response, sizeof(response)); /* auto report cmd */
    if( v < 6 ) {
      break;
    }

    /* collect until timeout */
    last = 0;
    t = drv->os->tick->get() + DRV_UART_IO_TIMEOUT;
    do {
      v = sensor_im948_send_buf(io, POS_NULL, 0, response, sizeof(response));
      if( v < 78 )
        continue;
      report++;
      if( report < 2 ) {
        last = (response[11]<<8) + response[10]; /* save last x */
        continue; /* for unkown reason, first severals report is not stable. ignore it */
      }      

      /* dbg report */
      if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) { 
        drv->log->buf("valid", response, v);
        sensor_im948_dbg(response);
      }
      v = (response[11]<<8) + response[10];
      if( last == v )
        break;
      last = v;
    } while( !drv->os->tick->is_timeout(drv->os->tick->get(), t) );
    
  } while(0);

  if( report ) {
#if 0    
    /* read C X/Y/Z */
    for( t = 0; t < 3; t++ ) {
      pos_i32_t d;
      d = response[28+t*2] + response[28+t*2+1]*256;   
      if( d & 0x8000 )
        d |= 0xffff0000;
      d = d * 15106;
      if( d >= 0 )
        d += 5000;
      else
        d -= 5000;
      d /= 10000;
      if( d > 32767 )
        d = 32767;
      else if( d < -32768 )
        d = -32768;
      data[t] = d;
    }
    drv->data->plss_put_u16(PLSS_PL_ACCELERATION, (pos_u16_t)data[0]);
#else
    /* read EULER X/Y/Z */
    for( t = 0; t < 3; t++ ) {
      pos_i32_t d;
      d = response[50+t*2] + response[50+t*2+1]*256;   
      if( d & 0x8000 )
        d |= 0xffff0000;
      d = d * 54931;
      if( d >= 0 )
        d += 50000;
      else
        d -= 50000;
      d /= 100000;
      if( d > 32767 )
        d = 32767;
      else if( d < -32768 )
        d = -32768;
      data[t] = d;
    }
    drv->data->plss_put_u16(PLSS_PL_INCLINE, (pos_u16_t)data[0]);  
#endif
    v = (pos_u16_t)data[1];
    drv->data->plss_put_u8(v>>8, v&0xff);
    v = (pos_u16_t)data[2];
    drv->data->plss_put_u8(v>>8, v&0xff);
    for( t = 0; t < 3; t++ ) {
      char s[8];
      drv->os->sprintf(s, "\"e%c\":", 'x'+t);
      drv->data->put_raw(s,drv->os->strlen(s));
      drv->data->put_float(data[t],100);
    }
  }
  if( !report ) { 
    /* refresh to ERROR stat */
    drv->s->ctrl |= MA_SENSOR_CTRL_STAT_ERROR;
    drv->log->data("axis9 err", drv->s->slot->io);
    /* 错误的Sensor, 上报出错 */
    drv->data->put("\"%s\":\"no axis9#0x%x\",", "err", drv->s->slot->io, 0, 0, 0);
  }

  if( io ) {      
    /* shutdown IO after read */
    io->release(io);
  }
  return ret;
}


/*
 * Driver export
 */
ma_drv_export_t g_export = {
  .version = DRV_SENSOR_VERSION(MA_SENSOR_IM948, DRV_IM948_VERSION),
  .name = DRV_IM948_NAME,
  .u.sensor={
    .power_set = sensor_im948_set,
    .collect = sensor_im948_collect,
  },
};

