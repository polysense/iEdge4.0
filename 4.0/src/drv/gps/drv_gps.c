/**
 * @file  drv_gps.c
 * @brief SENSOR gps driver
 * @author Runby F.
 * @date 2022-8-30
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * 版本修订历史
v1.0 @ 2022-08-31
  1) First draft
  2) Support GPS json/plss report

v1.1 @ 2023-11-18
  1) Using new PWR ID solution

v1.2 @ 2024-08-27
  1) Report 0xffffffff when GPS is not located
  
*/

/**
 * @brief Driver version
 */
#define DRV_GPS_VERSION 0x0102

/**
 * @brief Driver name
 */
#define DRV_GPS_NAME  "GPS"

/**
 * @brief BUF len
 */
#define DRV_GPS_BUF_LEN 1024


/** 
* Power control
* @param[in]  on   0:Power off, 1:Power on
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
* @note If sensor does NOT require any power control, this fucntion can be NULL
*/ 
pos_status_t sensor_gps_set(pos_u32_t on) {  
  pos_u16_t pwr;
  ma_board_t *b = g_drv->board;
  pwr = (g_drv->s->slot->io >> 8) & 0x7f;  
  if( !pwr ) {
    pwr = MB_PWR_ID_VCCN_PWR3;
  }

  b->pwr_set(pwr, on);
  
  return POS_STATUS_OK;
}

/**
* String search
*/
pos_size_t sensor_gps_strchpos(const char *str, pos_u8_t char_to_find, pos_size_t size, pos_size_t skip) {
  pos_size_t i;
  for( i = 0; i < size; i++ ) {
    if( str[i] == char_to_find ) {
      if( skip <= 0 )
        return i;
      skip--;
    }
  }
  return size;
}

/**
* String value
*/
pos_u32_t sensor_gps_strval(const char *str, pos_size_t size) {
  pos_u32_t v;
  pos_size_t i;
  v = 0;
  if( str[0] == '0' && str[1] == 'x' ) {
    for( i = 2; i < size; i++ ) {
      if( str[i] >= '0' && str[i] <= '9' )
        v = (v<<4) + str[i] - '0';
      else if( str[i] >= 'a' && str[i] <= 'f' )
        v = (v<<4) + str[i] - 'a' + 10;
      else if( str[i] >= 'A' && str[i] <= 'F' )
        v = (v<<4) + str[i] - 'F' + 10;
      else
        break;
    }
  } else {
    for( i = 0; i < size; i++ ) {
      if( str[i] >= '0' && str[i] <= '9' )
        v = v*10 + str[i] - '0';
      else 
        break;
    }
  }
  return v;
}

/**
* String to u32
*/
pos_u32_t sensor_gps_string_to_u32( pos_u8_t *str, pos_u32_t len, pos_u32_t fraction_len ) {
  pos_u32_t pos, i;
  uint32_t v1, v2;
  pos = sensor_gps_strchpos((char*)str, '.', len, 0);
  if( pos >= len )
    return 0; /* unable to parse */
  v1 = sensor_gps_strval((char*)str, pos);
  v2 = v1 / 100; /* v2 is DD */
  v1 = ((v1 - v2 * 100) + v2 * 60)*60; /* translate from DDDMM into seconds */
  str += pos+1;
  len -= pos+1;
  pos = sensor_gps_strchpos((char*)str, ',', len, 0);
  v2 = sensor_gps_strval((char*)str, pos);
  v2 *= 60; /* translate from minutes into seconds */
  for( i = 0; i < fraction_len; i++ ) {
    v1 *= 10;
  }
  if( pos >= fraction_len )
    for( i = pos; i > fraction_len; i-- )
      v2 /= 10;
  else    
    for( i = pos; i < fraction_len; i++ )
      v2 *= 10;
  v1 += v2;
  if( pos < len ) {
    if( str[pos+1] == 'S' || str[pos+1] == 'W' )
      v1 |= 0x80000000; /* for latitude South and longitude West, the bit31 will be SET */
  }
  return v1;
}

/**
* GPS String get U32 value by index
*/
pos_u32_t sensor_gps_get_u32_by_index( pos_u8_t *str, pos_u32_t len, pos_u32_t fraction_len, pos_u32_t index ) {
  pos_u32_t pos;
  pos = sensor_gps_strchpos((char*)str, ',', len, index);
  if( pos < len ) {
    return sensor_gps_string_to_u32(str+pos+1, len-pos-1, fraction_len);
  }
  return 0;
}

/**
* GLL Search (example string: $GNGLL,3106.59337,N,12134.00610,E,161059.00,A,A*70\r\n
*/
pos_status_t sensor_gps_find_gll(pos_u8_t *str, pos_u32_t str_len, pos_u32_t *start, pos_u32_t *end) {
  pos_u32_t id_start, id_end, base = 0;
  while(str_len) {
    id_start = sensor_gps_strchpos((char*)str, '$', str_len, 0);
    if( id_start >= str_len )
      break;
    id_end = sensor_gps_strchpos((char*)str+id_start, '\n', str_len-id_start, 0);
    id_end += id_start;
    if( id_end >= str_len )
      break;
    if( id_end - id_start < 11 )
      break;
    if( str[id_end-1] != '\r' ||
        str[id_end-4] != '*' ) /* -2/-3 can be any for CKSUM */
      break;
    if( str[id_start+1] == 'G' && /* id_start+2 can be anything */
        str[id_start+3] == 'G' && 
        str[id_start+4] == 'L' && 
        str[id_start+5] == 'L' ) {
      if( start )
        *start = id_start + base;
      if( end )
        *end = id_end + base;
      return POS_STATUS_OK;
    }    
    str += id_end+1;
    str_len -= id_end+1;    
    base += id_end+1;
  }
  return POS_STATUS_E_NOT_FOUND;
}

/**
* NMEA checksum
*/
pos_status_t sensor_gps_cksum_check(pos_u8_t *str, pos_u32_t len) {
  static pos_u8_t _hex16_char[] = "0123456789ABCDEF";
  pos_u8_t ck = 0;
  pos_u32_t i;
  for( i = 1; i < len - 5; i++ )
    ck ^= str[i];
  if( str[i+1] != _hex16_char[(ck>>4)&0xf] ||
      str[i+2] != _hex16_char[ck&0xf] ) {
    return POS_STATUS_ERROR;
  }
  return POS_STATUS_OK;
}

/** 
* Sensor gps json put
*/
pos_status_t sensor_gps_json_put(pos_u32_t v) {
  drv_api_t *drv = g_drv;  
  if( v & 0x80000000 ) {
    v &= 0x7fffffff;
    drv->data->put_raw("-",1);
  }
  v = v * 5 / 18; /* translate from 0.001s to 0.000001degree */
  return drv->data->put_ufloat(v, 1000000);
}

/** 
* Sensor collecting
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_gps_collect(void){
  pos_io_handle_t *io;
  pos_status_t ret = POS_STATUS_E_TIMEOUT;
  pos_u32_t addr, v;
  pos_u16_t i, j;
  drv_api_t *drv = g_drv;
  pos_u8_t *buf;
  
  /* 初始化串口 */
  i = drv->s->slot->io;
  addr = drv->s->slot->rsvd32;
  
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("gps io", i);        
    drv->log->data("gps srsv", addr);            
  }  
  i &= 0xff; /* keep io for uart only */  
  if( POS_IO_IS_SERIAL_PORT(i) ) {
  } else
    i = 0;
  v = (addr >> 8)&0xffffff;
  if( !v ) {    
    v = 9600; /* 默认用9600 */     
  }
  io = drv->os->io->init(i, v, 1); /* use IRQ mode */
  
  buf = drv->os->malloc(DRV_GPS_BUF_LEN);
  if( buf ) {
    pos_u32_t to;
    pos_u32_t id_start, id_end;    
    to = drv->os->tick->get() + (addr&0xff)*5000; /* searching time in unit of 5s (5000ms) */
    do {
      for( j = 0; j < 10; j++ ) {
#if 1
        v = io->read(io, buf, DRV_GPS_BUF_LEN, 400);
#else
#if 0
        static const char _dbg[]="$GPGSV,1,1,00*79\r\n\
  $GNGLL,3106.59337,N,12134.00610,E,161059.00,A,A*70\r\n\
  $GPGSA,A,1,,,,,,,,,,,,,25.5,25.5,25.5*02\r\n\
  $BDGSA,A,1,,,,,,,,,,,,,25.5,25.5,25.5*13\r\n\
  $GPGSV,1,1,00*79\r\n";
#endif
static const char _dbg[]="\
$GNRMC,123659.000,V,3439.3609,N,11220.9572,E,000.0,000.0,270824,,,N*61\r\n\
$GNGGA,123659.200,3439.3375,N,11220.9468,E,1,07,1.6,354.6,M,-21.7,M,,0000*6B\r\n\
$GNRMC,123659.200,A,3439.3375,N,11220.9468,E,000.0,000.0,270824,,,A*7F\r\n\
$GNGGA,123659.400,3439.3375,N,11220.9465,E,1,08,1.3,354.6,M,-21.7,M,,0000*6A\r\n\
$GNGLL,3439.3376,N,11220.9463,E,123659.800,A,A*41\r\n\
$GNGGA,123659.800,3439.3376,N,11220.9463,E,1,08,1.3,351.3,M,-21.7,M,,0000*63\r\n\
";
        v = drv->os->strlen(_dbg);
        drv->os->memcpy(buf, _dbg, v);      
        
#endif
        buf[v] = 0;

        if( v && (drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG) != 0 ) {
/*          drv->log->buf("rx", buf, v); */
          drv->os->puts((char*)buf);
          drv->os->flush();
        }

        if( sensor_gps_find_gll(buf, v, &id_start, &id_end) )
          continue; /* data is not got */
        if( sensor_gps_cksum_check(&buf[id_start], id_end-id_start+1) == POS_STATUS_OK )
          break; /* searched GLL with valid checksum */
      }
      if( j < 10 ) {
        pos_u32_t latitude, longitude;
/*         drv->os->printf("Find GLL: %u %u\n", id_start, id_end); */
        if( (buf[id_end-5] != 'A' && buf[id_end-5] != 'D' && buf[id_end-5] != 'E') || buf[id_end-7] != 'A' )
          continue; /* no valid data got (gps not locked) */
        
        latitude = sensor_gps_get_u32_by_index(&buf[id_start], id_end - id_start, 3, 0);
        longitude = sensor_gps_get_u32_by_index(&buf[id_start], id_end - id_start, 3, 2);
        drv->os->printf("latitude=0x%x\r\n",latitude);        
        drv->os->printf("longitude=0x%x\r\n",longitude);
        if( drv->data->plss_put_u32(PLSS_PL_GPS, latitude) == POS_STATUS_OK ) {
          v = POS_NTOHL(longitude);
          drv->data->plss_put_raw(4, &v);
        }
        drv->data->put_raw("\"long\":",7);
        sensor_gps_json_put(longitude);
        drv->data->put_raw("\"lati\":",7);
        sensor_gps_json_put(latitude);
        ret = POS_STATUS_OK;
        break;
      }
    } while( !drv->os->tick->is_timeout(drv->os->tick->get(), to));
  }
  
  if( ret != POS_STATUS_OK ) {   
    /* 错误的Sensor, 上报出错 */
    drv->data->put("\"%s\":\"no gps#0x%x/0x%x\",", "err", i, addr, 0, 0);
    v = 0xffffffff;
    if( drv->data->plss_put_u32(PLSS_PL_GPS, v) == POS_STATUS_OK ) {
        drv->data->plss_put_raw(4, &v);
    }
  }

  /* Free buffer before quit */
  if( buf ) {
    drv->os->free(buf);
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
  .version = DRV_SENSOR_VERSION(MA_SENSOR_GPS, DRV_GPS_VERSION),
  .name = DRV_GPS_NAME,
  .u.sensor={
    .power_set = sensor_gps_set,
    .collect = sensor_gps_collect,
  },
};

