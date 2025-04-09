/**
 * @file  drv_myuart.c
 * @brief SENSOR driver for generic UART example
 * @author Runby F.
 * @date 2025-4-8
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * Revision history
v1.0 @ 2025-04-08
  1) First revision

*/

/**
 * @brief Driver version
 */
#define DRV_MYUART_VERSION 0x0100

/**
 * @brief Driver name
 */
#define DRV_MYUART_NAME  "MYUART"

/**
 * @brief Operating timeout in unit ms
 */
#define DRV_UART_IO_TIMEOUT  100

/**
 * @brief Operating timeout retry times
 */
#define DRV_UART_RETRY_NUM  10

/** 
* Power control
* @param[in]  on   0:Power off, 1:Power on
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
* @note If sensor does NOT require any power control, this fucntion can be NULL
*/ 
pos_status_t sensor_myuart_set(pos_u32_t on) {
  ma_board_t *b = g_drv->board;
  b->pwr_set(MB_PWR_ID_VCCN_PWR3, on);
  
  return POS_STATUS_OK;
}

/** 
* Sensor collecting
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_myuart_collect(void){
  pos_io_handle_t *io;
  pos_status_t ret = POS_STATUS_E_TIMEOUT;
  pos_u32_t v, sum, cnt, max, min;
  pos_u16_t t;
  drv_api_t *drv = g_drv;
  pos_u8_t response[12];

  /* specify the UART ID */
  t = POS_IO_UART2;

  drv->log->data("myuart io", t);

  /* refresh to normal by default */
  drv->s->ctrl |= MA_SENSOR_CTRL_STAT_NORMAL;
  drv->s->ctrl &= ~MA_SENSOR_CTRL_STAT_ERROR;

  /* init io */
  io = drv->os->io->init(t, 9600, 0);

  /* read and report the average */
  sum = 0;
  cnt = 0;
  max = 0;
  min = 0xffffffff;
  for(t=0; t < DRV_UART_RETRY_NUM; t++ ) {
	const pos_u8_t cmd_read[9] = {0xff, 0x20, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5a};
	/* flush before read */
	while( io->read(io, POS_NULL, 1024, 50) >= 1024 );

	/* pull command */
	io->write(io, (pos_u8_t*)cmd_read, 9, DRV_UART_IO_TIMEOUT);

	/* read response */
	v = io->read(io, response, 9, DRV_UART_IO_TIMEOUT);
	drv->log->buf("RX", response, v);
	if( v != 9 )
	  continue;

	if( response[0] != 0xff && response[1] != 0x86 )
	  continue; /* wrong format */

	/* crc */
	{
	  pos_u32_t i;
	  pos_u8_t c;
	  c = response[0];
	  for( i = 1; i < v; i++ )
		c += response[i];
	  if( c != 0xff ) /* crc check */
	   continue; /* wrong crc */
	}

	v = (response[2]<<8) + response[3];
	cnt++;
	sum += v;

	if( cnt >= 1 )
	  break;

	drv->os->tick->sleep(1000);

  }

  if( !cnt ) {
	/* no valid data got */
	drv->data->put("\"%s\":\"no sensor data\",", "err", 0, 0, 0, 0);
	/* set error flag */
	drv->s->ctrl |= MA_SENSOR_CTRL_STAT_ERROR;
  } else {
	if( cnt > 2 ) {
	  sum = sum - min - max;
	  cnt -= 2;
	}
	v = sum / cnt;
	drv->data->plss_put_u16(61, v); /* PLSS data format put, 61 for PLSS_PL_EXT_O2: U16 in  unit of 0.1% */

	drv->data->put_raw("\"o2\":", 5);
    drv->data->put_ufloat(v, 10);
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
  .version = DRV_SENSOR_VERSION(162, DRV_MYUART_VERSION), /* 162 is registered sensor type */
  .name = DRV_MYUART_NAME,
  .u.sensor={
    .power_set = sensor_myuart_set,
    .collect = sensor_myuart_collect,
  },
};

