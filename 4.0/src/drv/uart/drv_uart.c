/**
 * @file  drv_uart.c
 * @brief UART LCD driver
 * @author Runby F.
 * @date 2023-2-4
 * @copyright Polysense
 */

#include "drv_api.h"

/* 
 * Revision History
v1.0 @ 2023-03-24
  1) Finish first draft

v1.1 @ 2023-11-18
  1) Using new PWR ID solution
  
*/

/**
 * @brief Driver version
 */
#define DRV_UART_VERSION 0x0101

/**
 * @brief Driver name
 */
#define DRV_UART_NAME  "UART"

/**
 * @brief Buffer Size
 */
#define DRV_UART_BUF_SIZE 200


/**
 * @brief IRQ控制结构
 */
typedef struct {
  drv_api_t     drv;      /**< 驱动克隆，指向固定SLOT的驱动指针 */
  pos_io_handle_t *io;    /**< UART驱动IO */
  pos_u8_t      buf[DRV_UART_BUF_SIZE]; /**< UART Buffer */
  pos_u32_t     arg;
} uart_param_t;


/** 
* Sensor collecting
* @return     0: Successful\n
            !=0: Failed, refer to @ref pos_status_t
*/ 
pos_status_t sensor_uart_collect(void){  
#if 0
  uart_param_t *param;
  ma_sensor_ctrl_t *s = g_drv->s;
  const pos_u8_t cmd[] = {0x11, 0x01, 0x01, 0xed};
  
  g_drv->log->buf("uart sim", (pos_u8_t*)cmd, 4);

  param = (uart_param_t *)s->drv_buf;
  param->io->write(param->io, (pos_u8_t*)cmd, 4, 100);
/* 
{
  pos_u8_t buf[32];
  pos_u32_t len;
  len = param->io->read(param->io, buf, sizeof(buf), 1000);
  g_drv->log->buf("uart read", buf, len);
}  
*/
#endif

  return POS_STATUS_OK;
}

/** 
 * Polling callback function
 * @note This function can NOT call g_drv. When polling, g_drv might piont to any sensor drivers. It should call the param stored driver pointer.
 */
void sensor_uart_poll(uart_param_t *p) {
  pos_size_t len;
  const drv_data_api_t *data;
  const pos_lib_t *os;
  pos_u8_t *t, *c;

#if 0
  /* log */
  if( g_drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    g_drv->log->data("uart poll run", (pos_u32_t)p);
    g_drv->log->data("buf", (pos_u32_t)p->buf);    
    g_drv->log->data("io", (pos_u32_t)p->io);
    g_drv->log->data("rsvd5", (pos_u32_t)p->io->rsvd[5]);    
    g_drv->log->data("fifo", ((pos_u32_t*)p->io->rsvd[5])[0]);  
    
    /*
    if( os->tick->is_timeout(os->tick->get(), p->arg) ) {
      p->drv.log->data("uart poll run", (pos_u32_t)p->io);
      p->arg = os->tick->get() + 10000;
    }
    */
  }  
#endif

  if( !p )
    return;
  if( !p->io )
    return;

#if 0
  /* debug code */
  if( 1 ) {
    pos_u32_t * u32 = (pos_u32_t*)p->io->rsvd[5];
    p->drv.log->data("uart io u32", (pos_u32_t)u32);
    p->drv.log->data("uart io write", u32[0]);
    p->drv.log->data("uart io read", u32[1]);    
    p->drv.log->data("uart io cnt", u32[3]);      
  }
#endif

  /* check uart and report */
  len = p->io->read(p->io, p->buf, DRV_UART_BUF_SIZE, 100);  
#if 0
  /* dummy debug code to insert data automatically */
  if( !len ) {
    pos_u32_t t = p->drv.os->tick->get();
    if( (t  & 0xffff ) < 3000 ) {
      for( len = 0; len < 10; len++ )
        p->buf[len]=len;
    }
  }
#endif  
  if( !len ) {
    return;
  }

  /* log */
  if( p->drv.cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    p->drv.log->buf("uart poll data", p->buf, len);
  }  

  /* init var */
  data = p->drv.data;    
  os = p->drv.os;

  /* prepare txt/buf */
  t = os->malloc(DRV_UART_BUF_SIZE*2 + 64);
  if( !t )
    return;

  /* json report */
  os->strcpy((char*)t, "\"uart\":\"");
  c = &t[os->strlen((char*)t)];
  os->bin2str((char*)c, p->buf, len, ENCODE_HEXSTR_LOWER);
  c += len*2;
  os->strcpy((char*)c, "\",");    
  c += 2;    
  data->put_raw(t, c - t); 

  /* plss report */
  data->plss_put_u16(0xfe, 0xd786); /* d7-86-xxx (UART/485 raw payload) */
  data->plss_put_raw(len, p->buf);
  
  /* free buf */
  os->free(t);
}


/** 
 * Driver init
 * @param[in]  load   0:Unload driver, 1:Load driver
 * @return     0: Successful\n
             !=0: Failed, refer to @ref pos_status_t
 * @note This function can be NULL if it does NOT require any init operatoin
 */ 
pos_status_t sensor_uart_init(pos_u32_t load) {
  drv_api_t *drv = g_drv;
  uart_param_t *param;
  pos_u32_t baud, pwr;
  ma_sensor_ctrl_t *s = g_drv->s;
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
    drv->log->data("uart init", load+(s->slot->io<<8));
    drv->log->data("uart baud", s->slot->rsvd32);
  }  
  
  if( load ) {
    param = drv->os->malloc(sizeof(*param));
    if( !param )
      return POS_STATUS_E_MEM;
    drv->os->memset(param, 0, sizeof(*param));

    drv->os->memcpy(&param->drv, drv, sizeof(*drv)); /* clone a g_drv */
    s->drv_buf = param;  /* record to drv_buf */

    /* get io */
    baud = s->slot->rsvd32;
    if( !baud )
      baud = 9600; /* 默认用9600 */
    if( POS_IO_IS_SERIAL_PORT(s->slot->io) ) {
      param->io = drv->os->io->get(s->slot->io);
    } 
    if( !param->io ) {
      param->io = drv->os->io->get(POS_IO_UART0);
    }

    /*
     * turn ON UART pwr 
     */
    pwr = (s->slot->io >> 8) & 0x7f;
    if( !pwr )
      pwr = MB_PWR_ID_VCCN_PWR3;
    drv->board->pwr_set(pwr, 1);
    
    /* 
     * turn ON UART 
     */
    param->io->setup(param->io, baud, 1 ); /* init IRQ mode */
    
    /*
     * turn on poll
     */
    drv->proc->event_register((pos_func_t)drv->s->drv.u.sensor.poll_cb, param);

    /* 
     * Disable Sleep as UART requires working under regular clock 
     */
   drv->os->call(POS_CALL_FUNC_ID_SET_NO_SLEEP, 1);
    
  } else {      
    /*
     * turn off poll
     */
    drv->proc->event_unregister((pos_func_t)drv->s->drv.u.sensor.poll_cb, s->drv_buf);

    /* 
     * turn off UART and release structure
     */
    param = (uart_param_t*)s->drv_buf;
    if( param ) {
      if( param->io )
        param->io->release(param->io);
      drv->os->free(param);
      s->drv_buf = POS_NULL;
    }

    /* 
     * turn OFF UART pwr 
     */
    drv->board->pwr_set(MB_PWR_ID_VCCN_PWR3, 0);
    
    /* 
     * Enable Sleep
     */
    drv->os->call(POS_CALL_FUNC_ID_SET_NO_SLEEP, 0);

  }
  return POS_STATUS_OK;
}

/*
 * Driver export
 */
ma_drv_export_t g_export = {
  .version = DRV_SENSOR_VERSION(MA_SENSOR_UART, DRV_UART_VERSION),
  .name = DRV_UART_NAME,
  .u.sensor={
    .init = sensor_uart_init,
    .collect = sensor_uart_collect,
    .poll_cb = (pos_func_t)sensor_uart_poll,    
  },
};

