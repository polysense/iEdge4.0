/**
 * @file  drv_rfid.c
 * @brief RFID SI522 driver
 * @author Runby F.
 * @date 2022-4-28
 * @copyright Polysense
 */

#include "drv_api.h"
#include "drv_mtepd.h"

#define RFCfgReg_Val  0x68    /**< Config */

//********************************************//
//MF522寄存器定义
//********************************************//
//Page 0: Command and status
#define    RFU00                0x00  /**< RSVD */
#define    CommandReg           0x01  /**< Command */
#define    ComIEnReg            0x02  /**< Irq1EnReg */
#define    DivIEnReg            0x03  /**< Register */
#define    ComIrqReg            0x04  /**< Register */
#define    DivIrqReg            0x05  /**< Register */
#define    ErrorReg             0x06  /**< Register */
#define    Status1Reg           0x07  /**< Register */
#define    Status2Reg           0x08  /**< Register */
#define    FIFODataReg          0x09  /**< Register */
#define    FIFOLevelReg         0x0A  /**< Register */
#define    WaterLevelReg        0x0B  /**< Register */
#define    ControlReg           0x0C  /**< Register */
#define    BitFramingReg        0x0D  /**< Register */
#define    CollReg              0x0E  /**< Register */
#define    ACDConfigReg         0x0F  /**< Register */

//Page 1: Command                  
#define    RFU10                0x10  /**< Register */
#define    ModeReg              0x11  /**< Register */
#define    TxModeReg            0x12  /**< Register */
#define    RxModeReg            0x13  /**< Register */
#define    TxControlReg         0x14  /**< Register */
#define    TxASKReg             0x15  /**< Register */
#define    TxSelReg             0x16  /**< Register */
#define    RxSelReg             0x17  /**< Register */
#define    RxThresholdReg       0x18  /**< Register */
#define    DemodReg             0x19  /**< Register */
#define    RFU1A                0x1A  /**< Register */
#define    RFU1B                0x1B  /**< Register */
#define    MfTxReg              0x1C  /**< Register */
#define    MfRxReg              0x1D  /**< Register */
#define    TypeBReg             0x1E  /**< Register */
#define    SerialSpeedReg       0x1F  /**< Register */
                          
// Page 2: Configuration           
#define    ACDConfigSelReg      0x20  /**< Register */
#define    CRCResultRegH        0x21  /**< Register */
#define    CRCResultRegL        0x22  /**< Register */
#define    RFU23                0x23  /**< Register */
#define    ModWidthReg          0x24  /**< Register */
#define    RFU25                0x25  /**< Register */
#define    RFCfgReg             0x26  /**< Register */
#define    GsNReg               0x27  /**< Register */
#define    CWGsPReg             0x28  /**< Register */
#define    ModGsPReg            0x29  /**< Register */
#define    TModeReg             0x2A  /**< Register */
#define    TPrescalerReg        0x2B  /**< Register */
#define    TReloadRegH          0x2C  /**< Register */
#define    TReloadRegL          0x2D  /**< Register */
#define    TCounterValueRegH    0x2E  /**< Register */
#define    TCounterValueRegL    0x2F  /**< Register */
                          
//Page 3: Test Register           
#define    RFU30                0x30  /**< Register */
#define    TestSel1Reg          0x31  /**< Register */
#define    TestSel2Reg          0x32  /**< Register */
#define    TestPinEnReg         0x33  /**< Register */
#define    TestPinValueReg      0x34  /**< Register */
#define    TestBusReg           0x35  /**< Register */
#define    AutoTestReg          0x36  /**< Register */
#define    VersionReg           0x37  /**< Register */
#define    AnalogTestReg        0x38  /**< Register */
#define    TestDAC1Reg          0x39  /**< Register */
#define    TestDAC2Reg          0x3A  /**< Register */
#define    TestADCReg           0x3B  /**< Register */
#define    RFU3C                0x3C  /**< Register */
#define    RFU3D                0x3D  /**< Register */
#define    RFU3E                0x3E  /**< Register */
#define    RFU3F                0x3F  /**< Register */


/////////////////////////////////////////////////////////////////////
//MF522命令字
/////////////////////////////////////////////////////////////////////
#define PCD_IDLE              0x00    /**< 取消当前命令 */
#define PCD_AUTHENT           0x0E    /**< 验证密钥 */
#define PCD_RECEIVE           0x08    /**< 接收数据 */
#define PCD_TRANSMIT          0x04    /**< 发送数据 */
#define PCD_TRANSCEIVE        0x0C    /**< 发送并接收数据 */
#define PCD_RESETPHASE        0x0F    /**< 复位 */
#define PCD_CALCCRC           0x03    /**< CRC计算 */



/////////////////////////////////////////////////////////////////////
//MF522 FIFO长度定义
/////////////////////////////////////////////////////////////////////
#define DEF_FIFO_LENGTH       64      /**< FIFO size=64byte */
#define MAXRLEN  18                   /**< MAX RX Length */

/////////////////////////////////////////////////////////////////////
//Mifare_One卡片命令字
/////////////////////////////////////////////////////////////////////
#define PICC_REQIDL           0x26    /**< 寻天线区内未进入休眠状态 */
#define PICC_REQALL           0x52    /**< 寻天线区内全部卡 */
#define PICC_ANTICOLL1        0x93    /**< 防冲撞 */
#define PICC_ANTICOLL2        0x95    /**< 防冲撞 */
#define PICC_ANTICOLL3        0x97    /**< 防冲撞 */

#define PICC_AUTHENT1A        0x60    /**< 验证A密钥 */
#define PICC_AUTHENT1B        0x61    /**< 验证B密钥 */
#define PICC_READ             0x30    /**< 读块 */
#define PICC_WRITE            0xA0    /**< 写块 */
#define PICC_DECREMENT        0xC0    /**< 扣款 */
#define PICC_INCREMENT        0xC1    /**< 充值 */
#define PICC_RESTORE          0xC2    /**< 调块数据到缓冲区 */
#define PICC_TRANSFER         0xB0    /**< 保存缓冲区中数据 */
#define PICC_HALT             0x50    /**< 休眠 */

#define RFID_BLOCK_ID         1       /**< 数据块ID */

/** 
 * RFID总线IO写
 */ 
void drv_rfid_mtepd_io_write(pos_io_handle_t *io, pos_u8_t reg, pos_u8_t value) {
  reg = (reg & 0x3f) << 1;
  io->write(io, &value, 1, POS_IO_WAIT_SPI_CMD(reg,0));
}

/** 
 * RFID总线IO读
 */ 
pos_u8_t drv_rfid_mtepd_io_read(pos_io_handle_t *io, pos_u8_t reg) {
  pos_u8_t value;
  reg = ((reg & 0x3f) << 1) | 0x80;
  io->read(io, &value, 1, POS_IO_WAIT_SPI_CMD(reg,0));
  return value;
}

/** 
 * RFID总线IO清除掩码Bit
 */ 
void drv_rfid_mtepd_io_clr(pos_io_handle_t *io, pos_u8_t reg, pos_u8_t mask) {
  pos_u8_t tmp;
  tmp = drv_rfid_mtepd_io_read(io, reg);
  drv_rfid_mtepd_io_write(io, reg, tmp & ~mask);  // clear bit mask
}

/** 
 * RFID总线IO设置掩码Bit
 */ 
void drv_rfid_mtepd_io_set(pos_io_handle_t *io, pos_u8_t reg, pos_u8_t mask) {
  pos_u8_t tmp;
  tmp = drv_rfid_mtepd_io_read(io, reg);
  drv_rfid_mtepd_io_write(io, reg, tmp | mask);  // set bit mask
}

/**
 * 通过RC522和ISO14443卡通讯
 * @param[in] io 总线操作句柄 
 * @param[in] command RC522命令字
 * @param[in] p_in_data 通过RC522发送到卡片的数据
 * @param[in] in_len_byte 发送数据的字节长度
 * @param[out] p_out_data 接收到的卡片返回数据
 * @param[out] p_out_len_bit 返回数据的位长度
 * @return     0: 执行成功\n
             非0: 失败，详见@ref pos_status_t
 */
pos_status_t drv_rfid_mtepd_pcd_com522(pos_io_handle_t *io, 
  pos_u8_t command, 
  pos_u8_t *p_in_data, 
  pos_u8_t in_len_byte,
  pos_u8_t *p_out_data, 
  pos_u32_t *p_out_len_bit){
  
  const pos_lib_tick_t *tick = g_drv->os->tick;
  pos_status_t status = POS_STATUS_ERROR;
  pos_u8_t irqEn   = 0x00;
  pos_u8_t waitFor = 0x00;
  pos_u8_t lastBits;
  pos_u8_t n;
  pos_u32_t i;
  switch (command)    {
  case PCD_AUTHENT:
    irqEn   = 0x12;
    waitFor = 0x10;
    break;
  case PCD_TRANSCEIVE:
    irqEn   = 0x77;
    waitFor = 0x30;
    break;
  default:
    break;
  }
 
  //drv_rfid_mtepd_io_write(io, ComIEnReg,irqEn|0x80);
  drv_rfid_mtepd_io_clr(io, ComIrqReg,0x80);
  drv_rfid_mtepd_io_write(io, CommandReg,PCD_IDLE);
  drv_rfid_mtepd_io_set(io, FIFOLevelReg,0x80);
  
  for( i = 0; i < in_len_byte; i++ )    {   
    drv_rfid_mtepd_io_write(io, FIFODataReg, p_in_data[i]);    
    
  }
  drv_rfid_mtepd_io_write(io, CommandReg, command);
 
  if (command == PCD_TRANSCEIVE)  {    
    drv_rfid_mtepd_io_set(io, BitFramingReg,0x80);  
    
  }
  
  //操作M1卡最大等待时间25ms
//  g_drv->os->task->lock();
  i = tick->get() + 25; 
  do {
    n = drv_rfid_mtepd_io_read(io, ComIrqReg);
    if( tick->is_timeout(tick->get(), i) )
      break;
    
  } while ( (n&0x01) == 0 && (n&waitFor) == 0 );
//  g_drv->os->task->unlock();  
  drv_rfid_mtepd_io_clr(io, BitFramingReg,0x80);

#if 0
  if( g_drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) 
    g_drv->log->data("rfid pcd ComIrqReg", n); 
#endif

  do {
    if( (n&0x01) == 0 && (n&waitFor) == 0 )
      break;
  
    drv_rfid_mtepd_io_read(io, ErrorReg);

    i = drv_rfid_mtepd_io_read(io, ErrorReg);
#if 0    
    if( g_drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) 
      g_drv->log->data("rfid pcd ErrorReg", i); 
#endif    
    if( i&0x1B )    
      break;
    
    status = POS_STATUS_OK;
    if (n & irqEn & 0x01)        {   
      status = POS_STATUS_E_NOT_FOUND; //MI_NOTAGERR;   
      
    }

    if( command != PCD_TRANSCEIVE )        
      break;
      
    n = drv_rfid_mtepd_io_read(io, FIFOLevelReg);
    lastBits = drv_rfid_mtepd_io_read(io, ControlReg) & 0x07;
    if (n == 0)      {   
      n = 1;    
      
    } else if (n > MAXRLEN) {   
      n = MAXRLEN;   
      
    }
    
    for (i=0; i<n; i++) {   
      p_out_data[i] = drv_rfid_mtepd_io_read(io, FIFODataReg);
      
    }
    
    if (lastBits)  {   
      *p_out_len_bit = (n-1)*8 + lastBits;   
      
    } else      {   
      *p_out_len_bit = n*8;   

    }
    
  }while(0);
 
  drv_rfid_mtepd_io_set(io, ControlReg,0x80);           // stop timer now
  drv_rfid_mtepd_io_write(io, CommandReg,PCD_IDLE); 
  return status;
}

/*
 * PCD发送寻卡请求
 * @param[in] io 总线操作句柄
 * @param[in] req_code 寻卡方式\n
                 0x52 = 寻感应区内所有符合14443A标准的卡\n
                 0x26 = 寻未进入休眠状态的卡
 * @param[out] p_tag_type 卡片类型代码\n
                 0x4400 = Mifare_UltraLight\n
                 0x0400 = Mifare_One(S50)\n
                 0x0200 = Mifare_One(S70)\n
                 0x0800 = Mifare_Pro(X)\n
                 0x4403 = Mifare_DESFire
 * @return     0: 执行成功\n
              非0: 失败，详见@ref pos_status_t
 */
pos_status_t drv_rfid_mtepd_pcd_request(pos_io_handle_t *io, pos_u8_t req_code, pos_u16_t *p_tag_type) {
  pos_status_t status;  
  pos_u32_t unLen;
  pos_u8_t ucComMF522Buf[MAXRLEN]; 

  drv_rfid_mtepd_io_clr(io, Status2Reg,0x08);
  drv_rfid_mtepd_io_write(io, BitFramingReg,0x07);
  drv_rfid_mtepd_io_set(io, TxControlReg,0x03);
 
  ucComMF522Buf[0] = req_code;

  status = drv_rfid_mtepd_pcd_com522(io, PCD_TRANSCEIVE, ucComMF522Buf, 1, ucComMF522Buf, &unLen);
  if( status == POS_STATUS_OK &&
      unLen == 0x10 )  {    
    *p_tag_type     = (ucComMF522Buf[0]<<8) + ucComMF522Buf[1];
    
  }
  else  {       
    if( status == POS_STATUS_OK )
      status = POS_STATUS_E_PARAMETER;   
    
  }
   
  return status;
}


/*
 * PCD防冲撞检测
 * @param[in] io 总线操作句柄
 * @param[out] p_snr 卡片序列号，4字节
 * @param[in] anticollision_level 防冲撞级别
 * @return     0: 执行成功\n
              非0: 失败，详见@ref pos_status_t
 */
pos_status_t drv_rfid_mtepd_pcd_anticoll(pos_io_handle_t *io, pos_u8_t *pSnr, pos_u8_t anticollision_level){
  pos_status_t status;
  pos_u8_t i,snr_check=0;
  pos_u32_t unLen;
  pos_u8_t ucComMF522Buf[MAXRLEN]; 
  

  drv_rfid_mtepd_io_clr(io, Status2Reg,0x08);
  drv_rfid_mtepd_io_write(io, BitFramingReg,0x00);
  drv_rfid_mtepd_io_clr(io, CollReg,0x80);

  ucComMF522Buf[0] = anticollision_level;
  ucComMF522Buf[1] = 0x20;

  status = drv_rfid_mtepd_pcd_com522(io, PCD_TRANSCEIVE, ucComMF522Buf, 2, ucComMF522Buf, &unLen);

  if( status == POS_STATUS_OK )  {
    for (i=0; i<4; i++) {   
      pSnr[i]  = ucComMF522Buf[i];
      snr_check ^= ucComMF522Buf[i];
      
    }
    if( snr_check != ucComMF522Buf[i] )   {   
      status = POS_STATUS_E_RESOURCE;
    }
  }
    
  drv_rfid_mtepd_io_set(io, CollReg,0x80);
  return status;
}

/*
 * CRC计算
 * @param[in] io 总线操作句柄
 * @param[in] pIndata 输入数据
 * @param[in] len 输入数据长度
 * @param[out] pOutData CRC输出数据
 */
void drv_rfid_mtepd_calc_crc(pos_io_handle_t *io, pos_u8_t *pIndata,pos_u8_t len,pos_u8_t *pOutData){
  pos_u32_t i, n;
  const pos_lib_tick_t *tick = g_drv->os->tick;    
  drv_rfid_mtepd_io_clr(io, DivIrqReg,0x04);
  drv_rfid_mtepd_io_write(io, CommandReg,PCD_IDLE);
  drv_rfid_mtepd_io_set(io, FIFOLevelReg,0x80);
  for (i=0; i<len; i++)
  {   
		drv_rfid_mtepd_io_write(io, FIFODataReg, *(pIndata+i));   
	}
  drv_rfid_mtepd_io_write(io, CommandReg, PCD_CALCCRC);
  i = tick->get() + 50;
//  g_drv->os->task->lock();  
  do {
    n = drv_rfid_mtepd_io_read(io, DivIrqReg);
    if( tick->is_timeout(tick->get(), i) )
      break;
  } while( (n&0x04) == 0 );
//  g_drv->os->task->unlock();  
  pOutData[0] = drv_rfid_mtepd_io_read(io, CRCResultRegL);
  pOutData[1] = drv_rfid_mtepd_io_read(io, CRCResultRegH);
}

/*
 * PCD防冲撞检测
 * @param[in] io 总线操作句柄
 * @param[in] p_snr 卡片序列号，4字节
 * @param[out] sak SAK
 * @return     0: 执行成功\n
              非0: 失败，详见@ref pos_status_t
 */
pos_status_t drv_rfid_mtepd_pcd_select1 (pos_io_handle_t *io, pos_u8_t * pSnr, pos_u8_t *sak) {
  pos_status_t status;
  pos_u8_t i;
  unsigned int unLen;
  pos_u8_t ucComMF522Buf[MAXRLEN]; 
  
  ucComMF522Buf[0] = PICC_ANTICOLL1;
  ucComMF522Buf[1] = 0x70;
  ucComMF522Buf[6] = 0;
  for (i=0; i<4; i++)    {
  	ucComMF522Buf[i+2] = *(pSnr+i);
  	ucComMF522Buf[6]  ^= *(pSnr+i);
  }

#if 0
  g_drv->log->data("crc16",g_drv->os->crc(POS_CRC16, ucComMF522Buf, 7));
  g_drv->log->data("crc16_mbus",g_drv->os->crc(POS_CRC16_MBUS, ucComMF522Buf, 7));  
#endif  
  drv_rfid_mtepd_calc_crc(io, ucComMF522Buf,7,&ucComMF522Buf[7]);
//  g_drv->log->buf("crc16_rfid", &ucComMF522Buf[7], 2);  

  drv_rfid_mtepd_io_clr(io, Status2Reg,0x08);

  status = drv_rfid_mtepd_pcd_com522(io, PCD_TRANSCEIVE, ucComMF522Buf, 9, ucComMF522Buf, &unLen);
  
  if( status == POS_STATUS_OK && unLen == 0x18 )  {   
  	*sak = ucComMF522Buf[0];
    
  }  else  {   
  	status = POS_STATUS_ERROR;
    
  }

  return status;
}

/*
 * PCD验证密码
 * @param[in] io 总线操作句柄
 * @param[in] auth_mode 验证模式
 * @param[in] addr 块地址
 * @param[out] pData 读出的数据，16字节
 * @return     0: 执行成功\n
              非0: 失败，详见@ref pos_status_t
 */
pos_status_t drv_rfid_mtepd_pcd_auth(pos_io_handle_t *io, pos_u8_t auth_mode, pos_u8_t addr, const pos_u8_t *key, pos_u8_t *snr)
{
  pos_status_t status;
  unsigned int unLen;
  pos_u8_t ucComMF522Buf[MAXRLEN]; 

  ucComMF522Buf[0] = auth_mode;
  ucComMF522Buf[1] = addr;
  g_drv->os->memcpy(&ucComMF522Buf[2], key, 6);  
  g_drv->os->memcpy(&ucComMF522Buf[8], snr, 4);    
 
  status = drv_rfid_mtepd_pcd_com522(io, PCD_AUTHENT, ucComMF522Buf, 12, ucComMF522Buf, &unLen);
  if( status != POS_STATUS_OK )  {   
  	return status;    
  }  

  if( (drv_rfid_mtepd_io_read(io, Status2Reg) & 0x08) == 0 ){
  	status = POS_STATUS_E_RESOURCE;    
  }

  return status;
}

/*
 * PCD读取M1卡一块数据
 * @param[in] io 总线操作句柄
 * @param[in] addr 块地址
 * @param[out] pData 读出的数据，16字节
 * @return     0: 执行成功\n
              非0: 失败，详见@ref pos_status_t
 */
pos_status_t drv_rfid_mtepd_pcd_read(pos_io_handle_t *io, pos_u8_t addr,pos_u8_t *pData)
{
  pos_status_t status;
  unsigned int unLen;
  pos_u8_t ucComMF522Buf[MAXRLEN]; 

  ucComMF522Buf[0] = PICC_READ;
  ucComMF522Buf[1] = addr;
  drv_rfid_mtepd_calc_crc(io, ucComMF522Buf, 2, &ucComMF522Buf[2]);
 
  status = drv_rfid_mtepd_pcd_com522(io, PCD_TRANSCEIVE, ucComMF522Buf, 4, ucComMF522Buf, &unLen);
  if( status == POS_STATUS_OK && unLen == 0x90 )	{   
	  g_drv->os->memcpy(pData, ucComMF522Buf, 16);  
    
  } else  if( status == POS_STATUS_OK ) {   
    if( g_drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) 
      g_drv->log->data("rfid pcd unLen", unLen);    
  	status = POS_STATUS_E_RESOURCE;
    
  }
  
  return status;
}


/*
 * PCD写入M1卡一块数据
 * @param[in] io 总线操作句柄
 * @param[in] addr 块地址
 * @param[in] pData 写入的数据，16字节
 * @return     0: 执行成功\n
              非0: 失败，详见@ref pos_status_t
 */
pos_status_t drv_rfid_mtepd_pcd_write(pos_io_handle_t *io, pos_u8_t addr,pos_u8_t *pData)
{
  pos_status_t status;
  unsigned int unLen, i;
  pos_u8_t ucComMF522Buf[MAXRLEN]; 

  ucComMF522Buf[0] = PICC_WRITE;
  ucComMF522Buf[1] = addr;
  drv_rfid_mtepd_calc_crc(io, ucComMF522Buf, 2, &ucComMF522Buf[2]);
 
  status = drv_rfid_mtepd_pcd_com522(io, PCD_TRANSCEIVE, ucComMF522Buf, 4, ucComMF522Buf, &unLen);
  if( status == POS_STATUS_OK && unLen == 4 && (ucComMF522Buf[0] & 0x0f) == 0x0a )	{   
    for( i = 0; i < 16; i++ )
      ucComMF522Buf[i] = pData[i];
    drv_rfid_mtepd_calc_crc(io, ucComMF522Buf, 16, &ucComMF522Buf[16]);
    status = drv_rfid_mtepd_pcd_com522(io, PCD_TRANSCEIVE, ucComMF522Buf, 18, ucComMF522Buf, &unLen);    
  } else {
    if( g_drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
      g_drv->log->data("rfid pcd write status", status);    
      g_drv->log->data("rfid pcd write unLen", unLen);          
      g_drv->log->data("rfid pcd write Buf[0]", ucComMF522Buf[0]); 
    }
  }
  
  if( status != POS_STATUS_OK || unLen != 4 || (ucComMF522Buf[0] & 0x0f) != 0x0a )	{   
    if( status == POS_STATUS_OK ) {   
      if( g_drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) {
        g_drv->log->data("rfid pcd tx2 unLen", unLen);    
        g_drv->log->data("rfid pcd tx2 Buf[0]", ucComMF522Buf[0]); 
      }
    	status = POS_STATUS_E_RESOURCE;    
    }
  }
  
  return status;
}

/** 
* 上电并初始化驱动
* @param[out] buf 读出的数据，16字节
* @return     0: 执行成功\n
             非0: 失败，详见@ref pos_status_t
*/
pos_status_t drv_rfid_mtepd_read(pos_u8_t *buf) {
  pos_u16_t ATQA;
  pos_u32_t UID;
  pos_u8_t SAK = 0;
//  const pos_u8_t CardWriteBuf[16] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
  const pos_u8_t DefaultKeyABuf[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
  const drv_api_t *drv = g_drv;
  const pos_lib_t *os = drv->os;
  pos_io_handle_t *io;
  pos_status_t status;
  
  /* GET SPI */
  io = os->io->get(MB_RFID_SPI);
  if( !io )
    return POS_STATUS_E_INIT;
#if 0
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) 
    drv->log->data("rfid read",0);  
#endif  
  //request 寻卡
  status = drv_rfid_mtepd_pcd_request(io, PICC_REQIDL, &ATQA);
  if( status != POS_STATUS_OK )  //寻天线区内未进入休眠状态的卡，返回卡片类型 2字节  
  {
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) 
      drv->log->data("rfid scan err", status);  
    return status;
  }

#if 0  
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) 
    drv->log->data("rfid scan ok", ATQA);  
#endif

  //Anticoll 冲突检测
  UID=0;
  status = drv_rfid_mtepd_pcd_anticoll(io, (pos_u8_t*)&UID, PICC_ANTICOLL1);
  if( status != POS_STATUS_OK ) {        
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) 
      drv->log->data("rfid coll err", status);      
    return status;
    
  }

#if 0    
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) 
    drv->log->data("rfid coll ok", UID);  
#endif
  
  //Select 选卡
  status = drv_rfid_mtepd_pcd_select1(io, (pos_u8_t*)&UID,&SAK);
  if( status != POS_STATUS_OK )  {
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) 
      drv->log->data("rfid sel failed", SAK);      
    return status;
    
  }

#if 0    
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) 
    drv->log->data("rfid sel ok", SAK);  
#endif

  //Authenticate 验证密码
  status = drv_rfid_mtepd_pcd_auth(io, PICC_AUTHENT1A, RFID_BLOCK_ID, DefaultKeyABuf, (pos_u8_t*)&UID);  
  if( status != POS_STATUS_OK )  {
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) 
      drv->log->data("rfid auth failed", status);      
    return status;
    
  }
#if 0  
  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) 
    drv->log->data("rfid auth ok", 0);  
#endif

  //读BLOCK原始数据 
  status = drv_rfid_mtepd_pcd_read(io, RFID_BLOCK_ID, buf); 
  if( status != POS_STATUS_OK ) {
    if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) 
      drv->log->data("rfid read failed", status); 
    return status;
    
  }

  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) 
    drv->log->buf( "rfid read ok", buf, 16 );   
  
  return POS_STATUS_OK;
}

/** 
 * RFID driver init
 * @return     0: Successful\n
             !=0: Failed, refer to @ref pos_status_t
 */ 
pos_status_t drv_rfid_mtepd_init(void) {
  const drv_api_t *drv = g_drv;
  const pos_lib_t *os = drv->os;
  pos_io_handle_t *io;
  pos_gpio_pin_t pin_rst, pwr; 
  pos_u32_t id;
  /* 再初始化SPI */
  io = os->io->init(MB_RFID_SPI, MB_RFID_SPI_SPEED, MB_RFID_SPI_MODE);
  if( !io )
    return POS_STATUS_E_INIT;


  id = drv->s->slot->io;
  pwr = (id >> 8) & 0x7f;
  id &= 0x80ff;
  if( !id )
    id = MB_CTRL_ID_RFID_RST;
  if( !pwr )
    pwr = MB_PWR_ID_VCCN_PWR3;
  pin_rst = drv->board->pin_map(MB_PT_CTRL, id); 
  
  /* 先拉高RST并上电 */
  os->gpio->mode_set(pin_rst, POS_GPIO_MODE_OUTPUT);
  os->gpio->set(pin_rst, 1);
  drv->board->pwr_set(pwr, 1);  
  os->tick->sleep(50);

  if( drv->cfg->ctrl & MA_CFG_CTRL_DRV_LOG ) 
    drv->log->data( "rfid init vesion", drv_rfid_mtepd_io_read(io, VersionReg) );

#if 0
  /* debug code to read version*/
  while(1)
    os->tick->sleep(6000000);
#endif

  drv_rfid_mtepd_io_clr(io, Status2Reg, 0x08);  
#if 1
  // Reset baud rates
  drv_rfid_mtepd_io_write(io, TxModeReg, 0x00);
  drv_rfid_mtepd_io_write(io, RxModeReg, 0x00);
  // Reset ModWidthReg
  drv_rfid_mtepd_io_write(io, ModWidthReg, 0x26);
  // RxGain:110,43dB by default;
  drv_rfid_mtepd_io_write(io, RFCfgReg, RFCfgReg_Val);
  // When communicating with a PICC we need a timeout if something goes wrong.
  // f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
  // TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.
  drv_rfid_mtepd_io_write(io, TModeReg, 0x80);// TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
  drv_rfid_mtepd_io_write(io, TPrescalerReg, 0xa9);// TPreScaler = TModeReg[3..0]:TPrescalerReg
  drv_rfid_mtepd_io_write(io, TReloadRegH, 0x03); // Reload timer 
  drv_rfid_mtepd_io_write(io, TReloadRegL, 0xe8); // Reload timer 
  drv_rfid_mtepd_io_write(io, TxASKReg, 0x40);  // Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
  drv_rfid_mtepd_io_write(io, ModeReg, 0x3D);  // Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
  drv_rfid_mtepd_io_write(io, CommandReg, 0x00);  // Turn on the analog part of receiver   
#else
  drv_rfid_mtepd_io_write(io, ModeReg, 0x3D);  // Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
  drv_rfid_mtepd_io_write(io, TReloadRegL, 30); // Reload timer 
  drv_rfid_mtepd_io_write(io, TReloadRegH, 0); // Reload timer 
  drv_rfid_mtepd_io_write(io, TModeReg, 0x8D);// TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
  drv_rfid_mtepd_io_write(io, TPrescalerReg, 0x3e);// TPreScaler = TModeReg[3..0]:TPrescalerReg
  drv_rfid_mtepd_io_write(io, TxASKReg, 0x40);  // Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting

  drv_rfid_mtepd_io_write(io, CommandReg, 0x00);  // Turn on the analog part of receiver   

#endif

  //PcdAntennaOn()
  drv_rfid_mtepd_io_set(io, TxControlReg, 0x03);

  return POS_STATUS_OK;
}

/** 
 * 断电RFID
 * @return     0: 执行成功\n
             非0: 失败，详见@ref pos_status_t
 */ 
pos_status_t drv_rfid_mtepd_done(void) {
  const drv_api_t *drv = g_drv;
  const pos_lib_t *os = drv->os;
  pos_gpio_pin_t pin_rst; 
  mb_pwr_id_t pwr;
  pos_u32_t id;
  drv->os->io->done(MB_RFID_SPI);

  id = drv->s->slot->io;
  pwr = (id >> 8) & 0x7f;
  id &= 0x80ff;
  if( !id )
    id = MB_CTRL_ID_RFID_RST;
  if( !pwr )
    pwr = MB_PWR_ID_VCCN_PWR3;
  pin_rst = drv->board->pin_map(MB_PT_CTRL, id); 
  os->gpio->mode_set(pin_rst, POS_GPIO_MODE_ANALOG_INPUT_PD);
  drv->board->pwr_set(pwr, 0);  
  return POS_STATUS_OK;
}

