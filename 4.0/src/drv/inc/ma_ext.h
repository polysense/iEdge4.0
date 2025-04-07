/**
 * @file  ma_ext.h  
 * @brief MA (main application) external module managment library
 * @author Runby F.
 * @date 2022-2-18
 * @copyright Polysense
 */

#ifndef __MA_EXT_H__
#define __MA_EXT_H__

/**
 * @brief AT command hex string uppercase or lowercase control
 */
#define MA_EXT_HEX_CASE ENCODE_HEXSTR_UPPER

/**
 * @brief External module driver init flag
 */
#define MA_EXT_FLAG_DRV_INIT          0x00000001

/**
 * @brief External module network ready flag
 */
#define MA_EXT_FLAG_MODULE_READY      0x00000002

/**
 * @brief External module send okay flag
 */
#define MA_EXT_FLAG_MODULE_TX_OK      0x00000004

/**
 * @brief External module receive okay flag
 */
#define MA_EXT_FLAG_MODULE_RX_OK      0x00000008

/**
 * @brief External module transparent flag (some modules will use this bit to switch transparent mode)
 */
#define MA_EXT_FLAG_MODULE_TRANS      0x00000010

/**
 * @brief External module socket connecting flag (some modules will use this bit)
 */
#define MA_EXT_FLAG_MODULE_CONNECT    0x00000020

/**
 * @brief External module Power Saving Mode confirmed
 */
#define MA_EXT_FLAG_MODULE_PSM_ACK    0x00000040

/**
 * @brief External module Operate ID for Signal Get
 */
#define MA_EXT_MODULE_OPR_GET_SIGNAL    0x0

/**
 * @brief Print IMEI CIMI (bit[0] of arg32 will define: 0-only print once, 1-print on each function call)
 */
#define MA_EXT_MODULE_OPR_PRINT_IMEI_CIMI 0x01

/**
 * @brief Module Readln. Arg32 will be used as read timeout (ms). Return read length.
 */
#define MA_EXT_MODULE_OPR_READLN 0x02

/**
 * @brief Module dummy send for triggering RAI
 */
#define MA_EXT_MODULE_OPR_DUMMY_TX 0x03

/**
 * @brief Get the id (0 based) U32 from a string (specified in arg32) and return it
 */
#define MA_EXT_MODULE_OPR_GET_U32(id) (0x0100+id)

/**
 * @brief External module mqtt/onenet control structure
 */
typedef struct {
  /**
  * @brief MQTT User
  */
  char *user;

  /**
  * @brief MQTT Password
  */
  char *password;

  /**
  * @brief MQTT Client ID
  */
  char *mcid;

  /**
  * @brief Buf for Client ID
  */  
  char *cid_buf;

  /**
  * @brief Buf for User
  */  
  char *usr_buf;

  /**
  * @brief Subscribe topic
  */
  char topic_sub[92];

  /**
  * @brief Publish topic
  */  
  char topic_pub[88];

  /**
  * @brief Command ARG (for lwm2m OBSERVE ID)
  */  
  pos_u32_t arg[2];
} ma_ext_onenet_ctrl_t;


/**
 * @brief External module control structure
 */
typedef struct {
  /**
  * @brief External module flag (accessible by each module)
  */
  pos_u32_t      flag; 

  /**
  * @brief Buffer length (Shoule reserve one byte for the ending \\0. So actual usable length is buf_size-1.)
  */  
  pos_size_t      buf_size; 

  /**
  * @brief  External module buffer
  */
  pos_u8_t        *buf; 

  /**
  * @brief   Command transmitting ending chars, [0] for length (2 max), [1]/[2] for actual ending bytes, [3] is \\0
  */  
  pos_u8_t        tx_rt[4]; 

  /**
  * @brief   Command receiving ending chars, [0] for length (2 max), [1]/[2] for actual ending bytes, [3] is \\0
  */  
  pos_u8_t        rx_rt[4]; 

  /**
  * @brief   Reporting type used to record the original type during initilization. \n
    Comparing the current EEPROM report type with this value will know report type change and reboot system accordingly.
  */    
  pos_u16_t      report_type; 

  /**
  * @brief   Running level indicator
  */      
  pos_u8_t      run_level; 

  /**
  * @brief   Module IO type
  */      
  pos_u8_t      module_io;

  /**
  * @brief   IO driving handle
  */        
  pos_io_handle_t *io; 

  /**
  * @brief   Externan module driver
  */
  pos_net_t    *net; 

  /**
  * @brief   DNS resolved server IP in network order
  */        
  pos_u8_t       svr_ip[4]; 

  /**
  * @brief   DNS timeout time
  */        
  pos_u32_t     svr_ip_timeout;

  /**
  * @brief Send AT command and query its response
  */
  pos_status_t (*cmd_query)(pos_u8_t* cmd, pos_u32_t timeout);

  /**
  * @brief Send AT command (no need to appending \\r\\n)
  * @note When len is zero, the actual command length will be the actual cmd string length
  */
  pos_status_t (*cmd_send)(pos_u8_t* cmd, pos_size_t len);

  /**
  * @brief Send AT command and check whether it's the expected response.
  * If res is POS_NULL, return successful when AT OK is detected; or else, reutrn successful when res is deteced
  * @note "cmd" can be null. In this case, only response is checked and no commands will be sent.
  */
  pos_status_t (*cmd_query_check)(pos_u8_t* cmd, pos_size_t offset, pos_u8_t *res, pos_u32_t timeout);

  /**
  * @brief Quit AT module transparent mode (using common quiting pattern: delay, +++, delay)
  */
  void (*quit_trans_mode)(void);

  /**
  * @brief External module power on and reset
  */
  void (*module_reset)(void);

  /**
  * @brief External module power set
  * @param[in] on 0: Power off, 1: Power on
  */
  void (*module_power_set)(pos_u32_t on);

  /**
  * @brief Search kw+rt in "buf" from "pos" byte of the buffer. If "rt" is NULL, "rt" will be ignored and only kw is used.
  * @note "kw"/"rt" are both \\0 ending. "rt" can be null pointer if not necessary to compare.
  */
  pos_u8_t * (*buf_search_pos)(pos_u8_t *buf, pos_size_t buf_size, pos_size_t pos, pos_u8_t *kw, pos_size_t kw_len, pos_u8_t *rt);

  /**
  * @brief Flush IO (UART ports) connecting the external module
  */
  void (*io_flush)(void);  

  /**
  * @brief Reserved (ext module might use this to record its private buffer pointer)
  */
  pos_u32_t     rsvd32;

  /**
  * @brief External module virtual driver header
  */
  void          *hdr; 

  /**
  * @brief Physical driver header mapped from virtual address
  */
  ma_drv_export_t drv;

  /**
   * @brief External module auto init control when continuous NACK times reached
   */
  pos_u32_t nack_init_timeout;

  /**
   * @brief External module auto reset control when continuous NACK times reached
   */
  pos_u32_t nack_reset_timeout;

  /**
   * @brief External module MTU length
   */
  pos_u32_t mtu;  

  /**
   * @brief External module recent receiving length
   */
  pos_u32_t rlen;

  /**
   * @brief MQTT/OneNET unified control
   */
  ma_ext_onenet_ctrl_t one;

  /**
  * @brief External module oper
  */
  pos_u32_t (*module_opr)(pos_u32_t opr_id, pos_u32_t arg);
  
} ma_ext_ctrl_t;


#endif
