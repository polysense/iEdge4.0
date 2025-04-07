/**
 * @file  ma_ota.h  
 * @brief OTA implementation librarty
 * @author Runby F.
 * @date 2022-3-21
 * @copyright Polysense
 */

#ifndef __MA_OTA_H__
#define __MA_OTA_H__

#define MA_OTA_FILE_TYPE_NONE   0 /**< Unknown OTA file type*/
#define MA_OTA_FILE_TYPE_DRV    1 /**< Single external module or sensor driver */
#define MA_OTA_FILE_TYPE_MAIN   2 /**< MA firmware */
#define MA_OTA_FILE_TYPE_POS    3 /**< POS firmware */
#define MA_OTA_FILE_TYPE_BIN_CFG   4 /**< Binary configuration file*/
#define MA_OTA_FILE_TYPE_ASC_CFG   5 /**< ASCII configurqation file */
#define MA_OTA_FILE_TYPE_DRV_FULL  6 /**< All-in-one multi-driver package */
#define MA_OTA_FILE_TYPE_DRV_MT    7 /**< MTT related multi-driver package */
#define MA_OTA_FILE_TYPE_ASC_PIN   8 /**< ASCII pin configuration file */
#define MA_OTA_FILE_TYPE_BIN_PIN   9 /**< Binary pin configuration file */


/**
 * @brief OTA request structure (SVR->DEV)
 */
typedef struct {
  pos_u16_t mode;     /**< Bit control definitions:\n
    bit[0] - Perform OTA operation even when version is same\n
    Other bits - Reserved */
  pos_u8_t  svr_src;  /**< Server location:\n
    0 - From Polysense OTA server\n
    1- From current reporting server */  
  pos_u8_t  req_type; /**< Request OTA file type. Refer to @ref MA_OTA_FILE_TYPE_NONE and other definitions */
  union {
    struct {
      pos_u16_t ver16;/**< External module or sensor version:\n
      0 - Latest version\n
      Others - Expected OTA version */
      pos_u16_t type; /**< External module or sensor type. Refer to @ref ma_report_type_t and @ref ma_sensor_type_t */      
    } drv;
    pos_u32_t ver32;  /**< POS or MA version:\n
    0 - Latest version\n
    Others - Expected OTA version */
  } u;                /**< UNION field:\n
  @ref MA_OTA_FILE_TYPE_DRV - Use "drv" field\n
  @ref MA_OTA_FILE_TYPE_MAIN - Use "ver32" field\n
  @ref MA_OTA_FILE_TYPE_POS - Use "ver32" field\n
  Others - Keep drv(ver32) as zero */
} ma_ota_req_t;

/**
 * @brief OTA file content request structure (DEV->SVR)
 */
typedef struct {
  pos_u16_t seg_ofs;  /**< Content offset in 64K segment */
  pos_u8_t  seg_no;   /**< Segment and MTU control:\n
  Bit[0..3] - Segment ID (Each segment is a 64KB block, zero based).\n
  Bit[4..7] - N*100 max trasmitting length in one time (Server can ignore and always use a safe lenght, ie. 100bytes) */
  pos_u8_t  req_type; /**< Request OTA file type. Refer to @ref MA_OTA_FILE_TYPE_NONE and other definitions */
  union {
    struct {
        pos_u16_t ver16;/**< External module or sensor version:\n
        0 - Latest version\n
        Others - Expected OTA version */
        pos_u16_t type; /**< External module or sensor type. Refer to @ref ma_report_type_t and @ref ma_sensor_type_t */      
      } drv;
      pos_u32_t ver32;  /**< POS or MA version:\n
      0 - Latest version\n
      Others - Expected OTA version */
  } u;                /**< UNION field:\n
  @ref MA_OTA_FILE_TYPE_DRV - Use "drv" field\n
  @ref MA_OTA_FILE_TYPE_MAIN - Use "ver32" field\n
  @ref MA_OTA_FILE_TYPE_POS - Use "ver32" field\n
  Others - Keep drv(ver32) as zero */
} ma_ota_req_file_t;

/**
 * @brief OTA file content response strcuture (SVR->DEV)
 */
typedef struct {
  ma_ota_req_file_t hdr; /**< Content header. Refer to @ref ma_ota_req_file_t. Typically, it's almost same the the previos content request from DEV to SVR */
  pos_u16_t ctrl;     /**< Control flags: \n
                      bit[0] - File content is transferred done \n
                      bit[1] - Error. Expect to cancel the OTA process
                      Others - Reserved */
  pos_u16_t size;     /**< Data length in this packet */
  pos_u8_t  data[0];  /**< File content data */
} ma_ota_file_t;

/**
 * @brief OTA global control structure
 */
typedef struct {
  ma_ota_req_t      req; /**< The current OTA reuest command */
  ma_ota_req_file_t hdr; /**< Latest OTA content header */  
  void             *buf; /**< Temporary buffer used by MA/OTA process */  
  pos_u32_t ota_failure; /**< OTA failure times */
} ma_ota_ctrl_t;

#endif
