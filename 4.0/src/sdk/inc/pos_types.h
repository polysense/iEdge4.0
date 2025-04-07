/**
 * @file  pos_types.h  
 * @brief Common types definition
 * @author Runby F.
 * @date 2022-2-8 
 * @copyright Polysense
 */

#ifndef __POS_TYPES_H__
#define __POS_TYPES_H__

#define POS_NULL    (0)   /**< NULL pointer */
#define POS_ARRAY_CNT(array)       (sizeof(array)/sizeof(array[0]))   /**< Count element number from array */
#define POS_NTOHS(value)           ((((value)&0xff)<<8)|(((value)>>8)&0xff))  /**< Network/host order translate */
#define POS_MIN( X, Y )  ((X) < (Y) ? (X) : (Y)) /**< Get minimum value */
#define POS_MAX( X, Y )  ((X) > (Y) ? (X) : (Y)) /**< Get maximum value */

typedef unsigned char pos_u8_t; /**< Unsigned 8-bit integer */

typedef signed char pos_i8_t;  /**< Signed 8-bit integer */

typedef unsigned short pos_u16_t; /**< Unsigned 16-bit integer */

typedef short pos_i16_t; /**< Signed 16-bit integer */

typedef unsigned int pos_u32_t; /**< Unsigned 32-bit integer */

typedef int pos_i32_t; /**< Signed 32-bit integer */

typedef unsigned long long pos_u64_t; /**< Unsigned 64-bit integer */

typedef long long pos_i64_t; /**< Signed 64-bit integer */

typedef int pos_status_t; /**< Common function returning status type */

typedef unsigned int pos_size_t; /**< Size type */

typedef pos_status_t (*pos_func_t) (void *arg); /**< Common function type */
typedef void (*pos_funcv_t) (void); /**< Void function type */
typedef void *(*pos_funcvp_t) (void); /**< Void* function type */
typedef void  (*pos_func_msg_cb_t)(void *msg, pos_size_t len); /**< Common message callback function type */
typedef pos_status_t (*pos_func2_t) (void *arg1, void *arg2); /**< Common function type with two arguments */

/**
 * @brief String structure with length field
 */
typedef struct
{
	pos_size_t len;  ///< String length
	char* data; ///< String
} pos_lstring_t;

/**
 * @brief Structure with both C string and length+string
 * @note If lstring.len is not zero, it's processed as length+string; or else, C string
 */
typedef struct
{
	char* cstring; ///< C string
	pos_lstring_t lstring; ///< Length+string
} pos_c_lstring_t;

/**
 * @brief Return status definition
 */
typedef enum{
  POS_STATUS_OK = 0, /**< Successful */

  POS_STATUS_E_INIT = 1,  /**< Init error */

  POS_STATUS_E_PARAMETER = 2, /**< Parameter error */

  POS_STATUS_E_MEM = 3, /**< Memory error */

  POS_STATUS_E_NOT_FOUND = 4, /**< Not found */

  POS_STATUS_E_TIMEOUT = 5, /**< Timeout error */

  POS_STATUS_E_NOT_SUPPORT = 6,  /**< Not support */
  
  POS_STATUS_E_DUPLICATE = 7,    /**< Duplicate */

  POS_STATUS_E_NULL = 8,    /**< Null pointer error */

  POS_STATUS_E_RESOURCE = 9,    /**< Resource error */

  POS_STATUS_ERROR = 0xff, /**< Generic error */
  
} pos_status_e; 

/**
* @brief Resource ID definition
*/
typedef enum {
  POS_RESOURCE_OS = 0, /**< POS area*/
    
  POS_RESOURCE_OTA = 1, /**< OTA buffer */
  
  POS_RESOURCE_MAIN = 2, /**< MA (main application) area */
  
  POS_RESOURCE_APP1 = 3, /**< Driver area */
  
  POS_RESOURCE_APP2 = 4, /**< Reserved area (not support) */
  
  POS_RESOURCE_CFG = 5, /**< Configuration area */  

  POS_RESOURCE_MAX = 6, /**< Max resource ID, should be < this MAX value */
  
} pos_resource_id_t;

/**
* @brief CRC type definition
*/
typedef enum {
  POS_CRC8 = 0, /**< CRC8 */

  POS_CRC8R = 1, /**< Reversed CRC8 */  

  POS_CRC16 = 2, /**< CRC16 */    
  
  POS_CRC16_MBUS = 3, /**< MODBUS CRC16 */
  
  POS_CRC_MAX, /**< Max type, should be < this value*/
  
} pos_crc_type_t;

/**
* @brief Resouce configuration structure
*/
typedef struct {
  pos_u32_t  rom_addr; /**< Storage starting address */

  pos_u32_t  rom_size; /**< Storage size in unit of bytes */
  
  pos_u32_t ram_addr; /**< Memory starting address */
  
  pos_u32_t ram_size; /**< Memory size in unit of bytes */
  
} pos_resource_t;

/**
* @brief Timer callback function type
*/
typedef void (*pos_timer_cb_t)(void *param);

/**
* @brief Timer handle
*/
typedef void *pos_timer_handle_t;


#endif
