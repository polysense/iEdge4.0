/**
 * @file  pos_wire1.h  
 * @brief Single wire librarty
 * @author Runby F.
 * @date 2022-2-24
 * @copyright Polysense
 */

#ifndef __POS_WIRE1_H__
#define __POS_WIRE1_H__

#include "pos_types.h"

/**
 * @brief Single wire handle
 */ 
typedef void *pos_wire1_handle_t;

/**
 * @brief Single wire bus operating library
 */ 
typedef struct {
  /**
  * @brief Allocate a single wire handle
  * @param[in] rw Read/write pin name
  * @param[in] pullup Pullup pin name
  * @param[in] pulldn Pulldown pin name
  * @return Single wire handle, or NULL if allocated error
  */
  pos_wire1_handle_t (*alloc)(pos_gpio_pin_t rw, pos_gpio_pin_t pullup, pos_gpio_pin_t pulldn);

  /**
  * @brief Release a single wire resource
  * @param[in] h Single wire handle
  */  
  void (*free)(pos_wire1_handle_t h);

  /**
  * @brief Single wire bus reset
  * @param[in] h Single wire handle
  */  
  void (*reset)(pos_wire1_handle_t h);

  /**
  * @brief Check device response from the bus
  * @param[in] h Single wire handle
  * @return 0: No response\n
            !=0: Response detected
  */  
  pos_u32_t (*check)(pos_wire1_handle_t h);

  /**
  * @brief Read a single bit from the bus
  * @param[in] h Single wire handle
  * @return Reading bit (0 or 1)
  */  
  pos_u8_t (*read_bit)(pos_wire1_handle_t h);

  /**
  * @brief Read a byte from the bus
  * @param[in] h Single wire handle
  * @return Reading byte
  */  
  pos_u8_t (*read_byte)(pos_wire1_handle_t h);

  /**
  * @brief Write a bit to the bus
  * @param[in] h Single wire handle
  * @param[in] bit Writing bit (0 or 1)
  */  
  void (*write_bit)(pos_wire1_handle_t wh, pos_u8_t bit);

  /**
  * @brief Write a byte to the bus
  * @param[in] h Single wire handle
  * @param[in] data Byte for writing
  */  
  void (*write_byte)(pos_wire1_handle_t wh,pos_u8_t data);  
} pos_lib_wire1_t;


#endif
