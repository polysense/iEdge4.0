/**
 * @file  pos_io.h  
 * @brief IO library definitions
 * @author Runby F.
 * @date 2022-2-13
 * @copyright Polysense
 */

#ifndef __POS_IO_H__
#define __POS_IO_H__

#include "pos_types.h"

/**
 * @brief IO wait forever
 */
#define POS_IO_WAIT_FOREVER 0xffffffff 

/**
 * @brief Special wait value for I2C address control (16 bit register, timeout in unit of ms)
 */
#define POS_IO_WAIT_I2C_READ16(reg16,ms) (((reg16)<<16)+ms)

/**
 * @brief Special wait value for I2C address control (8 bit register, timeout in unit of ms)
 */
#define POS_IO_WAIT_I2C_READ8(reg8,ms) (((reg8)<<24)+ms)

/**
 * @brief Special wait value for I2C address control (8 bit register, timeout in unit of ms)
 */
#define POS_IO_WAIT_SPI_CMD(reg8,ms) (0x80000000+((reg8)<<16)+ms)

/**
 * @brief Standard SPI operating wait in unit of ms
 */
#define POS_IO_WAIT_SPI_NORMAL(ms) ms

/**
 * @brief No wait, return immediately if IO is not ready
 */
#define POS_IO_WAIT_NO_DELAY   0 

/**
 * @brief SPI master control mode
 */
#define POS_IO_CTRL_SPI_MASTER          0x10 

/**
 * @brief SPI slave control mode
 */
#define POS_IO_CTRL_SPI_SLAVE           0

/**
 * @brief SPI polarity low
 */
#define POS_IO_CTRL_SPI_POLARITY_LOW    0 

/**
 * @brief SPI polarity high
 */
#define POS_IO_CTRL_SPI_POLARITY_HIGH   0x08

/**
 * @brief SPI phase control 1
 */
#define POS_IO_CTRL_SPI_PHASE_1    0 

/**
 * @brief SPI phase control 2
 */
#define POS_IO_CTRL_SPI_PHASE_2   0x04


/**
 * @brief I2C master control mode
 */
#define POS_IO_CTRL_I2C_MASTER          0x550040

/**
 * @brief I2C slave control mode
 */
#define POS_IO_CTRL_I2C_SLAVE(addr)     (((addr)<<16)+0x44)

/**
 * @brief I2C slave with normal broadcast
 */
#define POS_IO_CTRL_I2C_SLAVE_BC_NORMAL    0

/**
 * @brief I2C slave with normal broadcast ACK
 */
#define POS_IO_CTRL_I2C_SLAVE_BC_ANSWER    0x100

/**
 * @brief Check if io is UART
 */
#define POS_IO_IS_UART(io)          ((io)<POS_IO_LPUART0)

/**
 * @brief Check if io is LPUART
 */
#define POS_IO_IS_LPUART(io)        (((io)<POS_IO_I2C0) && ((io)>=POS_IO_LPUART0))

/**
 * @brief Check if io is serial port (UART or LPUART)
 */
#define POS_IO_IS_SERIAL_PORT(io)   ((io)<POS_IO_I2C0)

/**
 * @brief Setup CTRL flag indicating console IO
 */
#define POS_IO_SETUP_CTRL_CONSOLE   0x80000000

/**
 * @brief IO type definition
 * @note LPUART is only supported on certain boards
 */
typedef enum {
  POS_IO_UART0    = 0x00,
  POS_IO_UART1    = 0x01,    
  POS_IO_UART2    = 0x02,  
  POS_IO_UART3    = 0x03,
  POS_IO_UART4    = 0x04,
  POS_IO_UART5    = 0x05,    
  POS_IO_UART6    = 0x06,  
  POS_IO_UART7    = 0x07,
  POS_IO_UART8    = 0x08,
  POS_IO_UART9    = 0x09,    
  POS_IO_UART10   = 0x0a,  
  POS_IO_UART11   = 0x0b,
  POS_IO_UART12   = 0x0c,
  POS_IO_UART13   = 0x0d,    
  POS_IO_UART14   = 0x0e,  
  POS_IO_UART15   = 0x0f,  
  POS_IO_LPUART0  = 0x10,
  POS_IO_LPUART1  = 0x11,    
  POS_IO_LPUART2  = 0x12,  
  POS_IO_LPUART3  = 0x13,
  POS_IO_I2C0     = 0x20,
  POS_IO_I2C1     = 0x21,    
  POS_IO_I2C2     = 0x22,  
  POS_IO_I2C3     = 0x23,  
  POS_IO_I2C4     = 0x24,
  POS_IO_I2C5     = 0x25,
  POS_IO_I2C6     = 0x26,
  POS_IO_I2C7     = 0x27,
  POS_IO_I2C8     = 0x28,
  POS_IO_I2C9     = 0x29,
  POS_IO_I2C10    = 0x2a,
  POS_IO_I2C11    = 0x2b,
  POS_IO_I2C12    = 0x2c,
  POS_IO_I2C13    = 0x2d,
  POS_IO_I2C14    = 0x2e,
  POS_IO_I2C15    = 0x2f,
  POS_IO_SPI0     = 0x30,
  POS_IO_SPI1     = 0x31,    
  POS_IO_SPI2     = 0x32,  
  POS_IO_SPI3     = 0x33,  
} pos_io_t;

/**
 * @brief IO operating handle definition
 */ 
typedef struct pos_io_handle_s {
  /**
  * @brief Internal use for physical instance pointer
  */
  void *instance;    
  
  /**
  * @brief Read operation
  * @param[in] h IO handle pointer
  * @param[in] data Buffer for input data
  * @param[in] len Expecting reading length of bytes
  * @param[in] timeout Waiting timeout, refer to @ref POS_IO_WAIT_FOREVER @ref POS_IO_WAIT_SPI_NORMAL(ms) ...
  * @return Actual reading length of bytes
  */
  pos_size_t (*read)(struct pos_io_handle_s *h, pos_u8_t *data, pos_size_t len, pos_u32_t timeout);
  
  /**
  * @brief Write operation
  * @param[in] h IO handle pointer
  * @param[in] data Buffer for output
  * @param[in] len Expecting writing length of bytes
  * @param[in] timeout Waiting timeout, refer to @ref POS_IO_WAIT_FOREVER @ref POS_IO_WAIT_SPI_NORMAL(ms) ...
  * @return Actual writing length of bytes
  */
  pos_size_t (*write)(struct pos_io_handle_s *h, pos_u8_t *data, pos_size_t len, pos_u32_t timeout);

  /**
  * @brief Release resource and put related IO module into low-power state
  * @param[in] h IO handle pointer
  * @return     0: Successful\n
               !=0: Failed, refer to @ref pos_status_t
  * @note Calling pos_lib_io_t::done() will work same as this function
  */
  pos_status_t (*release)(struct pos_io_handle_s *h);

  /**
  * @brief Setup resource with new parameters
  * @param[in] h IO handle pointer
  * @param[in] speed Working speed
  * @param[in] ctrl Controling mode
  * @return     0: Successful\n
               !=0: Failed, refer to @ref pos_status_t
  */
  pos_status_t (*setup)(struct pos_io_handle_s *h, pos_u32_t speed, pos_u32_t ctrl);

  /**
  * @brief Reserved field
  */  
  pos_u32_t rsvd[7]; 
} pos_io_handle_t;

/**
 * @brief IO processing library structure
 */ 
typedef struct {
  /**
  * @brief Get IO handle pointer
  * @param[in] io IO resource type
  * @return IO handle pointer, or NULL if it's unavailable
  */
  pos_io_handle_t * (*get)(pos_io_t io);

  /**
  * @brief Init IO and return its handle pointer
  * @param[in] io IO resource type
  * @param[in] ... Init parameters, refer to\n
<pre>
UART/LPUART:\n
  @ref pos_u32_t arg1: Baudrate\n
SPI:\n
  @ref pos_u32_t arg1: Bus speed\n
  @ref pos_u32_t arg2: SPI control, refer to @ref POS_IO_CTRL_SPI_MASTER ...\n  
</pre>
  * @return IO handle pointer, or NULL if it's unavalable or init failed
  */
  pos_io_handle_t * (*init)(pos_io_t io, ...);

  /**
  * @brief Release IO resource and make it low-power mode
  * @param[in] io IO resource type
  * @return     0: Successful\n
               !=0: Failed, refer to @ref pos_status_t
  * @note Calling pos_io_handle_t::release() will work same as this function
  */  
  pos_status_t (*done)(pos_io_t io);

  /**
  * @brief Set the GPIO alternative mode with given IO resource type
  * @param[in] pin GPIO pin name
  * @param[in] io IO resource type
  * @return     0: Successful\n
               !=0: Failed, refer to @ref pos_status_t
  */  
  pos_status_t (*pin_af_set)(pos_gpio_pin_t pin, pos_io_t io);  
  
  /**
  * @brief Get the GPIO alternative mode for given IO resource type
  * @param[in] pin GPIO pin name
  * @param[in] io IO resource type
  * @return     0: unsupport this io type for this pin\n
               !=0: GPIO alternative mode value for this kind of IO type
  */  
  pos_u32_t (*pin_af_get)(pos_gpio_pin_t pin, pos_io_t io);  

 } pos_lib_io_t;


#endif
