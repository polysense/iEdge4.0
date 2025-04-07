/**
 * @file  pos_lib.h  
 * @brief Polysense OS (POS) API definition
 * @author Runby F.
 * @date 2022-2-8 
 * @copyright Polysense
 */

#ifndef __POS_LIB_H__
#define __POS_LIB_H__

#include "pos_types.h"
#include "pos_gpio.h"
#include "pos_wire1.h"
#include "pos_io.h"
#include "pos_pwm.h"
#include "pos_call.h"
#include "pos_task.h"
#include "pos_util.h"
#include "pos_net.h"
#include "pos_mm.h"

#undef POS_CHINESE

/**
 * @brief Use to detect whether this is a POS library structure
 */
#define POS_LIB_MAGIC_NUM      0xe782c631

/**
 * @brief MA APP binary header magic dword
 */
#define APP_HDR_MAGIC 0x7193be45

/**
 * @brief Drvier binary header magic dword
 */
#define DRV_HDR_MAGIC 0x8274ef37

/**
 * @brief Pin configuration binary header magic dword
 */
#define PIN_HDR_MAGIC 0x37b72348

/**
 * @brief Binary to string translation type definition
 */
typedef enum {
  ENCODE_HEXSTR_LOWER = 0,  ///> Translate to hex string in lowercase
  ENCODE_HEXSTR_UPPER = 1,  ///> Translate to hex string in uppercase
  ENCODE_BASE64 = 2,        ///> Translate to base64 string
} pos_encode_t;

/**
 * @brief String to binary translation type definition
 */
typedef enum {
  DECODE_HEXSTR = 0,  ///> Hex string
  DECODE_INT32 = 1,  ///> Signed integer string or IP address 
  DECODE_BASE64 = 2,        ///> BASE64 string
  DECODE_UINT32 = 3,  ///> Unsigned integer string or IP address 
  DECODE_FLOAT1000 = 4,  ///> Signed float string (x 1000) to integer binary
  DECODE_UFLOAT1000 = 5,  ///> Unsigned float string (x 1000) to integer binary
} pos_decode_t;


/**
 * @brief ADC libraty
 */
typedef struct {
  /** 
   * Start ADC and shutdown it until finished
   * @param[in]  channel   ADC channel
   * @return ADC sampling data
   */ 
  pos_u32_t (*read)(pos_u32_t channel);

  /** 
   * Start ADC
   * @param[in]  channel   ADC channel
   */ 
  void (*start)(pos_u32_t channel) ;   

  /** 
   * Stop ADC
   */ 
  void (*stop)(void);

  /** 
   * ADC convert
   * @param[in] wait 0: No wait, return 0xffffffff if conveting is not finished; 1: wait until convering done
   * @return ADC sampling data
   * @note When wait=0, it can check the result by 0xffffffff to know whether the converting is done
   */ 
  pos_u32_t (*convert)(pos_u32_t wait);

  /** 
   * ADC convert with filtering
   * @param[in]  channel   ADC channel
   * @param[in]  mode      Filtering mode (0: average, 1: max, others: not support)
   * @param[in]  cnt       Number of samplings for filtering
   * @param[in]  us_gap    Interval of gap during each sampling in unit of us
   * @return ADC sampling data after filtering
   */ 
  pos_u32_t (*read_filter)(pos_u32_t adc_channel, pos_u32_t mode, pos_u32_t cnt, pos_u32_t us_gap);
} pos_lib_adc_t;

/**
 * @brief Timer library
 */
typedef struct {
  /** 
   * Create a timer
   * @param[in]  cb_ptr   Callback function
   * @return     Timer handler, or NULL is creation is failed
   * @note The new created timer will not start until pos_lib_timer_t::set() is called with timeout_ms
   */ 
  pos_timer_handle_t (*create)(pos_timer_cb_t cb_ptr);

  /** 
   * Set timer's timeout time (unit of ms) and start the timer immediately
   * @param[in]  h   Timer handler
   * @param[in]  timeout_ms   Timeout time in unit of ms
   */ 
  void (*set)(pos_timer_handle_t h, pos_u32_t timeout_ms);

  /** 
   * Delete a timer
   * @param[in]  h   Timer handler
   */ 
  void (*destroy)(pos_timer_handle_t h);

  /** 
   * Return elapsed time in unit of ms from last timer trigger
   * @return Time in unit of ms
   */ 
  pos_u32_t (*elaps)(void);

  /** 
   * Return the next timeout time to now in unit of ms
   * @return Time in unot if ms, return 0xffffffff if no running timer
   */ 
  pos_u32_t (*next_timeout)(void);    
} pos_lib_timer_t;

/**
 * @brief RTC library
 */
typedef struct {
  /** 
   * Set RTC time
   * @param[in]  ymdhmsw_bcd   BCD bytes for YEAR/MONTH/DAY/HOUR/MINUTE/SECONDS/WEEKDAY
   */ 
  void (*time_set)(pos_u8_t *ymdhmsw_bcd);

  /** 
   * Get RTC time
   * @param[out]  ymdhmsw_bcd   BCD bytes for YEAR/MONTH/DAY/HOUR/MINUTE/SECONDS/WEEKDAY
   */ 
  void (*time_get)(pos_u8_t *ymdhmsw_bcd);


  /** 
   * Set RTC alarm
   * @param[in]  ymdhmsw_bcd   BCD bytes for YEAR/MONTH/DAY/HOUR/MINUTE/SECONDS/WEEKDAY
   * @param[in]  cb            Alarm callback function pointer
   * @param[in]  cookie        Alarm callback cookie poionter   
   */ 
  void (*alarm_set)(pos_u8_t *hmsw_bcd, void (*cb)(void *), void *cookie);

  /** 
   * Get RTC alarm
   * @param[out]  ymdhmsw_bcd   BCD bytes for YEAR/MONTH/DAY/HOUR/MINUTE/SECONDS/WEEKDAY
   * @param[out]  cb            Alarm callback function pointer
   * @param[out]  cookie        Alarm callback cookie poionter   
   * @return Current alarm valid (enable) state
   */ 
  pos_u32_t (*get_alarm)(pos_u8_t *hmsw_bcd, void (**cb)(void *), void **cookie);

  /** 
   * Set RTC time2 (ticks simulating RTC in bin format)
   * @param[in]  ymdhmsw_bin   BIN bytes for YEAR/MONTH/DAY/HOUR/MINUTE/SECONDS/WEEKDAY
   * @param[in]  ticks         Running ticks corresponding to the given time
   */ 
  void (*time2_set)(pos_u8_t *ymdhmsw_bin, pos_u32_t ticks);

  /** 
   * Get current RTC time2 (ticks simulating RTC in bin format)
   * @param[out]  ymdhmsw_bin   BIN bytes for YEAR/MONTH/DAY/HOUR/MINUTE/SECONDS/WEEKDAY
   */ 
  void (*time2_get)(pos_u8_t *ymdhmsw_bin);

  /** 
   * Get recent set RTC time2 (ticks simulating RTC in bin format)
   * @param[out]  ymdhmsw_bin   Recent setting BIN bytes for YEAR/MONTH/DAY/HOUR/MINUTE/SECONDS/WEEKDAY
   * @return                    Recent setting ticks
   */ 
  pos_u32_t (*time2_get_tick)(pos_u8_t *ymdhmsw_bin);

  /** 
   * Adjsut RTC time by increasing seconds (in bin format)
   * @param[in,out] ymdhmsw_bin   BIN bytes for YEAR/MONTH/DAY/HOUR/MINUTE/SECONDS/WEEKDAY
   * @param[in]  seconds     Seconds to be increased
   */ 
  void (*time_adj)(pos_u8_t *ymdhmsw_bin, pos_u32_t seconds);

  /** 
   * Translate RTC BIN to BCD format
   * @param[in]   ymdhmsw_bin   BIN bytes for YEAR/MONTH/DAY/HOUR/MINUTE/SECONDS/WEEKDAY
   * @param[out]  ymdhmsw_bcd   BCD bytes for YEAR/MONTH/DAY/HOUR/MINUTE/SECONDS/WEEKDAY
   */ 
  void (*bin2bcd)(pos_u8_t *ymdhmsw_bin, pos_u8_t *ymdhmsw_bcd);

  /** 
   * Translate RTC BCD to BIN format
   * @param[in]   ymdhmsw_bcd   BCD bytes for YEAR/MONTH/DAY/HOUR/MINUTE/SECONDS/WEEKDAY
   * @param[out]  ymdhmsw_bin   BIN bytes for YEAR/MONTH/DAY/HOUR/MINUTE/SECONDS/WEEKDAY
   */ 
  void (*bcd2bin)(pos_u8_t *ymdhmsw_bcd, pos_u8_t *ymdhmsw_bin);

} pos_lib_rtc_t;

/**
 * @brief Tick (unit of 1ms) librarty
 */
typedef struct {
  /** 
   * Get current running tick
   * @return     Current tick
   * @note System tick interval is 1 ms
   */ 
  pos_u32_t (*get)(void);

  /** 
   * Sleep until given ticks
   * @param[in] ticks Expecting sleeping ticks
   * @note Interrupt or timer events could be triggered during this sleep. But the wakeup time is always after the sleeping ticks.
   */ 
  void (*sleep)(pos_u32_t ticks);

  /** 
   * Try sleeping until given ticks
   * @param[in] ticks Expecting sleeping ticks
   * @return Actual sleeping ticks
   * @note Interrupt or timer eventer could end the sleeping before its expecting ticks
   */ 
  pos_u32_t (*try_sleep)(pos_u32_t ticks);

  /** 
   * Calculate the delta between now and old ticks
   * @param[in] now New tick time
   * @param[in] old Old tick time
   * @return     Delta ticks
   */ 
  pos_u32_t (*elaps_calc)(pos_u32_t now, pos_u32_t old);

  /** 
   * Calculate now to given old ticks delta time
   * @param[in] old Old tick time
   * @return     Delta ticks from now
   */ 
  pos_u32_t (*elaps)(pos_u32_t old);

  /** 
   * Check ticks is timeout
   * @param[in] now Checking tick time
   * @param[in] exp_timeout Expecting timeout ticks
   * @return    0: Not timeout, !=0: Timeout
   */ 
  pos_u32_t (*is_timeout)(pos_u32_t now, pos_u32_t exp_timeout);

  /** 
   * Wakeup app from sleeping/waiting state
   * @param[in] app_id Not used now
   */ 
  void (*wakeup)(pos_u32_t app_id);
} pos_lib_tick_t;

/**
 * @brief POS general library
 */
typedef struct {
  /**
   * @brief Use to detect whether this is a POS library structure. It should be equal to @ref POS_LIB_MAGIC_NUM
   */
  pos_u32_t  magic;

  /**
   * @brief POS version
   */
  pos_u32_t  version;
  
  /**
  * @brief ADC library
  */
  const pos_lib_adc_t *adc;

  /**
  * @brief GPIO library
  */
  const pos_lib_gpio_t *gpio;

  /**
  * @brief Single wire library (DS18b20)
  */
  const pos_lib_wire1_t *wire1;

  /**
  * @brief IO library
  */
  const pos_lib_io_t *io;  

  /**
  * @brief Task library
  */
  const pos_lib_task_t *task;

  /**
  * @brief Tick library
  */
  const pos_lib_tick_t *tick;

  /**
  * @brief Timer library
  */
  const pos_lib_timer_t *timer;

  /**
  * @brief Utility library
  */
  const pos_lib_util_t *util;

  /**
  * @brief Network protocol stack library
  */
  const pos_lib_nps_t *nps;  

  /**
  * @brief Memory map and virtual mem library
  */
  const pos_lib_mm_t *mm;
  
  /** 
   * Load application and execute it according to different mode 
   * @param[in]  app_res_id   Application resource ID, refer to @ref pos_resource_id_t
   * @param[in]  mode   \n
          0: Only check whether this application resource is available\n
          1: Check this application resource and init its data memory\n
          2: Check this application resource and run it without data memory initilization\n
          3: Check this application resource and run it after data memory initilization\n
          Others: Not support
   * @param[in]  arg   Parameter pointer passing to application entrance
   * @param[out] entry_pptr  Returning pointer to store the application entrance function
   * @return     0: Successful\n
                !=0: Failed, refer to @ref pos_status_t
   * @note entry_pptr can be NULL if the application entrance is not cared
   */ 
  pos_status_t (*app_load)(pos_resource_id_t app_res_id, pos_u32_t mode, void *arg, pos_func_t *entry_pptr);

  /** 
   * Memory copy
   * @param[in] dest  Destination memory pointer
   * @param[in] src   Source memory pointer
   * @param[in] n     Number of bytes to be copied
   * @note
   */ 
  void (*memcpy)(void *dest, const void *src, pos_size_t n);

  /** 
   * Memory set
   * @param[in] s   Destination memory pointer
   * @param[in] c   Byte value to be set with
   * @param[in] n   Number of btyes to be set
   * @return    Always return the pinter of s
   */ 
  void * (*memset)(void *s, int c, pos_size_t n);

  /** 
   * Memory compare
   * @param[in] dst   Destination memory pointer
   * @param[in] src   Source memory pointer
   * @param[in] n     Number of bytes to be compared
   * @return 0: all are equal, !=0: not euqal
   */ 
  int (*memcmp)( const void *dst, const void *src, pos_size_t n );

  /** 
   * Heap memory allocation
   * @param[in] n   Number of bytes to be allocated
   * @return    NULL: Allocate failed\n
   			Others: The new allocated memory pointer
   */ 
  void * (*malloc)(pos_size_t n);

  /** 
   * Heap memory release
   * @param[in] data   The memory pointer to be released
   */ 
  void (*free)(void *data);

  /** 
   * Print content to buffer with given format pattern
   * @param[in] buf   Output data buffer
   * @param[in] fmt   Format patterns, refer to @ref printf
   * @param[in] ...   Variable arguments
   * @return End of data buffer after the printing
   * @note Return pointer - buf is euqal to the final printing text length
   */ 
  char *(*sprintf)(char *buf, const char *fmt, ...);

  /** 
   * Parse string by given format
   * @param[in] buf   String buffer to be parsed
   * @param[in] fmt   Parsing format:
<pre>
\%\#b ... Hex string to binary array
\%\#n ... Singed or unsigned integer, IP address to binary U32/I32
\%\#m ... String bitmap to binary U32
1) The above "\%\#b" length must be specified by any length, for example: "\%6b"
2) Other "\#" must be value betwee 1..4, indicating the final parsing length of binary bytes
</pre>    
   * @param[in] ...   Variable paramter list
   * @return End of string buffer pointer after the given format parsing
   * @note 1) Return pointer - buf is euqal to the final parsing text length
   * @note 2) Examples:
<pre>
 pos_u8_t buf[6];
 pos_u32_t u32;
 pos_u8_t u8;
 g_pos->sscan("00112233aabb", "%6b", buf); ==> buf={0x00,0x11,0x22,0x33,0xaa,0xbb,} 
 g_pos->sscan("1.2.3.4", "%4n", &u32); ==> u32=0x01020304
 g_pos->sscan("1.2.3.4", "%1n", &u8); ==> u8=0x04 
 g_pos->sscan("0 6", "%1m", &u8); ==> u8=0x41
 g_pos->sscan("0 6 31", "%4m", &u32); ==> u32=0x80000041
</pre>
 */ 
  const char *(*sscan)(const char *buf, const char *fmt, ...);

  /** 
   * Print string to console
   * @param[in] s   String pointer
   */ 
  void (*puts)(const char *s);

  /** 
   * Print string to console to format pattern
   * @param[in] fmt   Format pattern:
<pre>
\%\#B ... Number of bytes in uppercase hex string
\%\#b ... Number of bytes in lowercase hex string
\%\#c ... Single char
\%\#d ... Signed integer
\%\#i ... IP address (A.B.C.D)
\%\#s ... String
\%\#u ... Unsigned integer
\%\#X ... Hex number if uppercase
\%\#x ... Hex number if lowercase
1) The above "\%\#b" or "\%\#B" length must be specified by any byte length, for example: "\%6b"
2) In other case, "\#" can be omitted, then shortest string will be printed
3) "\#" can be "-n" indicating printing alignment to left, or "n" indicating alignment to right
</pre>     
   * @param[in] ...   Variable paramter list
   * @note Examples:
<pre>
 char buf[6]={0,1,2,3,a,b};
 pos_u32_t u = 0x0102030f;
 g_pos->printf("%6b", buf); ==> 00112233aabb 
 g_pos->printf("%6B", buf); ==> 00112233AABB
 g_pos->printf("%x", u); ==> 102030f
 g_pos->printf("%08X", u); ==> 0102030F 
 g_pos->printf("%i", u); ==> 1.2.3.15
 g_pos->printf("%c%5c", 'a', 'b'); ==> a    b
 g_pos->printf("[%c%5c%-5c%c]", 'a', 'b', 'c','d'); ==> [a    bc    d]  
</pre>     

   */ 
  void (*printf)(const char *fmt, ...);

  /** 
   * Flush console printing buffer
   */ 
  void (*flush)(void);

  /** 
   * Encode binary bytes into text string by given type
   * @param[in] buf   Final text string buffer
   * @param[in] data  Bytes buffer to be encoded
   * @param[in] data_len   Number of bytes to be encoded
   * @param[in] encode   Encding type @ref pos_encode_t
   * @return 
      NULL: Encode error\n
      Others: End of the text string buffer after encoding\n
   * @note When not NULL returned, Return Pointer - buf is equal to text length encoded
   */ 
  char *(*bin2str)(char *buf, pos_u8_t *data, pos_size_t data_len, pos_encode_t encode);

  /** 
   * Decode text string to binary bytes by given type
   * @param[in] buf   Text string buffer
   * @param[in] data  Bytes buffer to be encoded
   * @param[in] p_data_len   Text string length pointer (after decoding this value will be updated as the final decoding length)
   * @param[in] decode   Decoding type @ref pos_decode_t
   * @return 
      NULL: Decode error\n
      Others: End of the bytes buffer after decoding\n
   * @note When not NULL returned, Return Pointer - buf is equal to text length decoded
   */ 
  char *(*str2bin)(char *buf, pos_u8_t *data, pos_size_t *p_data_len, pos_decode_t decode);

  /** 
   * CRC calculation
   * @param[in] type   CRC type, refer to @ref pos_crc_type_t
   * @param[in] data   Data bytes pointer
   * @param[in] data_len   Number of bytes to be processed
   * @return 
      CRC checksum result. Only lowest bit[0..7] is valid for CRC8 and bit[0..15] is valid for CRC16
   */ 
  pos_u32_t (*crc)(pos_crc_type_t type, pos_u8_t *data, pos_size_t data_len);

  /** 
   * Generate a random integer
   * @return 
      Random integer generated
   */ 
  pos_u32_t (*rand)( void );

  /** 
   * Generate a random valud between min and max
   * @param[in] min   Lower limit
   * @param[in] max   Upper limite
   * @return 
      Random integer generated
   */ 
  pos_u32_t (*randr)( pos_u32_t min, pos_u32_t max );

  /** 
   * String copy
   * @param[in] dst   Destination string buffer
   * @param[in] src   Source string buffer
   * @return 
      End of destination string buffer after copying
   */ 
  char * (*strcpy)(char *dst, const char *src);

  /** 
   * String copy with number of bytes
   * @param[in] dst   Destination string buffer
   * @param[in] src   Source string buffer
   * @param[in] n     At most number of bytes to be copied
   * @return 
      End of destination string buffer after copying
   */ 
  char * (*strncpy)(char *dst, const char *src, pos_size_t n);

  /** 
   * Count string length
   * @param[in] src   String buffer
   * @return 
      String lenth of bytes
   */ 
  pos_size_t (*strlen)(const char *str);

  /** 
   * Erase and program flash
   * @param[in] addr  Target flash address
   * @param[in] buf   Data bytes buffer (if NULL, the target flash will be erased onlye)
   * @param[in] n     Number of bytes to be erased or programmed.
   * @param[in] reset 0: Do NOT reset after programming, !=0: Reset after programming
   * @note When target is the POS kernel, the system will always reset after programming even if reset=0
   */ 
  void (*program)(pos_u32_t addr, const void *buf, pos_size_t n, pos_u32_t reset);

  /** 
   * System reset
   */ 
  void (*reset)(void);

  /** 
   * Generic function call with function ID
   * @param[in] func_id Function ID, refer to @ref POS_CALL_FUNC_ID_GET_INFO
   * @param[in] ...   Variable parameters
   * @return     0: Successful\n
                !=0: Failed, refer to @ref pos_status_t
   */ 
  pos_status_t (*call)(pos_u32_t func_id, ...);

  /**
  * @brief PWM library
  */
  const pos_lib_pwm_t *pwm;

  /**
  * @brief Pin configuration
  */
  const pos_board_pin_t *pin;

  /**
  * @brief RTC library
  */
  const pos_lib_rtc_t *rtc;
} pos_lib_t;

/**
 * @brief Global POS API calling instance \n
 Examples:\n
  g_pos->printf( "hello, world!" );
 * @note When application is running, the low layer POS kernel will initialize this variable pointing to the correct locations.
 */
extern pos_lib_t *g_pos; /**< Global POS API pointer */

#endif
