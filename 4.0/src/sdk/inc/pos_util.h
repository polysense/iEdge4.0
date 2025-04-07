/**
 * @file  pos_util.h  
 * @brief Utilities library
 * @author Runby F.
 * @date 2022-2-19
 * @copyright Polysense
 */

#ifndef __POS_UTIL_H__
#define __POS_UTIL_H__

#include "pos_types.h"

/**
 * @brief Utility library definition
 */ 
typedef struct {
  /**
  * @brief Get U32 from any address
  * @param[in] data Data address
  * @param[in] len Max data length in unit of buytes
  * @return Final U32 integer
  */
  pos_u32_t (*get_u32)(void *data, pos_size_t len);

  /**
  * @brief Translate from string to unsigned integer
  * @param[in] str String to be translated
  * @return Translated U32 integer, return zero of translation is failed
  */
  pos_u32_t (*str2u32)(const char *str );

  /**
  * @brief Translate from string to signed integer
  * @param[in] str String to be translated
  * @return Translated I32 integer, return zero of translation is failed
  */  
  pos_i32_t (*str2i32)(const char *str );

  /**
  * @brief Find pattern from string
  * @param[in] str String to be searched
  * @param[in] pattern Matching mattern
  * @param[in] skip Skip the number of matching from the beginning
  * @return String address matching with the pattern after "skip" times of matching (skip+1 matching address).
  If pattern is never matched or not enough "skip" matching times, return NULL.
  */
  const char *(*strfind_skip)(const char *str, const char *pattern, pos_size_t str_len, pos_size_t skip);

  /**
  * @brief Length counting of string by \\0 or specified EOF char
  * @param[in] str String to count
  * @param[in] eof_ch EOF character
  * @return String length
  * @note The length will never cross a \\0 boundary of the string
  */
  pos_size_t (*strlen_eof)(const char *str, char eof_ch);

  /**
  * @brief Xmodem loading file into OTA resource area
  * @return Loading file length of bytes
  * @note File content will be saved in the OTA resource area. File length should NOT beyond this OTA resource area. 
  */
  pos_size_t (*xload)(void);

  /**
  * @brief Delay in unit of us
  * param[in] us Delay of time in unit of us
  * @note This operation will delay with no task sleep. 
    For longer delay (for example longer than 10s), please use tick->sleep().
  */
  void  (*udelay)(pos_u32_t us);

  /**
  * @brief Console input for confirming
  * param[in] seconds Waiting seconds for confirming
  * @return POS_STATUS_OK: Confirmed, others; Not confirmed
  */
  pos_status_t (*console_confirm)(pos_u32_t seconds);

  /**
  * @brief String compare with case sensitive or insensitive
  * @param[in] dst String buffer
  * @param[in] src String buffer
  * @param[in] case_care Case sensitive, 0: insensitive, 1: sensitive
  * @return 0: String matched\n
          !=0: String not matched
  */
  int (*strcmp_case)( const char *dst, const char *src, int case_care );

  /**
  * @brief Unsigned float to integer/fraction translation 
  * @param[in] v_float IEEE754 floating value
  * @param[out] part_int Integer part of this float
  * @param[out] part_faction Fraction part of this float (in unit of 0.000001)
  */
  void (*ufloat_to_u32)(pos_u32_t v_float, pos_u32_t *part_int, pos_u32_t *part_fraction);

  /**
  * @brief Unsigned float to integer translation 
  * @param[in] v_float IEEE754 floating value
  * @param[in] precision Multiply by this number (1/10/10/.../1000000)
  * @param[out] Integer part multiplied by precision number
  */
  pos_u32_t (*ufloat_to_u32_precision)(pos_u32_t v_float, pos_u32_t precision);

  /**
  * @brief Interger absolute value
  * @param[in] v Signed integer
  * @return Absolute value
  */
  pos_u32_t (*abs)(pos_i32_t v);

  /**
  * @brief Unsigned integer multipication/division
  * @param[in] d Unsigned integer
  * @param[in] mul Multiply by
  * @param[in] div Divide by
  * @return d * mul / div
  */
  pos_u32_t (*umd)(pos_u32_t d, pos_u32_t mul, pos_u32_t div);

  /**
  * @brief Unsigned integer mod
  * @param[in] d Unsigned integer
  * @param[in] mod Mod by
  * @return d % mod
  */
  pos_u32_t (*umod)(pos_u32_t d, pos_u32_t mod);

  /**
  * @brief Unsigned integer SQRT
  * @param[in] d Unsigned integer
  * @return sqrt(d)
  */
  pos_u32_t (*sqrt)(pos_u32_t d);

  /**
  * @brief Sort an I16 array
  * @param[in] a Array of data
  * @param[in] cnt Array count
  * @param[in] descending Sort type: 0-ASCENDING, 1-DESCENDING
  * @return middle value of this array
  */
  pos_i16_t (*sort_i16)(pos_i16_t *a, pos_u32_t cnt, pos_u8_t descending);

  /**
  * @brief Sort an U16 array
  * @param[in] a Array of data
  * @param[in] cnt Array count
  * @param[in] descending Sort type: 0-ASCENDING, 1-DESCENDING
  * @return middle value of this array
  */
  pos_u16_t (*sort_u16)(pos_u16_t *a, pos_u32_t cnt, pos_u8_t descending);

  /**
  * @brief ASign
  * @param[in] d Sign vlaue in unit of 0.0001 (10000 for 1)
  * @return Angle in unit of 0.01 degree
  */
  pos_i16_t (*asin)(pos_i16_t d);

  /**
  * @brief Console raw payload write
  * @param[in] buf Payload for writing to console
  * @param[in] len Payload length
  * @return Actual wrtiting length
  */
  pos_size_t (*console_write)(const pos_u8_t *buf, pos_size_t len);

} pos_lib_util_t;


#endif
