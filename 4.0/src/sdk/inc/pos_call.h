/**
 * @file  pos_call.h
 * @brief POS call with function ID
 * @author Runby F.
 * @date 2022-2-11
 * @copyright Polysense
 */

#ifndef __POS_CALL_H__
#define __POS_CALL_H__

#include "pos_types.h"

/**
 * @brief Get POS version and low level running information\n
 * arg1 Provide @ref pos_call_info_t structre pointer for returning
 */
#define POS_CALL_FUNC_ID_GET_INFO    0 

/**
 * @brief Get POS sleeping time (unit:ms) for time correction or other purpose\n
 * arg1 Provide a U32 data
 */
#define POS_CALL_FUNC_ID_SET_SLEEP_MS 1

/**
 * @brief Get POS network protocol stack's waiting timeout (unit:ms)\n
 * arg1 Provide a U32 pointer for returning
 */
#define POS_CALL_FUNC_ID_GET_NPS_NET_TIMEOUT 2

/**
 * @brief Set POS network protocol stack's waiting timeout (unit:ms)\n
 * arg1 Provide a U32 data
 */
#define POS_CALL_FUNC_ID_SET_NPS_NET_TIMEOUT 3

/**
 * @brief Get POS network protocl stack's DNS retry times\n
 * arg1 Provide a U32 pointer for returning
 */
 
#define POS_CALL_FUNC_ID_GET_NPS_DNS_RETRY 4
/**
 * @brief Set POS network protocl stack's DNS retry times\n
 * arg1 Provide a U32 data
 */
#define POS_CALL_FUNC_ID_SET_NPS_DNS_RETRY 5

/**
 * @brief Get POS network protocl stack's controlling flags\n
 * arg1 Provide a U32 pointer for returning
 */
 
#define POS_CALL_FUNC_ID_GET_FLAG_NPS 6

/**
 * @brief Set POS network protocl stack's controlling flags\n
 * arg1 Provide a U32 data
 * @note bit0~7:MQTT Version, bit8~15:rsvd, bit16:UDP Connect is required
 */
#define POS_CALL_FUNC_ID_SET_FLAG_NPS 7

/**
 * @brief Get POS network protocl stack's TX sequence ID\n
 * arg1 Provide a U32 pointer for returning
 */
 
#define POS_CALL_FUNC_ID_GET_TX_ID 8

/**
 * @brief Set POS network protocl stack's TX sequence ID\n
 * arg1 Provide a U32 data
 */
#define POS_CALL_FUNC_ID_SET_TX_ID 9

/**
 * @brief Get POS network protocl stack's yielding delay (unit: ms)\n
 * arg1 Provide a U32 pointer for returning
 */
#define POS_CALL_FUNC_ID_GET_NPS_YIELD_MS 10

/**
 * @brief Set POS network protocl stack's yielding delay (unit: ms)\n
 * arg1 Provide a U32 data
 */
#define POS_CALL_FUNC_ID_SET_NPS_YIELD_MS 11

/**
 * @brief Get console read callback function\n
 * arg1 Provide a @ref pos_func_msg_cb_t * pointer
 */
#define POS_CALL_FUNC_ID_GET_CONSOLE_RD_CB 12

/**
 * @brief Set console read callback function\n
 * arg1 Provide a @ref pos_func_msg_cb_t as the new callback function
 */
#define POS_CALL_FUNC_ID_SET_CONSOLE_RD_CB 13

/**
 * @brief Get abnormal monitoring information for coredump diagnostics\n
 * arg1 Provide a U32* for information length and a VOID ** for information pointer
 */
#define POS_CALL_FUNC_ID_GET_ABNORMAL_INFO 14

/**
 * @brief Set abnormal monitoring information for coredump diagnostics\n
 * arg1 Provide a U32 for information length and a VOID * for monitoring data information
 */
#define POS_CALL_FUNC_ID_SET_ABNORMAL_INFO 15

/**
 * @brief Set UART looping until escaping character detected\n
 * arg1 Provide a U32 value: B[23:16] for escaping char, B[15:8] for operating POS_IO_UART_x, B[7:0] for device POS_IO_UART_x
 */
#define POS_CALL_FUNC_ID_SET_UART_LOOP 16

/**
 * @brief Set No-Sleeping flag\n
 * arg1 Provide a U32 value: 0: allow slee, others: never sleep
 */
#define POS_CALL_FUNC_ID_SET_NO_SLEEP 17

/**
 * @brief Set No-Confirm flag\n
 * arg1 Provide a U32 value: 0: require to confirm, others: not-require
 */
#define POS_CALL_FUNC_ID_SET_NO_CONFIRM 18

/**
 * @brief Set/Trigger WDT Feed\n
 * arg1 Provide a U32 data for next WDT timer
 */
#define POS_CALL_FUNC_ID_SET_WDT_FEED 19

/**
 * @brief Set/I2C Addr/RegSize\n
 * arg1 Provide a IO pointer for IO handler\n
 * arg2 Provide a U32 value: bit[0..7] - ADDR8, Bit[8..15] - Reg size (8/16)
 */
#define POS_CALL_FUNC_ID_SET_I2C_ADDR  20

/**
 * @brief Set compensating ticks
 * arg1 Provide a I32 value for ticks increasing/decreasing
 */
#define POS_CALL_FUNC_ID_SET_TICK_COMPENSATE 21

/**
 * @brief Set console baudrate
 * arg1 Provide a U32 value for new console baudrate
 */
#define POS_CALL_FUNC_ID_SET_CONSOLE_BAUDRATE 22

/**
 * @brief Calc float in drv area\n
 * arg1 Provide a U32[3] for float calc\n
 * U32[0] = R
 * U32[1] = OPR
 * U32[2] = DATA
 * When OPR = 0, R = DATA (u32 to float translation)
 * When OPR = 1, R = R / DATA (float=float / u32)
 * When OPR = 2, R = R + DATA (float=float + i32)
 * When OPR = 3, R = R * DATA (float=float * i32)
 * When OPR = 4, R = R + DATA (float=float + float)
 * When OPR = 5, R = R - DATA (float=float - float)
 * When OPR = 6, R = R * DATA (float=float * float)
 * When OPR = 7, R = R / DATA (float=float / float) 
 */
#define POS_CALL_FUNC_ID_CALC_FLOAT 23
/**
 * @brief Float operation: float = u32
 */
#define POS_CALL_OP_FLOAT_LOAD_U32(os, f, v) do {\
  pos_u32_t __buf[3];\
  __buf[1] = 0;\
  __buf[2] = (pos_u32_t)v;\
  (os)->call(POS_CALL_FUNC_ID_CALC_FLOAT, __buf);\
  f = __buf[0];\
} while(0)
/**
 * @brief Float operation: float /= u32
 */
#define POS_CALL_OP_FLOAT_DIV_U32(os, f, v) do {\
  pos_u32_t __buf[3];\
  __buf[0] = f;\
  __buf[1] = 1;\
  __buf[2] = (pos_u32_t)v;\
  (os)->call(POS_CALL_FUNC_ID_CALC_FLOAT, __buf);\
  f = __buf[0];\
} while(0)
/**
 * @brief Float operation: float += i32
 */
#define POS_CALL_OP_FLOAT_ADD_I32(os, f, v) do {\
  pos_u32_t __buf[3];\
  __buf[0] = f;\
  __buf[1] = 2;\
  __buf[2] = (pos_u32_t)v;\
  (os)->call(POS_CALL_FUNC_ID_CALC_FLOAT, __buf);\
  f = __buf[0];\
} while(0)
/**
 * @brief Float operation: float *= i32
 */
#define POS_CALL_OP_FLOAT_MUL_I32(os, f, v) do {\
  pos_u32_t __buf[3];\
  __buf[0] = f;\
  __buf[1] = 3;\
  __buf[2] = (pos_u32_t)v;\
  (os)->call(POS_CALL_FUNC_ID_CALC_FLOAT, __buf);\
  f = __buf[0];\
} while(0)
/**
 * @brief Float operation: float += float
 */
#define POS_CALL_OP_FLOAT_ADD_FLOAT(os, f, v) do {\
  pos_u32_t __buf[3];\
  __buf[0] = f;\
  __buf[1] = 4;\
  __buf[2] = (pos_u32_t)v;\
  (os)->call(POS_CALL_FUNC_ID_CALC_FLOAT, __buf);\
  f = __buf[0];\
} while(0)
/**
 * @brief Float operation: float -= float
 */
#define POS_CALL_OP_FLOAT_SUB_FLOAT(os, f, v) do {\
  pos_u32_t __buf[3];\
  __buf[0] = f;\
  __buf[1] = 5;\
  __buf[2] = (pos_u32_t)v;\
  (os)->call(POS_CALL_FUNC_ID_CALC_FLOAT, __buf);\
  f = __buf[0];\
} while(0)
/**
 * @brief Float operation: float *= float
 */
#define POS_CALL_OP_FLOAT_MUL_FLOAT(os, f, v) do {\
  pos_u32_t __buf[3];\
  __buf[0] = f;\
  __buf[1] = 6;\
  __buf[2] = (pos_u32_t)v;\
  (os)->call(POS_CALL_FUNC_ID_CALC_FLOAT, __buf);\
  f = __buf[0];\
} while(0)
/**
 * @brief Float operation: float /= float
 */
#define POS_CALL_OP_FLOAT_DIV_FLOAT(os, f, v) do {\
  pos_u32_t __buf[3];\
  __buf[0] = f;\
  __buf[1] = 7;\
  __buf[2] = (pos_u32_t)v;\
  (os)->call(POS_CALL_FUNC_ID_CALC_FLOAT, __buf);\
  f = __buf[0];\
} while(0)

/**
 * @brief Get POS console IO bus ID
 * arg1 Provide a U32 pointer for returning
 */
#define POS_CALL_FUNC_ID_GET_CONSOLE_IO 24

/**
 * @brief Calc float in drv area\n
 * arg1 Provide a U32[3] for float calc\n
 * U32[0] = RAW_TMP
 * U32[1] = RAW_PRESS
 * U8[0..20] = NVM_PAR
 * When return U32[0]/U32[1] will be the final TMP/PRESS in float
 */
#define POS_CALL_FUNC_ID_CALC_BMP690 25
typedef struct {
  pos_u32_t vrslt[2]; /* #0..TMP, #1..PRESS */
  pos_u8_t nvm[24]; /* only 0..20 (21 Bytes) used) */
} pos_call_calc_bmp690_t;

/**
 * @brief GET_INFO @ref POS_CALL_FUNC_ID_GET_INFO Structure
 */
typedef struct {
  pos_u32_t   version; ///< POS version
  pos_u32_t   free; ///< Free heap memory
  pos_u32_t   free_min; ///< Least healp memory in history
  pos_u32_t   ticks;    ///< System Running Ticks (ms) 
  pos_u32_t   ticks_sleep; ///< System Sleeping Ticks (ms)
  pos_u16_t   os_stack; ///< POS kernel stack free size
  pos_u16_t   app_stack; ///< APP stack free size
  pos_u16_t   idle_stack; ///< IDLE stack free size
  pos_u16_t   startup_stack; ///< Startup stack free size
  pos_u32_t   rsvd[5]; ///< Reserved information
} pos_call_info_t;

#endif
