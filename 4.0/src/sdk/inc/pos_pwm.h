/**
 * @file  pos_pwm.h  
 * @brief PWM library
 * @author Runby F.
 * @date 2022-3-31
 * @copyright Polysense
 */

#ifndef __POS_PWM_H__
#define __POS_PWM_H__

#include "pos_types.h"

/**
 *@brief PWM channel
 */
typedef enum {
  PWM_ID0 = 0,
  PWM_ID1 = 1,    
  PWM_ID2 = 2,
  PWM_ID3 = 3,
} pos_pwm_id_t;

/**
 *@brief PWM mode
 */
typedef enum {
  PWM_MODE_0 = 0, /**< Classical beep duzzer output mode */
} pos_pwm_mode_t;

/**
 * @brief PWM operating librarty
 */ 
typedef struct {
  /**
  * @brief Init PWM
  * @param[in] id PWM resource ID, refer to @ref pos_pwm_id_t
  * @param[in] hz Working frequency (0: will shutdown this PWM and release all resources)
  * @param[in] duty PWM duty ratio in unit of 0.01%
  */
  void (*init)(pos_pwm_id_t id, pos_u32_t hz, pos_u32_t duty);

  /**
  * @brief Start/stop PWM
  * @param[in] id PWM resource ID, refer to @ref pos_pwm_id_t
  * @param[in] on 0: Stop\n
                  Others: Start
  */
  void (*set)(pos_pwm_id_t id, pos_u32_t on);

  /**
  * @brief Get PWM status
  * @param[in] id PWM resource ID, refer to @ref pos_pwm_id_t
  * @param[out] p_hz Retrun PWM frequency (0: shutdown, others: running frequency)
  * @param[out] p_duty Return PWM duty ratio in unit of 0.01%
  * @note p_hz's Bit[31] is used to tell whether PWM is currently outputing (1: working, 0: stop)
  */
  void (*get)(pos_pwm_id_t id, pos_u32_t *p_hz, pos_u32_t *p_duty);  

} pos_lib_pwm_t;


#endif
