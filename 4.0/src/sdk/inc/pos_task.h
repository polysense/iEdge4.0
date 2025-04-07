/**
 * @file  pos_task.h  
 * @brief Task operation library
 * @author Runby F.
 * @date 2022-2-15
 * @copyright Polysense
 */

#ifndef __POS_TASK_H__
#define __POS_TASK_H__

#include "pos_types.h"

/**
 * @brief TASK handle type
 */ 
typedef void *pos_task_handle_t;

/**
 * @brief TASK library
 */ 
typedef struct {
  /**
  * @brief Disable task scheduling (used during accessing multi-tasks sharing resources)
  */
  void (*lock)(void);

  /**
  * @brief Re-enable task scheduling
  */
  void (*unlock)(void);

  /**
  * @brief Disable task scheduling and IRQ interrupt
  */
  void (*lock_irq)(void);

  /**
  * @brief Re-enable task schedulgin and IRQ interrupt
  */
  void (*unlock_irq)(void);

  /**
  * @brief Return current task handle
  */
  pos_task_handle_t (*self)(void);
 } pos_lib_task_t;


#endif
