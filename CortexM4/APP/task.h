/**************************************************************************************************************
 File   :
 Author :
 Date   :
 Brief  :
**************************************************************************************************************/

/* Define to prevent recursive inclusion *********************************************************************/
#ifndef __TASK_H__
#define __TASK_H__

/* __cplusplus ***********************************************************************************************/
#ifdef __cplusplus
  extern "C" {
#endif

/* Includes **************************************************************************************************/
#include "MAIN.h"

/* Includes **************************************************************************************************/
#include "BSP_STM32H7.h"
#include "LED_STM32H7.h"
#include "COM_STM32H7.h"
#include "TMC_STM32H7.h"
#include "ADC_STM32H7.h"
#include "EEP_STM32H7.h"

/* Variables *************************************************************************************************/
extern const osThreadAttr_t app_main_attr;
extern const osThreadAttr_t app_sock_attr;
extern const osThreadAttr_t app_test_attr;
extern const osThreadAttr_t app_idle_attr;

/* Variables *************************************************************************************************/
extern osThreadId_t app_main_tid;
extern osThreadId_t app_sock_tid;
extern osThreadId_t app_test_tid;
extern osThreadId_t app_idle_tid;

/* Functions *************************************************************************************************/
extern void app_main_task(void *argument);

/* __cplusplus ***********************************************************************************************/
#ifdef __cplusplus
  }
#endif

/* __TASK_H__ ************************************************************************************************/
#endif
