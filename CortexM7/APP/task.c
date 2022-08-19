/**************************************************************************************************************
 File   :
 Author :
 Date   :
 Brief  :
**************************************************************************************************************/

/* Includes **************************************************************************************************/
#include "TASK.h"
#include "TEST.h"
#include "idle.h"

/* Variables *************************************************************************************************/
const osThreadAttr_t app_main_attr = { .name = "main", .stack_size =  2048, .priority = osPriorityNormal2 };
const osThreadAttr_t app_test_attr = { .name = "test", .stack_size = 16384, .priority = osPriorityNormal3 };
const osThreadAttr_t app_idle_attr = { .name = "idle", .stack_size =  2048, .priority = osPriorityNormal1 };

/* Variables *************************************************************************************************/
osThreadId_t app_main_tid;
osThreadId_t app_test_tid;
osThreadId_t app_idle_tid;

/**************************************************************************************************************
 Declaration : app_main_task
 Parameters  : argument, Arbitrary user data set on osThreadNew
 Return Value: None
 Description : application main thread
**************************************************************************************************************/
__NO_RETURN void app_main_task(void *argument)
{
  /* Get the currently running thread */
  app_main_tid = osThreadGetId();
  ASSERT(app_main_tid != NULL);

  /* Create the application threads */
  VERIFY(osThreadNew(app_idle_task, (void *) 0x00000002UL, &app_idle_attr) != NULL);
	VERIFY(osThreadNew(app_test_task, (void *) 0x00000001UL, &app_test_attr) != NULL);
  

  /* Start thread infinite loop */
  while (true) {
    /* Check that all threads are alive */
    if ((osThreadFlagsGet() & 0x03) == 0x03) {
      VERIFY(osThreadFlagsClear(0x03) != osFlagsError);
    }

    /* LED flashes every half second */
    LED_Toggle(LED0);
    VERIFY(osDelay(500) == osOK);

    /* Pass control to the next thread */
    VERIFY(osThreadYield() == osOK);
  }

  /* Never happen */
  ErrorHandler(__FILE__, __LINE__);
}
