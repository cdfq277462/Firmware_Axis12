/**************************************************************************************************************
 File   :
 Author :
 Date   :
 Brief  :
**************************************************************************************************************/

/* Includes **************************************************************************************************/
#include "TEST.h"

/**************************************************************************************************************
 Declaration : app_test_task
 Parameters  : argument, Arbitrary user data set on osThreadNew
 Return Value: None
 Description : application test thread
**************************************************************************************************************/
__NO_RETURN void app_test_task(void *argument)
{
  /* Get the currently running thread */
  app_test_tid = osThreadGetId();
  ASSERT(app_test_tid != NULL);

  /* Start thread infinite loop */
  while (true) {
    /* Notify main thread activity flag */
    VERIFY(osThreadFlagsSet(app_main_tid, (uint32_t) argument) != osFlagsError);

    /* Please do something */
    VERIFY(osDelay(1000) == osOK);

    /* Passes the control to the next thread */
    VERIFY(osThreadYield() == osOK);
  }

  /* Never happen */
  ErrorHandler(__FILE__, __LINE__);
}
