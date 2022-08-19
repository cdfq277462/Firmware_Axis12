/**************************************************************************************************************
 File   :
 Author :
 Date   :
 Brief  :
**************************************************************************************************************/

/* Includes **************************************************************************************************/
#include "TEST.h"

/**************************************************************************************************************
 Declaration : app_uart_task
 Parameters  : argument, Arbitrary user data set on osThreadNew
 Return Value: None
 Description : application uart thread
**************************************************************************************************************/
__NO_RETURN void app_uart_task(void *argument)
{
  uint8_t Index = 0xFF;

  /* Get the currently running thread */
  app_uart_tid = osThreadGetId();
  ASSERT(app_test_tid != NULL);
  
  /* Wait for the equipment to be ready */
  VERIFY(osDelay(2000) == osOK);

  /* Infinite loop */
  while (true) {    
    VERIFY(osThreadYield() == osOK); /* Passes the control to the next thread */
    VERIFY(osThreadFlagsSet(app_main_tid, (uint32_t) argument) != osFlagsError);

    /* Sequentially read the TMC register value */
    Index += 0x01;
    Index &= 0x7F;

    VERIFY(TMC_Read(TMC_MOTOR_1, Index) == HAL_OK);
    VERIFY(osDelay(2) == osOK);
  }

  /* Never happen */
  ErrorHandler(__FILE__, __LINE__);
}
