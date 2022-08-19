/**************************************************************************************************************
 File  :
 Author:
 Date  :
 Brief :
**************************************************************************************************************/

/* Includes **************************************************************************************************/
#include "TASK.h"

/* Variables *************************************************************************************************/
struct LocalMS LocalMS;

/**************************************************************************************************************
 Declaration : LMS_Load
 Parameters  : None
 Return Value: Error Status
 Description : Load local machine settings
**************************************************************************************************************/
ErrorStatus LMS_Load(struct LocalMS *pLocalMS)
{
  pLocalMS->DeviceID = 1;

  /* Return function status */
  return SUCCESS;
}

/**************************************************************************************************************
 Declaration : LMS_Load
 Parameters  : None
 Return Value: Error Status
 Description : Load local machine settings
**************************************************************************************************************/
ErrorStatus LMS_Save(struct LocalMS *pLocalMS)
{
  /* Return function status */
  return SUCCESS;
}

/**************************************************************************************************************
 Declaration : HAL_Delay
 Parameters  : Delay, Specifies the delay time length, in milliseconds
 Return Value: None
 Description : Override default HAL_Delay function
**************************************************************************************************************/
void HAL_Delay(uint32_t Delay)
{
  /* If the kernel is running */
  if (osKernelGetState() == osKernelRunning) {
    VERIFY(osDelay(Delay) == osOK);
  }

  /* If Kernel is not running */
  else {
    /* Add a freq to guarantee minimum wait */
    uint32_t tickstart = HAL_GetTick();
    uint32_t wait = Delay;

    if (wait < HAL_MAX_DELAY) {
      wait += (uint32_t) (uwTickFreq);
    }

    while ((HAL_GetTick() - tickstart) < wait) { __NOP(); };
  }
}

/**************************************************************************************************************
 Declaration : HAL_InitTic
 Parameters  : TickPriority, Tick interrupt priority
 Return Value: Tick value
 Description : HAL status
**************************************************************************************************************/
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  UNUSED(TickPriority);

  /* Return HAL status */
  return HAL_OK;
}

/**************************************************************************************************************
 Declaration : HAL_GetTick
 Parameters  : None
 Return Value: Tick value
 Description : Override default HAL_GetTick function
**************************************************************************************************************/
uint32_t HAL_GetTick(void)
{
  /* If the kernel is running */
  if (osKernelGetState() == osKernelRunning) {
    VERIFY(osThreadYield() == osOK);
    return osKernelGetTickCount();
  }

  /* If Kernel is not running */
  else {
    static uint32_t ticks = 0U;

    for (uint32_t i = (SystemCoreClock >> 14U); i > 0U; i--) {
      __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
      __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    }

    return ++ticks;
  }
}

/**************************************************************************************************************
 Declaration : ErrorHandler
 Parameters  : file, Source file of the assertion
 Parameters  : line, Source line of the assertion
 Return Value: None
 Description : This function is executed in case of error occurrence
**************************************************************************************************************/
void ErrorHandler(const char *file, int line)
{
  /* Turn off all LEDs */
  LED_Off(LED0);
  LED_Off(LED1);

  /* Disable IRQ Interrupts */
  __disable_irq();

#ifdef NDEBUG
  /* Initiates a system reset request to reset the MCU */
  NVIC_SystemReset();
#else
  /* Causes the processor to enter Debug state */
  __BKPT(0);
#endif

  /* Infinite loop will never happen */
  while (true) { __NOP(); };
}

/**************************************************************************************************************
 Declaration : assert_failed
 Parameters  : file, Source file of the assertion
 Parameters  : line, Source line of the assertion
 Return Value: None
 Description :  Override default assert_failed function
**************************************************************************************************************/
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* Handle this error to ErrorHandler */
  ErrorHandler((const char *) file, (int) line);

  /* Infinite loop will never happen */
  while (true) { __NOP(); };
}
#endif

/**************************************************************************************************************
 Declaration : __aeabi_assert
 Parameters  : expr, Assert expression that was not true
 Parameters  : file, Source file of the assertion
 Parameters  : line, Source line of the assertion
 Return Value: None
 Description : Override default __aeabi_assert function
**************************************************************************************************************/
#ifndef NDEBUG
void __aeabi_assert(const char *expr, const char *file, int line)
{
  /* Handle this error to ErrorHandler */
  ErrorHandler(file, line);

  /* Infinite loop will never happen */
  while (true) { __NOP(); };
}
#endif

/**************************************************************************************************************
 Declaration : main
 Parameters  : None
 Return Value: None
 Description : Standard C program entry
**************************************************************************************************************/
int main(void)
{
  /* HW semaphore Clock enable */
  __HAL_RCC_HSEM_CLK_ENABLE();

  /* Activate HSEM notification for Cortex-M4 */
  HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));

  /* Clear pending event */
  HAL_PWREx_ClearPendingEvent();

  /* Enter a Domain to DSTOP mode */
  HAL_PWREx_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE, PWR_D2_DOMAIN);

  /* Clear HSEM flag */
  __HAL_HSEM_CLEAR_FLAG(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));

  /* HAL library initialization */
  HAL_Init();

  /* Initialize CMSIS-RTOS2 */
  if (osKernelInitialize() != osOK) {
    ErrorHandler(__FILE__, __LINE__);
  }

  /* Create application main thread */
  if (osThreadNew(app_main_task, NULL, &app_main_attr) == NULL) {
    ErrorHandler(__FILE__, __LINE__);
  }

  /* Start thread execution */
  if (osKernelStart() != osOK) {
    ErrorHandler(__FILE__, __LINE__);
  }

  /* Infinite loop will never happen */
  while (true) { __NOP(); };
}
