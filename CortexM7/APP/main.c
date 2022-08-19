/**************************************************************************************************************
 File  :
 Author:
 Date  :
 Brief :
**************************************************************************************************************/

/* Includes **************************************************************************************************/
#include "TASK.h"

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
  /* Turn off all MOTs */
  TMC_Disable(&hMOT1);
  TMC_Disable(&hMOT2);
  TMC_Disable(&hMOT3);
  TMC_Disable(&hMOT4);
  TMC_Disable(&hMOT5);
  TMC_Disable(&hMOT6);

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
 Declaration : SystemCoreClockConfig
 Parameters  : None
 Return Value: None
 Description : System Core Clock Configuration
**************************************************************************************************************/
static void SystemCoreClockConfig(void)
{
  RCC_ClkInitTypeDef       RCC_ClkInitStruct       = { 0 };
  RCC_OscInitTypeDef       RCC_OscInitStruct       = { 0 };
  RCC_PeriphCLKInitTypeDef RCC_PeriphClkInitStruct = { 0 };

  /* Supply configuration update enable */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /* Configure the main internal regulator output voltage */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
  while (__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY) == 0) { __NOP(); };

  /* Configure LSE Drive Capability */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState       = RCC_HSE_BYPASS;
  RCC_OscInitStruct.LSEState       = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM       = 2;
  RCC_OscInitStruct.PLL.PLLN       = 240;
  RCC_OscInitStruct.PLL.PLLP       = 2;
  RCC_OscInitStruct.PLL.PLLQ       = 15;
  RCC_OscInitStruct.PLL.PLLR       = 2;
  RCC_OscInitStruct.PLL.PLLRGE     = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL  = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN   = 0;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    ErrorHandler(__FILE__, __LINE__);
  }

  /* Initializes the CPU, AHB and APB buses clocks */
  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK    | RCC_CLOCKTYPE_SYSCLK
                                   | RCC_CLOCKTYPE_PCLK1   | RCC_CLOCKTYPE_PCLK2
                                   | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    ErrorHandler(__FILE__, __LINE__);
  }

  /* Initializes the peripherals clock */
  RCC_PeriphClkInitStruct.PeriphClockSelection      = RCC_PERIPHCLK_SPI1   | RCC_PERIPHCLK_SPI2
                                                    | RCC_PERIPHCLK_SPI3   | RCC_PERIPHCLK_SPI4
                                                    | RCC_PERIPHCLK_SPI5   | RCC_PERIPHCLK_SPI6
                                                    | RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_USART2
                                                    | RCC_PERIPHCLK_USART3 | RCC_PERIPHCLK_UART4
                                                    | RCC_PERIPHCLK_UART5  | RCC_PERIPHCLK_USART6
                                                    | RCC_PERIPHCLK_UART7  | RCC_PERIPHCLK_UART8;
  RCC_PeriphClkInitStruct.PLL3.PLL3M                = 2;
  RCC_PeriphClkInitStruct.PLL3.PLL3N                = 240;
  RCC_PeriphClkInitStruct.PLL3.PLL3P                = 15;
  RCC_PeriphClkInitStruct.PLL3.PLL3Q                = 15;
  RCC_PeriphClkInitStruct.PLL3.PLL3R                = 2;
  RCC_PeriphClkInitStruct.PLL3.PLL3RGE              = RCC_PLL3VCIRANGE_2;
  RCC_PeriphClkInitStruct.PLL3.PLL3VCOSEL           = RCC_PLL3VCOWIDE;
  RCC_PeriphClkInitStruct.PLL3.PLL3FRACN            = 0;
  RCC_PeriphClkInitStruct.Spi123ClockSelection      = RCC_SPI123CLKSOURCE_PLL;
  RCC_PeriphClkInitStruct.Spi45ClockSelection       = RCC_SPI45CLKSOURCE_PLL3;
  RCC_PeriphClkInitStruct.Spi6ClockSelection        = RCC_SPI6CLKSOURCE_PLL3;
  RCC_PeriphClkInitStruct.Usart16ClockSelection     = RCC_USART16CLKSOURCE_D2PCLK2;
  RCC_PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;

  if (HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInitStruct) != HAL_OK) {
    ErrorHandler(__FILE__, __LINE__);
  }

  /* Enable D2 domain SRAM1 Clock */
  __HAL_RCC_D2SRAM1_CLK_ENABLE();

  /* Enable D2 domain SRAM2 Clock */
  __HAL_RCC_D2SRAM2_CLK_ENABLE();

  /* Enable D2 domain SRAM3 Clock */
  __HAL_RCC_D2SRAM3_CLK_ENABLE();

#if 0U
  /* Activate CSI clock mondatory for I/O Compensation Cell */
  __HAL_RCC_CSI_ENABLE();

  /* Enable SYSCFG clock mondatory for I/O Compensation Cell */
  __HAL_RCC_SYSCFG_CLK_ENABLE();

  /* Enables the I/O Compensation Cell */
  HAL_EnableCompensationCell();
#endif
}

/**************************************************************************************************************
 Declaration : MPU_Config
 Parameters  : None
 Return Value: None
 Description : Configure MPU
**************************************************************************************************************/
static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = { 0 };

  /* Disables the MPU */
  HAL_MPU_Disable();

  /* Initializes and configures the Region and the memory to be protected */
  MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
  MPU_InitStruct.Number           = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress      = 0x30040000;
  MPU_InitStruct.Size             = MPU_REGION_SIZE_32KB;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable      = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable     = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Initializes and configures the Region and the memory to be protected */
  MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
  MPU_InitStruct.Number           = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress      = 0x38000000;
  MPU_InitStruct.Size             = MPU_REGION_SIZE_64KB;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Configure the MPU attributes as Normal Non Cacheable for Flash Bank B sectros 6 and 7 */
  MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
  MPU_InitStruct.Number           = MPU_REGION_NUMBER2;
  MPU_InitStruct.BaseAddress      = ADDR_FLASH_SECTOR_6_BANK2;
  MPU_InitStruct.Size             = MPU_REGION_SIZE_256KB;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**************************************************************************************************************
 Declaration : main
 Parameters  : None
 Return Value: None
 Description : Standard C program entry
**************************************************************************************************************/
int main(void)
{
  /* MPU Configuration */
  MPU_Config();

  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();

  /* Wait until CPU2 boots and enters in stop mode */
  while (__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) { __NOP(); }

  /* HAL library initialization */
  VERIFY(HAL_Init() == HAL_OK);

  /* Configure the system clock */
  SystemCoreClockConfig();
  SystemCoreClockUpdate();

  ASSERT(SystemCoreClock == 480000000UL); /* Core-M7 */
  ASSERT(SystemD2Clock   == 240000000UL); /* Core-M4 */

  /* GPIO Peripheral clock enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /* Clear 4671 all register values */
  memset((void *) hMOT1.Instance, 0, sizeof(TMC_TypeDef));
  memset((void *) hMOT2.Instance, 0, sizeof(TMC_TypeDef));
  memset((void *) hMOT3.Instance, 0, sizeof(TMC_TypeDef));
  memset((void *) hMOT4.Instance, 0, sizeof(TMC_TypeDef));
  memset((void *) hMOT5.Instance, 0, sizeof(TMC_TypeDef));
  memset((void *) hMOT6.Instance, 0, sizeof(TMC_TypeDef));

  /* Initialize the LED peripherals */
  VERIFY(LED_Init(LED0) == HAL_OK);
  VERIFY(LED_Init(LED1) == HAL_OK);
  VERIFY(LED_Init(LED2) == HAL_OK);

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_BDMA_CLK_ENABLE();

  /* Initializes the UDP FIFO Queue */
  VERIFY(FIFO_InitQueue(UDP_QueueStruct, UDP_QueueSpaces, 4096));

  /* HW semaphore Clock enable */
  __HAL_RCC_HSEM_CLK_ENABLE();

  /* Fast Take HSEM in order to notify the CPU2 */
  HAL_HSEM_FastTake(HSEM_ID_0);

  /* Release HSEM in order to notify the CPU2 */
  HAL_HSEM_Release(HSEM_ID_0, 0);

  /* Wait until CPU2 wakes up from stop mode */
  while (__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) { __NOP(); };

  /* Clear HSEM flag */
  __HAL_HSEM_CLEAR_FLAG(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));

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
