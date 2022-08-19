/**************************************************************************************************************
 File   :
 Author :
 Date   :
 Brief  :
**************************************************************************************************************/

/* Includes **************************************************************************************************/
#include "IRQ_STM32H7.h"

/**************************************************************************************************************
 Declaration : NMI_Handler
 Parameters  : None
 Return Value: None
 Description : This function handles Non maskable interrupt.
**************************************************************************************************************/
void NMI_Handler(void)
{
  ErrorHandler(__FILE__, __LINE__);
}

/**************************************************************************************************************
 Declaration : HardFault_Handler
 Parameters  : None
 Return Value: None
 Description : This function handles Hard fault interrupt
**************************************************************************************************************/
void HardFault_Handler(void)
{
  ErrorHandler(__FILE__, __LINE__);
}

/**************************************************************************************************************
 Declaration : MemManage_Handle
 Parameters  : None
 Return Value: None
 Description : This function handles Memory management fault
**************************************************************************************************************/
void MemManage_Handler(void)
{
  ErrorHandler(__FILE__, __LINE__);
}

/**************************************************************************************************************
 Declaration : BusFault_Handler
 Parameters  : None
 Return Value: None
 Description : This function handles Pre-fetch fault, memory access fault
**************************************************************************************************************/
void BusFault_Handler(void)
{
  ErrorHandler(__FILE__, __LINE__);
}

/**************************************************************************************************************
 Declaration : UsageFault_Handle
 Parameters  : None
 Return Value: None
 Description : This function handles Undefined instruction or illegal state
**************************************************************************************************************/
void UsageFault_Handler(void)
{
  ErrorHandler(__FILE__, __LINE__);
}

/**************************************************************************************************************
 Declaration : DebugMon_Handler
 Parameters  : None
 Return Value: None
 Description : This function handles Debug monitor
**************************************************************************************************************/
void DebugMon_Handler(void)
{
  ErrorHandler(__FILE__, __LINE__);
}

/**************************************************************************************************************
 Declaration : SPI1_IRQHandler
 Parameters  : None
 Return Value: None
 Description : This function handles SPI1 global interrupt
**************************************************************************************************************/
#if defined(HAL_SPI_MODULE_ENABLED) && defined(CORE_CM7)
void SPI1_IRQHandler(void)
{
  HAL_SPI_IRQHandler(&hSPI1);
}
#endif

/**************************************************************************************************************
 Declaration : SPI2_IRQHandler
 Parameters  : None
 Return Value: None
 Description : This function handles SPI2 global interrupt
**************************************************************************************************************/
#if defined(HAL_SPI_MODULE_ENABLED) && defined(CORE_CM7)
void SPI2_IRQHandler(void)
{
  HAL_SPI_IRQHandler(&hSPI2);
}
#endif

/**************************************************************************************************************
 Declaration : SPI3_IRQHandler
 Parameters  : None
 Return Value: None
 Description : This function handles SPI3 global interrupt
**************************************************************************************************************/
#if defined(HAL_SPI_MODULE_ENABLED) && defined(CORE_CM7)
void SPI3_IRQHandler(void)
{
  HAL_SPI_IRQHandler(&hSPI3);
}
#endif

/**************************************************************************************************************
 Declaration : SPI4_IRQHandler
 Parameters  : None
 Return Value: None
 Description : This function handles SPI4 global interrupt
**************************************************************************************************************/
#if defined(HAL_SPI_MODULE_ENABLED) && defined(CORE_CM7)
void SPI4_IRQHandler(void)
{
  HAL_SPI_IRQHandler(&hSPI4);
}
#endif

/**************************************************************************************************************
 Declaration : SPI5_IRQHandler
 Parameters  : None
 Return Value: None
 Description : This function handles SPI5 global interrupt
**************************************************************************************************************/
#if defined(HAL_SPI_MODULE_ENABLED) && defined(CORE_CM7)
void SPI5_IRQHandler(void)
{
  HAL_SPI_IRQHandler(&hSPI5);
}
#endif

/**************************************************************************************************************
 Declaration : SPI6_IRQHandler
 Parameters  : None
 Return Value: None
 Description : This function handles SPI6 global interrupt
**************************************************************************************************************/
#if defined(HAL_SPI_MODULE_ENABLED) && defined(CORE_CM7)
void SPI6_IRQHandler(void)
{
  HAL_SPI_IRQHandler(&hSPI6);
}
#endif

/**************************************************************************************************************
 Declaration : DMA1_Stream0_IRQHandler
 Parameters  : None
 Return Value: None
 Description : This function handles DMA1 stream0 global interrupt
**************************************************************************************************************/
#if defined(HAL_DMA_MODULE_ENABLED) && defined(CORE_CM7)
void DMA1_Stream0_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hDMA_SPI1_RX);
}
#endif

/**************************************************************************************************************
 Declaration : DMA1_Stream1_IRQHandler
 Parameters  : None
 Return Value: None
 Description : This function handles DMA1 stream1 global interrupt
**************************************************************************************************************/
#if defined(HAL_DMA_MODULE_ENABLED) && defined(CORE_CM7)
void DMA1_Stream1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hDMA_SPI1_TX);
}
#endif

/**************************************************************************************************************
 Declaration : DMA1_Stream2_IRQHandler
 Parameters  : None
 Return Value: None
 Description : This function handles DMA1 stream2 global interrupt
**************************************************************************************************************/
#if defined(HAL_DMA_MODULE_ENABLED) && defined(CORE_CM7)
void DMA1_Stream2_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hDMA_SPI2_RX);
}
#endif

/**************************************************************************************************************
 Declaration : DMA1_Stream3_IRQHandler
 Parameters  : None
 Return Value: None
 Description : This function handles DMA1 stream3 global interrupt
**************************************************************************************************************/
#if defined(HAL_DMA_MODULE_ENABLED) && defined(CORE_CM7)
void DMA1_Stream3_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hDMA_SPI2_TX);
}
#endif

/**************************************************************************************************************
 Declaration : DMA1_Stream4_IRQHandler
 Parameters  : None
 Return Value: None
 Description : This function handles DMA1 stream4 global interrupt
**************************************************************************************************************/
#if defined(HAL_DMA_MODULE_ENABLED) && defined(CORE_CM7)
void DMA1_Stream4_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hDMA_SPI3_RX);
}
#endif

/**************************************************************************************************************
 Declaration : DMA1_Stream5_IRQHandler
 Parameters  : None
 Return Value: None
 Description : This function handles DMA1 stream5 global interrupt
**************************************************************************************************************/
#if defined(HAL_DMA_MODULE_ENABLED) && defined(CORE_CM7)
void DMA1_Stream5_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hDMA_SPI3_TX);
}
#endif

/**************************************************************************************************************
 Declaration : DMA1_Stream6_IRQHandler
 Parameters  : None
 Return Value: None
 Description : This function handles DMA1 stream6 global interrupt
**************************************************************************************************************/
#if defined(HAL_DMA_MODULE_ENABLED) && defined(CORE_CM7)
void DMA1_Stream6_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hDMA_SPI4_RX);
}
#endif

/**************************************************************************************************************
 Declaration : DMA1_Stream7_IRQHandler
 Parameters  : None
 Return Value: None
 Description : This function handles DMA1 stream7 global interrupt
**************************************************************************************************************/
#if defined(HAL_DMA_MODULE_ENABLED) && defined(CORE_CM7)
void DMA1_Stream7_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hDMA_SPI4_TX);
}
#endif

/**************************************************************************************************************
 Declaration : DMA2_Stream0_IRQHandler
 Parameters  : None
 Return Value: None
 Description : This function handles DMA2 stream0 global interrupt
**************************************************************************************************************/
#if defined(HAL_DMA_MODULE_ENABLED) && defined(CORE_CM7)
void DMA2_Stream0_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hDMA_SPI5_RX);
}
#endif

/**************************************************************************************************************
 Declaration : DMA2_Stream1_IRQHandler
 Parameters  : None
 Return Value: None
 Description : This function handles DMA2 stream1 global interrupt
**************************************************************************************************************/
#if defined(HAL_DMA_MODULE_ENABLED) && defined(CORE_CM7)
void DMA2_Stream1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hDMA_SPI5_TX);
}
#endif

/**************************************************************************************************************
 Declaration : DMA2_Stream2_IRQHandler
 Parameters  : None
 Return Value: None
 Description : This function handles DMA1_Stream0_IRQHandler interrupt request
**************************************************************************************************************/
#if defined(HAL_ADC_MODULE_ENABLED) && defined(CORE_CM4)
void DMA2_Stream2_IRQHandler(void)
{
  HAL_DMA_IRQHandler(hADC1.DMA_Handle);
}
#endif

/**************************************************************************************************************
 Declaration : BDMA_Channel0_IRQHandler
 Parameters  : None
 Return Value: None
 Description : This function handles BDMA channel0 global interrupt
**************************************************************************************************************/
#if defined(HAL_DMA_MODULE_ENABLED) && defined(CORE_CM7)
void BDMA_Channel0_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hDMA_SPI6_RX);
}
#endif

/**************************************************************************************************************
 Declaration : BDMA_Channel1_IRQHandler
 Parameters  : None
 Return Value: None
 Description : This function handles BDMA channel1 global interrupt
**************************************************************************************************************/
#if defined(HAL_DMA_MODULE_ENABLED) && defined(CORE_CM7)
void BDMA_Channel1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hDMA_SPI6_TX);
}
#endif

/**************************************************************************************************************
 Declaration : USART1_IRQHandler
 Parameters  : None
 Return Value: None
 Description : This function handles USART1 global interrupt
**************************************************************************************************************/
#if defined(HAL_UART_MODULE_ENABLED) && defined(CORE_CM4)
void USART1_IRQHandler(void)
{
  HAL_UART_IRQHandler(&hUART1);
}
#endif

/**************************************************************************************************************
 Declaration : USART2_IRQHandler
 Parameters  : None
 Return Value: None
 Description : This function handles USART2 global interrupt
**************************************************************************************************************/
#if defined(HAL_UART_MODULE_ENABLED) && defined(CORE_CM4)
void USART2_IRQHandler(void)
{
  HAL_UART_IRQHandler(&hUART2);
}
#endif

/**************************************************************************************************************
 Declaration : USART3_IRQHandler
 Parameters  : None
 Return Value: None
 Description : This function handles USART3 global interrupt
**************************************************************************************************************/
#if defined(HAL_UART_MODULE_ENABLED) && defined(CORE_CM4)
void USART3_IRQHandler(void)
{
  HAL_UART_IRQHandler(&hUART3);
}
#endif

/**************************************************************************************************************
 Declaration : UART4_IRQHandler
 Parameters  : None
 Return Value: None
 Description : This function handles UART4 global interrupt
**************************************************************************************************************/
#if defined(HAL_UART_MODULE_ENABLED) && defined(CORE_CM4)
void UART4_IRQHandler(void)
{
  HAL_UART_IRQHandler(&hUART4);
}
#endif

/**************************************************************************************************************
 Declaration : UART5_IRQHandler
 Parameters  : None
 Return Value: None
 Description : This function handles UART5 global interrupt
**************************************************************************************************************/
#if defined(HAL_UART_MODULE_ENABLED) && defined(CORE_CM4)
void UART5_IRQHandler(void)
{
  HAL_UART_IRQHandler(&hUART5);
}
#endif

/**************************************************************************************************************
 Declaration : USART6_IRQHandler
 Parameters  : None
 Return Value: None
 Description : This function handles USART6 global interrupt
**************************************************************************************************************/
#if defined(HAL_UART_MODULE_ENABLED) && defined(CORE_CM4)
void USART6_IRQHandler(void)
{
  HAL_UART_IRQHandler(&hUART6);
}
#endif

/**************************************************************************************************************
 Declaration : UART7_IRQHandler
 Parameters  : None
 Return Value: None
 Description : This function handles UART7 global interrupt
**************************************************************************************************************/
#if defined(HAL_UART_MODULE_ENABLED) && defined(CORE_CM4)
void UART7_IRQHandler(void)
{
  HAL_UART_IRQHandler(&hUART7);
}
#endif

/**************************************************************************************************************
 Declaration : UART8_IRQHandler
 Parameters  : None
 Return Value: None
 Description : This function handles UART8 global interrupt
**************************************************************************************************************/
#if defined(HAL_UART_MODULE_ENABLED) && defined(CORE_CM4)
void UART8_IRQHandler(void)
{
  HAL_UART_IRQHandler(&hUART8);
}
#endif

/**************************************************************************************************************
 Declaration : HSEM1_IRQHandler
 Parameters  : None
 Return Value: None
 Description : This function handles HSEM1 global interrupt
**************************************************************************************************************/
#if defined(HAL_HSEM_MODULE_ENABLED) && defined(CORE_CM7)
void HSEM1_IRQHandler(void)
{
  HAL_HSEM_IRQHandler();
}
#endif

/**************************************************************************************************************
 Declaration : HSEM2_IRQHandler
 Parameters  : None
 Return Value: None
 Description : This function handles HSEM2 global interrupt
**************************************************************************************************************/
#if defined(HAL_HSEM_MODULE_ENABLED) && defined(CORE_CM4)
void HSEM2_IRQHandler(void)
{
  HAL_HSEM_IRQHandler();
}
#endif

