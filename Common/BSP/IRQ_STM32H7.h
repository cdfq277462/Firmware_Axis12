/**************************************************************************************************************
 File   :
 Author :
 Date   :
 Brief  :
**************************************************************************************************************/

/* Define to prevent recursive inclusion *********************************************************************/
#ifndef __IRQ_STM32H7_H__
#define __IRQ_STM32H7_H__

/* __cplusplus ***********************************************************************************************/
#ifdef __cplusplus
  extern "C" {
#endif

/* Includes **************************************************************************************************/
#include "BSP_STM32H7.h"

/* Includes **************************************************************************************************/
#if defined (CORE_CM7)
  #include "SPI_STM32H7.h"
#elif defined (CORE_CM4)
  #include "COM_STM32H7.h"
  #include "ADC_STM32H7.h"
#endif

/* Function **************************************************************************************************/
extern void DebugMon_Handler  (void);
extern void UsageFault_Handler(void);
extern void BusFault_Handler  (void);
extern void MemManage_Handler (void);
extern void HardFault_Handler (void);
extern void NMI_Handler       (void);

/* __cplusplus ***********************************************************************************************/
#ifdef __cplusplus
  }
#endif

/* __IRQ_STM32H7_H__ *****************************************************************************************/
#endif
