/**************************************************************************************************************
 File   :
 Author :
 Date   :
 Brief  :
**************************************************************************************************************/

/* Define to prevent recursive inclusion *********************************************************************/
#ifndef __LED_STM32H7_H__
#define __LED_STM32H7_H__

/* __cplusplus ***********************************************************************************************/
#ifdef __cplusplus
  extern "C" {
#endif

/* Includes **************************************************************************************************/
#include "BSP_STM32H7.h"

/* Typedef ***************************************************************************************************/
typedef struct {
  GPIO_TypeDef *Port;
  uint32_t      Pin;
} const LED_TypeDef;

/* Variables *************************************************************************************************/
extern LED_TypeDef * const LED0;
extern LED_TypeDef * const LED1;
extern LED_TypeDef * const LED2;

/* Macros ****************************************************************************************************/
#define LED_Off(LEDx)    WRITE_REG(LEDx->Port->BSRR, LEDx->Pin << 0x10)
#define LED_On(LEDx)     WRITE_REG(LEDx->Port->BSRR, LEDx->Pin)
#define LED_Toggle(LEDx) WRITE_REG(LEDx->Port->ODR, READ_REG(LEDx->Port->ODR) ^ LEDx->Pin)

/* Functions *************************************************************************************************/
extern HAL_StatusTypeDef LED_Init  (LED_TypeDef * const LEDx);
extern HAL_StatusTypeDef LED_DeInit(LED_TypeDef * const LEDx);

/* __cplusplus ***********************************************************************************************/
#ifdef __cplusplus
  }
#endif

/* __LED_STM32H7_H__ *****************************************************************************************/
#endif
