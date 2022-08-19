/**************************************************************************************************************
 File   :
 Author :
 Date   :
 Brief  :
**************************************************************************************************************/

/* Includes **************************************************************************************************/
#include "LED_STM32H7.h"

/* Macros ****************************************************************************************************/
#define IS_LED_ALL_INSTANCE(LEDx) (((LEDx) == LED0) || ((LEDx) == LED1) || ((LEDx) == LED2))

/* Variables *************************************************************************************************/
static LED_TypeDef LED_Struct[] = {
  { .Port = GPIOB, .Pin = GPIO_PIN_0 , },
  { .Port = GPIOE, .Pin = GPIO_PIN_1 , },
  { .Port = GPIOB, .Pin = GPIO_PIN_14, },
};

/* Variables *************************************************************************************************/
LED_TypeDef * const LED0 = &LED_Struct[0];
LED_TypeDef * const LED1 = &LED_Struct[1];
LED_TypeDef * const LED2 = &LED_Struct[2];

/**************************************************************************************************************
 Declaration : LED_Init
 Parameters  : LEDx, LED to be configured
 Return Value: None
 Description : Configures LED on GPIO
**************************************************************************************************************/
HAL_StatusTypeDef LED_Init(LED_TypeDef * const LEDx)
{
  /* Check the parameters */
  if (LEDx == NULL) {
    return HAL_ERROR;
  }

  assert_param(IS_LED_ALL_INSTANCE (LEDx      ));
  assert_param(IS_GPIO_ALL_INSTANCE(LEDx->Port));
  assert_param(IS_GPIO_PIN         (LEDx->Pin ));

  /* Enable the GPIO peripheral clock */
  __HAL_RCC_GPIO_CLK_ENABLE(LEDx->Port);

  /* Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LEDx->Port, LEDx->Pin, GPIO_PIN_RESET);

  /* Initializes the LED peripheral */
  GPIO_InitTypeDef GPIO_InitStruct = { .Pin   = LEDx->Pin,
                                       .Mode  = GPIO_MODE_OUTPUT_PP,
                                       .Pull  = GPIO_PULLUP,
                                       .Speed = GPIO_SPEED_FREQ_LOW };

  HAL_GPIO_Init(LEDx->Port, &GPIO_InitStruct);

  /* Return HAL status */
  return HAL_OK;
}

/**************************************************************************************************************
 Declaration : LED_DeInit
 Parameters  : LEDx, LED to be configured
 Return Value: None
 Description : DeInit LEDs
**************************************************************************************************************/
HAL_StatusTypeDef LED_DeInit(LED_TypeDef * const LEDx)
{
  /* Check the parameters */
  if (LEDx == NULL) {
    return HAL_ERROR;
  }

  assert_param(IS_LED_ALL_INSTANCE (LEDx      ));
  assert_param(IS_GPIO_ALL_INSTANCE(LEDx->Port));
  assert_param(IS_GPIO_PIN         (LEDx->Pin ));

  /* Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LEDx->Port, LEDx->Pin, GPIO_PIN_SET);

  /* De-initializes the GPIOx peripheral */
  HAL_GPIO_DeInit(LEDx->Port, LEDx->Pin);

  /* Return HAL status */
  return HAL_OK;
}
