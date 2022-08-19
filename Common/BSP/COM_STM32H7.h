/**************************************************************************************************************
 File   :
 Author :
 Date   :
 Brief  :
**************************************************************************************************************/

/* Define to prevent recursive inclusion *********************************************************************/
#ifndef __COM_STM32H7_H__
#define __COM_STM32H7_H__

/* __cplusplus ***********************************************************************************************/
#ifdef __cplusplus
  extern "C" {
#endif

/* Includes **************************************************************************************************/
#include "BSP_STM32H7.h"

/* Constants *************************************************************************************************/
#define UART1 USART1
#define UART2 USART2
#define UART3 USART3
#define UART6 USART6

/* Variables *************************************************************************************************/
extern MOT_HandleTypeDef hMOT1;

/* Variables *************************************************************************************************/
extern UART_HandleTypeDef hUART1;
extern UART_HandleTypeDef hUART2;
extern UART_HandleTypeDef hUART3;
extern UART_HandleTypeDef hUART4;
extern UART_HandleTypeDef hUART5;
extern UART_HandleTypeDef hUART6;
extern UART_HandleTypeDef hUART7;
extern UART_HandleTypeDef hUART8;

/* Functions *************************************************************************************************/
extern void HAL_UART_TxCpltCallback(UART_HandleTypeDef *hUARTx);
extern void HAL_UART_RxCpltCallback(UART_HandleTypeDef *hUARTx);
extern void HAL_UART_ErrorCallback (UART_HandleTypeDef *hUARTx);

extern HAL_StatusTypeDef MOT_Init (MOT_HandleTypeDef *hMOTx);
extern HAL_StatusTypeDef MOT_Read (MOT_HandleTypeDef *hMOTx, uint8_t Address);
extern HAL_StatusTypeDef MOT_Write(MOT_HandleTypeDef *hMOTx, uint8_t Address, int32_t Value);

/* __cplusplus ***********************************************************************************************/
#ifdef __cplusplus
  }
#endif

/* __COM_STM32H7_H__ *****************************************************************************************/
#endif
