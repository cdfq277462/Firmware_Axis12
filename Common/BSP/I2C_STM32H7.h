/**************************************************************************************************************
 File   :
 Author :
 Date   :
 Brief  :
**************************************************************************************************************/

/* Define to prevent recursive inclusion *********************************************************************/
#ifndef __I2C_STM32H7_H__
#define __I2C_STM32H7_H__

/* __cplusplus ***********************************************************************************************/
#ifdef __cplusplus
  extern "C" {
#endif

/* Includes **************************************************************************************************/
#include "MAIN.h"

/* Macro *****************************************************************************************************/
extern I2C_HandleTypeDef hI2C4;
extern DMA_HandleTypeDef hDMA_I2C4_RX;
extern DMA_HandleTypeDef hDMA_I2C4_TX;

/* Functions *************************************************************************************************/
#define EEP_Init()   I2C_Init(I2C4)
#define EEP_DeInit() I2C_DeInit(I2C4)

#define EEP_Read(MemAddress, pData, Size)  I2C_Read(I2C4, MemAddress, pData, Size)
#define EEP_Write(MemAddress, pData, Size) I2C_Write(I2C4, MemAddress, pData, Size)

extern HAL_StatusTypeDef I2C_Init  (I2C_TypeDef *I2Cx);
extern HAL_StatusTypeDef I2C_DeInit(I2C_TypeDef *I2Cx);

extern HAL_StatusTypeDef I2C_Read (I2C_TypeDef *I2Cx, uint16_t MemAddress, void       *pData, uint16_t Size);
extern HAL_StatusTypeDef I2C_Write(I2C_TypeDef *I2Cx, uint16_t MemAddress, const void *pData, uint16_t Size);

/* __I2C_STM32H7_H__ *****************************************************************************************/
#endif
