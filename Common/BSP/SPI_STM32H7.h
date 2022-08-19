/**************************************************************************************************************
 File   :
 Author :
 Date   :
 Brief  :
**************************************************************************************************************/

/* Define to prevent recursive inclusion *********************************************************************/
#ifndef __SPI_STM32H7_H__
#define __SPI_STM32H7_H__

/* __cplusplus ***********************************************************************************************/
#ifdef __cplusplus
  extern "C" {
#endif

/* Includes **************************************************************************************************/
#include "BSP_STM32H7.h"

/* Variables *************************************************************************************************/
extern SPI_HandleTypeDef hSPI1;
extern SPI_HandleTypeDef hSPI2;
extern SPI_HandleTypeDef hSPI3;
extern SPI_HandleTypeDef hSPI4;
extern SPI_HandleTypeDef hSPI5;
extern SPI_HandleTypeDef hSPI6;

/* Variables *************************************************************************************************/
extern DMA_HandleTypeDef hDMA_SPI1_RX;
extern DMA_HandleTypeDef hDMA_SPI1_TX;
extern DMA_HandleTypeDef hDMA_SPI2_RX;
extern DMA_HandleTypeDef hDMA_SPI2_TX;
extern DMA_HandleTypeDef hDMA_SPI3_RX;
extern DMA_HandleTypeDef hDMA_SPI3_TX;
extern DMA_HandleTypeDef hDMA_SPI4_RX;
extern DMA_HandleTypeDef hDMA_SPI4_TX;
extern DMA_HandleTypeDef hDMA_SPI5_RX;
extern DMA_HandleTypeDef hDMA_SPI5_TX;
extern DMA_HandleTypeDef hDMA_SPI6_RX;
extern DMA_HandleTypeDef hDMA_SPI6_TX;

/* Variables *************************************************************************************************/
extern MOT_HandleTypeDef hMOT1;
extern MOT_HandleTypeDef hMOT2;
extern MOT_HandleTypeDef hMOT3;
extern MOT_HandleTypeDef hMOT4;
extern MOT_HandleTypeDef hMOT5;
extern MOT_HandleTypeDef hMOT6;

/* Functions *************************************************************************************************/
extern HAL_StatusTypeDef SPI_Init  (SPI_HandleTypeDef *hSPIx);
extern HAL_StatusTypeDef SPI_DeInit(SPI_HandleTypeDef *hSPIx);

extern void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hSPIx);
extern void HAL_SPI_ErrorCallback   (SPI_HandleTypeDef *hSPIx);

extern HAL_StatusTypeDef MOT_Init (MOT_HandleTypeDef *hMOTx);
extern HAL_StatusTypeDef MOT_Read (MOT_HandleTypeDef *hMOTx, uint8_t Address);
extern HAL_StatusTypeDef MOT_Write(MOT_HandleTypeDef *hMOTx, uint8_t Address, int32_t Value);

/* __cplusplus ***********************************************************************************************/
#ifdef __cplusplus
  }
#endif

/* __SPI_STM32H7_H__ *****************************************************************************************/
#endif
