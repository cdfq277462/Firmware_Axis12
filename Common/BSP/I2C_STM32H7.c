/**************************************************************************************************************
 File   :
 Author :
 Date   :
 Brief  :
**************************************************************************************************************/

/* Includes **************************************************************************************************/
#include "SPI_STM32H7.h"

/* I2C4 Resources ********************************************************************************************/
#define I2C1_TRANSFER_COMPLETED 0x40000000

#define I2C4_MEMADD_SIZE I2C_MEMADD_SIZE_16BIT
#define I2C4_DEVICE_ADDR 0xA0
#define I2C4_DEVICE_PGSZ 0x20

static osThreadId_t I2C4_RxThreadID = NULL;
static osThreadId_t I2C4_TxThreadID = NULL;

I2C_HandleTypeDef hI2C4        = { 0 };
DMA_HandleTypeDef hDMA_I2C4_RX = { 0 };
DMA_HandleTypeDef hDMA_I2C4_TX = { 0 };

/**************************************************************************************************************
 Declaration : HAL_I2C_MemRxCpltCallback
 Parameters  : hI2Cx, Pointer to a I2C_HandleTypeDef structure
 Return Value: None
 Description : I2C Memory Rx Transfer completed callback
**************************************************************************************************************/
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hI2Cx)
{
  if (hI2Cx->Instance == I2C4) {
    /* Set thread event flag */
    VERIFY(osThreadFlagsSet(I2C4_RxThreadID, I2C1_TRANSFER_COMPLETED) != osFlagsError);
  }
  else {
    ErrorHandler(__FILE__, __LINE__);
  }
}

/**************************************************************************************************************
 Declaration : HAL_I2C_MemTxCpltCallback
 Parameters  : hI2Cx, Pointer to a I2C_HandleTypeDef structure
 Return Value: None
 Description : I2C Memory Tx Transfer completed callback
**************************************************************************************************************/
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hI2Cx)
{
  if (hI2Cx->Instance == I2C4) {
    /* Set thread event flag */
    VERIFY(osThreadFlagsSet(I2C4_TxThreadID, I2C1_TRANSFER_COMPLETED) != osFlagsError);
  }
  else {
    ErrorHandler(__FILE__, __LINE__);
  }
}

/**************************************************************************************************************
 Declaration : HAL_I2C_ErrorCallback
 Parameters  : hI2Cx, Pointer to a I2C_HandleTypeDef structure
 Return Value: None
 Description : I2C error callback
**************************************************************************************************************/
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hI2Cx)
{
  ErrorHandler(__FILE__, __LINE__);
}

/**************************************************************************************************************
 Declaration : I2C_DeInit
 Parameters  : I2Cx, Pointer to a I2C_TypeDef structure
 Return Value: HAL status
 Description : De-Initialize the I2C peripherals
**************************************************************************************************************/
HAL_StatusTypeDef I2C_DeInit(I2C_TypeDef *I2Cx)
{
  if (I2Cx == I2C4) {
    return HAL_OK;
  }
  else {
    ErrorHandler(__FILE__, __LINE__);
  }

  /* Return HAL status */
  return HAL_ERROR;
}

/**************************************************************************************************************
 Declaration : I2C_Init
 Parameters  : I2Cx, Pointer to a I2C_TypeDef structure
 Return Value: HAL status
 Description : Initialize the I2C peripherals
**************************************************************************************************************/
HAL_StatusTypeDef I2C_Init(I2C_TypeDef *I2Cx)
{
  if (I2Cx == I2C4) {
    /* Configure the I2C peripheral */
    hI2C4.Instance              = I2C4;
    hI2C4.Mode                  = HAL_I2C_MODE_MASTER;
    hI2C4.Init.Timing           = 0x00B03FDB;
    hI2C4.Init.OwnAddress1      = 0;
    hI2C4.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    hI2C4.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    hI2C4.Init.OwnAddress2      = 0;
    hI2C4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hI2C4.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    hI2C4.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;

    /* Initializes the I2C peripheral */
    if (HAL_I2C_Init(&hI2C4) != HAL_OK) {
      return HAL_ERROR;
    }

    /* Configure I2C Analogue filter */
    if (HAL_I2CEx_ConfigAnalogFilter(&hI2C4,I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
      return HAL_ERROR;
    }

    /* Configure I2C Digital filter */
    if (HAL_I2CEx_ConfigDigitalFilter(&hI2C4, 0) != HAL_OK) {
      return HAL_ERROR;
    }

    /* Enable EV I2C interrupt */
    HAL_NVIC_SetPriority(I2C4_EV_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C4_EV_IRQn);

    /* Enable ER I2C interrupt */
    HAL_NVIC_SetPriority(I2C4_ER_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C4_ER_IRQn);

    /* BDMA_Channel0_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(BDMA_Channel0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(BDMA_Channel0_IRQn);

    /* BDMA_Channel1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(BDMA_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(BDMA_Channel1_IRQn);

    /* Checks if target device is ready for communication */
    if (HAL_I2C_IsDeviceReady(&hI2C4, I2C4_DEVICE_ADDR, UINT_MAX, HAL_MAX_DELAY) != HAL_OK) {
      return HAL_ERROR;
    }

    /* Checks the I2C handle state */
    while (HAL_I2C_GetState(&hI2C4) != HAL_I2C_STATE_READY) {
      VERIFY(osThreadYield() == osOK);
    }

    /* Return HAL status */
    return HAL_OK;
  }
  else {
    ErrorHandler(__FILE__, __LINE__);
  }

  /* Return HAL status */
  return HAL_ERROR;
}

/**************************************************************************************************************
 Declaration : I2C_Read
 Parameters  : I2Cx      , Pointer to a I2C_TypeDef structure
 Parameters  : MemAddress, Internal memory address
 Parameters  : pData     , Pointer to data buffer
 Parameters  : Size      , Amount of data to be sent
 Return Value: HAL status
 Description : Read an amount of data from a specific memory address
**************************************************************************************************************/
HAL_StatusTypeDef I2C_Read(I2C_TypeDef *I2Cx, uint16_t MemAddress, void *pData, uint16_t Size)
{
  if (I2Cx == I2C4) {
    /* Check the parameters */
    assert_param(pData && Size);

    /* Get the currently running thread */
    I2C4_RxThreadID = osThreadGetId();
    ASSERT(I2C4_RxThreadID != NULL);

    /* Read data from I2C peripheral */
    if (HAL_I2C_Mem_Read_DMA(&hI2C4, I2C4_DEVICE_ADDR, MemAddress, I2C4_MEMADD_SIZE, pData, Size) != HAL_OK) {
      return HAL_ERROR;
    }

    /* Wait for completion event */
    if (osThreadFlagsWait(I2C1_TRANSFER_COMPLETED, osFlagsWaitAny, osWaitForever) == osFlagsError) {
      return HAL_ERROR;
    }

    /* Checks the I2C handle state */
    while (HAL_I2C_GetState(&hI2C4) != HAL_I2C_STATE_READY) {
      return HAL_ERROR;
    }

    /* Return HAL status */
    return HAL_OK;
  }
  else {
    ErrorHandler(__FILE__, __LINE__);
  }

  /* Return HAL status */
  return HAL_ERROR;
}

/**************************************************************************************************************
 Declaration : I2C_Write
 Parameters  : I2Cx      , Pointer to a I2C_TypeDef structure
 Parameters  : MemAddress, Internal memory address
 Parameters  : pData     , Pointer to data buffer
 Parameters  : Size      , Amount of data to be sent
 Return Value: HAL status
 Description : Write an amount of data from a specific memory address
**************************************************************************************************************/
HAL_StatusTypeDef I2C_Write(I2C_TypeDef *I2Cx, uint16_t MemAddress, const void *pData, uint16_t Size)
{
  if (I2Cx == I2C4) {
    /* Check the parameters */
    assert_param(pData && Size);

    /* Get the currently running thread */
    I2C4_TxThreadID = osThreadGetId();
    ASSERT(I2C4_TxThreadID != NULL);

    /* Write address is not I2C1_DEVICE_PGSZ aligned */
    uint8_t *buf = (uint8_t *) pData;
    uint16_t len = MemAddress % I2C4_DEVICE_PGSZ;

    if (len != 0) {
      len = I2C4_DEVICE_PGSZ - len;
      if (len > Size) { len = Size; }

      if (HAL_I2C_Mem_Write_DMA(&hI2C4, I2C4_DEVICE_ADDR, MemAddress, I2C4_MEMADD_SIZE, buf, len)) {
        return HAL_ERROR;
      }
      else {
        Size       -= len;
        MemAddress += len;
        buf        += len;
      }

      /* Wait for completion event */
      if (osThreadFlagsWait(I2C1_TRANSFER_COMPLETED, osFlagsWaitAny, osWaitForever) == osFlagsError) {
        return HAL_ERROR;
      }

      /* Checks the I2C handle state */
      while (HAL_I2C_GetState(&hI2C4) != HAL_I2C_STATE_READY) {
        return HAL_ERROR;
      }

      /* Checks if target device is ready for communication */
      if (HAL_I2C_IsDeviceReady(&hI2C4, I2C4_DEVICE_ADDR, UINT_MAX, HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
      }

      /* Checks the I2C handle state */
      while (HAL_I2C_GetState(&hI2C4) != HAL_I2C_STATE_READY) {
        return HAL_ERROR;
      }
    }

    /* Write address is I2C1_DEVICE_PGSZ aligned */
    while (Size > 0) {
      len = Size < I2C4_DEVICE_PGSZ ? Size : I2C4_DEVICE_PGSZ;

      if (HAL_I2C_Mem_Write_DMA(&hI2C4, I2C4_DEVICE_ADDR, MemAddress, I2C4_MEMADD_SIZE, buf, len)) {
        return HAL_ERROR;
      }
      else {
        Size       -= len;
        MemAddress += len;
        buf        += len;
      }

      /* Wait for completion event */
      if (osThreadFlagsWait(I2C1_TRANSFER_COMPLETED, osFlagsWaitAny, osWaitForever) == osFlagsError) {
        return HAL_ERROR;
      }

      /* Checks the I2C handle state */
      while (HAL_I2C_GetState(&hI2C4) != HAL_I2C_STATE_READY) {
        return HAL_ERROR;
      }

      /* Checks if target device is ready for communication */
      if (HAL_I2C_IsDeviceReady(&hI2C4, I2C4_DEVICE_ADDR, UINT_MAX, HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
      }

      /* Checks the I2C handle state */
      while (HAL_I2C_GetState(&hI2C4) != HAL_I2C_STATE_READY) {
        return HAL_ERROR;
      }
    }

    /* Return HAL status */
    return HAL_OK;
  }
  else {
    ErrorHandler(__FILE__, __LINE__);
  }

  /* Return HAL status */
  return HAL_ERROR;
}

