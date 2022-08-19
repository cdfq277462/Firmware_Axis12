/**************************************************************************************************************
 File   :
 Author :
 Date   :
 Brief  :
**************************************************************************************************************/

/* Includes **************************************************************************************************/
#include "COM_STM32H7.h"

/* UARTx handle **********************************************************************************************/
UART_HandleTypeDef hUART1 = { .Instance = UART1, };
UART_HandleTypeDef hUART2 = { .Instance = UART2, };
UART_HandleTypeDef hUART3 = { .Instance = UART3, };
UART_HandleTypeDef hUART4 = { .Instance = UART4, };
UART_HandleTypeDef hUART5 = { .Instance = UART5, };
UART_HandleTypeDef hUART6 = { .Instance = UART6, };
UART_HandleTypeDef hUART7 = { .Instance = UART7, };
UART_HandleTypeDef hUART8 = { .Instance = UART8, };

/* MOTx handle ***********************************************************************************************/
MOT_HandleTypeDef hMOT1 = {.hUARTx = &hUART1, .Instance = TMC2, .Flags = UART1_TRANSFER_COMPLETED };
MOT_HandleTypeDef hMOT2 = {.hUARTx = &hUART1, .Instance = TMC1 };
MOT_HandleTypeDef hMOT3 = {.hUARTx = &hUART1, .Instance = TMC1 };
MOT_HandleTypeDef hMOT4 = {.hUARTx = &hUART1, .Instance = TMC1 };
MOT_HandleTypeDef hMOT5 = {.hUARTx = &hUART1, .Instance = TMC1 };
MOT_HandleTypeDef hMOT6 = {.hUARTx = &hUART1, .Instance = TMC1 };

/**************************************************************************************************************
 Declaration : HAL_UART_TxCpltCallback
 Parameters  : hUARTx, UART handle
 Return Value: None
 Description : Tx Transfer completed callback
**************************************************************************************************************/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *hUARTx)
{
  /* Check the parameters */
  assert_param(hUARTx == &hUART1 || hUARTx == &hUART2 || hUARTx == &hUART3 || hUARTx == &hUART4 ||
               hUARTx == &hUART5 || hUARTx == &hUART6 || hUARTx == &hUART7 || hUARTx == &hUART8);
}

/**************************************************************************************************************
 Declaration : HAL_UART_RxCpltCallback
 Parameters  : hUARTx, UART handle
 Return Value: None
 Description : Rx Transfer completed callback
**************************************************************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *hUARTx)
{
  /* Check the parameters */
  assert_param(hUARTx == &hUART1 || hUARTx == &hUART2 || hUARTx == &hUART3 || hUARTx == &hUART4 ||
               hUARTx == &hUART5 || hUARTx == &hUART6 || hUARTx == &hUART7 || hUARTx == &hUART8);

  /* Check the object pointer */
  if (hUARTx == hMOT1.hUARTx) {
    assert_param(hMOT1.TxRxThreadID != NULL);

    if ((hMOT1.TxData[0] & 0x80) == 0) {
      hMOT1.Instance->Registers[hMOT1.TxData[0] & 0x7F] = __REV(*(uint32_t *) &hMOT1.RxData[1]);
      hMOT1.Instance->Registers[hMOT1.TxData[0] | 0x80] = __REV(*(uint32_t *) &hMOT1.RxData[1]);
    }
    else {
      hMOT1.Instance->Registers[hMOT1.TxData[0] | 0x80] = hMOT1.Instance->Registers[hMOT1.TxData[0] & 0x7F];
    }

    VERIFY((osThreadFlagsSet(hMOT1.TxRxThreadID, hMOT1.Flags) & osFlagsError) == 0U);
  }
  else {
    ErrorHandler(__FILE__, __LINE__);
  }
}

/**************************************************************************************************************
 Declaration : HAL_UART_ErrorCallback
 Parameters  : hUARTx, UART handle
 Return Value: None
 Description : UART error callback
**************************************************************************************************************/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *hUARTx)
{
  /* Check the parameters */
  assert_param(hUARTx == &hUART1 || hUARTx == &hUART2 || hUARTx == &hUART3 || hUARTx == &hUART4 ||
               hUARTx == &hUART5 || hUARTx == &hUART6 || hUARTx == &hUART7 || hUARTx == &hUART8);
}

/**************************************************************************************************************
 Declaration : UART_Init
 Parameters  : UARTx, UART Instance
 Return Value: HAL status
 Description : Initialize the UART peripherals
**************************************************************************************************************/
HAL_StatusTypeDef UART_Init(UART_HandleTypeDef *hUARTx)
{
  /* Check the parameters */
  assert_param(hUARTx == &hUART1 || hUARTx == &hUART2 || hUARTx == &hUART3 || hUARTx == &hUART4 ||
               hUARTx == &hUART5 || hUARTx == &hUART6 || hUARTx == &hUART7 || hUARTx == &hUART8);

  /* Configure the UART peripheral */
  hUARTx->Init.BaudRate               = 115200;
  hUARTx->Init.WordLength             = UART_WORDLENGTH_8B;
  hUARTx->Init.StopBits               = UART_STOPBITS_1;
  hUARTx->Init.Parity                 = UART_PARITY_NONE;
  hUARTx->Init.Mode                   = UART_MODE_TX_RX;
  hUARTx->Init.HwFlowCtl              = UART_HWCONTROL_NONE;
  hUARTx->Init.OverSampling           = UART_OVERSAMPLING_16;
  hUARTx->Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
  hUARTx->Init.ClockPrescaler         = UART_PRESCALER_DIV1;
  hUARTx->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hUARTx->FifoMode                    = UART_FIFOMODE_ENABLE;

  /* De-Initialize the UART peripheral */
  if (HAL_UART_DeInit(hUARTx) != HAL_OK) {
    return HAL_ERROR;
  }

  /* Re-Initialize the UART peripheral */
  if (HAL_UART_Init(hUARTx)) {
    return HAL_ERROR;
  }

  /* Enable the FIFO mode */
  if (HAL_UARTEx_EnableFifoMode(hUARTx)) {
    return HAL_ERROR;
  }

  /* Set the TX FIFO threshold */
  if (HAL_UARTEx_SetTxFifoThreshold(hUARTx, UART_TXFIFO_THRESHOLD_1_8)) {
    return HAL_ERROR;
  }

  /* Set the RX FIFO threshold */
  if (HAL_UARTEx_SetRxFifoThreshold(hUARTx, UART_RXFIFO_THRESHOLD_1_8)) {
    return HAL_ERROR;
  }

  /* Return HAL status */
  return HAL_OK;
}

/**************************************************************************************************************
 Declaration : UART_DeInit
 Parameters  : UARTx, UART Instance
 Return Value: HAL status
 Description : DeInitialize the UART peripherals
**************************************************************************************************************/
HAL_StatusTypeDef UART_DeInit(UART_HandleTypeDef *hUARTx)
{
  /* Check the parameters */
  assert_param(hUARTx == &hUART1 || hUARTx == &hUART2 || hUARTx == &hUART3 || hUARTx == &hUART4 ||
               hUARTx == &hUART5 || hUARTx == &hUART6 || hUARTx == &hUART7 || hUARTx == &hUART8);

  /* De-Initialize the UART peripheral */
  return HAL_UART_DeInit(hUARTx);
}

/**************************************************************************************************************
 Declaration : UART_HandleTypeDef
 Parameters  : hUARTx , pointer to UART_HandleTypeDef structure
 Parameters  : pTxData, pointer to transmission data buffer
 Parameters  : pRxData, pointer to reception data buffer
 Parameters  : Size   , amount of data to be sent and received
 Return Value: HAL status
 Description : Transmit and Receive an amount of data
**************************************************************************************************************/
static HAL_StatusTypeDef UART_TransmitReceive(UART_HandleTypeDef *hUARTx ,
                                              uint8_t            *pTxData,
                                              uint8_t            *pRxData,
                                              uint16_t            Size   )
{
  /* Check the parameters */
  assert_param(hUARTx == &hUART1 || hUARTx == &hUART2 || hUARTx == &hUART3 || hUARTx == &hUART4 ||
               hUARTx == &hUART5 || hUARTx == &hUART6 || hUARTx == &hUART7 || hUARTx == &hUART8);

  /* Check the parameters */
  assert_param(Size   );
  assert_param(pTxData);
  assert_param(pRxData);

  /* Receive an amount of data */
  if (HAL_UART_Receive_IT(hUARTx, pRxData, Size) != HAL_OK) {
    HAL_UART_AbortReceive_IT(hUARTx);
    return HAL_ERROR;
  }

  /* Transmit an amount of data */
  if (HAL_UART_Transmit_IT(hUARTx, pTxData, Size) != HAL_OK) {
    HAL_UART_AbortTransmit_IT(hUARTx);
    return HAL_ERROR;
  }

  /* Return HAL status */
  return HAL_OK;
}

/**************************************************************************************************************
 Declaration : TMC_Read
 Parameters  : Motor  , Motor number
 Parameters  : Address, Register address
 Return Value: HAL Status
 Description : Read TMC register
**************************************************************************************************************/
HAL_StatusTypeDef MOT_Read(MOT_HandleTypeDef *hMOTx, uint8_t Address)
{
  /* Check the parameters */
  assert_param(&hMOT1 == hMOTx || &hMOT2 == hMOTx || &hMOT3 == hMOTx ||
               &hMOT4 == hMOTx || &hMOT5 == hMOTx || &hMOT6 == hMOTx);

  /* Wait for last access to complete */
  if (hMOTx->hUARTx->RxState != HAL_UART_STATE_READY) {
    VERIFY((osThreadFlagsClear(hMOTx->Flags) & osFlagsError) == 0U);

    if (hMOTx->hUARTx->RxState != HAL_UART_STATE_READY) {
      VERIFY((osThreadFlagsWait(hMOTx->Flags, osFlagsWaitAll, osWaitForever) & osFlagsError) == 0U);
    }

    ASSERT(hMOTx->hUARTx->RxState == HAL_UART_STATE_READY);
  }

  /* Clear Write Bit */
  hMOTx->TxData[0] = Address & 0x7F;

  /* Transmit and receive amount of data */
  return UART_TransmitReceive(hMOTx->hUARTx, hMOTx->TxData, hMOTx->RxData, 5);
}

/**************************************************************************************************************
 Declaration : TMC_Write
 Parameters  : Motor  , Motor number
 Parameters  : Address, Register address
 Parameters  : Value  , Value to be written
 Return Value: HAL Status
 Description : Write TMC register
**************************************************************************************************************/
HAL_StatusTypeDef MOT_Write(MOT_HandleTypeDef *hMOTx, uint8_t Address, int32_t Value)
{
  /* Check the parameters */
  assert_param(&hMOT1 == hMOTx || &hMOT2 == hMOTx || &hMOT3 == hMOTx ||
               &hMOT4 == hMOTx || &hMOT5 == hMOTx || &hMOT6 == hMOTx);

  /* Wait for last access to complete */
  if (hMOTx->hUARTx->RxState != HAL_UART_STATE_READY) {
    VERIFY((osThreadFlagsClear(hMOTx->Flags) & osFlagsError) == 0U);

    if (hMOTx->hUARTx->RxState != HAL_UART_STATE_READY) {
      VERIFY((osThreadFlagsWait(hMOTx->Flags, osFlagsWaitAll, osWaitForever) & osFlagsError) == 0U);
    }

    ASSERT(hMOTx->hUARTx->RxState == HAL_UART_STATE_READY);
  }

  /* Setup Write Bit */
  hMOTx->TxData[0] = Address | 0x80;

  /* Reverse byte order (32 bit) */
  *((uint32_t *) &(hMOTx->TxData[1])) = __REV(Value);

  /* Transmit and receive amount of data */
  return UART_TransmitReceive(hMOTx->hUARTx, hMOTx->TxData, hMOTx->RxData, 5);
}

/**************************************************************************************************************
 Declaration : MOT_Init
 Parameters  : hMOTx, MOT handle
 Return Value: HAL Status
 Description : Initialize MOT communication
**************************************************************************************************************/
HAL_StatusTypeDef MOT_Init(MOT_HandleTypeDef *hMOTx)
{
  /* Check hMOTx parameters */
  assert_param(&hMOT1 == hMOTx || &hMOT2 == hMOTx || &hMOT3 == hMOTx ||
               &hMOT4 == hMOTx || &hMOT5 == hMOTx || &hMOT6 == hMOTx);

  /* Get current thread IDs */
  hMOTx->TxRxThreadID = osThreadGetId();

  /* Zero MOT all registers */
  memset((void *) hMOTx->Instance, 0, sizeof(TMC_TypeDef));

  /* Init TMC communication */
  return UART_Init(hMOTx->hUARTx);
}
