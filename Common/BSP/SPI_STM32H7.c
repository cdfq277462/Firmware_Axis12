/**************************************************************************************************************
 File   :
 Author :
 Date   :
 Brief  :
**************************************************************************************************************/

/* Includes **************************************************************************************************/
#include "SPI_STM32H7.h"
#include "TMC_STM32H7.h"

/* Variables *************************************************************************************************/
static uint8_t SPI1_TxData[32] __MEMORY_AT(0x38000000) = { 0 };
static uint8_t SPI1_RxData[32] __MEMORY_AT(0x38000020) = { 0 };
static uint8_t SPI2_TxData[32] __MEMORY_AT(0x38000040) = { 0 };
static uint8_t SPI2_RxData[32] __MEMORY_AT(0x38000060) = { 0 };
static uint8_t SPI3_TxData[32] __MEMORY_AT(0x38000080) = { 0 };
static uint8_t SPI3_RxData[32] __MEMORY_AT(0x380000A0) = { 0 };
static uint8_t SPI4_TxData[32] __MEMORY_AT(0x380000C0) = { 0 };
static uint8_t SPI4_RxData[32] __MEMORY_AT(0x380000E0) = { 0 };
static uint8_t SPI5_TxData[32] __MEMORY_AT(0x38000100) = { 0 };
static uint8_t SPI5_RxData[32] __MEMORY_AT(0x38000120) = { 0 };
static uint8_t SPI6_TxData[32] __MEMORY_AT(0x38000140) = { 0 };
static uint8_t SPI6_RxData[32] __MEMORY_AT(0x38000160) = { 0 };

/* SPIx handle ***********************************************************************************************/
SPI_HandleTypeDef hSPI1 = { .Instance = SPI1, };
SPI_HandleTypeDef hSPI2 = { .Instance = SPI2, };
SPI_HandleTypeDef hSPI3 = { .Instance = SPI3, };
SPI_HandleTypeDef hSPI4 = { .Instance = SPI4, };
SPI_HandleTypeDef hSPI5 = { .Instance = SPI5, };
SPI_HandleTypeDef hSPI6 = { .Instance = SPI6, };

/* Variables *************************************************************************************************/
DMA_HandleTypeDef hDMA_SPI1_RX = { 0 };
DMA_HandleTypeDef hDMA_SPI1_TX = { 0 };
DMA_HandleTypeDef hDMA_SPI2_RX = { 0 };
DMA_HandleTypeDef hDMA_SPI2_TX = { 0 };
DMA_HandleTypeDef hDMA_SPI3_RX = { 0 };
DMA_HandleTypeDef hDMA_SPI3_TX = { 0 };
DMA_HandleTypeDef hDMA_SPI4_RX = { 0 };
DMA_HandleTypeDef hDMA_SPI4_TX = { 0 };
DMA_HandleTypeDef hDMA_SPI5_RX = { 0 };
DMA_HandleTypeDef hDMA_SPI5_TX = { 0 };
DMA_HandleTypeDef hDMA_SPI6_RX = { 0 };
DMA_HandleTypeDef hDMA_SPI6_TX = { 0 };

/* MOTx handle ***********************************************************************************************/
MOT_HandleTypeDef hMOT1 = {.hSPIx = &hSPI1, .Instance = TMC1, .Flags = SPI1_TRANSFER_COMPLETED,
                          .TxData = SPI1_TxData, .RxData = SPI1_RxData };
MOT_HandleTypeDef hMOT2 = {.hSPIx = &hSPI2, .Instance = TMC2, .Flags = SPI2_TRANSFER_COMPLETED,
                           .TxData = SPI2_TxData, .RxData = SPI2_RxData };
MOT_HandleTypeDef hMOT3 = {.hSPIx = &hSPI3, .Instance = TMC3, .Flags = SPI3_TRANSFER_COMPLETED,
                           .TxData = SPI3_TxData, .RxData = SPI3_RxData };
MOT_HandleTypeDef hMOT4 = {.hSPIx = &hSPI4, .Instance = TMC4, .Flags = SPI4_TRANSFER_COMPLETED,
                           .TxData = SPI4_TxData, .RxData = SPI4_RxData };
MOT_HandleTypeDef hMOT5 = {.hSPIx = &hSPI5, .Instance = TMC5, .Flags = SPI5_TRANSFER_COMPLETED,
                           .TxData = SPI5_TxData, .RxData = SPI5_RxData };
MOT_HandleTypeDef hMOT6 = {.hSPIx = &hSPI6, .Instance = TMC6, .Flags = SPI6_TRANSFER_COMPLETED,
                           .TxData = SPI6_TxData, .RxData = SPI6_RxData };

/**************************************************************************************************************
 Declaration : HAL_SPI_TxRxCpltCallback
 Parameters  : hSPIx, SPI handle
 Return Value: None
 Description : TxRx Transfer completed callback.
**************************************************************************************************************/
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hSPIx)
{
  MOT_HandleTypeDef *hMOTx = NULL;

  /* Check the parameters */
  assert_param(hSPIx == &hSPI1 || hSPIx == &hSPI2 || hSPIx == &hSPI3 ||
               hSPIx == &hSPI4 || hSPIx == &hSPI5 || hSPIx == &hSPI6);

  /* Verify SPI registers base address */
  if (hSPIx->Instance == SPI1) {
    hMOTx = &hMOT1;
  } else
  if (hSPIx->Instance == SPI2) {
    hMOTx = &hMOT2;
  } else
  if (hSPIx->Instance == SPI3) {
    hMOTx = &hMOT3;
  } else
  if (hSPIx->Instance == SPI4) {
    hMOTx = &hMOT4;
  } else
  if (hSPIx->Instance == SPI5) {
    hMOTx = &hMOT5;
  } else
  if (hSPIx->Instance == SPI6) {
    hMOTx = &hMOT6;
  }
  else {
    ErrorHandler(__FILE__, __LINE__);
  }

  /* Verify MOT registers base address */
  assert_param(hMOTx->TxRxThreadID != NULL);

  if ((hMOTx->TxData[0] & 0x80) == 0) {
    hMOTx->Instance->Registers[hMOTx->TxData[0]] = __REV(*(uint32_t *) &hMOTx->RxData[1]);
  }
  else {
    /* Write action record the timestamp */
    hMOTx->Instance->R.RESERVE_22 = osKernelGetSysTimerCount() / 480;
  }

   /* Sets the thread flags for the thread */
  VERIFY((osThreadFlagsSet(hMOTx->TxRxThreadID, hMOTx->Flags) & osFlagsError) == 0U);
}

/**************************************************************************************************************
 Declaration : HAL_SPI_ErrorCallback
 Parameters  : hSPIx, SPI handle
 Return Value: None
 Description : SPI error callbacks
**************************************************************************************************************/
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hSPIx)
{
  /* Check the parameters */
  assert_param(hSPIx == &hSPI1 || hSPIx == &hSPI2 || hSPIx == &hSPI3 ||
               hSPIx == &hSPI4 || hSPIx == &hSPI5 || hSPIx == &hSPI6);
}

/**************************************************************************************************************
 Declaration : SPI_Init
 Parameters  : SPIx, SPI Instance
 Return Value: HAL status
 Description : Initialize the SPI peripherals
**************************************************************************************************************/
HAL_StatusTypeDef SPI_Init(SPI_HandleTypeDef *hSPIx)
{
  GPIO_InitTypeDef GPIO_InitStruct = { .Mode  = GPIO_MODE_OUTPUT_PP,
                                       .Pull  = GPIO_PULLUP,
                                       .Speed = GPIO_SPEED_LOW };

  /* Check the parameters */
  assert_param(hSPIx == &hSPI1 || hSPIx == &hSPI2 || hSPIx == &hSPI3 ||
               hSPIx == &hSPI4 || hSPIx == &hSPI5 || hSPIx == &hSPI6);

  /* Disable AT25128 CS# */
  if (hSPIx == &hSPI1) {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
  } else
  if (hSPIx == &hSPI2) {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
  } else
  if (hSPIx == &hSPI3) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
  } else
  if (hSPIx == &hSPI4) {
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, GPIO_PIN_SET);
  } else
  if (hSPIx == &hSPI5) {
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
  } else
  if (hSPIx == &hSPI6) {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
  }

  /* Configure the SPI peripheral */
  hSPIx->Init.Mode                    = SPI_MODE_MASTER;
  hSPIx->Init.BaudRatePrescaler       = SPI_BAUDRATEPRESCALER_64;
  hSPIx->Init.Direction               = SPI_DIRECTION_2LINES;
  hSPIx->Init.CLKPhase                = SPI_PHASE_2EDGE;
  hSPIx->Init.CLKPolarity             = SPI_POLARITY_HIGH;
  hSPIx->Init.DataSize                = SPI_DATASIZE_8BIT;
  hSPIx->Init.FirstBit                = SPI_FIRSTBIT_MSB;
  hSPIx->Init.TIMode                  = SPI_TIMODE_DISABLE;
  hSPIx->Init.CRCCalculation          = SPI_CRCCALCULATION_DISABLE;
  hSPIx->Init.CRCPolynomial           = 0;
  hSPIx->Init.CRCLength               = SPI_CRC_LENGTH_8BIT;
  hSPIx->Init.FifoThreshold           = SPI_FIFO_THRESHOLD_05DATA;
  hSPIx->Init.NSS                     = SPI_NSS_HARD_OUTPUT;
  hSPIx->Init.NSSPMode                = SPI_NSS_PULSE_DISABLE;
  hSPIx->Init.MasterSSIdleness        = SPI_MASTER_SS_IDLENESS_15CYCLE;
  hSPIx->Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_15CYCLE;
  hSPIx->Init.MasterKeepIOState       = SPI_MASTER_KEEP_IO_STATE_ENABLE;

  /* Initialize the SPI peripheral */
  return HAL_SPI_Init(hSPIx);
}

/**************************************************************************************************************
 Declaration : SPI_DeInit
 Parameters  : SPIx, SPI Instance
 Return Value: HAL status
 Description : De-Initialize the SPI peripherals
**************************************************************************************************************/
HAL_StatusTypeDef SPI_DeInit(SPI_HandleTypeDef *hSPIx)
{
  /* Check the parameters */
  assert_param(hSPIx == &hSPI1 || hSPIx == &hSPI2 || hSPIx == &hSPI3 ||
               hSPIx == &hSPI4 || hSPIx == &hSPI5 || hSPIx == &hSPI6);

  /* De-Initialize the SPI peripheral */
  return HAL_SPI_DeInit(hSPIx);
}

/**************************************************************************************************************
 Declaration : SPI_TransmitReceive
 Parameters  : pSPIx  , pointer to a SPI_TypeDef structure
 Parameters  : pTxData, pointer to transmission data buffer
 Parameters  : pRxData, pointer to reception data buffer
 Parameters  : Size   , amount of data to be sent and received
 Return Value: HAL status
 Description : Transmit and Receive an amount of data
**************************************************************************************************************/
static HAL_StatusTypeDef SPI_TransmitReceive(SPI_HandleTypeDef *hSPIx  ,
                                             uint8_t           *pTxData,
                                             uint8_t           *pRxData,
                                             uint16_t           Size   )
{
  /* Check the parameters */
  assert_param(hSPIx == &hSPI1 || hSPIx == &hSPI2 || hSPIx == &hSPI3 ||
               hSPIx == &hSPI4 || hSPIx == &hSPI5 || hSPIx == &hSPI6);

  /* Check the parameters */
  assert_param(Size != 0);
  assert_param(pTxData != NULL);
  assert_param(pRxData != NULL);

  /* Transmit and Receive an amount of data in non-blocking mode */
  return HAL_SPI_TransmitReceive_DMA(hSPIx, pTxData, pRxData, Size);
}

/**************************************************************************************************************
 Declaration : MOT_Read
 Parameters  : hMOTx  , Motor handle
 Parameters  : Address, Register address
 Return Value: HAL Status
 Description : Read MOT register
**************************************************************************************************************/
HAL_StatusTypeDef MOT_Read(MOT_HandleTypeDef *hMOTx, uint8_t Address)
{
  /* Check the parameters */
  assert_param(&hMOT1 == hMOTx || &hMOT2 == hMOTx || &hMOT3 == hMOTx ||
               &hMOT4 == hMOTx || &hMOT5 == hMOTx || &hMOT6 == hMOTx);

  /* Wait for last access to complete */
  if (hMOTx->hSPIx->State != HAL_SPI_STATE_READY) {
    VERIFY((osThreadFlagsClear(hMOTx->Flags) & osFlagsError) == 0U);

    if (hMOTx->hSPIx->State != HAL_SPI_STATE_READY) {
      VERIFY((osThreadFlagsWait(hMOTx->Flags, osFlagsWaitAll, osWaitForever) & osFlagsError) == 0U);
    }

    ASSERT(hMOTx->hSPIx->State == HAL_SPI_STATE_READY);
  }

  /* Clear Write Bit */
  hMOTx->TxData[0] = Address & 0x7F;

  /* Reverse byte order (32 bit) */
  *((uint32_t *) &(hMOTx->TxData[1])) = 0U;

  /* Transmit and receive amount of data */
  return SPI_TransmitReceive(hMOTx->hSPIx, hMOTx->TxData, hMOTx->RxData, 5);
}

/**************************************************************************************************************
 Declaration : MOT_Write
 Parameters  : hMOTx  , Motor handle
 Parameters  : Address, Register address
 Parameters  : Value  , Value to be written
 Return Value: HAL Status
 Description : Write MOT register
**************************************************************************************************************/
HAL_StatusTypeDef MOT_Write(MOT_HandleTypeDef *hMOTx, uint8_t Address, int32_t Value)
{
  /* Check the parameters */
  assert_param(&hMOT1 == hMOTx || &hMOT2 == hMOTx || &hMOT3 == hMOTx ||
               &hMOT4 == hMOTx || &hMOT5 == hMOTx || &hMOT6 == hMOTx);

  /* Wait for last access to complete */
  if (hMOTx->hSPIx->State != HAL_SPI_STATE_READY) {
    VERIFY((osThreadFlagsClear(hMOTx->Flags) & osFlagsError) == 0U);

    if (hMOTx->hSPIx->State != HAL_SPI_STATE_READY) {
      VERIFY((osThreadFlagsWait(hMOTx->Flags, osFlagsWaitAll, osWaitForever) & osFlagsError) == 0U);
    }

    ASSERT(hMOTx->hSPIx->State == HAL_SPI_STATE_READY);
  }

  /* Setup Write Bit */
  hMOTx->TxData[0] = Address | 0x80;

  /* Reverse byte order (32 bit) */
  *((uint32_t *) &(hMOTx->TxData[1])) = __REV(Value);

  /* Transmit and receive amount of data */
  return SPI_TransmitReceive(hMOTx->hSPIx, hMOTx->TxData, hMOTx->RxData, 5);
}

/**************************************************************************************************************
 Declaration : MOT_Init
 Parameters  : hMOTx, Motor handle
 Return Value: HAL Status
 Description : Initialize MOT communication
**************************************************************************************************************/
HAL_StatusTypeDef MOT_Init(MOT_HandleTypeDef *hMOTx)
{
  /* Check parameters */
  assert_param(&hMOT1 == hMOTx || &hMOT2 == hMOTx || &hMOT3 == hMOTx ||
               &hMOT4 == hMOTx || &hMOT5 == hMOTx || &hMOT6 == hMOTx);

  /* Get current thread IDs */
  hMOTx->TxRxThreadID = osThreadGetId();

  /* Init TMC communication */
  return SPI_Init(hMOTx->hSPIx);
}
