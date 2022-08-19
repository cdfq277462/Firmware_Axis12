/**************************************************************************************************************
 File   :
 Author :
 Date   :
 Brief  :
**************************************************************************************************************/

/* Includes **************************************************************************************************/
#include "MSP_STM32H7.h"

/**************************************************************************************************************
 Declaration : HAL_MspInit
 Parameters  : None
 Return Value: None
 Description : Initializes the Global MSP
**************************************************************************************************************/
void HAL_MspInit(void)
{
  /* Enable SYSCFG Clock */
  __HAL_RCC_SYSCFG_CLK_ENABLE();

  /* SVCall_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SVCall_IRQn, 14, 0);

  /* PendSV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);
}

/**************************************************************************************************************
 Declaration : HAL_MspDeInit
 Parameters  : None
 Return Value: None
 Description : DeInitializes the Global MSP
**************************************************************************************************************/
void HAL_MspDeInit(void)
{
}

/**************************************************************************************************************
 Declaration : HAL_ETH_MspInit
 Parameters  : ETH handle pointer
 Return Value: None
 Description : ETH MSP Initialization
**************************************************************************************************************/
#if defined(HAL_ETH_MODULE_ENABLED) && defined(CORE_CM4)
void HAL_ETH_MspInit(ETH_HandleTypeDef *hETHx)
{
  if (hETHx->Instance == ETH) {
    GPIO_InitTypeDef GPIO_InitStruct = { .Alternate = GPIO_AF11_ETH,
                                         .Mode      = GPIO_MODE_AF_PP,
                                         .Pull      = GPIO_NOPULL,
                                         .Speed     = GPIO_SPEED_FREQ_HIGH };

    /* EMAC Peripheral clock enable */
    __HAL_RCC_ETH1MAC_CLK_ENABLE();
    __HAL_RCC_ETH1TX_CLK_ENABLE ();
    __HAL_RCC_ETH1RX_CLK_ENABLE ();

    /* ETH GPIOA Configuration */
    GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ETH GPIOB Configuration */
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* ETH GPIOC Configuration */
    GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* ETH GPIOG Configuration */
    GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_13;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    /* ETH interrupt Init */
    HAL_NVIC_SetPriority(ETH_IRQn, 13, 0);
    HAL_NVIC_EnableIRQ(ETH_IRQn);
  }
}
#endif

/**************************************************************************************************************
 Declaration : HAL_ETH_MspDeInit
 Parameters  : ETH handle pointer
 Return Value: None
 Description : ETH MSP De-Initialization
**************************************************************************************************************/
#if defined(HAL_ETH_MODULE_ENABLED) && defined(CORE_CM4)
void HAL_ETH_MspDeInit(ETH_HandleTypeDef *hETHx)
{
  if (hETHx->Instance == ETH) {
    /* EMAC Peripheral clock disable */
    __HAL_RCC_ETH1MAC_CLK_DISABLE();
    __HAL_RCC_ETH1TX_CLK_DISABLE ();
    __HAL_RCC_ETH1RX_CLK_DISABLE ();

    /* ETH GPIO Configuration */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_7);
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_13);
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 );
    HAL_GPIO_DeInit(GPIOG, GPIO_PIN_11 | GPIO_PIN_13);

    /* ETH interrupt DeInit */
    HAL_NVIC_DisableIRQ(ETH_IRQn);
  }
}
#endif

/**************************************************************************************************************
 Declaration : HAL_SPI_MspInit
 Parameters  : SPI handle pointer
 Return Value: None
 Description : Initializes the Global SPI
**************************************************************************************************************/
#if defined(HAL_SPI_MODULE_ENABLED) && defined(CORE_CM7)
void HAL_SPI_MspInit(SPI_HandleTypeDef *hSPIx)
{
  GPIO_InitTypeDef GPIO_InitStruct = { .Mode  = GPIO_MODE_AF_PP,
                                       .Pull  = GPIO_NOPULL,
                                       .Speed = GPIO_SPEED_FREQ_LOW };

  /* Initializes the Global SPI1 */
  if /**/ (hSPIx->Instance == SPI1) {
    /* Peripherals clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    /* Initializes SPI GPIO Pin */
    GPIO_InitStruct.Pin       = GPIO_PIN_5;  /* SPI_SCK  */
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = GPIO_PIN_4;  /* SPI_NSS  */
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = GPIO_PIN_5;  /* SPI_MOSI */
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = GPIO_PIN_6;  /* SPI_MISO */
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* SPI1 RX DMA Initializes */
    hDMA_SPI1_RX.Instance                 = DMA1_Stream0;
    hDMA_SPI1_RX.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hDMA_SPI1_RX.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hDMA_SPI1_RX.Init.MemBurst            = DMA_MBURST_INC4;
    hDMA_SPI1_RX.Init.PeriphBurst         = DMA_PBURST_INC4;
    hDMA_SPI1_RX.Init.Request             = DMA_REQUEST_SPI1_RX;
    hDMA_SPI1_RX.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hDMA_SPI1_RX.Init.PeriphInc           = DMA_PINC_DISABLE;
    hDMA_SPI1_RX.Init.MemInc              = DMA_MINC_ENABLE;
    hDMA_SPI1_RX.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hDMA_SPI1_RX.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hDMA_SPI1_RX.Init.Mode                = DMA_NORMAL;
    hDMA_SPI1_RX.Init.Priority            = DMA_PRIORITY_LOW;

    if (HAL_DMA_Init(&hDMA_SPI1_RX) != HAL_OK) {
      ErrorHandler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(hSPIx, hdmarx, hDMA_SPI1_RX);

    /* SPI1 TX DMA Initializes */
    hDMA_SPI1_TX.Instance                 = DMA1_Stream1;
    hDMA_SPI1_TX.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hDMA_SPI1_TX.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hDMA_SPI1_TX.Init.MemBurst            = DMA_MBURST_INC4;
    hDMA_SPI1_TX.Init.PeriphBurst         = DMA_PBURST_INC4;
    hDMA_SPI1_TX.Init.Request             = DMA_REQUEST_SPI1_TX;
    hDMA_SPI1_TX.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hDMA_SPI1_TX.Init.PeriphInc           = DMA_PINC_DISABLE;
    hDMA_SPI1_TX.Init.MemInc              = DMA_MINC_ENABLE;
    hDMA_SPI1_TX.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hDMA_SPI1_TX.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hDMA_SPI1_TX.Init.Mode                = DMA_NORMAL;
    hDMA_SPI1_TX.Init.Priority            = DMA_PRIORITY_LOW;

    if (HAL_DMA_Init(&hDMA_SPI1_TX) != HAL_OK) {
      ErrorHandler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(hSPIx, hdmatx, hDMA_SPI1_TX);

    /* NVIC configuration for DMA transfer complete interrupt */
    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ  (DMA1_Stream0_IRQn);

    /* NVIC configuration for DMA transfer complete interrupt */
    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ  (DMA1_Stream1_IRQn);

    /* NVIC configuration for SPI transfer complete interrupt */
    HAL_NVIC_SetPriority(SPI1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ  (SPI1_IRQn);
  }

  /* Initializes the Global SPI2 */
  else if (hSPIx->Instance == SPI2) {
    /* Peripherals clock enable */
    __HAL_RCC_SPI2_CLK_ENABLE();

    /* Initializes SPI GPIO Pin */
    GPIO_InitStruct.Pin       = GPIO_PIN_10; /* SPI_SCK  */
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = GPIO_PIN_12; /* SPI_NSS  */
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = GPIO_PIN_3;  /* SPI_MOSI */
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = GPIO_PIN_2;  /* SPI_MISO */
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* SPI2 RX DMA Initializes */
    hDMA_SPI2_RX.Instance                 = DMA1_Stream2;
    hDMA_SPI2_RX.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hDMA_SPI2_RX.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hDMA_SPI2_RX.Init.MemBurst            = DMA_MBURST_INC4;
    hDMA_SPI2_RX.Init.PeriphBurst         = DMA_PBURST_INC4;
    hDMA_SPI2_RX.Init.Request             = DMA_REQUEST_SPI2_RX;
    hDMA_SPI2_RX.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hDMA_SPI2_RX.Init.PeriphInc           = DMA_PINC_DISABLE;
    hDMA_SPI2_RX.Init.MemInc              = DMA_MINC_ENABLE;
    hDMA_SPI2_RX.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hDMA_SPI2_RX.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hDMA_SPI2_RX.Init.Mode                = DMA_NORMAL;
    hDMA_SPI2_RX.Init.Priority            = DMA_PRIORITY_LOW;

    if (HAL_DMA_Init(&hDMA_SPI2_RX) != HAL_OK) {
      ErrorHandler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(hSPIx, hdmarx, hDMA_SPI2_RX);

    /* SPI2 TX DMA Initializes */
    hDMA_SPI2_TX.Instance                 = DMA1_Stream3;
    hDMA_SPI2_TX.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hDMA_SPI2_TX.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hDMA_SPI2_TX.Init.MemBurst            = DMA_MBURST_INC4;
    hDMA_SPI2_TX.Init.PeriphBurst         = DMA_PBURST_INC4;
    hDMA_SPI2_TX.Init.Request             = DMA_REQUEST_SPI2_TX;
    hDMA_SPI2_TX.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hDMA_SPI2_TX.Init.PeriphInc           = DMA_PINC_DISABLE;
    hDMA_SPI2_TX.Init.MemInc              = DMA_MINC_ENABLE;
    hDMA_SPI2_TX.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hDMA_SPI2_TX.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hDMA_SPI2_TX.Init.Mode                = DMA_NORMAL;
    hDMA_SPI2_TX.Init.Priority            = DMA_PRIORITY_LOW;

    if (HAL_DMA_Init(&hDMA_SPI2_TX) != HAL_OK) {
      ErrorHandler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(hSPIx, hdmatx, hDMA_SPI2_TX);

    /* NVIC configuration for DMA transfer complete interrupt */
    HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ  (DMA1_Stream2_IRQn);

    /* NVIC configuration for DMA transfer complete interrupt */
    HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ  (DMA1_Stream3_IRQn);

    /* NVIC configuration for SPI transfer complete interrupt */
    HAL_NVIC_SetPriority(SPI2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ  (SPI2_IRQn);
  }

  /* Initializes the Global SPI3 */
  else if (hSPIx->Instance == SPI3) {
    /* Peripherals clock enable */
    __HAL_RCC_SPI3_CLK_ENABLE();

    /* Initializes SPI GPIO Pin */
    GPIO_InitStruct.Pin       = GPIO_PIN_10; /* SPI_SCK  */
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = GPIO_PIN_15; /* SPI_NSS  */
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = GPIO_PIN_2;  /* SPI_MOSI */
    GPIO_InitStruct.Alternate = GPIO_AF7_SPI3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = GPIO_PIN_11; /* SPI_MISO */
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* SPI3 RX DMA Initializes */
    hDMA_SPI3_RX.Instance                 = DMA1_Stream4;
    hDMA_SPI3_RX.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hDMA_SPI3_RX.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hDMA_SPI3_RX.Init.MemBurst            = DMA_MBURST_INC4;
    hDMA_SPI3_RX.Init.PeriphBurst         = DMA_PBURST_INC4;
    hDMA_SPI3_RX.Init.Request             = DMA_REQUEST_SPI3_RX;
    hDMA_SPI3_RX.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hDMA_SPI3_RX.Init.PeriphInc           = DMA_PINC_DISABLE;
    hDMA_SPI3_RX.Init.MemInc              = DMA_MINC_ENABLE;
    hDMA_SPI3_RX.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hDMA_SPI3_RX.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hDMA_SPI3_RX.Init.Mode                = DMA_NORMAL;
    hDMA_SPI3_RX.Init.Priority            = DMA_PRIORITY_LOW;

    if (HAL_DMA_Init(&hDMA_SPI3_RX) != HAL_OK) {
      ErrorHandler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(hSPIx, hdmarx, hDMA_SPI3_RX);

    /* SPI3 TX DMA Initializes */
    hDMA_SPI3_TX.Instance                 = DMA1_Stream5;
    hDMA_SPI3_TX.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hDMA_SPI3_TX.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hDMA_SPI3_TX.Init.MemBurst            = DMA_MBURST_INC4;
    hDMA_SPI3_TX.Init.PeriphBurst         = DMA_PBURST_INC4;
    hDMA_SPI3_TX.Init.Request             = DMA_REQUEST_SPI3_TX;
    hDMA_SPI3_TX.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hDMA_SPI3_TX.Init.PeriphInc           = DMA_PINC_DISABLE;
    hDMA_SPI3_TX.Init.MemInc              = DMA_MINC_ENABLE;
    hDMA_SPI3_TX.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hDMA_SPI3_TX.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hDMA_SPI3_TX.Init.Mode                = DMA_NORMAL;
    hDMA_SPI3_TX.Init.Priority            = DMA_PRIORITY_LOW;

    if (HAL_DMA_Init(&hDMA_SPI3_TX) != HAL_OK) {
      ErrorHandler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(hSPIx, hdmatx, hDMA_SPI3_TX);

    /* NVIC configuration for DMA transfer complete interrupt */
    HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ  (DMA1_Stream4_IRQn);

    /* NVIC configuration for DMA transfer complete interrupt */
    HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ  (DMA1_Stream5_IRQn);

    /* NVIC configuration for SPI transfer complete interrupt */
    HAL_NVIC_SetPriority(SPI3_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ  (SPI3_IRQn);
  }

  /* Initializes the Global SPI4 */
  else if (hSPIx->Instance == SPI4) {
    /* Peripherals clock enable */
    __HAL_RCC_SPI4_CLK_ENABLE();

    /* Initializes SPI GPIO Pin */
    GPIO_InitStruct.Pin       = GPIO_PIN_2;  /* SPI_SCK  */
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI4;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = GPIO_PIN_4;  /* SPI_NSS  */
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI4;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = GPIO_PIN_6;  /* SPI_MOSI */
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI4;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = GPIO_PIN_5;  /* SPI_MISO */
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI4;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* SPI4 RX DMA Initializes */
    hDMA_SPI4_RX.Instance                 = DMA1_Stream6;
    hDMA_SPI4_RX.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hDMA_SPI4_RX.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hDMA_SPI4_RX.Init.MemBurst            = DMA_MBURST_INC4;
    hDMA_SPI4_RX.Init.PeriphBurst         = DMA_PBURST_INC4;
    hDMA_SPI4_RX.Init.Request             = DMA_REQUEST_SPI4_RX;
    hDMA_SPI4_RX.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hDMA_SPI4_RX.Init.PeriphInc           = DMA_PINC_DISABLE;
    hDMA_SPI4_RX.Init.MemInc              = DMA_MINC_ENABLE;
    hDMA_SPI4_RX.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hDMA_SPI4_RX.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hDMA_SPI4_RX.Init.Mode                = DMA_NORMAL;
    hDMA_SPI4_RX.Init.Priority            = DMA_PRIORITY_LOW;

    if (HAL_DMA_Init(&hDMA_SPI4_RX) != HAL_OK) {
      ErrorHandler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(hSPIx, hdmarx, hDMA_SPI4_RX);

    /* SPI4 TX DMA Initializes */
    hDMA_SPI4_TX.Instance                 = DMA1_Stream7;
    hDMA_SPI4_TX.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hDMA_SPI4_TX.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hDMA_SPI4_TX.Init.MemBurst            = DMA_MBURST_INC4;
    hDMA_SPI4_TX.Init.PeriphBurst         = DMA_PBURST_INC4;
    hDMA_SPI4_TX.Init.Request             = DMA_REQUEST_SPI4_TX;
    hDMA_SPI4_TX.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hDMA_SPI4_TX.Init.PeriphInc           = DMA_PINC_DISABLE;
    hDMA_SPI4_TX.Init.MemInc              = DMA_MINC_ENABLE;
    hDMA_SPI4_TX.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hDMA_SPI4_TX.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hDMA_SPI4_TX.Init.Mode                = DMA_NORMAL;
    hDMA_SPI4_TX.Init.Priority            = DMA_PRIORITY_LOW;

    if (HAL_DMA_Init(&hDMA_SPI4_TX) != HAL_OK) {
      ErrorHandler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(hSPIx, hdmatx, hDMA_SPI4_TX);

    /* NVIC configuration for DMA transfer complete interrupt */
    HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ  (DMA1_Stream6_IRQn);

    /* NVIC configuration for DMA transfer complete interrupt */
    HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ  (DMA1_Stream7_IRQn);

    /* NVIC configuration for SPI transfer complete interrupt */
    HAL_NVIC_SetPriority(SPI4_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ  (SPI4_IRQn);
  }

  /* Initializes the Global SPI5 */
  else if (hSPIx->Instance == SPI5) {
    /* Peripherals clock enable */
    __HAL_RCC_SPI5_CLK_ENABLE();

    /* Initializes SPI GPIO Pin */
    GPIO_InitStruct.Pin       = GPIO_PIN_7;  /* SPI_SCK  */
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI5;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = GPIO_PIN_6;  /* SPI_NSS  */
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI5;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = GPIO_PIN_9;  /* SPI_MOSI */
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI5;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = GPIO_PIN_8;  /* SPI_MISO */
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI5;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    /* SPI5 RX DMA Initializes */
    hDMA_SPI5_RX.Instance                 = DMA2_Stream0;
    hDMA_SPI5_RX.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hDMA_SPI5_RX.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hDMA_SPI5_RX.Init.MemBurst            = DMA_MBURST_INC4;
    hDMA_SPI5_RX.Init.PeriphBurst         = DMA_PBURST_INC4;
    hDMA_SPI5_RX.Init.Request             = DMA_REQUEST_SPI5_RX;
    hDMA_SPI5_RX.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hDMA_SPI5_RX.Init.PeriphInc           = DMA_PINC_DISABLE;
    hDMA_SPI5_RX.Init.MemInc              = DMA_MINC_ENABLE;
    hDMA_SPI5_RX.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hDMA_SPI5_RX.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hDMA_SPI5_RX.Init.Mode                = DMA_NORMAL;
    hDMA_SPI5_RX.Init.Priority            = DMA_PRIORITY_LOW;

    if (HAL_DMA_Init(&hDMA_SPI5_RX) != HAL_OK) {
      ErrorHandler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(hSPIx, hdmarx, hDMA_SPI5_RX);

    /* SPI5 TX DMA Initializes */
    hDMA_SPI5_TX.Instance                 = DMA2_Stream1;
    hDMA_SPI5_TX.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hDMA_SPI5_TX.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hDMA_SPI5_TX.Init.MemBurst            = DMA_MBURST_INC4;
    hDMA_SPI5_TX.Init.PeriphBurst         = DMA_PBURST_INC4;
    hDMA_SPI5_TX.Init.Request             = DMA_REQUEST_SPI5_TX;
    hDMA_SPI5_TX.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hDMA_SPI5_TX.Init.PeriphInc           = DMA_PINC_DISABLE;
    hDMA_SPI5_TX.Init.MemInc              = DMA_MINC_ENABLE;
    hDMA_SPI5_TX.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hDMA_SPI5_TX.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hDMA_SPI5_TX.Init.Mode                = DMA_NORMAL;
    hDMA_SPI5_TX.Init.Priority            = DMA_PRIORITY_LOW;

    if (HAL_DMA_Init(&hDMA_SPI5_TX) != HAL_OK) {
      ErrorHandler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(hSPIx, hdmatx, hDMA_SPI5_TX);

    /* NVIC configuration for DMA transfer complete interrupt */
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ  (DMA2_Stream0_IRQn);

    /* NVIC configuration for DMA transfer complete interrupt */
    HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ  (DMA2_Stream1_IRQn);

    /* NVIC configuration for SPI transfer complete interrupt */
    HAL_NVIC_SetPriority(SPI5_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ  (SPI5_IRQn);
  }

  /* Initializes the Global SPI6 */
  else if (hSPIx->Instance == SPI6) {
    /* Peripherals clock enable */
    __HAL_RCC_SPI6_CLK_ENABLE();

    /* Initializes SPI GPIO Pin */
    GPIO_InitStruct.Pin       = GPIO_PIN_3;  /* SPI_SCK  */
    GPIO_InitStruct.Alternate = GPIO_AF8_SPI6;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = GPIO_PIN_8;  /* SPI_NSS  */
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI6;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = GPIO_PIN_14; /* SPI_MOSI */
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI6;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = GPIO_PIN_12; /* SPI_MISO */
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI6;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    /* SPI6 RX DMA Initializes */
    hDMA_SPI6_RX.Instance                 = BDMA_Channel0;
    hDMA_SPI6_RX.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hDMA_SPI6_RX.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hDMA_SPI6_RX.Init.MemBurst            = DMA_MBURST_INC4;
    hDMA_SPI6_RX.Init.PeriphBurst         = DMA_PBURST_INC4;
    hDMA_SPI6_RX.Init.Request             = BDMA_REQUEST_SPI6_RX;
    hDMA_SPI6_RX.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hDMA_SPI6_RX.Init.PeriphInc           = DMA_PINC_DISABLE;
    hDMA_SPI6_RX.Init.MemInc              = DMA_MINC_ENABLE;
    hDMA_SPI6_RX.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hDMA_SPI6_RX.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hDMA_SPI6_RX.Init.Mode                = DMA_NORMAL;
    hDMA_SPI6_RX.Init.Priority            = DMA_PRIORITY_LOW;

    if (HAL_DMA_Init(&hDMA_SPI6_RX) != HAL_OK) {
      ErrorHandler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(hSPIx, hdmarx, hDMA_SPI6_RX);

    /* SPI6 TX DMA Initializes */
    hDMA_SPI6_TX.Instance                 = BDMA_Channel1;
    hDMA_SPI6_TX.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hDMA_SPI6_TX.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hDMA_SPI6_TX.Init.MemBurst            = DMA_MBURST_INC4;
    hDMA_SPI6_TX.Init.PeriphBurst         = DMA_PBURST_INC4;
    hDMA_SPI6_TX.Init.Request             = BDMA_REQUEST_SPI6_TX;
    hDMA_SPI6_TX.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hDMA_SPI6_TX.Init.PeriphInc           = DMA_PINC_DISABLE;
    hDMA_SPI6_TX.Init.MemInc              = DMA_MINC_ENABLE;
    hDMA_SPI6_TX.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hDMA_SPI6_TX.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hDMA_SPI6_TX.Init.Mode                = DMA_NORMAL;
    hDMA_SPI6_TX.Init.Priority            = DMA_PRIORITY_LOW;

    if (HAL_DMA_Init(&hDMA_SPI6_TX) != HAL_OK) {
      ErrorHandler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(hSPIx, hdmatx, hDMA_SPI6_TX);

    /* BDMA_Channel0_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(BDMA_Channel0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ  (BDMA_Channel0_IRQn);

    /* BDMA_Channel1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(BDMA_Channel1_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ  (BDMA_Channel1_IRQn);

    /* NVIC configuration for SPI transfer complete interrupt */
    HAL_NVIC_SetPriority(SPI6_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ  (SPI6_IRQn);
  }
}
#endif

/**************************************************************************************************************
 Declaration : HAL_SPI_MspDeInit
 Parameters  : SPI handle pointer
 Return Value: None
 Description : DeInitializes the Global SPI
**************************************************************************************************************/
#if defined(HAL_SPI_MODULE_ENABLED) && defined(CORE_CM7)
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hSPIx)
{
  /* De-Initializes the Global SPI1 */
  if /**/ (hSPIx->Instance == SPI1) {
    __HAL_RCC_SPI1_CLK_DISABLE();
  }

  /* De-Initializes the Global SPI2 */
  else if (hSPIx->Instance == SPI2) {
    __HAL_RCC_SPI2_CLK_DISABLE();
  }

  /* De-Initializes the Global SPI3 */
  else if (hSPIx->Instance == SPI3) {
    __HAL_RCC_SPI3_CLK_DISABLE();
  }

  /* De-Initializes the Global SPI4 */
  else if (hSPIx->Instance == SPI4) {
    __HAL_RCC_SPI4_CLK_DISABLE();
  }

  /* De-Initializes the Global SPI5 */
  else if (hSPIx->Instance == SPI5) {
    __HAL_RCC_SPI5_CLK_DISABLE();
  }

  /* De-Initializes the Global SPI6 */
  else if (hSPIx->Instance == SPI6) {
    __HAL_RCC_SPI6_CLK_DISABLE();
  }
}
#endif

/**************************************************************************************************************
 Declaration : HAL_UART_MspInit
 Parameters  : UART handle pointer
 Return Value: None
 Description : Initializes the Global UART
**************************************************************************************************************/
#if defined(HAL_UART_MODULE_ENABLED) && defined(CORE_CM4)
void HAL_UART_MspInit(UART_HandleTypeDef* hUARTx)
{
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };

  /* Initializes the Global SPI1 */
  if /**/ (hUARTx->Instance == UART1) {
    /* Reset the peripherals */
     __HAL_RCC_USART1_FORCE_RESET ();
    __HAL_RCC_USART1_RELEASE_RESET();

    /* Peripheral clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    /* The GPIO Configuration */
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_USART1;

    /* The GPIO Initializes */
    GPIO_InitStruct.Pin = GPIO_PIN_15; /* USART1_RX */
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* The GPIO Initializes */
    GPIO_InitStruct.Pin = GPIO_PIN_14; /* USART1_TX */
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* The interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 12, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  }

  /* Initializes the Global USART2 */
  else if (hUARTx->Instance == UART2) {
    /* Peripheral clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    /* The GPIO Configuration */
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;

    /* The GPIO Initializes */
    GPIO_InitStruct.Pin = GPIO_PIN_3;  /* USART2_RX */
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* The GPIO Initializes */
    GPIO_InitStruct.Pin = GPIO_PIN_5;  /* USART2_TX */
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  }

  /* Initializes the Global USART3 */
  else if (hUARTx->Instance == UART3) {
    /* Peripheral clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    /* The GPIO Configuration */
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;

    /* The GPIO Initializes */
    GPIO_InitStruct.Pin = GPIO_PIN_11; /* USART3_RX */
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* The GPIO Initializes */
    GPIO_InitStruct.Pin = GPIO_PIN_8;  /* USART3_TX */
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  }

  /* Initializes the Global UART4 */
  else if (hUARTx->Instance == UART4) {
    /* Peripheral clock enable */
    __HAL_RCC_UART4_CLK_ENABLE();

    /* The GPIO Configuration */
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;

    /* The GPIO Initializes */
    GPIO_InitStruct.Pin = GPIO_PIN_11; /* USART4_RX */
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* The GPIO Initializes */
    GPIO_InitStruct.Pin = GPIO_PIN_0;  /* USART4_TX */
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }

  /* Initializes the Global UART5 */
  else if (hUARTx->Instance == UART5) {
    __HAL_RCC_UART5_CLK_ENABLE();

    /* The GPIO Configuration */
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART5;

    /* The GPIO Initializes */
    GPIO_InitStruct.Pin = GPIO_PIN_2;  /* USART5_RX */
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* The GPIO Initializes */
    GPIO_InitStruct.Pin = GPIO_PIN_12; /* USART5_TX */
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  }

  /* Initializes the Global USART6 */
  else if (hUARTx->Instance == UART6) {
    /* Peripheral clock enable */
    __HAL_RCC_USART6_CLK_ENABLE();

    /* The GPIO Configuration */
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART6;

    /* The GPIO Initializes */
    GPIO_InitStruct.Pin = GPIO_PIN_7;  /* USART6_RX */
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* The GPIO Initializes */
    GPIO_InitStruct.Pin = GPIO_PIN_6;  /* USART6_TX */
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  }

  /* Initializes the Global UART7 */
  else if (hUARTx->Instance == UART7) {
    __HAL_RCC_UART7_CLK_ENABLE();

    /* The GPIO Configuration */
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_UART7;

    /* The GPIO Initializes */
    GPIO_InitStruct.Pin = GPIO_PIN_7;  /* USART7_RX */
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* The GPIO Initializes */
    GPIO_InitStruct.Pin = GPIO_PIN_8;  /* USART7_TX */
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  }

  /* Initializes the Global UART8 */
  else if (hUARTx->Instance == UART8) {
    __HAL_RCC_UART8_CLK_ENABLE();

    /* The GPIO Configuration */
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART8;

    /* The GPIO Initializes */
    GPIO_InitStruct.Pin = GPIO_PIN_0;  /* USART8_RX */
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* The GPIO Initializes */
    GPIO_InitStruct.Pin = GPIO_PIN_1;  /* USART8_TX */
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  }
}
#endif

/**************************************************************************************************************
 Declaration : HAL_UART_MspDeInit
 Parameters  : UART handle pointer
 Return Value: None
 Description : DeInitializes the Global UART
**************************************************************************************************************/
#if defined(HAL_UART_MODULE_ENABLED) && defined(CORE_CM4)
void HAL_UART_MspDeInit(UART_HandleTypeDef* hUARTx)
{
  /* De-Initializes the Global SPI1 */
  if /**/ (hUARTx->Instance == UART1) {
     __HAL_RCC_USART1_FORCE_RESET();
     __HAL_RCC_USART1_RELEASE_RESET();

    __HAL_RCC_USART1_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  }

  /* De-Initializes the Global USART2 */
  else if (hUARTx->Instance == UART2) {
    __HAL_RCC_USART2_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  }

  /* De-Initializes the Global USART3 */
  else if (hUARTx->Instance == UART3) {
    __HAL_RCC_USART3_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  }

  /* De-Initializes the Global UART4 */
  else if (hUARTx->Instance == UART4) {
    __HAL_RCC_UART4_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(UART4_IRQn);
  }

  /* De-Initializes the Global UART5 */
  else if (hUARTx->Instance == UART5) {
    __HAL_RCC_UART5_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(UART5_IRQn);
  }

  /* De-Initializes the Global USART6 */
  else if (hUARTx->Instance == UART6) {
    __HAL_RCC_USART6_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(USART6_IRQn);
  }

  /* De-Initializes the Global UART7 */
  else if (hUARTx->Instance == UART7) {
    __HAL_RCC_UART7_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(UART8_IRQn);
  }

  /* De-Initializes the Global UART8 */
  else if (hUARTx->Instance == UART8) {
    __HAL_RCC_UART8_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(UART7_IRQn);
  }
}
#endif

/**************************************************************************************************************
 Declaration :
 Parameters  :
 Return Value:
 Description :
**************************************************************************************************************/
#if defined(HAL_ADC_MODULE_ENABLED) && defined(CORE_CM4)
void HAL_ADC_MspInit(ADC_HandleTypeDef *hADC)
{
  if (hADC->Instance == ADC1) {
    GPIO_InitTypeDef         GPIO_InitStruct  = { 0 };
    static DMA_HandleTypeDef DMA_HandleStruct = { 0 };

    /* Peripheral clock enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    /* ADC Periph clock enable */
    __HAL_RCC_ADC12_CLK_ENABLE();

    /* ADC Periph interface clock configuration */
    __HAL_RCC_ADC_CONFIG(RCC_ADCCLKSOURCE_CLKP);

    /* Enable DMA clock */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* Configure peripheral GPIO */
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;

    GPIO_InitStruct.Pin = GPIO_PIN_3;  /* ADC1_INP15 */
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0;  /* ADC1_INP10 */
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_11; /* ADC1_INP2  */
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    /* Configure DMA parameters */
    DMA_HandleStruct.Instance                 = DMA2_Stream2;
    DMA_HandleStruct.Init.Request             = DMA_REQUEST_ADC1;
    DMA_HandleStruct.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    DMA_HandleStruct.Init.PeriphInc           = DMA_PINC_DISABLE;
    DMA_HandleStruct.Init.MemInc              = DMA_MINC_ENABLE;
    DMA_HandleStruct.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    DMA_HandleStruct.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    DMA_HandleStruct.Init.Mode                = DMA_CIRCULAR;
    DMA_HandleStruct.Init.Priority            = DMA_PRIORITY_LOW;
    DMA_HandleStruct.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;

    /* Initialize the DMA */
    HAL_DMA_DeInit(&DMA_HandleStruct);
    HAL_DMA_Init(&DMA_HandleStruct);

    /* Associate the DMA handle */
    __HAL_LINKDMA(hADC, DMA_Handle, DMA_HandleStruct);

    /* NVIC configuration for DMA Input data interrupt */
    HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 12, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  }
}
#endif

/**************************************************************************************************************
 Declaration :
 Parameters  :
 Return Value:
 Description :
**************************************************************************************************************/
#if defined(HAL_ADC_MODULE_ENABLED) && defined(CORE_CM4)
void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hADC)
{
  if (hADC->Instance == ADC1) {
    /* Reset peripherals */
    __HAL_RCC_ADC12_FORCE_RESET();
    __HAL_RCC_ADC12_RELEASE_RESET();

    /* ADC Periph clock disable */
    __HAL_RCC_ADC12_CLK_DISABLE();

    /* De-initialize the ADC Channel GPIO pin */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_3);
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_0);
    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_11);

    /* Initialize the DMA */
    HAL_DMA_DeInit(hADC->DMA_Handle);
  }
}
#endif
