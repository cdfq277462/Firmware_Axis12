/**************************************************************************************************************
 File   :
 Author :
 Date   :
 Brief  :
**************************************************************************************************************/

/* Includes **************************************************************************************************/
#include "ADC_STM32H7.h"

/* Variables *************************************************************************************************/
ADC_HandleTypeDef hADC1 = { .Instance = ADC1 };
uint16_t ADC1_Values[3] __MEMORY_AT(0x30046000);

/**************************************************************************************************************
 Declaration :
 Parameters  :
 Return Value:
 Description :
**************************************************************************************************************/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hADC)
{
	int __useSimulateLight = (TMC5->W.RESERVE_16 >> 8) & 0x000F;
	if(__useSimulateLight != 1)
		TMC1->R.RESERVE_21 = (uint32_t) ((3.3 / 65535.0 * (double) ADC1_Values[0]) * 1000.0);
  TMC2->R.RESERVE_21 = (uint32_t) ((3.3 / 65535.0 * (double) ADC1_Values[1]) * 1000.0);
  TMC3->R.RESERVE_21 = (uint32_t) ((3.3 / 65535.0 * (double) ADC1_Values[2]) * 1000.0);
}

/**************************************************************************************************************
 Declaration :
 Parameters  :
 Return Value:
 Description :
**************************************************************************************************************/
HAL_StatusTypeDef ADC_Init(void)
{
  ADC_ChannelConfTypeDef ChannelConfStruct = { 0 };

  /* Initialize the ADC peripheral */
  hADC1.Init.ClockPrescaler           = ADC_CLOCK_ASYNC_DIV8;
  hADC1.Init.Resolution               = ADC_RESOLUTION_16B;
  hADC1.Init.ScanConvMode             = ADC_SCAN_ENABLE;
  hADC1.Init.EOCSelection             = ADC_EOC_SINGLE_CONV;
  hADC1.Init.LowPowerAutoWait         = DISABLE;
  hADC1.Init.ContinuousConvMode       = ENABLE;
  hADC1.Init.NbrOfConversion          = 3;
  hADC1.Init.DiscontinuousConvMode    = DISABLE;
  hADC1.Init.ExternalTrigConv         = ADC_SOFTWARE_START;
  hADC1.Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hADC1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hADC1.Init.Overrun                  = ADC_OVR_DATA_OVERWRITTEN;
  hADC1.Init.LeftBitShift             = ADC_LEFTBITSHIFT_NONE;
  hADC1.Init.OversamplingMode         = DISABLE;

  if (HAL_ADC_Init(&hADC1) != HAL_OK) {
    ErrorHandler(__FILE__, __LINE__);
  }

  /* Start the ADC calibration */
  if (HAL_ADCEx_Calibration_Start(&hADC1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) {
    ErrorHandler(__FILE__, __LINE__);
  }

  /* Configure Regular Channel */
  ChannelConfStruct.SamplingTime           = ADC_SAMPLETIME_810CYCLES_5;
  ChannelConfStruct.SingleDiff             = ADC_SINGLE_ENDED;
  ChannelConfStruct.OffsetNumber           = ADC_OFFSET_NONE;
  ChannelConfStruct.Offset                 = 0;
  ChannelConfStruct.OffsetSignedSaturation = DISABLE;

  ChannelConfStruct.Channel = ADC_CHANNEL_15;
  ChannelConfStruct.Rank    = ADC_REGULAR_RANK_1;
  if (HAL_ADC_ConfigChannel(&hADC1, &ChannelConfStruct) != HAL_OK) {
    ErrorHandler(__FILE__, __LINE__);
  }

  ChannelConfStruct.Channel = ADC_CHANNEL_10;
  ChannelConfStruct.Rank    = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hADC1, &ChannelConfStruct) != HAL_OK) {
    ErrorHandler(__FILE__, __LINE__);
  }

  ChannelConfStruct.Channel = ADC_CHANNEL_2;
  ChannelConfStruct.Rank    = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hADC1, &ChannelConfStruct) != HAL_OK) {
    ErrorHandler(__FILE__, __LINE__);
  }

  /* Start conversion in DMA mode */
  if (HAL_ADC_Start_DMA(&hADC1, (uint32_t *) ADC1_Values, countof(ADC1_Values))  != HAL_OK) {
    ErrorHandler(__FILE__, __LINE__);
  }

  /* Return HAL status */
  return HAL_OK;
}