/**************************************************************************************************************
 File   :
 Author :
 Date   :
 Brief  :
**************************************************************************************************************/

/* Includes **************************************************************************************************/
#include "TMC_STM32H7.h"

/**************************************************************************************************************
 Declaration : MOT_Init
 Parameters  : hMOTx, MOT handle
 Return Value: HAL Status
 Description : Initialize TMC
**************************************************************************************************************/
HAL_StatusTypeDef TMC_Init(MOT_HandleTypeDef *hMOTx)
{
  GPIO_InitTypeDef GPIO_InitStruct = { .Mode  = GPIO_MODE_OUTPUT_PP,
                                       .Pull  = GPIO_NOPULL,
                                       .Speed = GPIO_SPEED_HIGH };

  /* Check parameters */
  assert_param(&hMOT1 == hMOTx || &hMOT2 == hMOTx || &hMOT3 == hMOTx ||
               &hMOT4 == hMOTx || &hMOT5 == hMOTx || &hMOT6 == hMOTx);

  if (TMC1 == hMOTx->Instance) {
    /* EN_TMC4671 */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = GPIO_PIN_14;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
  } else
  if (TMC2 == hMOTx->Instance) {
    /* EN_TMC4671 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
  } else
  if (TMC3 == hMOTx->Instance) {
    /* EN_TMC4671 */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
  } else
  if (TMC4 == hMOTx->Instance) {
    /* EN_TMC4671 */
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
  } else
  if (TMC5 == hMOTx->Instance) {
    /* EN_TMC4671 */
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = GPIO_PIN_14;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
  } else
  if (TMC6 == hMOTx->Instance) {
    /* EN_TMC4671 */
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_10, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_10, GPIO_PIN_RESET);
  }
  else {
		
    return HAL_ERROR;
  }
	
	
	  
			/* Enable the TMC4671 ENI pin */
  VERIFY(TMC_Enable(&hMOT1) == HAL_OK);
  VERIFY(TMC_Enable(&hMOT2) == HAL_OK);
  VERIFY(TMC_Enable(&hMOT3) == HAL_OK);
  VERIFY(TMC_Enable(&hMOT4) == HAL_OK);
  VERIFY(TMC_Enable(&hMOT5) == HAL_OK);
  VERIFY(TMC_Enable(&hMOT6) == HAL_OK);
		  /* Wait a whlie */
  VERIFY(osDelay(100) == osOK);
	
	
	if (TMC1 == hMOTx->Instance | TMC2 == hMOTx->Instance | TMC3 == hMOTx->Instance) {
	/* Set Motor Type */
		VERIFY(MOT_Write(hMOTx, TMC4671_MOTOR_TYPE_N_POLE_PAIRS       , 0x00020032) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_PWM_POLARITIES                , 0x00000000) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_PWM_MAXCNT                    , 0x00000F9F) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_PWM_BBM_H_BBM_L               , 0x00001919) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_PWM_SV_CHOP                   , 0x00000007) == HAL_OK);

		/* Set ADC Configuration */
		VERIFY(MOT_Write(hMOTx, TMC4671_ADC_I_SELECT                  , 0x18000100) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_dsADC_MCFG_B_MCFG_A           , 0x00100010) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_dsADC_MCLK_A                  , 0x20000000) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_dsADC_MCLK_B                  , 0x00000000) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_dsADC_MDEC_B_MDEC_A           , 0x014E014E) == HAL_OK);

#if 0
		VERIFY(MOT_Write(hMOTx, TMC4671_ADC_I1_SCALE_OFFSET           , 0x010081BF) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_ADC_I0_SCALE_OFFSET           , 0x010081D5) == HAL_OK);
#else
		VERIFY(MOT_Write(hMOTx, TMC4671_ADC_I0_SCALE_OFFSET           , 0x0100816F) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_ADC_I1_SCALE_OFFSET           , 0x0100821B) == HAL_OK);
		
#endif

		/* Set ABN Decoder */
		VERIFY(MOT_Write(hMOTx, TMC4671_ABN_DECODER_MODE              , 0x00000000) == HAL_OK);
		//VERIFY(MOT_Write(hMOTx, TMC4671_ABN_DECODER_PPR               , 0x00010000) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_ABN_DECODER_PPR               , 0x00003E80) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_ABN_DECODER_COUNT             , 0x0000154C) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_ABN_DECODER_PHI_E_PHI_M_OFFSET, 0x00000000) == HAL_OK);
		
		
		//VERIFY(MOT_Write(hMOTx, TMC4671_ABN_DECODER_PHI_E_PHI_M_OFFSET, 0x00000000) == HAL_OK);

		/* Set Limit */
#if 0
		VERIFY(MOT_Write(hMOTx, TMC4671_PIDOUT_UQ_UD_LIMITS           , 0x00007FFF) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_PID_TORQUE_FLUX_LIMITS        , 0x00000BB8) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_PID_ACCELERATION_LIMIT        , 0x00000BB8) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_PID_VELOCITY_LIMIT            , 0x00000BB8) == HAL_OK);
#else
		VERIFY(MOT_Write(hMOTx, TMC4671_PID_TORQUE_FLUX_LIMITS        , 0x00001999) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_PIDOUT_UQ_UD_LIMITS           , 0x00005A81) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_PID_ACCELERATION_LIMIT        , 0x000007D0) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_PID_VELOCITY_LIMIT            , 0x000003E8) == HAL_OK);
#endif
		
	}
	else if (TMC4 == hMOTx->Instance | TMC5 == hMOTx->Instance | TMC6 == hMOTx->Instance) {
		/* Set Motor Type */
		VERIFY(MOT_Write(hMOTx, TMC4671_MOTOR_TYPE_N_POLE_PAIRS       , 0x00020032) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_PWM_POLARITIES                , 0x00000000) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_PWM_MAXCNT                    , 0x00000F9F) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_PWM_BBM_H_BBM_L               , 0x00001919) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_PWM_SV_CHOP                   , 0x00000007) == HAL_OK);

		/* Set ADC Configuration */
		VERIFY(MOT_Write(hMOTx, TMC4671_ADC_I_SELECT                  , 0x14000100) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_dsADC_MCFG_B_MCFG_A           , 0x00100010) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_dsADC_MCLK_A                  , 0x20000000) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_dsADC_MCLK_B                  , 0x00000000) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_dsADC_MDEC_B_MDEC_A           , 0x00FF00FF) == HAL_OK);

#if 0
		VERIFY(MOT_Write(hMOTx, TMC4671_ADC_I1_SCALE_OFFSET           , 0x010081BF) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_ADC_I0_SCALE_OFFSET           , 0x010081D5) == HAL_OK);
#else
		VERIFY(MOT_Write(hMOTx, TMC4671_ADC_I0_SCALE_OFFSET           , 0x01008162) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_ADC_I1_SCALE_OFFSET           , 0x01008227) == HAL_OK);
		
#endif

		/* Set ABN Decoder */
		VERIFY(MOT_Write(hMOTx, TMC4671_ABN_DECODER_MODE              , 0x00000000) == HAL_OK);
		//VERIFY(MOT_Write(hMOTx, TMC4671_ABN_DECODER_PPR               , 0x00010000) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_ABN_DECODER_PPR               , 0x00002580) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_ABN_DECODER_COUNT             , 0x0000000A) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_ABN_DECODER_PHI_E_PHI_M_OFFSET, 0x00000000) == HAL_OK);
		
		
		//VERIFY(MOT_Write(hMOTx, TMC4671_ABN_DECODER_PHI_E_PHI_M_OFFSET, 0x00000000) == HAL_OK);

		/* Set Limit */
#if 0
		VERIFY(MOT_Write(hMOTx, TMC4671_PIDOUT_UQ_UD_LIMITS           , 0x00007FFF) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_PID_TORQUE_FLUX_LIMITS        , 0x00000BB8) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_PID_ACCELERATION_LIMIT        , 0x00000BB8) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_PID_VELOCITY_LIMIT            , 0x00000BB8) == HAL_OK);
#else
		VERIFY(MOT_Write(hMOTx, TMC4671_PID_TORQUE_FLUX_LIMITS        , 0x000005DC) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_PIDOUT_UQ_UD_LIMITS           , 0x00007FFF) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_PID_ACCELERATION_LIMIT        , 0x000007D0) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_PID_VELOCITY_LIMIT            , 0x000003E8) == HAL_OK);
#endif
	}
  /* Set PI value */
#if 0
  VERIFY(MOT_Write(hMOTx, TMC4671_PID_TORQUE_P_TORQUE_I         , 0x025801A9) == HAL_OK);
  VERIFY(MOT_Write(hMOTx, TMC4671_PID_FLUX_P_FLUX_I             , 0x025801A9) == HAL_OK);
  VERIFY(MOT_Write(hMOTx, TMC4671_PID_VELOCITY_P_VELOCITY_I     , 0x0140007D) == HAL_OK);
  VERIFY(MOT_Write(hMOTx, TMC4671_PID_POSITION_P_POSITION_I     , 0x001F0000) == HAL_OK);
	
	// old
	VERIFY(MOT_Write(hMOTx, TMC4671_PID_TORQUE_P_TORQUE_I         , 0x00FF0020) == HAL_OK);
	VERIFY(MOT_Write(hMOTx, TMC4671_PID_FLUX_P_FLUX_I             , 0x00FF0020) == HAL_OK);
	VERIFY(MOT_Write(hMOTx, TMC4671_PID_VELOCITY_P_VELOCITY_I     , 0x0140007D) == HAL_OK);
	VERIFY(MOT_Write(hMOTx, TMC4671_PID_POSITION_P_POSITION_I     , 0x00200000) == HAL_OK);
#else
	//VERIFY(MOT_Write(hMOTx, TMC4671_PID_TORQUE_P_TORQUE_I         , 0x00FF0020) == HAL_OK);
	//VERIFY(MOT_Write(hMOTx, TMC4671_PID_FLUX_P_FLUX_I             , 0x00FF0020) == HAL_OK);
  if (TMC1 == hMOTx->Instance) {
		VERIFY(MOT_Write(hMOTx, TMC4671_PID_TORQUE_P_TORQUE_I         , 0x0C9A547C) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_PID_FLUX_P_FLUX_I             , 0x0C9A547C) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_PID_VELOCITY_P_VELOCITY_I     , 0x024302FF) == HAL_OK);
		//VERIFY(MOT_Write(hMOTx, TMC4671_PID_POSITION_P_POSITION_I     , 0x00030000) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_PID_POSITION_P_POSITION_I     , 0x00190000) == HAL_OK);

  } else
  if (TMC2 == hMOTx->Instance) {
		VERIFY(MOT_Write(hMOTx, TMC4671_PID_TORQUE_P_TORQUE_I         , 0x0C9A547C) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_PID_FLUX_P_FLUX_I             , 0x0C9A547C) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_PID_VELOCITY_P_VELOCITY_I     , 0x024302FF) == HAL_OK);
		//VERIFY(MOT_Write(hMOTx, TMC4671_PID_POSITION_P_POSITION_I     , 0x00030000) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_PID_POSITION_P_POSITION_I     , 0x00190000) == HAL_OK);

  } else
  if (TMC3 == hMOTx->Instance) {
		VERIFY(MOT_Write(hMOTx, TMC4671_PID_TORQUE_P_TORQUE_I         , 0x0C9A547C) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_PID_FLUX_P_FLUX_I             , 0x0C9A547C) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_PID_VELOCITY_P_VELOCITY_I     , 0x024302FF) == HAL_OK);
		//VERIFY(MOT_Write(hMOTx, TMC4671_PID_POSITION_P_POSITION_I     , 0x00030000) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_PID_POSITION_P_POSITION_I     , 0x00230000) == HAL_OK);

  } else
  if (TMC4 == hMOTx->Instance) {
//		VERIFY(MOT_Write(hMOTx, TMC4671_PID_TORQUE_P_TORQUE_I         , 0x05DC2EE0) == HAL_OK);
//		VERIFY(MOT_Write(hMOTx, TMC4671_PID_FLUX_P_FLUX_I             , 0x05DC2EE0) == HAL_OK);
//		VERIFY(MOT_Write(hMOTx, TMC4671_PID_VELOCITY_P_VELOCITY_I     , 0x01900258) == HAL_OK);
//		VERIFY(MOT_Write(hMOTx, TMC4671_PID_POSITION_P_POSITION_I     , 0x00070000) == HAL_OK);
		
		VERIFY(MOT_Write(hMOTx, TMC4671_PID_TORQUE_P_TORQUE_I         , 0x04B02328) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_PID_FLUX_P_FLUX_I             , 0x04B02328) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_PID_VELOCITY_P_VELOCITY_I     , 0x00C8012C) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_PID_POSITION_P_POSITION_I     , 0x00960000) == HAL_OK);
		
  } else
  if (TMC5 == hMOTx->Instance) {
//		VERIFY(MOT_Write(hMOTx, TMC4671_PID_TORQUE_P_TORQUE_I         , 0x05DC1770) == HAL_OK);
//		VERIFY(MOT_Write(hMOTx, TMC4671_PID_FLUX_P_FLUX_I             , 0x05DC1770) == HAL_OK);
//		VERIFY(MOT_Write(hMOTx, TMC4671_PID_VELOCITY_P_VELOCITY_I     , 0x019002BC) == HAL_OK);
//		VERIFY(MOT_Write(hMOTx, TMC4671_PID_POSITION_P_POSITION_I     , 0x00080000) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_PID_TORQUE_P_TORQUE_I         , 0x04B02328) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_PID_FLUX_P_FLUX_I             , 0x04B02328) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_PID_VELOCITY_P_VELOCITY_I     , 0x00C8012C) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_PID_POSITION_P_POSITION_I     , 0x00960000) == HAL_OK);
  } else
  if (TMC6 == hMOTx->Instance) {
//		VERIFY(MOT_Write(hMOTx, TMC4671_PID_TORQUE_P_TORQUE_I         , 0x05DC1770) == HAL_OK);
//		VERIFY(MOT_Write(hMOTx, TMC4671_PID_FLUX_P_FLUX_I             , 0x05DC1770) == HAL_OK);
//		VERIFY(MOT_Write(hMOTx, TMC4671_PID_VELOCITY_P_VELOCITY_I     , 0x019002BC) == HAL_OK);
//		VERIFY(MOT_Write(hMOTx, TMC4671_PID_POSITION_P_POSITION_I     , 0x00080000) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_PID_TORQUE_P_TORQUE_I         , 0x04B02328) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_PID_FLUX_P_FLUX_I             , 0x04B02328) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_PID_VELOCITY_P_VELOCITY_I     , 0x00C8012C) == HAL_OK);
		VERIFY(MOT_Write(hMOTx, TMC4671_PID_POSITION_P_POSITION_I     , 0x00960000) == HAL_OK);
  }
#endif

  /* ABN Encoder Initial */
  VERIFY(MOT_Write(hMOTx, TMC4671_MODE_RAMP_MODE_MOTION         , 0x00000008) == HAL_OK);
  VERIFY(MOT_Write(hMOTx, TMC4671_ABN_DECODER_PHI_E_PHI_M_OFFSET, 0x00000000) == HAL_OK);
  VERIFY(MOT_Write(hMOTx, TMC4671_PHI_E_SELECTION               , 0x00000001) == HAL_OK);
  VERIFY(MOT_Write(hMOTx, TMC4671_PHI_E_EXT                     , 0x00000000) == HAL_OK);
  VERIFY(MOT_Write(hMOTx, TMC4671_UQ_UD_EXT                     , 0x000007D0) == HAL_OK);
	VERIFY(osDelay(1000) == osOK);
	
	VERIFY(MOT_Write(hMOTx, TMC4671_ABN_DECODER_COUNT             , 0x00000000) == HAL_OK);
//	VERIFY(MOT_Write(hMOTx, TMC4671_ABN_DECODER_COUNT_N           , 0x00000000) == HAL_OK);
//	VERIFY(MOT_Write(hMOTx, TMC4671_ABN_2_DECODER_COUNT           , 0x00000000) == HAL_OK);
//	VERIFY(MOT_Write(hMOTx, TMC4671_ABN_2_DECODER_COUNT_N         , 0x00000000) == HAL_OK);
//	VERIFY(MOT_Write(hMOTx, TMC4671_ABN_DECODER_COUNT             , 0x00000EB3) == HAL_OK);
//	VERIFY(MOT_Write(hMOTx, TMC4671_ABN_DECODER_COUNT_N           , 0x00002AEF) == HAL_OK);
//	VERIFY(MOT_Write(hMOTx, TMC4671_ABN_2_DECODER_COUNT           , 0x0000913C) == HAL_OK);
//	VERIFY(MOT_Write(hMOTx, TMC4671_ABN_2_DECODER_COUNT_N         , 0x0000913C) == HAL_OK);
	
  /* Set FOC Selection */
  VERIFY(MOT_Write(hMOTx, TMC4671_PHI_E_SELECTION               , 0x00000003) == HAL_OK);
  VERIFY(MOT_Write(hMOTx, TMC4671_VELOCITY_SELECTION            , 0x00000009) == HAL_OK);
  VERIFY(MOT_Write(hMOTx, TMC4671_POSITION_SELECTION            , 0x00000000) == HAL_OK);
	
  // Switch to torque mode
  VERIFY(MOT_Write(hMOTx, TMC4671_MODE_RAMP_MODE_MOTION         , 0x00000001) == HAL_OK);

  // Rotate right
	VERIFY(MOT_Write(hMOTx, TMC4671_PID_TORQUE_FLUX_TARGET         , 0x01900000) == HAL_OK);
	
	VERIFY(osDelay(100) == osOK);
	
	// Rotate left
	VERIFY(MOT_Write(hMOTx, TMC4671_PID_TORQUE_FLUX_TARGET         , 0xFE700000) == HAL_OK);

	VERIFY(osDelay(100) == osOK);

  /* Switch stop mode */
  //VERIFY(MOT_Write(hMOTx, TMC4671_MODE_RAMP_MODE_MOTION         , 0x00000001) == HAL_OK);
  //VERIFY(MOT_Write(hMOTx, TMC4671_PID_POSITION_TARGET           , 0x00000000) == HAL_OK);
  //VERIFY(MOT_Write(hMOTx, TMC4671_PID_POSITION_ACTUAL           , 0x00000000) == HAL_OK);
  //VERIFY(MOT_Write(hMOTx, TMC4671_MODE_RAMP_MODE_MOTION         , 0x00000003) == HAL_OK);
	//VERIFY(MOT_Write(hMOTx, TMC4671_STATUS_FLAGS           , 0xF0788080) == HAL_OK);
	VERIFY(MOT_Write(hMOTx, TMC4671_PID_TORQUE_FLUX_TARGET         , 0x00000000) == HAL_OK);
	
  VERIFY(osDelay(100) == osOK);

  if (TMC1 == hMOTx->Instance | TMC2 == hMOTx->Instance | TMC3 == hMOTx->Instance) {
	
	// Switch to velocity mode
  VERIFY(MOT_Write(hMOTx, TMC4671_MODE_RAMP_MODE_MOTION         , 0x00000002) == HAL_OK);
  VERIFY(osDelay(100) == osOK);
	VERIFY(MOT_Write(hMOTx, TMC4671_PID_TORQUE_FLUX_TARGET         , 0x0000FA24) == HAL_OK);
	
  }


	
	
	//Test
	//VERIFY(MOT_Write(hMOTx, TMC4671_PID_TORQUE_FLUX_TARGET           , 0x000003E8) == HAL_OK);
	//VERIFY(osDelay(1000) == osOK);
	//VERIFY(MOT_Write(hMOTx, TMC4671_PID_TORQUE_FLUX_TARGET           , 0xFFFFFC18) == HAL_OK);
	//VERIFY(osDelay(1000) == osOK);
  //VERIFY(MOT_Write(hMOTx, TMC4671_PID_TORQUE_FLUX_TARGET           , 0x00000000) == HAL_OK);
		
  /* Return HAL status */
  return HAL_OK;
}

/**************************************************************************************************************
 Declaration : TMC_Enable
 Parameters  : hMOTx, MOT handle
 Return Value: HAL Status
 Description : Enable TMC
**************************************************************************************************************/
HAL_StatusTypeDef TMC_Enable (MOT_HandleTypeDef *hMOTx)
{
  /* EN_TMC4671 IO Pin */
  if (TMC1 == hMOTx->Instance) {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
  } else
  if (TMC2 == hMOTx->Instance) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1 , GPIO_PIN_SET);
  } else
  if (TMC3 == hMOTx->Instance) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8 , GPIO_PIN_SET);
  } else
  if (TMC4 == hMOTx->Instance) {
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3 , GPIO_PIN_SET);
  } else
  if (TMC5 == hMOTx->Instance) {
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);
  } else
  if (TMC6 == hMOTx->Instance) {
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_10, GPIO_PIN_SET);
  }
  else {
    return HAL_ERROR;
  }

  /* Return HAL status */
  return HAL_OK;
}

/**************************************************************************************************************
 Declaration : TMC_Disable
 Parameters  : hMOTx, MOT handle
 Return Value: HAL Status
 Description : Disable TMC
**************************************************************************************************************/
HAL_StatusTypeDef TMC_Disable(MOT_HandleTypeDef *hMOTx)
{
  /* EN_TMC4671 IO Pin */
  if (TMC1 == hMOTx->Instance) {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
  } else
  if (TMC2 == hMOTx->Instance) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1 , GPIO_PIN_RESET);
  } else
  if (TMC3 == hMOTx->Instance) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8 , GPIO_PIN_RESET);
  } else
  if (TMC4 == hMOTx->Instance) {
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3 , GPIO_PIN_RESET);
  } else
  if (TMC5 == hMOTx->Instance) {
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
  } else
  if (TMC6 == hMOTx->Instance) {
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_10, GPIO_PIN_RESET);
  }
  else {
    return HAL_ERROR;
  }

  /* Return HAL status */
  return HAL_OK;
}
