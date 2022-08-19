/**************************************************************************************************************
 File   :
 Author :
 Date   :
 Brief  : Responsible for all TMC4671 initialization and reading and writing
**************************************************************************************************************/

/* Includes **************************************************************************************************/
#include "IDLE.h"

/* Constants *************************************************************************************************/
const uint8_t AccessPermission[128] = { /* 0: Reserved, 1: Readable, 3: Readable And Writable */
  /* TMC4671_CHIPINFO_DATA                     0x00 */ (1 & 3),
  /* TMC4671_CHIPINFO_ADDR                     0x01 */ (3 & 3),
  /* TMC4671_ADC_RAW_DATA                      0x02 */ (1 & 3),
  /* TMC4671_ADC_RAW_ADDR                      0x03 */ (3 & 3),
  /* TMC4671_dsADC_MCFG_B_MCFG_A               0x04 */ (3 & 3),
  /* TMC4671_dsADC_MCLK_A                      0x05 */ (3 & 3),
  /* TMC4671_dsADC_MCLK_B                      0x06 */ (3 & 3),
  /* TMC4671_dsADC_MDEC_B_MDEC_A               0x07 */ (3 & 3),
  /* TMC4671_ADC_I1_SCALE_OFFSET               0x08 */ (3 & 3),
  /* TMC4671_ADC_I0_SCALE_OFFSET               0x09 */ (3 & 3),
  /* TMC4671_ADC_I_SELECT                      0x0A */ (3 & 3),
  /* TMC4671_ADC_I1_I0_EXT                     0x0B */ (3 & 3),
  /* TMC4671_DS_ANALOG_INPUT_STAGE_CFG         0x0C */ (3 & 3),
  /* TMC4671_AENC_0_SCALE_OFFSET               0x0D */ (3 & 3),
  /* TMC4671_AENC_1_SCALE_OFFSET               0x0E */ (3 & 3),
  /* TMC4671_AENC_2_SCALE_OFFSET               0x0F */ (3 & 3),
  /* TMC4671_RESERVE_01                        0x10 */ (0 & 3),
  /* TMC4671_AENC_SELECT                       0x11 */ (3 & 3),
  /* TMC4671_ADC_IWY_IUX                       0x12 */ (1 & 3),
  /* TMC4671_ADC_IV                            0x13 */ (1 & 3),
  /* TMC4671_RESERVE_02                        0x14 */ (0 & 3),
  /* TMC4671_AENC_WY_UX                        0x15 */ (1 & 3),
  /* TMC4671_AENC_VN                           0x16 */ (1 & 3),
  /* TMC4671_PWM_POLARITIES                    0x17 */ (3 & 3),
  /* TMC4671_PWM_MAXCNT                        0x18 */ (3 & 3),
  /* TMC4671_PWM_BBM_H_BBM_L                   0x19 */ (3 & 3),
  /* TMC4671_PWM_SV_CHOP                       0x1A */ (3 & 3),
  /* TMC4671_MOTOR_TYPE_N_POLE_PAIRS           0x1B */ (3 & 3),
  /* TMC4671_PHI_E_EXT                         0x1C */ (3 & 3),
  /* TMC4671_PHI_M_EXT                         0x1D */ (3 & 3),
  /* TMC4671_POSITION_EXT                      0x1E */ (3 & 3),
  /* TMC4671_OPENLOOP_MODE                     0x1F */ (3 & 3),
  /* TMC4671_OPENLOOP_ACCELERATION             0x20 */ (3 & 3),
  /* TMC4671_OPENLOOP_VELOCITY_TARGET          0x21 */ (3 & 3),
  /* TMC4671_OPENLOOP_VELOCITY_ACTUAL          0x22 */ (3 & 3),
  /* TMC4671_OPENLOOP_PHI                      0x23 */ (3 & 3),
  /* TMC4671_UQ_UD_EXT                         0x24 */ (3 & 3),
  /* TMC4671_ABN_DECODER_MODE                  0x25 */ (3 & 3),
  /* TMC4671_ABN_DECODER_PPR                   0x26 */ (3 & 3),
  /* TMC4671_ABN_DECODER_COUNT                 0x27 */ (3 & 3),
  /* TMC4671_ABN_DECODER_COUNT_N               0x28 */ (3 & 3),
  /* TMC4671_ABN_DECODER_PHI_E_PHI_M_OFFSET    0x29 */ (3 & 3),
  /* TMC4671_ABN_DECODER_PHI_E_PHI_M           0x2A */ (1 & 3),
  /* TMC4671_RESERVE_03                        0x2B */ (0 & 3),
  /* TMC4671_ABN_2_DECODER_MODE                0x2C */ (3 & 3),
  /* TMC4671_ABN_2_DECODER_PPR                 0x2D */ (3 & 3),
  /* TMC4671_ABN_2_DECODER_COUNT               0x2E */ (3 & 3),
  /* TMC4671_ABN_2_DECODER_COUNT_N             0x2F */ (3 & 3),
  /* TMC4671_ABN_2_DECODER_PHI_M_OFFSET        0x30 */ (3 & 3),
  /* TMC4671_ABN_2_DECODER_PHI_M               0x31 */ (1 & 3),
  /* TMC4671_HALL_MODE                         0x33 */ (3 & 3),
  /* TMC4671_RESERVE_04                        0x32 */ (0 & 3),
  /* TMC4671_HALL_POSITION_060_000             0x34 */ (3 & 3),
  /* TMC4671_HALL_POSITION_180_120             0x35 */ (3 & 3),
  /* TMC4671_HALL_POSITION_300_240             0x36 */ (3 & 3),
  /* TMC4671_HALL_PHI_E_PHI_M_OFFSET           0x37 */ (3 & 3),
  /* TMC4671_HALL_DPHI_MAX                     0x38 */ (3 & 3),
  /* TMC4671_HALL_PHI_E_INTERPOLATED_PHI_E     0x39 */ (1 & 3),
  /* TMC4671_HALL_PHI_M                        0x3A */ (1 & 3),
  /* TMC4671_AENC_DECODER_MODE                 0x3B */ (3 & 3),
  /* TMC4671_AENC_DECODER_N_MASK_N_THRESHOLD   0x3C */ (3 & 3),
  /* TMC4671_AENC_DECODER_PHI_A_RAW            0x3D */ (1 & 3),
  /* TMC4671_AENC_DECODER_PHI_A_OFFSET         0x3E */ (3 & 3),
  /* TMC4671_AENC_DECODER_PHI_A                0x3F */ (1 & 3),
  /* TMC4671_AENC_DECODER_PPR                  0x40 */ (3 & 3),
  /* TMC4671_AENC_DECODER_COUNT                0x41 */ (1 & 3),
  /* TMC4671_AENC_DECODER_COUNT_N              0x42 */ (3 & 3),
  /* TMC4671_RESERVE_05                        0x43 */ (0 & 3),
  /* TMC4671_RESERVE_06                        0x44 */ (0 & 3),
  /* TMC4671_AENC_DECODER_PHI_E_PHI_M_OFFSET   0x45 */ (3 & 3),
  /* TMC4671_AENC_DECODER_PHI_E_PHI_M          0x46 */ (1 & 3),
  /* TMC4671_AENC_DECODER_POSITION             0x47 */ (1 & 3),
  /* TMC4671_RESERVE_07                        0x48 */ (0 & 3),
  /* TMC4671_RESERVE_08                        0x49 */ (0 & 3),
  /* TMC4671_RESERVE_09                        0x4A */ (0 & 3),
  /* TMC4671_RESERVE_10                        0x4B */ (0 & 3),
  /* TMC4671_RESERVE_11                        0x4C */ (0 & 3),
  /* TMC4671_CONFIG_DATA                       0x4D */ (3 & 3),
  /* TMC4671_CONFIG_ADDR                       0x4E */ (3 & 3),
  /* TMC4671_RESERVE_12                        0x4F */ (0 & 3),
  /* TMC4671_VELOCITY_SELECTION                0x50 */ (3 & 3),
  /* TMC4671_POSITION_SELECTION                0x51 */ (3 & 3),
  /* TMC4671_PHI_E_SELECTION                   0x52 */ (3 & 3),
  /* TMC4671_PHI_E                             0x53 */ (1 & 3),
  /* TMC4671_PID_FLUX_P_FLUX_I                 0x54 */ (3 & 3),
  /* TMC4671_RESERVE_13                        0x55 */ (0 & 3),
  /* TMC4671_PID_TORQUE_P_TORQUE_I             0x56 */ (3 & 3),
  /* TMC4671_RESERVE_14                        0x57 */ (0 & 3),
  /* TMC4671_PID_VELOCITY_P_VELOCITY_I         0x58 */ (3 & 3),
  /* TMC4671_RESERVE_15                        0x59 */ (0 & 3),
  /* TMC4671_PID_POSITION_P_POSITION_I         0x5A */ (3 & 3),
  /* TMC4671_RESERVE_16                        0x5B */ (0 & 3),
  /* TMC4671_PID_TORQUE_FLUX_TARGET_DDT_LIMITS 0x5C */ (3 & 3),
  /* TMC4671_PIDOUT_UQ_UD_LIMITS               0x5D */ (3 & 3),
  /* TMC4671_PID_TORQUE_FLUX_LIMITS            0x5E */ (3 & 3),
  /* TMC4671_PID_ACCELERATION_LIMIT            0x5F */ (3 & 3),
  /* TMC4671_PID_VELOCITY_LIMIT                0x60 */ (3 & 3),
  /* TMC4671_PID_POSITION_LIMIT_LOW            0x61 */ (3 & 3),
  /* TMC4671_PID_POSITION_LIMIT_HIGH           0x62 */ (3 & 3),
  /* TMC4671_MODE_RAMP_MODE_MOTION             0x63 */ (3 & 3),
  /* TMC4671_PID_TORQUE_FLUX_TARGET            0x64 */ (3 & 3),
  /* TMC4671_PID_TORQUE_FLUX_OFFSET            0x65 */ (3 & 3),
  /* TMC4671_PID_VELOCITY_TARGET               0x66 */ (3 & 3),
  /* TMC4671_PID_VELOCITY_OFFSET               0x67 */ (3 & 3),
  /* TMC4671_PID_POSITION_TARGET               0x68 */ (3 & 3),
  /* TMC4671_PID_TORQUE_FLUX_ACTUAL            0x69 */ (1 & 3),
  /* TMC4671_PID_VELOCITY_ACTUAL               0x6A */ (1 & 3),
  /* TMC4671_PID_POSITION_ACTUAL               0x6B */ (3 & 3),
  /* TMC4671_PID_ERROR_DATA                    0x6C */ (1 & 3),
  /* TMC4671_PID_ERROR_ADDR                    0x6D */ (3 & 3),
  /* TMC4671_INTERIM_DATA                      0x6E */ (3 & 3),
  /* TMC4671_INTERIM_ADDR                      0x6F */ (3 & 3),
  /* TMC4671_RESERVE_17                        0x70 */ (0 & 3),
  /* TMC4671_RESERVE_18                        0x71 */ (0 & 3),
  /* TMC4671_RESERVE_19                        0x72 */ (0 & 3),
  /* TMC4671_RESERVE_20                        0x73 */ (0 & 3),
  /* TMC4671_WATCHDOG_CFG                      0x74 */ (3 & 3),
  /* TMC4671_ADC_VM_LIMITS                     0x75 */ (3 & 3),
  /* TMC4671_INPUTS_RAW                        0x76 */ (1 & 3),
  /* TMC4671_OUTPUTS_RAW                       0x77 */ (1 & 3),
  /* TMC4671_STEP_WIDTH                        0x78 */ (3 & 3),
  /* TMC4671_UART_BPS                          0x79 */ (3 & 3),
  /* TMC4671_UART_ADDRS                        0x7A */ (3 & 3),
  /* TMC4671_GPIO_dsADCI_CONFIG                0x7B */ (3 & 3),
  /* TMC4671_STATUS_FLAGS                      0x7C */ (3 & 3),
  /* TMC4671_STATUS_MASK                       0x7D */ (3 & 3),
  /* TMC4671_RESERVE_21                        0x7E */ (0 & 3),
  /* TMC4671_RESERVE_22                        0x7F */ (0 & 3),
};

/**************************************************************************************************************
 Declaration : app_idle_task
 Parameters  : argument, Arbitrary user data set on osThreadNew
 Return Value: None
 Description : application idle thread
**************************************************************************************************************/
__NO_RETURN void app_idle_task(void *argument)
{
  uint8_t  ReadIndex = 0;
  uint32_t TickCount = 0;

  /* Get the currently running thread */
  app_idle_tid = osThreadGetId();
  ASSERT(app_idle_tid != NULL);
	
  VERIFY(osDelay(100) == osOK);
	
  /* Initialize the SPI devices */
  VERIFY(MOT_Init(&hMOT1) == HAL_OK);
  VERIFY(MOT_Init(&hMOT2) == HAL_OK);
  VERIFY(MOT_Init(&hMOT3) == HAL_OK);
  VERIFY(MOT_Init(&hMOT4) == HAL_OK);
  VERIFY(MOT_Init(&hMOT5) == HAL_OK);
  VERIFY(MOT_Init(&hMOT6) == HAL_OK);

  /* Wait a whlie */
  VERIFY(osDelay(100) == osOK);
	
//#if 0U
  /* Wait all devices to be ready */
  while (true) { /* Infinite loop */
    VERIFY(MOT_Write(&hMOT1, TMC4671_CHIPINFO_ADDR, 0U) == HAL_OK);
    VERIFY(MOT_Write(&hMOT2, TMC4671_CHIPINFO_ADDR, 0U) == HAL_OK);
    VERIFY(MOT_Write(&hMOT3, TMC4671_CHIPINFO_ADDR, 0U) == HAL_OK);
    VERIFY(MOT_Write(&hMOT4, TMC4671_CHIPINFO_ADDR, 0U) == HAL_OK);
    VERIFY(MOT_Write(&hMOT5, TMC4671_CHIPINFO_ADDR, 0U) == HAL_OK);
    VERIFY(MOT_Write(&hMOT6, TMC4671_CHIPINFO_ADDR, 0U) == HAL_OK);
		
    VERIFY(MOT_Read(&hMOT1, TMC4671_CHIPINFO_DATA) == HAL_OK);
    VERIFY(MOT_Read(&hMOT2, TMC4671_CHIPINFO_DATA) == HAL_OK);
    VERIFY(MOT_Read(&hMOT3, TMC4671_CHIPINFO_DATA) == HAL_OK);
    VERIFY(MOT_Read(&hMOT4, TMC4671_CHIPINFO_DATA) == HAL_OK);
    VERIFY(MOT_Read(&hMOT5, TMC4671_CHIPINFO_DATA) == HAL_OK);
    VERIFY(MOT_Read(&hMOT6, TMC4671_CHIPINFO_DATA) == HAL_OK);

    if (hMOT1.Instance->Registers[TMC4671_CHIPINFO_DATA] == 0x34363731 && /* 0x34363731 */
        hMOT2.Instance->Registers[TMC4671_CHIPINFO_DATA] == 0x34363731 && /* 0x34363731 */
        hMOT3.Instance->Registers[TMC4671_CHIPINFO_DATA] == 0x34363731 && /* 0x34363731 */
        hMOT4.Instance->Registers[TMC4671_CHIPINFO_DATA] == 0x34363731 && /* 0x34363731 */
        hMOT5.Instance->Registers[TMC4671_CHIPINFO_DATA] == 0x34363731 && /* 0x34363731 */
        hMOT6.Instance->Registers[TMC4671_CHIPINFO_DATA] == 0x34363731) { /* 0x34363731 */
      break;
    } /* 0x34363731 */
				
				//Test:只開第一軸
//		if (hMOT1.Instance->Registers[TMC4671_CHIPINFO_DATA] == 0x34363731) { /* 0x34363731 */
//      break;
//    } /* 0x34363731 */
		
		
  } /* Infinite loop */
//#endif
	
  /* Wait a whlie */
//  VERIFY(osDelay(100) == osOK);
	
	  /* Enable the TMC4671 ENI pin */
//  VERIFY(TMC_Enable(&hMOT1) == HAL_OK);
//  VERIFY(TMC_Enable(&hMOT2) == HAL_OK);
//  VERIFY(TMC_Enable(&hMOT3) == HAL_OK);
//  VERIFY(TMC_Enable(&hMOT4) == HAL_OK);
//  VERIFY(TMC_Enable(&hMOT5) == HAL_OK);
//  VERIFY(TMC_Enable(&hMOT6) == HAL_OK);
	
	  /* Wait a whlie */
  VERIFY(osDelay(100) == osOK);

  /* Initialize the TMC motions */
  VERIFY(TMC_Init(&hMOT1) == HAL_OK);
  VERIFY(TMC_Init(&hMOT2) == HAL_OK);
  VERIFY(TMC_Init(&hMOT3) == HAL_OK);
  VERIFY(TMC_Init(&hMOT4) == HAL_OK);
  VERIFY(TMC_Init(&hMOT5) == HAL_OK);
  VERIFY(TMC_Init(&hMOT6) == HAL_OK);
	
	
  /* Access all register values */
  for (uint16_t adr = 0; adr < countof(AccessPermission); adr++) { /* Permission */
    if (AccessPermission[adr] & 1) { /* Readable */
      VERIFY(MOT_Read(&hMOT1, adr) == HAL_OK);
      VERIFY(MOT_Read(&hMOT2, adr) == HAL_OK);
      VERIFY(MOT_Read(&hMOT3, adr) == HAL_OK);
      VERIFY(MOT_Read(&hMOT4, adr) == HAL_OK);
      VERIFY(MOT_Read(&hMOT5, adr) == HAL_OK);
      VERIFY(MOT_Read(&hMOT6, adr) == HAL_OK);
    } /* Readable */
  } /* Permission */

  /* Wait a whlie */
  VERIFY(osDelay(100) == osOK);

  /* Enable the TMC4671 ENI pin */
//  VERIFY(TMC_Enable(&hMOT1) == HAL_OK);
//  VERIFY(TMC_Enable(&hMOT2) == HAL_OK);
//  VERIFY(TMC_Enable(&hMOT3) == HAL_OK);
//  VERIFY(TMC_Enable(&hMOT4) == HAL_OK);
//  VERIFY(TMC_Enable(&hMOT5) == HAL_OK);
//  VERIFY(TMC_Enable(&hMOT6) == HAL_OK);

  /* Wait a whlie */
  VERIFY(osDelay(500) == osOK);

  /* All TMC4671 initialization has been completed */
  VERIFY(osThreadFlagsSet(app_test_tid, 0x7FFFFFFF) != osFlagsError);

  /* LightUp LED to notify user */
  LED_On(LED2);


//while(true)
//{
//	VERIFY(osDelay(500) == osOK);
//}


  /* Start thread infinite loop */
  while (true) {
    /* Notify main thread activity flag */
    VERIFY(osThreadFlagsSet(app_main_tid, (uint32_t) argument) != osFlagsError);

    /* Write the TMC4671 register value​​​​ */
    for (uint16_t adr = 0; adr < countof(AccessPermission); adr++) { /* Permission */
      if ((AccessPermission[adr] & 2) == 0) { /* Unwritable */
        continue;
      } /* Unwritable */
      if (hMOT1.Instance->Registers[adr | 0x80] != hMOT1.Instance->Registers[adr | 0x0100]) { /* hMOT1 */
        VERIFY(MOT_Write(&hMOT1, adr, hMOT1.Instance->Registers[adr | 0x0080]) == HAL_OK);
        hMOT1.Instance->Registers[adr | 0x0000] = hMOT1.Instance->Registers[adr | 0x0080];
        hMOT1.Instance->Registers[adr | 0x0100] = hMOT1.Instance->Registers[adr | 0x0080];
      } /* hMOT1 */
      if (hMOT2.Instance->Registers[adr | 0x80] != hMOT2.Instance->Registers[adr | 0x0100]) { /* hMOT2 */
        VERIFY(MOT_Write(&hMOT2, adr, hMOT2.Instance->Registers[adr | 0x0080]) == HAL_OK);
        hMOT2.Instance->Registers[adr | 0x0000] = hMOT2.Instance->Registers[adr | 0x0080];
        hMOT2.Instance->Registers[adr | 0x0100] = hMOT2.Instance->Registers[adr | 0x0080];
      } /* hMOT2 */
      if (hMOT3.Instance->Registers[adr | 0x80] != hMOT3.Instance->Registers[adr | 0x0100]) { /* hMOT3 */
        VERIFY(MOT_Write(&hMOT3, adr, hMOT3.Instance->Registers[adr | 0x0080]) == HAL_OK);
        hMOT3.Instance->Registers[adr | 0x0000] = hMOT3.Instance->Registers[adr | 0x0080];
        hMOT3.Instance->Registers[adr | 0x0100] = hMOT3.Instance->Registers[adr | 0x0080];
      } /* hMOT3 */
      if (hMOT4.Instance->Registers[adr | 0x80] != hMOT4.Instance->Registers[adr | 0x0100]) { /* hMOT4 */
        VERIFY(MOT_Write(&hMOT4, adr, hMOT4.Instance->Registers[adr | 0x0080]) == HAL_OK);
        hMOT4.Instance->Registers[adr | 0x0000] = hMOT4.Instance->Registers[adr | 0x0080];
        hMOT4.Instance->Registers[adr | 0x0100] = hMOT4.Instance->Registers[adr | 0x0080];
      } /* hMOT4 */
      if (hMOT5.Instance->Registers[adr | 0x80] != hMOT5.Instance->Registers[adr | 0x0100]) { /* hMOT5 */
        VERIFY(MOT_Write(&hMOT5, adr, hMOT5.Instance->Registers[adr | 0x0080]) == HAL_OK);
        hMOT5.Instance->Registers[adr | 0x0000] = hMOT5.Instance->Registers[adr | 0x0080];
        hMOT5.Instance->Registers[adr | 0x0100] = hMOT5.Instance->Registers[adr | 0x0080];
      } /* hMOT5 */
      if (hMOT6.Instance->Registers[adr | 0x80] != hMOT6.Instance->Registers[adr | 0x0100]) { /* hMOT6 */
        VERIFY(MOT_Write(&hMOT6, adr, hMOT6.Instance->Registers[adr | 0x0080]) == HAL_OK);
        hMOT6.Instance->Registers[adr | 0x0000] = hMOT6.Instance->Registers[adr | 0x0080];
        hMOT6.Instance->Registers[adr | 0x0100] = hMOT6.Instance->Registers[adr | 0x0080];
      } /* hMOT6 */

      while (((hMOT6.hSPIx->State) != HAL_SPI_STATE_READY)) {__NOP(); };
      while (((hMOT5.hSPIx->State) != HAL_SPI_STATE_READY)) {__NOP(); };
      while (((hMOT4.hSPIx->State) != HAL_SPI_STATE_READY)) {__NOP(); };
      while (((hMOT3.hSPIx->State) != HAL_SPI_STATE_READY)) {__NOP(); };
      while (((hMOT2.hSPIx->State) != HAL_SPI_STATE_READY)) {__NOP(); };
      while (((hMOT1.hSPIx->State) != HAL_SPI_STATE_READY)) {__NOP(); };
    } /* Permission */

    /* Read the TMC4671 register value */
    if (osKernelGetTickCount() < TickCount + 0) {
      continue;
    }
    else {
      TickCount = osKernelGetTickCount();
    }

    ReadIndex += 0x01;
    ReadIndex &= 0x7F;

    if (AccessPermission[ReadIndex] & 1) { /* Readable */
      VERIFY(MOT_Read(&hMOT1, ReadIndex) == HAL_OK);
      VERIFY(MOT_Read(&hMOT2, ReadIndex) == HAL_OK);
      VERIFY(MOT_Read(&hMOT3, ReadIndex) == HAL_OK);
      VERIFY(MOT_Read(&hMOT4, ReadIndex) == HAL_OK);
      VERIFY(MOT_Read(&hMOT5, ReadIndex) == HAL_OK);
      VERIFY(MOT_Read(&hMOT6, ReadIndex) == HAL_OK);
    } /* Readable */

    while (((hMOT6.hSPIx->State) != HAL_SPI_STATE_READY)) {__NOP(); };
    while (((hMOT5.hSPIx->State) != HAL_SPI_STATE_READY)) {__NOP(); };
    while (((hMOT4.hSPIx->State) != HAL_SPI_STATE_READY)) {__NOP(); };
    while (((hMOT3.hSPIx->State) != HAL_SPI_STATE_READY)) {__NOP(); };
    while (((hMOT2.hSPIx->State) != HAL_SPI_STATE_READY)) {__NOP(); };
    while (((hMOT1.hSPIx->State) != HAL_SPI_STATE_READY)) {__NOP(); };

    /* Pass control to the next thread */
    VERIFY(osThreadYield() == osOK);
  }

  /* Never happen */
  ErrorHandler(__FILE__, __LINE__);
}
