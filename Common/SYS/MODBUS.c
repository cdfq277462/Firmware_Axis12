/**************************************************************************************************************
 File  :
 Author:
 Date  :
 Brief :
**************************************************************************************************************/

/* Includes **************************************************************************************************/
#include "MODBUS.h"

/* Variables *************************************************************************************************/
static volatile const int32_t * const R_Registers[128] = { /* Actual Values */
  /* 0 - 11 */
	&(TMC1->R.PID_POSITION_TARGET   ), /* PID_POSITION_TARGET     */
  &(TMC2->R.PID_POSITION_TARGET   ), /* PID_POSITION_TARGET     */
  &(TMC3->R.PID_POSITION_TARGET   ), /* PID_POSITION_TARGET     */
  &(TMC4->R.PID_POSITION_TARGET   ), /* PID_POSITION_TARGET     */
  &(TMC5->R.PID_POSITION_TARGET   ), /* PID_POSITION_TARGET     */
  &(TMC6->R.PID_POSITION_TARGET   ), /* PID_POSITION_TARGET     */
  /* 12 - 23 */
  &(TMC1->R.PID_POSITION_ACTUAL   ), /* PID_POSITION_ACTUAL     */
  &(TMC2->R.PID_POSITION_ACTUAL   ), /* PID_POSITION_ACTUAL     */
  &(TMC3->R.PID_POSITION_ACTUAL   ), /* PID_POSITION_ACTUAL     */
  &(TMC4->R.PID_POSITION_ACTUAL   ), /* PID_POSITION_ACTUAL     */
  &(TMC5->R.PID_POSITION_ACTUAL   ), /* PID_POSITION_ACTUAL     */
  &(TMC6->R.PID_POSITION_ACTUAL   ), /* PID_POSITION_ACTUAL     */
  /* 24 - 35 */
  &(TMC1->R.RESERVE_22            ), /* TRANSMIT_TIME_STAMP R22 */
  &(TMC2->R.RESERVE_22            ), /* TRANSMIT_TIME_STAMP R22 */
  &(TMC3->R.RESERVE_22            ), /* TRANSMIT_TIME_STAMP R22 */
  &(TMC4->R.RESERVE_22            ), /* TRANSMIT_TIME_STAMP R22 */
  &(TMC5->R.RESERVE_22            ), /* TRANSMIT_TIME_STAMP R22 */
  &(TMC6->R.RESERVE_22            ), /* TRANSMIT_TIME_STAMP R22 */
  /* 36 - 47 */
  &(TMC1->R.RESERVE_21            ), /* ANALOG_ACTUAL_VALUE R21 */
  &(TMC2->R.RESERVE_21            ), /* ANALOG_ACTUAL_VALUE R21 */
  &(TMC3->R.RESERVE_21            ), /* ANALOG_ACTUAL_VALUE R21 */
  &(TMC4->R.RESERVE_21            ), /* ANALOG_ACTUAL_VALUE R21 */
  &(TMC5->R.RESERVE_21            ), /* ANALOG_ACTUAL_VALUE R21 */
  &(TMC6->R.RESERVE_21            ), /* ANALOG_ACTUAL_VALUE R21 */
  /* 48 - 59 */
  &(TMC1->R.RESERVE_20            ), /* REF_ORIGIN_POSITION R17 */
  &(TMC2->R.RESERVE_20            ), /* REF_ORIGIN_POSITION R17 */
  &(TMC3->R.RESERVE_20            ), /* REF_ORIGIN_POSITION R17 */
  &(TMC4->R.RESERVE_20            ), /* REF_ORIGIN_POSITION R17 */
  &(TMC5->R.RESERVE_20            ), /* REF_ORIGIN_POSITION R17 */
  &(TMC6->R.RESERVE_20            ), /* REF_ORIGIN_POSITION R17 */
  /* 60 - 71 */
  &(TMC1->R.RESERVE_19            ), /* REF_OFFSET_POSITION R19 */
  &(TMC2->R.RESERVE_19            ), /* REF_OFFSET_POSITION R19 */
  &(TMC3->R.RESERVE_19            ), /* REF_OFFSET_POSITION R19 */
  &(TMC4->R.RESERVE_19            ), /* REF_OFFSET_POSITION R19 */
  &(TMC5->R.RESERVE_19            ), /* REF_OFFSET_POSITION R19 */
  &(TMC6->R.RESERVE_19            ), /* REF_OFFSET_POSITION R19 */
  /* 72 - 83 */
  &(TMC1->R.PID_TORQUE_FLUX_ACTUAL), /* PID_TORQUE_FLUX_ACTUAL  */
  &(TMC2->R.PID_TORQUE_FLUX_ACTUAL), /* PID_TORQUE_FLUX_ACTUAL  */
  &(TMC3->R.PID_TORQUE_FLUX_ACTUAL), /* PID_TORQUE_FLUX_ACTUAL  */
  &(TMC4->R.PID_TORQUE_FLUX_ACTUAL), /* PID_TORQUE_FLUX_ACTUAL  */
  &(TMC5->R.PID_TORQUE_FLUX_ACTUAL), /* PID_TORQUE_FLUX_ACTUAL  */
  &(TMC6->R.PID_TORQUE_FLUX_ACTUAL), /* PID_TORQUE_FLUX_ACTUAL  */
	/* 84 - 95 */
	// POSITION LIMIT
	&(TMC1->R.RESERVE_01),
	&(TMC2->R.RESERVE_01),
	&(TMC3->R.RESERVE_01),
	&(TMC4->R.RESERVE_01),
	&(TMC5->R.RESERVE_01),
	&(TMC6->R.RESERVE_01),
	
		// ADC
  //&(TMC1->R.RESERVE_18), 
  //&(TMC2->R.RESERVE_18), 
  //&(TMC3->R.RESERVE_18), 
	/* 96 - 99 */
	// FLAG
	&(TMC4->R.RESERVE_03), /* 84 85 */
	&(TMC4->R.RESERVE_04),
  /* 100 - 111 */
	// CALCULATE POSE RESPONSE
	&(TMC1->R.RESERVE_06),
	&(TMC2->R.RESERVE_06),
	&(TMC3->R.RESERVE_06),
	&(TMC4->R.RESERVE_06),
	&(TMC5->R.RESERVE_06),
	&(TMC6->R.RESERVE_06),
	/* 112 - 123 */
	// CALCULATE LENGTH RESPONSE
	&(TMC1->R.RESERVE_07),
	&(TMC2->R.RESERVE_07),
	&(TMC3->R.RESERVE_07),
	&(TMC4->R.RESERVE_07),
	&(TMC5->R.RESERVE_07), 
	&(TMC6->R.RESERVE_07),
	/* 124 - 135 */
	// FROM FK POSE RESPONSE
	&(TMC1->R.RESERVE_08),
	&(TMC2->R.RESERVE_08),
	&(TMC3->R.RESERVE_08),
	&(TMC4->R.RESERVE_08),
	&(TMC5->R.RESERVE_08), 
	&(TMC6->R.RESERVE_08),
	/* 136 - 147 */
	// TOOL POSE WRITE
	&(TMC1->R.RESERVE_15),
	&(TMC2->R.RESERVE_15),
	&(TMC3->R.RESERVE_15),
	&(TMC4->R.RESERVE_15),
	&(TMC5->R.RESERVE_15),
	&(TMC6->R.RESERVE_15),
	/* 148 - 159 */
	// POSE WRITE
	// Tstart 
	&(TMC1->R.RESERVE_05), 
	&(TMC2->R.RESERVE_05), 
	&(TMC3->R.RESERVE_05),
	&(TMC4->R.RESERVE_05),
	&(TMC5->R.RESERVE_05), 
	&(TMC6->R.RESERVE_05), 
	
	// Reserve
	/* 160 - 171 */
	// PARAMETERS
	&(TMC1->R.RESERVE_16),	// velocity
	&(TMC2->R.RESERVE_16), 	// scan range
	&(TMC3->R.RESERVE_16), 	// step range
	&(TMC4->R.RESERVE_16), 	// line space
	&(TMC5->R.RESERVE_16), 	// timeScale_scanMode_flag 0000 0000 0000 0000
	&(TMC6->R.RESERVE_16), 	// thd	
	
	&(TMC1->R.RESERVE_17),	// simulate_light_source_config
	&(TMC2->R.RESERVE_17),	// simulate_light_source_config
	&(TMC3->R.RESERVE_17),	// resetFlag
	&(TMC4->R.RESERVE_17),	// sinFrequency


};

/* Variables *************************************************************************************************/
static volatile int32_t * const W_Registers[128][2] = { /* Target Values */
  { &(TMC1->W.PID_POSITION_TARGET), &(TMC1->M.PID_POSITION_TARGET) }, /* PID_POSITION_TARGET     */
  { &(TMC2->W.PID_POSITION_TARGET), &(TMC2->M.PID_POSITION_TARGET) }, /* PID_POSITION_TARGET     */
  { &(TMC3->W.PID_POSITION_TARGET), &(TMC3->M.PID_POSITION_TARGET) }, /* PID_POSITION_TARGET     */
  { &(TMC4->W.PID_POSITION_TARGET), &(TMC4->M.PID_POSITION_TARGET) }, /* PID_POSITION_TARGET     */
  { &(TMC5->W.PID_POSITION_TARGET), &(TMC5->M.PID_POSITION_TARGET) }, /* PID_POSITION_TARGET     */
  { &(TMC6->W.PID_POSITION_TARGET), &(TMC6->M.PID_POSITION_TARGET) }, /* PID_POSITION_TARGET     */

  { &(TMC1->W.PID_POSITION_ACTUAL), &(TMC1->M.PID_POSITION_ACTUAL) }, /* PID_POSITION_ACTUAL     */
  { &(TMC2->W.PID_POSITION_ACTUAL), &(TMC2->M.PID_POSITION_ACTUAL) }, /* PID_POSITION_ACTUAL     */
  { &(TMC3->W.PID_POSITION_ACTUAL), &(TMC3->M.PID_POSITION_ACTUAL) }, /* PID_POSITION_ACTUAL     */
  { &(TMC4->W.PID_POSITION_ACTUAL), &(TMC4->M.PID_POSITION_ACTUAL) }, /* PID_POSITION_ACTUAL     */
  { &(TMC5->W.PID_POSITION_ACTUAL), &(TMC5->M.PID_POSITION_ACTUAL) }, /* PID_POSITION_ACTUAL     */
  { &(TMC6->W.PID_POSITION_ACTUAL), &(TMC6->M.PID_POSITION_ACTUAL) }, /* PID_POSITION_ACTUAL     */

  { NULL                          , NULL                           }, /* TRANSMIT_TIME_STAMP R22 */
  { NULL                          , NULL                           }, /* TRANSMIT_TIME_STAMP R22 */
  { NULL                          , NULL                           }, /* TRANSMIT_TIME_STAMP R22 */
  { NULL                          , NULL                           }, /* TRANSMIT_TIME_STAMP R22 */
  { NULL                          , NULL                           }, /* TRANSMIT_TIME_STAMP R22 */
  { NULL                          , NULL                           }, /* TRANSMIT_TIME_STAMP R22 */

  { NULL                          , NULL                           }, /* ANALOG_ACTUAL_VALUE R21 */
  { NULL                          , NULL                           }, /* ANALOG_ACTUAL_VALUE R21 */
  { NULL                          , NULL                           }, /* ANALOG_ACTUAL_VALUE R21 */
  { NULL                          , NULL                           }, /* ANALOG_ACTUAL_VALUE R21 */
  { NULL                          , NULL                           }, /* ANALOG_ACTUAL_VALUE R21 */
  { NULL                          , NULL                           }, /* ANALOG_ACTUAL_VALUE R21 */

  { NULL                          , NULL                           }, /* REF_ORIGIN_POSITION R20 */
  { NULL                          , NULL                           }, /* REF_ORIGIN_POSITION R20 */
  { NULL                          , NULL                           }, /* REF_ORIGIN_POSITION R20 */
  { NULL                          , NULL                           }, /* REF_ORIGIN_POSITION R20 */
  { NULL                          , NULL                           }, /* REF_ORIGIN_POSITION R20 */
  { NULL                          , NULL                           }, /* REF_ORIGIN_POSITION R20 */


  { &(TMC1->W.RESERVE_19         ), &(TMC1->M.RESERVE_19         ) }, /* REF_OFFSET_POSITION R19 */
  { &(TMC2->W.RESERVE_19         ), &(TMC2->M.RESERVE_19         ) }, /* REF_OFFSET_POSITION R19 */
  { &(TMC3->W.RESERVE_19         ), &(TMC3->M.RESERVE_19         ) }, /* REF_OFFSET_POSITION R19 */
  { &(TMC4->W.RESERVE_19         ), &(TMC4->M.RESERVE_19         ) }, /* REF_OFFSET_POSITION R19 */
  { &(TMC5->W.RESERVE_19         ), &(TMC5->M.RESERVE_19         ) }, /* REF_OFFSET_POSITION R19 */
  { &(TMC6->W.RESERVE_19         ), &(TMC6->M.RESERVE_19         ) }, /* REF_OFFSET_POSITION R19 */

  { NULL                          , NULL                           }, /* PID_TORQUE_FLUX_ACTUAL  */
  { NULL                          , NULL                           }, /* PID_TORQUE_FLUX_ACTUAL  */
  { NULL                          , NULL                           }, /* PID_TORQUE_FLUX_ACTUAL  */
  { NULL                          , NULL                           }, /* PID_TORQUE_FLUX_ACTUAL  */
  { NULL                          , NULL                           }, /* PID_TORQUE_FLUX_ACTUAL  */
  { NULL                          , NULL                           }, /* PID_TORQUE_FLUX_ACTUAL  */
	
	// POSITION LIMIT
	{ NULL                          , NULL                           }, /*  POSITION LIMIT  */
  { NULL                          , NULL                           }, /*  POSITION LIMIT  */
  { NULL                          , NULL                           }, /*  POSITION LIMIT  */
  { NULL                          , NULL                           }, /*  POSITION LIMIT  */
  { NULL                          , NULL                           }, /*  POSITION LIMIT  */
  { NULL                          , NULL                           }, /*  POSITION LIMIT  */
		// ADC
  //{ &(TMC1->W.RESERVE_18), &(TMC1->M.RESERVE_18) },
  //{ &(TMC2->W.RESERVE_18), &(TMC2->M.RESERVE_18) },
  //{ &(TMC3->W.RESERVE_18), &(TMC3->M.RESERVE_18) },
	
	// FLAG
	{ &(TMC4->W.RESERVE_03), &(TMC4->M.RESERVE_03) },
	{ &(TMC4->W.RESERVE_04), &(TMC4->M.RESERVE_04) },

	// CALCULATE POSE RESPONSE
	{ NULL                          , NULL                           }, /*    */
  { NULL                          , NULL                           }, /*    */
  { NULL                          , NULL                           }, /*    */
  { NULL                          , NULL                           }, /*    */
  { NULL                          , NULL                           }, /*    */
  { NULL                          , NULL                           }, /*    */
	
	// CALCULATE LENGTH RESPONSE
	{ NULL                          , NULL                           }, /*    */
  { NULL                          , NULL                           }, /*    */
  { NULL                          , NULL                           }, /*    */
  { NULL                          , NULL                           }, /*    */
  { NULL                          , NULL                           }, /*    */
  { NULL                          , NULL                           }, /*    */
	
		// FROM FK POSE RESPONSE
	{ NULL                          , NULL                           }, /*    */
  { NULL                          , NULL                           }, /*    */
  { NULL                          , NULL                           }, /*    */
  { NULL                          , NULL                           }, /*    */
  { NULL                          , NULL                           }, /*    */
  { NULL                          , NULL                           }, /*    */
	
	
	// TOOL POSE WRITE
	{ &(TMC1->W.RESERVE_15), &(TMC1->M.RESERVE_15) },
	{ &(TMC2->W.RESERVE_15), &(TMC2->M.RESERVE_15) },
	{ &(TMC3->W.RESERVE_15), &(TMC3->M.RESERVE_15) },
	{ &(TMC4->W.RESERVE_15), &(TMC4->M.RESERVE_15) },
	{ &(TMC5->W.RESERVE_15), &(TMC5->M.RESERVE_15) },
	{ &(TMC6->W.RESERVE_15), &(TMC6->M.RESERVE_15) },
		
	// POSE WRITE
	{ &(TMC1->W.RESERVE_05), &(TMC1->M.RESERVE_05) },
	{ &(TMC2->W.RESERVE_05), &(TMC2->M.RESERVE_05) },
	{ &(TMC3->W.RESERVE_05), &(TMC3->M.RESERVE_05) },
	{ &(TMC4->W.RESERVE_05), &(TMC4->M.RESERVE_05) },
	{ &(TMC5->W.RESERVE_05), &(TMC5->M.RESERVE_05) },
	{ &(TMC6->W.RESERVE_05), &(TMC6->M.RESERVE_05) },
	
	
	// PARAMETERS
	{ &(TMC1->W.RESERVE_16), &(TMC1->M.RESERVE_16) },
	{ &(TMC2->W.RESERVE_16), &(TMC2->M.RESERVE_16) },
	{ &(TMC3->W.RESERVE_16), &(TMC3->M.RESERVE_16) },
	{ &(TMC4->W.RESERVE_16), &(TMC4->M.RESERVE_16) },
	{ &(TMC5->W.RESERVE_16), &(TMC5->M.RESERVE_16) },
	{ &(TMC6->W.RESERVE_16), &(TMC6->M.RESERVE_16) },
	
	{ &(TMC1->W.RESERVE_17), &(TMC1->M.RESERVE_17) }, 	
	{ &(TMC2->W.RESERVE_17), &(TMC2->M.RESERVE_17) },
	{ &(TMC3->W.RESERVE_17), &(TMC3->M.RESERVE_17) },
	{ &(TMC4->W.RESERVE_17), &(TMC4->M.RESERVE_17) },
};

/**************************************************************************************************************
 Declaration :
 Parameters  :
 Return Value:
 Description :
**************************************************************************************************************/
__weak bool MODBUS_GetStatus(uint8_t num, uint16_t adr)
{
  return false;
}

/**************************************************************************************************************
 Declaration :
 Parameters  :
 Return Value:
 Description :
**************************************************************************************************************/
__weak void MODBUS_SetStatus(uint8_t num, uint16_t adr, bool val)
{
}

/**************************************************************************************************************
 Declaration :
 Parameters  :
 Return Value:
 Description :
**************************************************************************************************************/
__weak uint16_t MODBUS_GetRegister(uint8_t DeviceID, uint16_t MemAddr)
{
  if (MemAddr >= 0x0100) {
    return 0;
  }
  else if (DeviceID == 1) {
    return (MemAddr & 1) ? LOWORD(TMC1->Registers[(MemAddr >> 1) & 0x7F])
                         : HIWORD(TMC1->Registers[(MemAddr >> 1) & 0x7F]);
  }
  else if (DeviceID == 2) {
    return (MemAddr & 1) ? LOWORD(TMC2->Registers[(MemAddr >> 1) & 0x7F])
                         : HIWORD(TMC2->Registers[(MemAddr >> 1) & 0x7F]);
  }
  else if (DeviceID == 3) {
    return (MemAddr & 1) ? LOWORD(TMC3->Registers[(MemAddr >> 1) & 0x7F])
                         : HIWORD(TMC3->Registers[(MemAddr >> 1) & 0x7F]);
  }
  else if (DeviceID == 4) {
    return (MemAddr & 1) ? LOWORD(TMC4->Registers[(MemAddr >> 1) & 0x7F])
                         : HIWORD(TMC4->Registers[(MemAddr >> 1) & 0x7F]);
  }
  else if (DeviceID == 5) {
    return (MemAddr & 1) ? LOWORD(TMC5->Registers[(MemAddr >> 1) & 0x7F])
                         : HIWORD(TMC5->Registers[(MemAddr >> 1) & 0x7F]);
  }
  else if (DeviceID == 6) {
    return (MemAddr & 1) ? LOWORD(TMC6->Registers[(MemAddr >> 1) & 0x7F])
                         : HIWORD(TMC6->Registers[(MemAddr >> 1) & 0x7F]);
  }
  else if (DeviceID == 255 && R_Registers[MemAddr >> 1]) {
    return (MemAddr & 1) ? LOWORD(*R_Registers[MemAddr >> 1])
                         : HIWORD(*R_Registers[MemAddr >> 1]);
  }

  return 0;
}

/**************************************************************************************************************
 Declaration :
 Parameters  :
 Return Value:
 Description :
**************************************************************************************************************/
__weak void MODBUS_SetRegister(uint8_t DeviceID, uint16_t MemAddr, uint16_t Value)
{
  if (DeviceID == 255 && MemAddr == 255 && Value == 255) {
    __set_FAULTMASK(1);
    NVIC_SystemReset();
  }

  if (MemAddr >= 0x0100) {
    return;
  }
  else if (DeviceID == 1) {
    static int32_t lValue = 0;

    if (MemAddr & 1) {
      lValue &= (uint32_t) 0xFFFF0000U;
      lValue |= (uint32_t) Value << 0U;

      TMC1->Registers[(MemAddr >> 1) | 0x0080] =  lValue;
      TMC1->Registers[(MemAddr >> 1) | 0x0100] = ~lValue;
    }
    else {
      lValue &= (uint32_t) 0x0000FFFFU;
      lValue |= (uint32_t) Value << 16;
    }
  }
  else if (DeviceID == 2) {
    static int32_t lValue = 0;

    if (MemAddr & 1) {
      lValue &= (uint32_t) 0xFFFF0000U;
      lValue |= (uint32_t) Value << 0U;

      TMC2->Registers[(MemAddr >> 1) | 0x0080] =  lValue;
      TMC2->Registers[(MemAddr >> 1) | 0x0100] = ~lValue;
    }
    else {
      lValue &= (uint32_t) 0x0000FFFFU;
      lValue |= (uint32_t) Value << 16;
    }
  }
  else if (DeviceID == 3) {
    static int32_t lValue = 0;

    if (MemAddr & 1) {
      lValue &= (uint32_t) 0xFFFF0000U;
      lValue |= (uint32_t) Value << 0U;

      TMC3->Registers[(MemAddr >> 1) | 0x0080] =  lValue;
      TMC3->Registers[(MemAddr >> 1) | 0x0100] = ~lValue;
    }
    else {
      lValue &= (uint32_t) 0x0000FFFFU;
      lValue |= (uint32_t) Value << 16;
    }
  }
  else if (DeviceID == 4) {
    static int32_t lValue = 0;

    if (MemAddr & 1) {
      lValue &= (uint32_t) 0xFFFF0000U;
      lValue |= (uint32_t) Value << 0U;

      TMC4->Registers[(MemAddr >> 1) | 0x0080] =  lValue;
      TMC4->Registers[(MemAddr >> 1) | 0x0100] = ~lValue;
    }
    else {
      lValue &= (uint32_t) 0x0000FFFFU;
      lValue |= (uint32_t) Value << 16;
    }
  }
  else if (DeviceID == 5) {
    static int32_t lValue = 0;

    if (MemAddr & 1) {
      lValue &= (uint32_t) 0xFFFF0000U;
      lValue |= (uint32_t) Value << 0U;

      TMC5->Registers[(MemAddr >> 1) | 0x0080] =  lValue;
      TMC5->Registers[(MemAddr >> 1) | 0x0100] = ~lValue;
    }
    else {
      lValue &= (uint32_t) 0x0000FFFFU;
      lValue |= (uint32_t) Value << 16;
    }
  }
  else if (DeviceID == 6) {
    static int32_t lValue = 0;

    if (MemAddr & 1) {
      lValue &= (uint32_t) 0xFFFF0000U;
      lValue |= (uint32_t) Value << 0U;

      TMC6->Registers[(MemAddr >> 1) | 0x0080] =  lValue;
      TMC6->Registers[(MemAddr >> 1) | 0x0100] = ~lValue;
    }
    else {
      lValue &= (uint32_t) 0x0000FFFFU;
      lValue |= (uint32_t) Value << 16;
    }
  }
  else if (DeviceID == 255) {
    static int32_t lValue = 0;

    if (MemAddr & 1) {
      lValue &= (uint32_t) 0xFFFF0000U;
      lValue |= (uint32_t) Value << 0U;

      if (W_Registers[MemAddr >> 1][0]) { *W_Registers[MemAddr >> 1][0] =  lValue; }
      if (W_Registers[MemAddr >> 1][1]) { *W_Registers[MemAddr >> 1][1] = ~lValue; };
    }
    else {
      lValue &= (uint32_t) 0x0000FFFFU;
      lValue |= (uint32_t) Value << 16;
    }
  }
}

/**************************************************************************************************************
 Declaration :
 Parameters  :
 Return Value:
 Description : 01 Read Coil Status
**************************************************************************************************************/
static uint16_t MODBUS_ReadCoilStatus(uint8_t ID, uint8_t *Ques, uint8_t *Answ)
{
  uint16_t StartAD = __REVSH(*(uint16_t *) &Ques[0]);
  uint16_t PointNO = __REVSH(*(uint16_t *) &Ques[2]);

  if (PointNO > 2000) { // No. of Points
    return 0;
  } // No. of Points

  *((uint8_t *) Answ++) = ((PointNO - 1) >> 3) + 1; // Byte Count

  for (uint16_t Index = 0; Index < PointNO; Index++) { // No. of Points
    if (MODBUS_GetStatus(ID, StartAD + Index)) {
      Answ[Index >> 3] |=  (1 << (Index & 7));
    }
    else {
      Answ[Index >> 3] &= ~(1 << (Index & 7));
    }
  } // No. of Points

  return ((PointNO - 1) >> 3) + 2; // Data Length Of Answer
}

/**************************************************************************************************************
 Declaration :
 Parameters  :
 Return Value:
 Description : 02 Read Input Status
**************************************************************************************************************/
static uint16_t MODBUS_ReadInputStatus(uint8_t ID, uint8_t *Ques, uint8_t *Answ)
{
  uint16_t StartAD = __REVSH(*(uint16_t *) &Ques[0]);
  uint16_t PointNO = __REVSH(*(uint16_t *) &Ques[2]);

  if (PointNO > 2000) { // No. of Points
    return 0;
  } // No. of Points

  *((uint8_t *) Answ++) = ((PointNO - 1) >> 3) + 1; // Byte Count

  for (uint16_t Index = 0; Index < PointNO; Index++) { // No. of Points
    if (MODBUS_GetStatus(ID, StartAD + Index)) {
      Answ[Index >> 3] |=  (1 << (Index & 7));
    }
    else {
      Answ[Index >> 3] &= ~(1 << (Index & 7));
    }
  } // No. of Points

  return ((PointNO - 1) >> 3) + 2; // Data Length Of Answer
}

/**************************************************************************************************************
 Declaration :
 Parameters  :
 Return Value:
 Description : 03H Read Holding Registers
**************************************************************************************************************/
static uint16_t MODBUS_ReadHoldingRegisters(uint8_t ID, uint8_t *Ques, uint8_t *Answ)
{
  uint16_t StartAD = __REVSH(*(uint16_t *) &Ques[0]);
  uint16_t PointNO = __REVSH(*(uint16_t *) &Ques[2]);

  if (PointNO > 125) { // No. of Points
    return 0;
  } // No. of Points

  *((uint8_t *) Answ++) = PointNO * sizeof(uint16_t); // Byte Count

  for (uint16_t Index = 0; Index < PointNO; Index++) { // No. of Points
    uint16_t Register = (uint16_t) MODBUS_GetRegister(ID, StartAD + Index);

    *((uint8_t *) Answ++) = HIBYTE(Register);
    *((uint8_t *) Answ++) = LOBYTE(Register);
  } // No. of Points

  return PointNO + PointNO + 1; // Data Length Of Answer
}

/**************************************************************************************************************
 Declaration :
 Parameters  :
 Return Value:
 Description : 04H Read Input Registers
**************************************************************************************************************/
static uint16_t MODBUS_ReadInputRegisters(uint8_t ID, uint8_t *Ques, uint8_t *Answ)
{
  uint16_t StartAD = __REVSH(*(uint16_t *) &Ques[0]);
  uint16_t PointNO = __REVSH(*(uint16_t *) &Ques[2]);

  if (PointNO > 125) { // No. of Points
    return 0;
  } // No. of Points

  *((uint8_t *) Answ++) = PointNO * sizeof(uint16_t); // Byte Count

  for (uint16_t Index = 0; Index < PointNO; Index++) { // No. of Points
    uint16_t Value = (uint16_t) MODBUS_GetRegister(ID, StartAD + Index);

    *((uint8_t *) Answ++) = HIBYTE(Value);
    *((uint8_t *) Answ++) = LOBYTE(Value);
  } // No. of Points

  return PointNO + PointNO + 1; // Data Length Of Answer
}

/*************************************************************************************************************
 Declaration :
 Parameters  :
 Return Value:
 Description : 05 Force Single Coil
**************************************************************************************************************/
static void MODBUS_ForceSingleCoil(uint8_t ID, uint8_t *Ques)
{
  uint16_t StartAD = __REVSH(*(uint16_t *) &Ques[0]);
  uint16_t DataVAL = __REVSH(*(uint16_t *) &Ques[2]);

  if (DataVAL == 0xFF00) {
    MODBUS_SetStatus(ID, StartAD,  true);
  }
  else {
    MODBUS_SetStatus(ID, StartAD, false);
  }
}

/**************************************************************************************************************
 Declaration :
 Parameters  :
 Return Value:
 Description : 06H Preset Single Register
**************************************************************************************************************/
static void MODBUS_PresetSingleRegister(uint8_t DeviceID, uint8_t *Ques)
{
  uint16_t StartAD = __REVSH(*(uint16_t *) &Ques[0]);
  uint16_t DataVAL = __REVSH(*(uint16_t *) &Ques[2]);

  MODBUS_SetRegister(DeviceID, StartAD, DataVAL);
}

/**************************************************************************************************************
 Declaration :
 Parameters  :
 Return Value:
 Description :
**************************************************************************************************************/
static void MODBUS_ForceMultipleCoils(uint8_t ID, uint8_t *Ques)
{
  uint16_t StartAD = __REVSH(*(uint16_t *) &Ques[0]);
  uint16_t PointNO = __REVSH(*(uint16_t *) &Ques[2]);

  if (PointNO > 2000) {// No. of Points
    PointNO = 2000;
  } // No. of Points

  for (uint16_t n = 0; n < PointNO; n++) { // No. of Points
    if (Ques[5 + (n >> 3)] & (1 << (n & 7))) {
      MODBUS_SetStatus(ID, StartAD + n, true);
    }
    else {
      MODBUS_SetStatus(ID, StartAD + n, false);
    }
  } // No. of Points
}

/**************************************************************************************************************
 Declaration :
 Parameters  :
 Return Value:
 Description : 10H Preset Multiple Registers
**************************************************************************************************************/
static void MODBUS_PresetMultipleRegisters(uint8_t DeviceID, uint8_t *Ques)
{
  uint16_t  StartAD = __REVSH(*(uint16_t *) &Ques[0]);
  uint16_t  PointNO = __REVSH(*(uint16_t *) &Ques[2]);
  uint16_t *DataPtr = (uint16_t *) &Ques[5];

  if (PointNO > 125) { // No. of Points
    PointNO = 125;
  } // No. of Points

  while (PointNO--) { // No. of Points
    MODBUS_SetRegister(DeviceID, StartAD++, __REVSH(*DataPtr++));
  } // No. of Points
}

/**************************************************************************************************************
 Declaration :
 Parameters  :
 Return Value:
 Description :
**************************************************************************************************************/
uint16_t MODBUS_ReplyToQuery(uint8_t *buf, uint16_t len, uint16_t size)
{
  if (len < 6) { // Length Is Too Small
    return 0;
  } // Length Is Too Small
  else if (buf[0] != UCHAR_MAX &&
           buf[0] != 1 &&  buf[0] != 2 && buf[0] != 3 &&
           buf[0] != 4 &&  buf[0] != 5 && buf[0] != 6) { // Compare Slave Address
    return 0;
  } // Compare Slave Address
  else if (buf[1] == 0x01) { // 01H Read Coil Status
    len = MODBUS_ReadCoilStatus(buf[0], &buf[2], &buf[2]);
    return len + 2; // Length
  } // 01H Read Coil Status
  else if (buf[1] == 0x02) { // 02H Read Input Status
    len = MODBUS_ReadInputStatus(buf[0], &buf[2], &buf[2]);
    return len + 2; // Length
  } // 02H Read Input Status
  else if (buf[1] == 0x03)  { // 03H Read Holding Registers
    len = MODBUS_ReadHoldingRegisters(buf[0], &buf[2], &buf[2]);
    return len + 2; // Length
  } // 03H Read Holding Registers
  else if (buf[1] == 0x04) { // 04H Read Input Registers
    len = MODBUS_ReadInputRegisters(buf[0], &buf[2], &buf[2]);
    return len + 2; // Length
  } // 04 Read Input Registers
  else if (buf[1] == 0x05) { // 05H Force Single Coil
    MODBUS_ForceSingleCoil(buf[0], &buf[2]);
    return len;
  } // 05H Force Single Coil
  else if (buf[1] == 0x06) { // 06H Preset Single Register
    MODBUS_PresetSingleRegister(buf[0], &buf[2]);
    return len;
  } // 06 Preset Single Register
  else if (buf[1] == 0x0F) { // 0FH Force Multiple Coils
    MODBUS_ForceMultipleCoils(buf[0], &buf[2]);
    return 6; /* Fixed Length */
  } // 0F Force Multiple Coils
  else if (buf[1] == 0x10) { // 10H Preset Multiple Registers
    MODBUS_PresetMultipleRegisters(buf[0], &buf[2]);
    return 6; /* Fixed Length */
  } // 10H Preset Multiple Registers

  return 0;
}
