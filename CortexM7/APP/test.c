/**************************************************************************************************************
 File   :
 Author :
 Date   :
 Brief  : Execute the zero point and the algorithm
**************************************************************************************************************/

/* Includes **************************************************************************************************/
#include "TEST.h"
#include "FIFO.h"
#include <stdio.h>

/* Functions *************************************************************************************************/
extern void GMT_N_Traj(void);
extern void initStewartParameter(void);
extern void GMT_Test(void);
extern void GMT_Test2(void);
extern void initAxis12Parameter(void);
/**************************************************************************************************************
 Declaration : SetValueForAllMotors
 Parameters  : Address - Register address
 Parameters  : Value   - The value written
 Parameters  : Delay   - Extra delay time
 Return Value: None
 Description : Set all motor register values ​​at the same time
**************************************************************************************************************/
static void WriteValuesToAllMotors(uint32_t Address, int32_t Value, uint32_t Delay)
{
  /* Block invalid addresses */
  Address &= 0x7F;

  /* Write Value to all motors */
  hMOT1.Instance->Registers[Address | 0x0080] =  Value;
  hMOT1.Instance->Registers[Address | 0x0100] = ~Value;

  hMOT2.Instance->Registers[Address | 0x0080] =  Value;
  hMOT2.Instance->Registers[Address | 0x0100] = ~Value;

  hMOT3.Instance->Registers[Address | 0x0080] =  Value;
  hMOT3.Instance->Registers[Address | 0x0100] = ~Value;

  hMOT4.Instance->Registers[Address | 0x0080] =  Value;
  hMOT4.Instance->Registers[Address | 0x0100] = ~Value;

  hMOT5.Instance->Registers[Address | 0x0080] =  Value;
  hMOT5.Instance->Registers[Address | 0x0100] = ~Value;

  hMOT6.Instance->Registers[Address | 0x0080] =  Value;
  hMOT6.Instance->Registers[Address | 0x0100] = ~Value;

  /* Wait for all operations to complete */
  while (hMOT1.Instance->Registers[Address | 0x0080] == ~hMOT1.Instance->Registers[Address | 0x0100] ||
         hMOT2.Instance->Registers[Address | 0x0080] == ~hMOT2.Instance->Registers[Address | 0x0100] ||
         hMOT3.Instance->Registers[Address | 0x0080] == ~hMOT3.Instance->Registers[Address | 0x0100] ||
         hMOT4.Instance->Registers[Address | 0x0080] == ~hMOT4.Instance->Registers[Address | 0x0100] ||
         hMOT5.Instance->Registers[Address | 0x0080] == ~hMOT5.Instance->Registers[Address | 0x0100] ||
         hMOT6.Instance->Registers[Address | 0x0080] == ~hMOT6.Instance->Registers[Address | 0x0100]) {
    VERIFY(osDelay(1) == osOK); /* Wait a while */
  }

  /* Wait for additional waiting time */
  if (Delay) { VERIFY(osDelay(Delay) == osOK); }
}

/**************************************************************************************************************
 Declaration : app_main_task
 Parameters  : argument, Arbitrary user data set on osThreadNew
 Return Value: None
 Description : application test thread
**************************************************************************************************************/
__NO_RETURN void app_test_task(void *argument)
{
  /* Get the currently running thread */
  app_test_tid = osThreadGetId();
  ASSERT(app_test_tid != NULL);
	
  /* Wait for all devices to be ready */
  VERIFY(osThreadFlagsWait(0x7FFFFFFFU, osFlagsWaitAll, osWaitForever) != osFlagsError);
	
	
	WriteValuesToAllMotors(TMC4671_MODE_RAMP_MODE_MOTION, 0x00000002, 10); /* VELOCITY_MOTION */
	
	hMOT1.Instance->W.PID_VELOCITY_TARGET = 0x000003E8;
	hMOT2.Instance->W.PID_VELOCITY_TARGET = 0x000003E8;
	hMOT3.Instance->W.PID_VELOCITY_TARGET = 0x000003E8;
	hMOT4.Instance->W.PID_VELOCITY_TARGET = 0x000000C8;
	hMOT5.Instance->W.PID_VELOCITY_TARGET = 0x000000C8;
	hMOT6.Instance->W.PID_VELOCITY_TARGET = 0x000000C8;
  VERIFY(osDelay(1000) == osOK);

	WriteValuesToAllMotors(TMC4671_CONFIG_ADDR, 0x00000033, 10); /* REF_SWITCH_CONFIG */
	WriteValuesToAllMotors(TMC4671_INTERIM_ADDR, 0x0000001E, 10); /* REF_SWITCH_STATUS */
  WriteValuesToAllMotors(TMC4671_CONFIG_DATA, 0x00000001, 10); /* REF_SWITCH_ENABLE */
	WriteValuesToAllMotors(TMC4671_CONFIG_DATA, 0x00000000, 10); /* REF_SWITCH_ENABLE */
	WriteValuesToAllMotors(TMC4671_CONFIG_DATA, 0x00000001, 10); /* REF_SWITCH_ENABLE */
	
	hMOT1.Instance->W.PID_VELOCITY_TARGET = 0xFFFFFC18;
	hMOT2.Instance->W.PID_VELOCITY_TARGET = 0xFFFFFC18;
	hMOT3.Instance->W.PID_VELOCITY_TARGET = 0xFFFFFC18;
	hMOT4.Instance->W.PID_VELOCITY_TARGET = 0xFFFFFF38;
	hMOT5.Instance->W.PID_VELOCITY_TARGET = 0xFFFFFF38;
	hMOT6.Instance->W.PID_VELOCITY_TARGET = 0xFFFFFF38;

	VERIFY(osDelay(100) == osOK);

	while( !((hMOT1.Instance->R.INTERIM_DATA) & 0X00000004) || /* LEFT_SWITCH_PASSED ? */
         !((hMOT2.Instance->R.INTERIM_DATA) & 0X00000004) || /* LEFT_SWITCH_PASSED ? */
         !((hMOT3.Instance->R.INTERIM_DATA) & 0X00000004) || /* LEFT_SWITCH_PASSED ? */
         !((hMOT4.Instance->R.INTERIM_DATA) & 0X00000004) || /* LEFT_SWITCH_PASSED ? */
         !((hMOT5.Instance->R.INTERIM_DATA) & 0X00000004) || /* LEFT_SWITCH_PASSED ? */
         !((hMOT6.Instance->R.INTERIM_DATA) & 0X00000004))
	{
		if(((hMOT1.Instance->R.INTERIM_DATA) & 0X00000004))
		{
	    hMOT1.Instance->W.PID_VELOCITY_TARGET = 0x00000000;
			VERIFY(osDelay(1) == osOK);
		}
		if(((hMOT2.Instance->R.INTERIM_DATA) & 0X00000004))
		{
	    hMOT2.Instance->W.PID_VELOCITY_TARGET = 0x00000000;
			VERIFY(osDelay(1) == osOK);
		}
		if(((hMOT3.Instance->R.INTERIM_DATA) & 0X00000004))
		{
	    hMOT3.Instance->W.PID_VELOCITY_TARGET = 0x00000000;
			VERIFY(osDelay(1) == osOK);
		}
		if(((hMOT4.Instance->R.INTERIM_DATA) & 0X00000004))
		{
	    hMOT4.Instance->W.PID_VELOCITY_TARGET = 0x00000000;
			VERIFY(osDelay(1) == osOK);
		}
		if(((hMOT5.Instance->R.INTERIM_DATA) & 0X00000004))
		{
	    hMOT5.Instance->W.PID_VELOCITY_TARGET = 0x00000000;
			VERIFY(osDelay(1) == osOK);
		}
		if(((hMOT6.Instance->R.INTERIM_DATA) & 0X00000004))
		{
	    hMOT6.Instance->W.PID_VELOCITY_TARGET = 0x00000000;
			VERIFY(osDelay(1) == osOK);
		}
		VERIFY(osDelay(1) == osOK);
	}
	
	VERIFY(osDelay(100) == osOK);
	WriteValuesToAllMotors(TMC4671_PID_POSITION_ACTUAL, 0x00000000, 10); 

	
	VERIFY(osDelay(100) == osOK);
	
	
	WriteValuesToAllMotors(TMC4671_CONFIG_DATA, 0x00000001, 10); /* REF_SWITCH_ENABLE */
	WriteValuesToAllMotors(TMC4671_CONFIG_DATA, 0x00000000, 10); /* REF_SWITCH_ENABLE */
	WriteValuesToAllMotors(TMC4671_CONFIG_DATA, 0x00000001, 10); /* REF_SWITCH_ENABLE */

	hMOT1.Instance->W.PID_VELOCITY_TARGET = 0x000003E8;
	hMOT2.Instance->W.PID_VELOCITY_TARGET = 0x000003E8;
	hMOT3.Instance->W.PID_VELOCITY_TARGET = 0x000003E8;
	hMOT4.Instance->W.PID_VELOCITY_TARGET = 0x000000C8;
	hMOT5.Instance->W.PID_VELOCITY_TARGET = 0x000000C8;
	hMOT6.Instance->W.PID_VELOCITY_TARGET = 0x000000C8;
	
	VERIFY(osDelay(100) == osOK);
	
	while( !((hMOT1.Instance->R.INTERIM_DATA) & 0X00000008) || /* LEFT_SWITCH_PASSED ? */
         !((hMOT2.Instance->R.INTERIM_DATA) & 0X00000008) || /* LEFT_SWITCH_PASSED ? */
         !((hMOT3.Instance->R.INTERIM_DATA) & 0X00000008) || /* LEFT_SWITCH_PASSED ? */
         !((hMOT4.Instance->R.INTERIM_DATA) & 0X00000008) || /* LEFT_SWITCH_PASSED ? */
         !((hMOT5.Instance->R.INTERIM_DATA) & 0X00000008) || /* LEFT_SWITCH_PASSED ? */
         !((hMOT6.Instance->R.INTERIM_DATA) & 0X00000008))
	{
		if(((hMOT1.Instance->R.INTERIM_DATA) & 0X00000008))
		{
	    hMOT1.Instance->W.PID_VELOCITY_TARGET = 0x00000000;
			VERIFY(osDelay(1) == osOK);
		}
		if(((hMOT2.Instance->R.INTERIM_DATA) & 0X00000008))
		{
	    hMOT2.Instance->W.PID_VELOCITY_TARGET = 0x00000000;
			VERIFY(osDelay(1) == osOK);
		}
		if(((hMOT3.Instance->R.INTERIM_DATA) & 0X00000008))
		{
	    hMOT3.Instance->W.PID_VELOCITY_TARGET = 0x00000000;
			VERIFY(osDelay(1) == osOK);
		}
		if(((hMOT4.Instance->R.INTERIM_DATA) & 0X00000008))
		{
	    hMOT4.Instance->W.PID_VELOCITY_TARGET = 0x00000000;
			VERIFY(osDelay(1) == osOK);
		}
		if(((hMOT5.Instance->R.INTERIM_DATA) & 0X00000008))
		{
	    hMOT5.Instance->W.PID_VELOCITY_TARGET = 0x00000000;
			VERIFY(osDelay(1) == osOK);

		}
		if(((hMOT6.Instance->R.INTERIM_DATA) & 0X00000008))
		{
	    hMOT6.Instance->W.PID_VELOCITY_TARGET = 0x00000000;
			VERIFY(osDelay(1) == osOK);
		}
		VERIFY(osDelay(1) == osOK);
	}
	
	VERIFY(osDelay(100) == osOK);
	hMOT1.Instance->R.RESERVE_01 = hMOT1.Instance->R.PID_POSITION_ACTUAL; // 把最頂點，記錄在 R.RESERVE_01
  hMOT2.Instance->R.RESERVE_01 = hMOT2.Instance->R.PID_POSITION_ACTUAL; // 把最頂點，記錄在 R.RESERVE_01
  hMOT3.Instance->R.RESERVE_01 = hMOT3.Instance->R.PID_POSITION_ACTUAL; // 把最頂點，記錄在 R.RESERVE_01
  hMOT4.Instance->R.RESERVE_01 = hMOT4.Instance->R.PID_POSITION_ACTUAL; // 把最頂點，記錄在 R.RESERVE_01
  hMOT5.Instance->R.RESERVE_01 = hMOT5.Instance->R.PID_POSITION_ACTUAL; // 把最頂點，記錄在 R.RESERVE_01
  hMOT6.Instance->R.RESERVE_01 = hMOT6.Instance->R.PID_POSITION_ACTUAL; // 把最頂點，記錄在 R.RESERVE_01
	
	VERIFY(osDelay(100) == osOK);
	WriteValuesToAllMotors(TMC4671_PID_POSITION_LIMIT_LOW, 0x00000000, 10); 
	
	VERIFY(osDelay(100) == osOK);
	hMOT1.Instance->W.PID_POSITION_LIMIT_HIGH = hMOT1.Instance->R.PID_POSITION_ACTUAL;
	hMOT2.Instance->W.PID_POSITION_LIMIT_HIGH = hMOT2.Instance->R.PID_POSITION_ACTUAL;
	hMOT3.Instance->W.PID_POSITION_LIMIT_HIGH = hMOT3.Instance->R.PID_POSITION_ACTUAL;
	hMOT4.Instance->W.PID_POSITION_LIMIT_HIGH = hMOT4.Instance->R.PID_POSITION_ACTUAL;
	hMOT5.Instance->W.PID_POSITION_LIMIT_HIGH = hMOT5.Instance->R.PID_POSITION_ACTUAL;
	hMOT6.Instance->W.PID_POSITION_LIMIT_HIGH = hMOT6.Instance->R.PID_POSITION_ACTUAL;
//	VERIFY(osDelay(100) == osOK);
//	hMOT1.Instance->W.PID_POSITION_LIMIT_LOW = 0x00000000;
//	hMOT2.Instance->W.PID_POSITION_LIMIT_LOW = 0x00000000;
//	hMOT3.Instance->W.PID_POSITION_LIMIT_LOW = 0x00000000;
//	hMOT4.Instance->W.PID_POSITION_LIMIT_LOW = 0x00000000;
//	hMOT5.Instance->W.PID_POSITION_LIMIT_LOW = 0x00000000;
//	hMOT6.Instance->W.PID_POSITION_LIMIT_LOW = 0x00000000;
	
	
	
	
	VERIFY(osDelay(100) == osOK);

  WriteValuesToAllMotors(TMC4671_MODE_RAMP_MODE_MOTION, 0x00000003, 100);

	hMOT1.Instance->W.PID_POSITION_TARGET = (hMOT1.Instance->R.RESERVE_01 )/2;
	hMOT2.Instance->W.PID_POSITION_TARGET = (hMOT2.Instance->R.RESERVE_01 )/2;
	hMOT3.Instance->W.PID_POSITION_TARGET = (hMOT3.Instance->R.RESERVE_01 )/2;
	hMOT4.Instance->W.PID_POSITION_TARGET = (hMOT4.Instance->R.RESERVE_01 )/2;
	hMOT5.Instance->W.PID_POSITION_TARGET = (hMOT5.Instance->R.RESERVE_01 )/2;
	hMOT6.Instance->W.PID_POSITION_TARGET = (hMOT6.Instance->R.RESERVE_01 )/2;
	
	VERIFY(osDelay(500) == osOK);
	
//	hMOT1.Instance->W.PID_POSITION_TARGET = hMOT1.Instance->R.PID_POSITION_ACTUAL; 
//	hMOT2.Instance->W.PID_POSITION_TARGET = hMOT2.Instance->R.PID_POSITION_ACTUAL; 
//	hMOT3.Instance->W.PID_POSITION_TARGET = hMOT3.Instance->R.PID_POSITION_ACTUAL; 
//	hMOT4.Instance->W.PID_POSITION_TARGET = hMOT4.Instance->R.PID_POSITION_ACTUAL; 
//	hMOT5.Instance->W.PID_POSITION_TARGET = hMOT5.Instance->R.PID_POSITION_ACTUAL; 
//	hMOT6.Instance->W.PID_POSITION_TARGET = hMOT6.Instance->R.PID_POSITION_ACTUAL; 
//	
//	VERIFY(osDelay(2000) == osOK);
	
	//hMOT1.Instance->W.PID_POSITION_TARGET = -1000000;
	
	GMT_Test();
	
	initAxis12Parameter();
	VERIFY(osDelay(1000) == osOK);

  while (true) {
    /* Notify main thread activity flag */
    VERIFY(osThreadFlagsSet(app_main_tid, (uint32_t) argument) != osFlagsError);

#if 0U
    while (hMOT1.Instance->W.PID_POSITION_TARGET > 0 &&
           hMOT2.Instance->W.PID_POSITION_TARGET > 0 &&
           hMOT3.Instance->W.PID_POSITION_TARGET > 0 &&
           hMOT4.Instance->W.PID_POSITION_TARGET > 0 &&
           hMOT5.Instance->W.PID_POSITION_TARGET > 0 &&
           hMOT6.Instance->W.PID_POSITION_TARGET > 0) {
      hMOT1.Instance->W.PID_POSITION_TARGET -= 10000;
      hMOT2.Instance->W.PID_POSITION_TARGET -= 10000;
      hMOT3.Instance->W.PID_POSITION_TARGET -= 10000;
      hMOT4.Instance->W.PID_POSITION_TARGET -= 10000;
      hMOT5.Instance->W.PID_POSITION_TARGET -= 10000;
      hMOT6.Instance->W.PID_POSITION_TARGET -= 10000;
      VERIFY(osDelay(1) == osOK);
    }
    while (hMOT1.Instance->W.PID_POSITION_TARGET < TMC1->R.RESERVE_01 &&
           hMOT2.Instance->W.PID_POSITION_TARGET < TMC2->R.RESERVE_01 &&
           hMOT3.Instance->W.PID_POSITION_TARGET < TMC3->R.RESERVE_01 &&
           hMOT4.Instance->W.PID_POSITION_TARGET < TMC4->R.RESERVE_01 &&
           hMOT5.Instance->W.PID_POSITION_TARGET < TMC5->R.RESERVE_01 &&
           hMOT6.Instance->W.PID_POSITION_TARGET < TMC6->R.RESERVE_01) {
      hMOT1.Instance->W.PID_POSITION_TARGET += 10000;
      hMOT2.Instance->W.PID_POSITION_TARGET += 10000;
      hMOT3.Instance->W.PID_POSITION_TARGET += 10000;
      hMOT4.Instance->W.PID_POSITION_TARGET += 10000;
      hMOT5.Instance->W.PID_POSITION_TARGET += 10000;
      hMOT6.Instance->W.PID_POSITION_TARGET += 10000;
      VERIFY(osDelay(1) == osOK);
    }

#endif		
		GMT_N_Traj();
		//GMT_Test2();
    VERIFY(osDelay(1) == osOK);

    /* Pass control to the next thread */
    VERIFY(osThreadYield() == osOK);
  }

  /* Never happen */
  ErrorHandler(__FILE__, __LINE__);
}
