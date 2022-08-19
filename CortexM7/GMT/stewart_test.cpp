#include <iostream>
#include <ctime>
/***********************************************************
	// FLAG
	&(TMC4->R.RESERVE_3), 
	&(TMC4->R.RESERVE_4),
	
	// POSE RESPONSE
	&(TMC1->R.RESERVE_06), 
	&(TMC2->R.RESERVE_06),
	&(TMC3->R.RESERVE_06), 
	&(TMC4->R.RESERVE_06), 
	&(TMC5->R.RESERVE_06),
	&(TMC6->R.RESERVE_06),
	
	// PIVOT WRITE
	&(TMC1->R.RESERVE_15),
	&(TMC2->R.RESERVE_15),
	&(TMC3->R.RESERVE_15),
	&(TMC4->R.RESERVE_15),
	&(TMC5->R.RESERVE_15),
	&(TMC6->R.RESERVE_15),
	
	// POSE WRITE
	&(TMC1->R.RESERVE_5),
	&(TMC1->R.RESERVE_6),
	&(TMC2->R.RESERVE_5),
	&(TMC2->R.RESERVE_6),
	&(TMC3->R.RESERVE_5),
	&(TMC3->R.RESERVE_6),
	
	// MTool POSE WRITER
	&(TMC1->R.RESERVE_7),
	&(TMC1->R.RESERVE_8),
	&(TMC2->R.RESERVE_7),
	&(TMC2->R.RESERVE_8),
	&(TMC3->R.RESERVE_7),
	&(TMC3->R.RESERVE_8),
	
	&(TMC4->R.RESERVE_5),
	&(TMC4->R.RESERVE_6),
	&(TMC4->R.RESERVE_7),
	&(TMC4->R.RESERVE_8), 
	
	// PARAMETERS
	&(TMC1->R.RESERVE_16), 	// velocity
	&(TMC2->R.RESERVE_16),	// scan range
	&(TMC3->R.RESERVE_16), 	// step range
	&(TMC4->R.RESERVE_16),	// line space
	&(TMC5->R.RESERVE_16), 	// timeScale_scanMode_flag 0000 0000 0000 0000
	&(TMC6->R.RESERVE_16), 	//
*/

//#include "stewart_lib.h"
#include "FIFO.h"
#include "header\StewartFA.h"
#include "header\messageTable.h"
#include "BSP_STM32H7.h"
#include "arm_math.h"
#include "math.h"

/*
bool __force_stop = false;
bool __is_busy = false;
int __selected_traj_function = 0;
bool __error = false;
int __return_message;


double __pivot  [32];
double __Tstart [32] = {0,0,0,0,
												0,0,0,0,
												0,0,0,0,
												0,0,0,0};
double __alpha;
double __beta;			
double __scanRange = 0;	
double __lineSpacing = 0;		
double __velocity	 = 0;	
	
												
double __P  [32] = { 0, 0, 0, 0, 0, 0 };
// ID = 7
*/

#define RESPONSE_BUSY 						1
#define RESPONSE_NORMAL_FINISHED 	2
#define RESPONSE_ERROR_STOP 			3

#define MESSAGE_SCAN_FIND_MAX 		1
#define MESSAGE_SCAN_FIND_AVARAGE 2
#define MESSAGE_SCAN_FAIL 				3

#define TRAJ_FUNCTION_FA_SPIRAL 	1
#define TRAJ_FUNCTION_MOVELPP 		2
#define TRAJ_FUNCTION_FA_SIN 			3
#define TRAJ_FUNCTION_N_FA_TRACK 	4

//#define NStewart 	0U
//#define Axis12 		1U


static bool isInit = false;
// Flag
static int32_t __force_stop;
static int32_t __is_busy;
static int32_t __selected_traj_function;
static int32_t __error;
static int32_t __return_message;

static int32_t __message_code_;

static int32_t __reset_falgs;


// Variables
static double __pivot  [12];
static double __poseStart [12];
static double __poseCurrent [12];

static VectorXd __currentPose = VectorXd(6);

//static double __TMtool [16];

static double __alpha;
static double __beta;		

static double __velocity;
static double __velocityMoveL;

static double __scanRange;	
static double __stepRange;
static double __lineSpacing;
static double __frequencySin;

static double __velocity_second;
static double __scanRange_second;	
static double __stepRange_second;
static double __lineSpacing_second;

static int32_t __timeScale;
static int32_t __scanMode;
static int32_t __spiralOption;

static int32_t __timeScale_second;
static int32_t __scanMode_second;
static int __FA_step_buffer[2];
static int __stopOption;


static double __thd0;
static double __thd1;
static double __ts = 0.002;

static double __LSourceY;	//light_center r(0)
static double __LSourceZ;	//light_center r(1)
static double __LRadA;		//light_radius u(0)
static double __LRadB;		//light_radius u(1)
static int32_t __useSimulateLight;

static double __lightSourceMax;

// Return Variables
//static double Pose[6];
static int32_t ADCReciveValue[3];
//static TrajOnceOutput out;
//static NIKOutput nik_result;
static FATrajctoryResultP out;
static WSOutput outputws;

static StewartFA stewartFA;

//local use varialble
int i = 1;
int __iter = 1;		
int __stage = 1;		
int __fa_stage = 1;
bool _new_traj = true;


//#define NStewart
#define Axis12

uint16_t ElapsedTimeMS = 0;
#ifdef Axis12
void setAllMotorPosition(VectorXd outputPose)
{
	VectorXd positionLimit (6);

	positionLimit << double(TMC1->R.RESERVE_01),
									double(TMC2->R.RESERVE_01),
									double(TMC3->R.RESERVE_01),
									double(TMC4->R.RESERVE_01),
									double(TMC5->R.RESERVE_01),
									double(TMC6->R.RESERVE_01);

	//(PoseTarget - PoseMin)/PoseTotal * MotorPositionTotal = MotorPositionTarget
	TMC1->W.PID_POSITION_TARGET = (outputPose(0) - (-25.67)) /51.55 * positionLimit(0);//mm
	TMC2->W.PID_POSITION_TARGET = (outputPose(1) - (-30.11)) /50.42 * positionLimit(1);
	TMC3->W.PID_POSITION_TARGET = (outputPose(2) - (-6.33)) /10.3 * positionLimit(2);
	TMC4->W.PID_POSITION_TARGET = (outputPose(3) - MathTool::degree2Radian(-6.1)) /MathTool::degree2Radian(12.64) * positionLimit(3);//rad
	TMC5->W.PID_POSITION_TARGET = (outputPose(4) - MathTool::degree2Radian(-4.89)) /MathTool::degree2Radian(10.19) * positionLimit(4);
	TMC6->W.PID_POSITION_TARGET = (outputPose(5) - MathTool::degree2Radian(-7.66)) /MathTool::degree2Radian(14.84) * positionLimit(5);
	
//	TMC1->W.PID_POSITION_TARGET = (outputPose(0) - (-22)) /44 * positionLimit(0);//mm
//	TMC2->W.PID_POSITION_TARGET = (outputPose(1) - (-25)) /40 * positionLimit(1);
//	TMC3->W.PID_POSITION_TARGET = (outputPose(2) - (-5)) /7 * positionLimit(2);
//	TMC4->W.PID_POSITION_TARGET = (outputPose(3) - MathTool::degree2Radian(-5)) /MathTool::degree2Radian(10) * positionLimit(3);//rad
//	TMC5->W.PID_POSITION_TARGET = (outputPose(4) - MathTool::degree2Radian(-3)) /MathTool::degree2Radian(7) * positionLimit(4);
//	TMC6->W.PID_POSITION_TARGET = (outputPose(5) - MathTool::degree2Radian(-6)) /MathTool::degree2Radian(12) * positionLimit(5);
									
}

#else
void setAllMotorPosition(VectorXd outputPose)
{
	VectorXd positionLimit (6);

	positionLimit << double(TMC1->R.RESERVE_01),
									double(TMC2->R.RESERVE_01),
									double(TMC3->R.RESERVE_01),
									double(TMC4->R.RESERVE_01),
									double(TMC5->R.RESERVE_01),
									double(TMC6->R.RESERVE_01);

	//(PoseTarget - PoseMin)/PoseTotal * MotorPositionTotal = MotorPositionTarget
	
	TMC1->W.PID_POSITION_TARGET = (outputPose(0) -50.5002) /10 *positionLimit(0);
	TMC2->W.PID_POSITION_TARGET = (outputPose(1) -50.5002) /10 *positionLimit(1);
	TMC3->W.PID_POSITION_TARGET = (outputPose(2) -50.5002) /10 *positionLimit(2);
	TMC4->W.PID_POSITION_TARGET = (outputPose(3) -50.5002) /10 *positionLimit(3);
	TMC5->W.PID_POSITION_TARGET = (outputPose(4) -50.5002) /10 *positionLimit(4);
	TMC6->W.PID_POSITION_TARGET = (outputPose(5) -50.5002) /10 *positionLimit(5);
									
}
#endif
void move2Pose(VectorXd targetPose)
{
	IKOutput nik_result = stewartFA.N_IK( targetPose);
	
	VectorXd g;
	g = nik_result.g;

	TMC1->R.RESERVE_07 = g(0) * 10000000;
	TMC2->R.RESERVE_07 = g(1) * 10000000;
	TMC3->R.RESERVE_07 = g(2) * 10000000;
	TMC4->R.RESERVE_07 = g(3) * 10000000;
	TMC5->R.RESERVE_07 = g(4) * 10000000;
	TMC6->R.RESERVE_07 = g(5) * 10000000;
	
	setAllMotorPosition(g);
}




double lightSource()
{
	// simulate light
	if( __useSimulateLight == 1){
		TMC1->R.RESERVE_21 = out.lightDataCurr* 1000;
		return out.lightDataCurr * 1000;
	}
	else{
		if( __lightSourceMax != 0)
			return (double( TMC1->R.RESERVE_21) /double( __lightSourceMax));
		else
			return double( TMC1->R.RESERVE_21);
	}
}

void updateModbusDisplay(void)
{
	// Update Modbus
	// tool pose TMtool
	TMC1->R.RESERVE_15 = TMC1->W.RESERVE_15;
	TMC2->R.RESERVE_15 = TMC2->W.RESERVE_15;
	TMC3->R.RESERVE_15 = TMC3->W.RESERVE_15;
	TMC4->R.RESERVE_15 = TMC4->W.RESERVE_15;
	//TMC5->R.RESERVE_15 = __pivot[4] *10000000;
	//TMC6->R.RESERVE_15 = __pivot[5] *10000000;
	TMC5->R.RESERVE_15 = TMC5->W.RESERVE_15;
	TMC6->R.RESERVE_15 = TMC6->W.RESERVE_15;
	
	// Tstart
	TMC1->R.RESERVE_05 = TMC1->W.RESERVE_05;
	TMC2->R.RESERVE_05 = TMC2->W.RESERVE_05;
	TMC3->R.RESERVE_05 = TMC3->W.RESERVE_05;
	TMC4->R.RESERVE_05 = TMC4->W.RESERVE_05;
	TMC5->R.RESERVE_05 = TMC5->W.RESERVE_05;
	TMC6->R.RESERVE_05 = TMC6->W.RESERVE_05;
	
	// parameters
	TMC1->R.RESERVE_16 = TMC1->W.RESERVE_16;	// velocity
	TMC2->R.RESERVE_16 = TMC2->W.RESERVE_16;	// scan range
	TMC3->R.RESERVE_16 = TMC3->W.RESERVE_16;	// step range
	TMC4->R.RESERVE_16 = TMC4->W.RESERVE_16;	// line space
	
	TMC5->R.RESERVE_16 = TMC5->W.RESERVE_16;	// timeScale_scanMode_flag 0000 0000 0000 0000
	TMC6->R.RESERVE_16 = TMC6->W.RESERVE_16;	// thd0 thd1	
	
	TMC1->R.RESERVE_17 = TMC1->W.RESERVE_17;	// simulate_light_source_config
	TMC2->R.RESERVE_17 = TMC2->W.RESERVE_17;	// simulate_light_source_config
	TMC3->R.RESERVE_17 = TMC3->W.RESERVE_17;	// resetFlag
	TMC4->R.RESERVE_17 = TMC4->W.RESERVE_17;  // frequency
}

extern "C" void stop_runing(void)
{
		__is_busy = 0;
		__selected_traj_function = 0;
		TMC4->W.RESERVE_04 = 0;
	
		//TMC4->R.RESERVE_4 = TMC4->W.RESERVE_4;
		int32_t flag = 0;
		flag = (__force_stop) | (__is_busy << 1) | (__error << 2) | (__selected_traj_function << 8) | (__return_message << 12);
		TMC4->R.RESERVE_03 = flag;

		__iter = 1;	
		__stage = 1;
		__fa_stage = 1;
}


extern "C" void initAxis12Parameter(void)
{
	#if 0U
	
	Stewart_GMT::set_A12_DefaultParameter();
	// Flags
	__force_stop 							= 0;
	__is_busy 								= 0;
	__selected_traj_function 	= 0;
	__error 									= 0;
	__return_message 					= 0;
		
	// reset Tstart
	// reset target pose
	TMC1->W.RESERVE_05 = 0; 
	TMC2->W.RESERVE_05 = 0; 
	TMC3->W.RESERVE_05 = 0; 
	TMC4->W.RESERVE_05 = 0;
	TMC5->W.RESERVE_05 = 0;
	TMC6->W.RESERVE_05 = 0;


	
	// velocity
	TMC1->W.RESERVE_16 = (1000 << 16) | 1000;
	// scan range
	TMC2->W.RESERVE_16 = 10000000; 
	// step range
	TMC3->W.RESERVE_16 = 10000000; 
	// line space
	TMC4->W.RESERVE_16 = 2000000; 
	
	// reset Pose Response
	TMC1->R.RESERVE_06 = 0;
	TMC2->R.RESERVE_06 = 0;
	TMC3->R.RESERVE_06 = 0;
	TMC4->R.RESERVE_06 = 0;
	TMC5->R.RESERVE_06 = 0;
	TMC6->R.RESERVE_06 = 0;
	
	// reset Length Response
	TMC1->W.RESERVE_07 = 0; 
	TMC2->W.RESERVE_07 = 0; 
	TMC3->W.RESERVE_07 = 0; 
	TMC4->W.RESERVE_07 = 0;
	TMC5->W.RESERVE_07 = 0;
	TMC6->W.RESERVE_07 = 0;
	
	out.is_finished = true;
	
	VectorXd initPose(6);
	initPose << 0, 0, 0, 0, 0, 0;

	setAllMotorPosition(initPose);
	lightSource();
	/*
	// parameters
	__alpha 			= TMC5->W.RESERVE_15 /10000000;
	__beta 				= TMC6->W.RESERVE_15 /10000000;			
	
	__velocity 		= TMC1->W.RESERVE_17 /10000000;
	//__velocity 		= 1;
	
	__scanRange 	= TMC2->W.RESERVE_16 /10000000;	
	__lineSpacing = TMC4->W.RESERVE_16 /10000000;
	*/
	
	#endif
	stewartFA = StewartFA(TypeA12);
	// Flags
	__force_stop 							= 0;
	__is_busy 								= 0;
	__selected_traj_function 	= 0;
	__error 									= 0;
	__return_message 					= 0;
		
	// reset Tstart
	// reset target pose
	TMC1->W.RESERVE_05 = 0; 
	TMC2->W.RESERVE_05 = 0; 
	TMC3->W.RESERVE_05 = 0; 
	TMC4->W.RESERVE_05 = 0;
	TMC5->W.RESERVE_05 = 0;
	TMC6->W.RESERVE_05 = 0;


	
	// velocity
	TMC1->W.RESERVE_16 = 10000000;
	// scan range
	TMC2->W.RESERVE_16 = 10000000; 
	// step range
	TMC3->W.RESERVE_16 = 10000000; 
	// line space
	TMC4->W.RESERVE_16 = 2000000; 
	
	// reset Pose Response
	TMC1->R.RESERVE_06 = 0;
	TMC2->R.RESERVE_06 = 0;
	TMC3->R.RESERVE_06 = 0;
	TMC4->R.RESERVE_06 = 0;
	TMC5->R.RESERVE_06 = 0;
	TMC6->R.RESERVE_06 = 0;
	
	// reset Length Response
	TMC1->W.RESERVE_07 = 0; 
	TMC2->W.RESERVE_07 = 0; 
	TMC3->W.RESERVE_07 = 0; 
	TMC4->W.RESERVE_07 = 0;
	TMC5->W.RESERVE_07 = 0;
	TMC6->W.RESERVE_07 = 0;
	
	VectorXd initPose(6);
	initPose << 0, 0, 0, 0, 0, 0;

	setAllMotorPosition(initPose);
	lightSource();


}

extern "C" void initStewartParameter(void){
	stewartFA = StewartFA(TypeN);
	//Stewart_GMT::set_N_Type_Stewart_DefaultParameter();
	// Flags
	__force_stop 							= 0;
	__is_busy 								= 0;
	__selected_traj_function 	= 0;
	__error 									= 0;
	__return_message 					= 0;
		
	// reset Tstart
	// reset target pose
	TMC1->W.RESERVE_05 = 0; 
	TMC2->W.RESERVE_05 = 0; 
	TMC3->W.RESERVE_05 = 664236000; 
	TMC4->W.RESERVE_05 = 0;
	TMC5->W.RESERVE_05 = 0;
	TMC6->W.RESERVE_05 = 0;


	
	// velocity
	TMC1->W.RESERVE_16 = (1000 << 16) | 1000;
	// scan range
	TMC2->W.RESERVE_16 = 10000000; 
	// step range
	TMC3->W.RESERVE_16 = 10000000; 
	// line space
	TMC4->W.RESERVE_16 = 2000000; 
	
	// reset Pose Response
	TMC1->R.RESERVE_06 = 0;
	TMC2->R.RESERVE_06 = 0;
	TMC3->R.RESERVE_06 = 664236000;
	TMC4->R.RESERVE_06 = 0;
	TMC5->R.RESERVE_06 = 0;
	TMC6->R.RESERVE_06 = 0;
	
	// reset Length Response
	TMC1->W.RESERVE_07 = 0; 
	TMC2->W.RESERVE_07 = 0; 
	TMC3->W.RESERVE_07 = 0; 
	TMC4->W.RESERVE_07 = 0;
	TMC5->W.RESERVE_07 = 0;
	TMC6->W.RESERVE_07 = 0;
	
	out.messageCode = MESSAGE_OK;
	
	VectorXd initPose(6);
	initPose << 0, 0, 66.4236, 0, 0, 0;

	move2Pose(initPose);
	/*
	// parameters
	__alpha 			= TMC5->W.RESERVE_15 /10000000;
	__beta 				= TMC6->W.RESERVE_15 /10000000;			
	
	__velocity 		= TMC1->W.RESERVE_17 /10000000;
	//__velocity 		= 1;
	
	__scanRange 	= TMC2->W.RESERVE_16 /10000000;	
	__lineSpacing = TMC4->W.RESERVE_16 /10000000;
	*/

}

extern "C" void updateStewartFlags(void)
{
	static int counter = 0;
	
	__selected_traj_function = TMC4->W.RESERVE_04;
	//std::cout << "__selected_traj_function :" << __selected_traj_function << std::endl;
	TMC4->R.RESERVE_04 = TMC4->W.RESERVE_04;
//	static int32_t __return_message_class;
//	static int32_t __return_message_item;
	
	
	if(TMC4->W.RESERVE_03 == 0xFFFFFFFF)
	{
		__force_stop = 0;
		__is_busy = 0;
		__error = 0;
		__selected_traj_function = 0;
		__return_message = 0;
		__message_code_ = 0;
		TMC4->W.RESERVE_03 = 0;
	}
	
	int32_t flag = 0;
	flag = (__force_stop) | (__is_busy << 1) | (__error << 2) | (__selected_traj_function << 8) | (__return_message << 12);

	//std::cout << "flag :" << flag << std::endl;
	TMC4->R.RESERVE_03 = flag;
}
extern "C" void updateStewartParameter(void)
{	
	// Update Variables
	// TMtool
	__pivot[0] = ( double(TMC1->W.RESERVE_15) /10000000);
  __pivot[1] = ( double(TMC2->W.RESERVE_15) /10000000);
	__pivot[2] = ( double(TMC3->W.RESERVE_15) /10000000);
	__pivot[3] = ( double(TMC4->W.RESERVE_15) /10000000);
	__alpha 	 = ( double(TMC5->W.RESERVE_15) /10000000);
	__beta 		 = ( double(TMC6->W.RESERVE_15) /10000000);	
	
	// Tstart
	__poseStart [0] = ( double(TMC1->W.RESERVE_05) /10000000);
	__poseStart [1] = ( double(TMC2->W.RESERVE_05) /10000000);
	__poseStart [2] = ( double(TMC3->W.RESERVE_05) /10000000);
	__poseStart [3] = ( double(TMC4->W.RESERVE_05) /10000000);
	__poseStart [4] = ( double(TMC5->W.RESERVE_05) /10000000);
	__poseStart [5] = ( double(TMC6->W.RESERVE_05) /10000000);
	
	// current pose
	__poseCurrent [0] = ( double(TMC1->R.RESERVE_06) /10000000);
	__poseCurrent [1] = ( double(TMC2->R.RESERVE_06) /10000000);
	__poseCurrent [2] = ( double(TMC3->R.RESERVE_06) /10000000);
	__poseCurrent [3] = ( double(TMC4->R.RESERVE_06) /10000000);
	__poseCurrent [4] = ( double(TMC5->R.RESERVE_06) /10000000);
	__poseCurrent [5] = ( double(TMC6->R.RESERVE_06) /10000000);
	
	// parameters
	__velocity 			=  double(TMC1->W.RESERVE_16 & 0x0000FFFF) /1000;	
	__velocityMoveL =  double((TMC1->W.RESERVE_16 >> 16) & 0x0000FFFF) /1000;
	__scanRange 	= ( double(TMC2->W.RESERVE_16) /10000000);	
  __stepRange	  = ( double(TMC3->W.RESERVE_16) /10000000);	
	__lineSpacing = ( double(TMC4->W.RESERVE_16) /10000000);

	__velocity_second = __velocity;
	__scanRange_second = __scanRange /5;	
	__stepRange_second = __stepRange /5;
	__lineSpacing_second = __lineSpacing /15;
	
	int32_t timeScale_scanMode_flag_First = TMC5->W.RESERVE_16 & 0x0000FFFF;
	int32_t timeScale_scanMode_flag_Second = ( TMC5->W.RESERVE_16 >> 16) & 0x0000FFFF;

	//std::cout << timeScale_scanMode_flag << std::endl;
	__timeScale = timeScale_scanMode_flag_First & 0x000F;
	__scanMode  = ( timeScale_scanMode_flag_First >> 4) & 0x000F ;
	__FA_step_buffer[0] = ( timeScale_scanMode_flag_First >> 8) & 0x000F;
	__stopOption = ( timeScale_scanMode_flag_First >> 12) & 0x000F;
	
	__timeScale_second = timeScale_scanMode_flag_Second & 0x000F;
	__scanMode_second  = ( timeScale_scanMode_flag_Second >> 4) & 0x000F ;
	__FA_step_buffer[1] = ( timeScale_scanMode_flag_Second >> 8) & 0x000F;
	//__stopOption = ( timeScale_scanMode_flag_Second >> 12) & 0x000F;

	//__useSimulateLight = (timeScale_scanMode_flag >> 8) & 0x000F ;
	
	//std::cout << __FA_step_buffer[0] << "\t" << __FA_step_buffer[1] << std::endl;
	//__FA_step_buffer[2] = ( N_FA_StpeBuffer >> 4) & 0x0005;
	//__FA_step_buffer[3] = ( N_FA_StpeBuffer >> 6) & 0x0005;
	//__FA_step_buffer[4] = ( N_FA_StpeBuffer >> 8) & 0x0005;
	//__FA_step_buffer[5] = ( N_FA_StpeBuffer >> 10) & 0x0005;

	int32_t thd = TMC6->W.RESERVE_16;
	__thd0 = double(thd >> 16) /100;
	__thd1 = double(thd & 0x0000FFFF) /100;
	stewartFA.SetFAThreshold(__thd0, __thd1);
	
	int32_t lightSourceConfigCenter = TMC1->W.RESERVE_17;
	__LSourceY 	= double((lightSourceConfigCenter >> 16) & 0x0000FFFF);
	__LSourceZ 	= double(lightSourceConfigCenter & 0x0000FFFF);
	
	int32_t lightSourceConfigRadius = TMC2->W.RESERVE_17;
	__LRadA 		= double((lightSourceConfigRadius >> 16) & 0x0000FFFF) /100;
	__LRadB 		= double(lightSourceConfigRadius & 0x0000FFFF) /100;
	
	__frequencySin = double(TMC4->W.RESERVE_17)/100;
		
	//test
	Vector2d r;
	Vector2d u;
	r << __LSourceY, __LSourceZ;
	u << __LRadA, __LRadB;
	stewartFA.SetFALightSource(r, u);
	
	__lightSourceMax = double( TMC3->W.RESERVE_17);
	
	thd = (( int(__thd0 *100) << 16)) | int( __thd1 *100);

	//lightSourceConfigCenter = ( int(__LSourceY) << 16) | int( __LSourceZ);
	//lightSourceConfigRadius = ( int(__LRadA *100) << 16) | int( __LRadB *100);
	
	stewartFA.SetTrajctoryTs(__ts);
	stewartFA.SetTrajctoryTimeScaling(static_cast<enumTimeScaling>(__timeScale));
	stewartFA.SetFALightSource(&lightSource);
	
	Vector3d pivot;
	pivot << __pivot[0], __pivot[1], __pivot[2];
	stewartFA.SetToolCoordinate(pivot, __alpha, __alpha);
	
	VectorXd toolposeWorking_rad(6);
	toolposeWorking_rad << __poseStart[0],
												 __poseStart[1],
												 __poseStart[2],
												 __poseStart[3],
												 __poseStart[4],
												 __poseStart[5];
	outputws = stewartFA.ToolWS(
		toolposeWorking_rad,
		//base
		tool
	);
	
	
//	Matrix3d ry = MathTool::rotY(__alpha);
//	Matrix3d rx = MathTool::rotX(__beta);
//	Vector3d PMtool{__pivot[0], __pivot[1], __pivot[2]};
//	Matrix3d RMtool = ry * rx;
//	TMtool = MathTool::rp2t(RMtool, PMtool);
	
	// update parameters to modbus
	updateModbusDisplay();
	
	VERIFY(osDelay(10) == osOK);
}

void updateCurrentPose(void)
{
	__currentPose (0) = ( double(TMC1->R.RESERVE_06) /10000000);
	__currentPose (1) = ( double(TMC2->R.RESERVE_06) /10000000);
	__currentPose (2) = ( double(TMC3->R.RESERVE_06) /10000000);
	__currentPose (3) = ( double(TMC4->R.RESERVE_06) /10000000);
	__currentPose (4) = ( double(TMC5->R.RESERVE_06) /10000000);
	__currentPose (5) = ( double(TMC6->R.RESERVE_06) /10000000);
}
void sinTrack(void)
{
}

void spiralTrack(void)
{
}


extern "C" void GMT_N_Traj(void)
{		
	static uint32_t StartTick = 0;
	//std::cout << "debug 1 :" << std::endl;
	VectorXd poseStart(6);
	if(__iter == 1){
		// update stewart parameters
		updateStewartParameter();
		//StartTick = HAL_GetTick();
		poseStart (0) = __poseStart[0];
		poseStart (1) = __poseStart[1];
		poseStart (2) = __poseStart[2];
		poseStart (3) = __poseStart[3];
		poseStart (4) = __poseStart[4];
		poseStart (5) = __poseStart[5];
	}
	// update stewart flags
	updateStewartFlags();
	updateCurrentPose();

	
	Matrix4d Tstart;
	Tstart = MathTool::poseRPY2SE3(__currentPose);
	//std::cout << "debug 2 :" << std::endl;

	if( __force_stop == 1)
	{
		stop_runing();
		return;
	}

	/* Spiral Track */
	if( __selected_traj_function == TRAJ_FUNCTION_FA_SPIRAL)
	{

		//manual clean up Ldata record
		if(__iter == 1)
		{
		  stewartFA.ClearLdataRecord();
		}
		
		Vector3d pivot{__pivot[0], __pivot[1], __pivot[2]};
				
		enumSpiralTEvaluate spiralOption = static_cast<enumSpiralTEvaluate>(__stopOption);
		enumFAEndPointOption endOption = static_cast<enumFAEndPointOption>(__stopOption);
		
		out = stewartFA.FASpiral(
																outputws.Tstart,
																__scanRange,
																__lineSpacing,
																__velocity,
																__velocityMoveL,
																spiralOption,
																__iter,
																__stage,
																__currentPose,
																endOption);
		
		__stage = out.stageNext;		
		//std::cout << out.message << std::endl;
//		if ( out.messageCode == MESSAGE_OK )
//			__return_message = out.messageCode;
		__return_message = out.messageCode;		
	}
	
	/* Sin Track */
	if( __selected_traj_function == TRAJ_FUNCTION_FA_SIN)
	{
				//manual clean up Ldata record
		if(__iter == 1)
		{
		  stewartFA.ClearLdataRecord();
		}
		enumSinScanOption 		scanMode 	= static_cast<enumSinScanOption>(__scanMode);
		enumFAEndPointOption 	endOption = static_cast<enumFAEndPointOption>(__stopOption);
		
		out = stewartFA.FASin(
													Tstart,
													__scanRange,
													__stepRange,
													__lineSpacing,
													__velocity,
													__velocityMoveL,
													__frequencySin,
													scanMode,
													__iter,
													__stage,
													__currentPose,
													endOption
													);
		
//		out = Stewart_GMT::A12_FA_SinTrackOnece(
//			Tstart,
//			__velocity,
//			__ts,
//			timeScale,
//			__lineSpacing,
//			__stepRange,
//			__scanRange,
//			scanMode,
//			__stage,
//			__iter, r, u, __thd0, __thd1
//		);
		//std::cout << "lightSource :" << lightSource() << std::endl;

		__stage = out.stageNext;	
//		if ( out.messageCode == MESSAGE_OK )
//			__return_message = out.messageCode;
		__return_message = out.messageCode;	

	}
	
	/* MoveL */
	if( __selected_traj_function == TRAJ_FUNCTION_MOVELPP)
	{	
		static VectorXd poseEnd_moveL(6);
		static VectorXd poseStart_moveL(6);
		// test
		Vector2d r;
		Vector2d u;
		r << __LSourceY, __LSourceZ;
		u << __LRadA, __LRadB;

		if(__iter == 1){
			//updateStewartParameter();
			poseStart_moveL = __currentPose;
	
			poseEnd_moveL(0) = __poseStart[0];
			poseEnd_moveL(1) = __poseStart[1];
			poseEnd_moveL(2) = __poseStart[2];
			poseEnd_moveL(3) = __poseStart[3];
			poseEnd_moveL(4) = __poseStart[4];
			poseEnd_moveL(5) = __poseStart[5];
		
// @old near zero test			
//			if(	__poseCurrent[0] == __poseStart[0] && __poseCurrent[1] == __poseStart[1] &&
//					__poseCurrent[2] == __poseStart[2] && __poseCurrent[3] == __poseStart[3] &&
//					__poseCurrent[4] == __poseStart[4] && __poseCurrent[5] == __poseStart[5] ){
//				//out.is_finished = true;
//				__return_message = out.messageCode;
//				//std::cout << "__return_message1 :" << __return_message << std::endl;
//				//std::cout << "start same as end" << std::endl;
//				stop_runing();
//				return;
//			}
		}
		
		double tf = __velocityMoveL;
		
		out = stewartFA.FAMoveLPP(
				MathTool::poseRPY2SE3(poseStart_moveL),
				MathTool::poseRPY2SE3(poseEnd_moveL),
				tf,
				__iter,
				__currentPose
		);
		
		//out = Stewart_GMT::A12_moveL_PPOnce(poseStart_moveL, poseEnd, timeScale, __velocity, __ts, __iter, r, u);
		
		//std::cout << "poseStart_moveL :" << poseStart_moveL  << std::endl;
		//std::cout << "poseEnd :" << poseEnd << std::endl;
		//std::cout << "out.trajp 2:" << out.trajp << std::endl;
		//std::cout << "__iter :" << __iter << std::endl;

		//std::cout << "out.is_finished :" << out.is_finished << std::endl;

	}
	
#if 0U
	if( __selected_traj_function == TRAJ_FUNCTION_N_FA_TRACK)
	{
		enumTimeScaling timeScale = static_cast<enumTimeScaling>(__timeScale);
		enumSinScanMode scanMode = static_cast<enumSinScanMode>(__scanMode);
		enumFAScanMode  firstScan = static_cast<enumFAScanMode>( __FA_step_buffer[0] -1);
		enumFAScanMode  secondScan = static_cast<enumFAScanMode>( __FA_step_buffer[1] -1);
		//std::cout << "debug 3 :" << std::endl;

		Vector2d r;
		Vector2d u;
		r << __LSourceY, __LSourceZ;
		u << __LRadA, __LRadB;
		//std::cout << "debug 4 :" << std::endl;

		std::cout << "__iter :" << __iter << std::endl;
		std::cout << "__stage :" << __stage << std::endl;
		std::cout << "__fa_stage :" << __fa_stage << std::endl;
		std::cout << "__thd0 :" << __thd0 << std::endl;
		std::cout << "__thd1 :" << __thd1 << std::endl;

		out = Stewart_GMT::A12_FA_TrackOnce(TMtool, Tstart, __velocity, __ts, timeScale, 
																			__lineSpacing, __stepRange, __scanRange, firstScan, 
																			__lineSpacing_second, __stepRange_second, __scanRange_second, secondScan, 
																			scanMode, __thd0, __thd1, r, u, __iter, __stage, __fa_stage);
		std::cout << "__message_code_ :" << out.message_code << std::endl;
		std::cout << "Ldata :" << out.Ldata << std::endl;

		//std::cout << "Traj P :" << out.trajp << std::endl;
		//std::cout << " out.is_finished :" <<  out.is_finished << std::endl;

		//std::cout << "debug 5 :" << std::endl;

		
		__stage = out.next_stage;		
		__fa_stage = out.fa_next_stage;

		if ( out.is_finished )
			__return_message = RESPONSE_NORMAL_FINISHED;
		__message_code_ = out.message_code;		
		//std::cout << "debug 6 :" << std::endl;

	}
#endif
	/*	
	//MESSAGE_OK = 1000001
	if( out.message_code > 3)
		__message_code_ = __message_code_;
	else
		__message_code_ = out.message_code;
	*/
	
	if ( out.messageCode == MESSAGE_OK )
	{
		__return_message = out.messageCode;
		//std::cout << "__message_code_ :" << __message_code_ << std::endl;
		stop_runing();
		return;
	}
	
	//Error
	if ( out.messageCode != MESSAGE_OK && out.messageCode != MESSAGE_TRAJCTORY_NORMAL_END )
	{
		__return_message = out.messageCode;
		//std::cout << "Error Code:" << out.messageCode << std::endl;
		stop_runing();
		__error = 1;
		return;
	}
	//std::cout << "__return_message3 :" << __return_message << std::endl;
	//std::cout << "debug 7 :" << std::endl;
	//CalPose
	TMC1->R.RESERVE_06 = out.trajP(0) * 10000000;
	TMC2->R.RESERVE_06 = out.trajP(1) * 10000000;
	TMC3->R.RESERVE_06 = out.trajP(2) * 10000000;
	TMC4->R.RESERVE_06 = out.trajP(3) * 10000000;
	TMC5->R.RESERVE_06 = out.trajP(4) * 10000000;
	TMC6->R.RESERVE_06 = out.trajP(5) * 10000000;
	
//	// VectorXd outLength(6);
//	nik_result = Stewart_GMT::N_Stewart_IK(out.trajp);
//	auto dg = nik_result.ls;
//	VectorXd g;
//	double _h0 = 21.2;
//	MatrixXd tarr = nik_result.alpha.array().square() + SQUARE(_h0) - nik_result.ls.array().square();
//	g = nik_result.alpha.array() - tarr.array().sqrt();
//	//std::cout << "debug 8 :" << std::endl;
	VectorXd g;

#ifdef Axis12
	g = out.trajP;
#else
	IKOutput nik_result = stewartFA.N_IK(out.trajP);
	g = nik_result.g;
#endif
  //CalLength
	TMC1->R.RESERVE_07 = g(0) * 10000000;
	TMC2->R.RESERVE_07 = g(1) * 10000000;
	TMC3->R.RESERVE_07 = g(2) * 10000000;
	TMC4->R.RESERVE_07 = g(3) * 10000000;
	TMC5->R.RESERVE_07 = g(4) * 10000000;
	TMC6->R.RESERVE_07 = g(5) * 10000000;
	
	//std::cout << "debug 8.1 :" << std::endl;

	setAllMotorPosition(g);


	//__iter = out.next_iter;
	__iter = out.kNext;
	//std::cout << "debug 9 :" << std::endl;
	// std::cout << "__iter :" <<__iter << "\t__stage :" << __stage << std::endl;
}


extern "C" void GMT_Test(void)
{
//	Stewart_GMT::Set_LightSource(&lightSource);
//	lightSource();
//	VectorXd input (6);
//	input << 15,10,1,4,3,5;
//	setAllMotorPosition(input);
//	
//	VERIFY(osDelay(5000) == osOK);
	 
  VectorXd input2 (6);
	input2 << 0,0,0,0,0,0;
//	input2 << 0,0,0,0,0,MathTool::degree2Radian(-5);
	setAllMotorPosition(input2);
	VERIFY(osDelay(2000) == osOK);
}

extern "C" void GMT_Test2(void)
{
	updateStewartParameter();
	updateStewartFlags();
	if( __selected_traj_function == TRAJ_FUNCTION_MOVELPP)
	{
			double x = 1000000;
			TMC1->W.PID_POSITION_TARGET = TMC1->R.PID_POSITION_ACTUAL + x;
			TMC2->W.PID_POSITION_TARGET = TMC2->R.PID_POSITION_ACTUAL;
			TMC3->W.PID_POSITION_TARGET = TMC3->R.PID_POSITION_ACTUAL;
			TMC4->W.PID_POSITION_TARGET = TMC4->R.PID_POSITION_ACTUAL;
			TMC5->W.PID_POSITION_TARGET = TMC5->R.PID_POSITION_ACTUAL;
			TMC6->W.PID_POSITION_TARGET = TMC6->R.PID_POSITION_ACTUAL;
		stop_runing();
		
	}
}
