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

#include "stewart_lib.h"
#include "FIFO.h"

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

#define TRAJ_FUNCTION_FA_SPIRAL 1
#define TRAJ_FUNCTION_MOVELPP 	2
#define TRAJ_FUNCTION_FA_SIN 		3
										
static bool isInit = false;
// Flag
static int32_t __force_stop;
static int32_t __is_busy;
static int32_t __selected_traj_function;
static int32_t __error;
static int32_t __return_message;
static int32_t __message_code_;

// Variables
static double __pivot  [12];
static double __poseStart [12];
static double __poseCurrent [12];

static Matrix4d TMtool;

//static double __TMtool [16];

static double __alpha;
static double __beta;		

static double __velocity;
static double __scanRange;	
static double __stepRange;
static double __lineSpacing;

static int32_t __timeScale;
static int32_t __scanMode;
static int32_t __useSimulateLight;

static double __thd0;
static double __thd1;
static double __ts = 0.005;

static double __LSourceY = 0;//light_center r(0)
static double __LSourceZ = 67;//light_center r(1)
static double __LRadA = 0.5;//light_radius u(0)
static double __LRadB = 0.5;//light_radius u(1)

// Return Variables
//static double Pose[6];
static int32_t ADCReciveValue[3];
static TrajOnceOutput out;
static NIKOutput nik_result;
					
//local use varialble
int i = 1;
int __iter = 1;		
int __stage = 1;		
bool _new_traj = true;


void setAllMotorLength(VectorXd outputLength)
{
	VectorXd positionLimit (6);
	positionLimit << double(TMC1->R.RESERVE_01),
									double(TMC2->R.RESERVE_01),
									double(TMC3->R.RESERVE_01),
									double(TMC4->R.RESERVE_01),
									double(TMC5->R.RESERVE_01),
									double(TMC6->R.RESERVE_01);
									
	TMC1->W.PID_POSITION_TARGET = (outputLength(0) -50.5002) /10 *positionLimit(0);
	TMC2->W.PID_POSITION_TARGET = (outputLength(1) -50.5002) /10 *positionLimit(1);
	TMC3->W.PID_POSITION_TARGET = (outputLength(2) -50.5002) /10 *positionLimit(2);
	TMC4->W.PID_POSITION_TARGET = (outputLength(3) -50.5002) /10 *positionLimit(3);
	TMC5->W.PID_POSITION_TARGET = (outputLength(4) -50.5002) /10 *positionLimit(4);
	TMC6->W.PID_POSITION_TARGET = (outputLength(5) -50.5002) /10 *positionLimit(5);
}

void move2Pose(VectorXd targetPose)
{
	nik_result = Stewart_GMT::N_Stewart_IK(targetPose);
	auto dg = nik_result.ls;
	VectorXd g;
	double _h0 = 21.2;
	MatrixXd tarr = nik_result.alpha.array().square() + SQUARE(_h0) - nik_result.ls.array().square();
	g = nik_result.alpha.array() - tarr.array().sqrt();
	
	TMC1->R.RESERVE_07 = g(0) * 10000000;
	TMC2->R.RESERVE_07 = g(1) * 10000000;
	TMC3->R.RESERVE_07 = g(2) * 10000000;
	TMC4->R.RESERVE_07 = g(3) * 10000000;
	TMC5->R.RESERVE_07 = g(4) * 10000000;
	TMC6->R.RESERVE_07 = g(5) * 10000000;
	
	setAllMotorLength(g);
}
double lightSource()
{
	// simulate light
	if( __useSimulateLight == 1){
		TMC1->R.RESERVE_21 = out.Ldata * 1000;
		return out.Ldata * 1000;
	}
	else{
		return (TMC1->R.RESERVE_21);
	}
}
extern "C" void stop_runing(void)
{
		__is_busy = 0;
		__selected_traj_function = 0;
		TMC4->W.RESERVE_04 = 0;
	
		//TMC4->R.RESERVE_4 = TMC4->W.RESERVE_4;
		int32_t flag = 0;
		flag = (__force_stop) | (__is_busy << 1) | (__error << 2) | (__selected_traj_function << 8) | (__return_message << 12) | (__message_code_ << 16);
		TMC4->R.RESERVE_03 = flag;

		__iter = 1;	
		__stage = 1;
}

extern "C" void initStewartParameter(void)
{
	Stewart_GMT::set_N_Type_Stewart_DefaultParameter();
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
	TMC4->R.RESERVE_04 = __selected_traj_function;
	
	int32_t flag = 0;
	flag = (__force_stop) | (__is_busy << 1) | (__error << 2) | (__selected_traj_function << 8) | (__return_message << 12) | (__message_code_ << 16);

	//std::cout << "flag :" << flag << std::endl;

	TMC4->R.RESERVE_03 = flag;
	
	
	
	if( __return_message != 0)
	{
		counter++;
		if (counter >= 50){
			__return_message = 0;
			counter = 0;
		}
	}
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
	__velocity 		= ( double(TMC1->W.RESERVE_16) /10000000);	
	__scanRange 	= ( double(TMC2->W.RESERVE_16) /10000000);	
  __stepRange	  = ( double(TMC3->W.RESERVE_16) /10000000);	
	__lineSpacing = ( double(TMC4->W.RESERVE_16) /10000000);

	int32_t timeScale_scanMode_flag = TMC5->W.RESERVE_16;
	//std::cout << timeScale_scanMode_flag << std::endl;
	__timeScale = timeScale_scanMode_flag & 0x000F;
	__scanMode  = (timeScale_scanMode_flag >> 4) & 0x000F ;
	__useSimulateLight = (timeScale_scanMode_flag >> 8) & 0x000F ;

	int32_t thd = TMC6->W.RESERVE_16;
	//__thd0 = double(thd >> 16) /100;
	//__thd1 = double(thd & 0x0000FFFF) /100;
	
	
		
	int32_t lightSourceConfigCenter = TMC1->W.RESERVE_17;
	__LSourceY 	= double((lightSourceConfigCenter >> 16) & 0x0000FFFF);
	__LSourceZ 	= double(lightSourceConfigCenter & 0x0000FFFF);
	
	int32_t lightSourceConfigRadius = TMC2->W.RESERVE_17;
	__LRadA 		= double((lightSourceConfigRadius >> 16) & 0x0000FFFF) /100;
	__LRadB 		= double(lightSourceConfigRadius & 0x0000FFFF) /100;
	
	//std::cout << "__LSourceY:" << __LSourceY << "__LSourceZ:" << __LSourceZ << "__LRadA:" << __LRadA << "__LRadB:"<< __LRadB << std::endl;
	 
	//std::cout << "__timeScale:" << __timeScale << "__scanMode:"<< __scanMode << std::endl;

	
	// Update Modbus
	// pivot
	TMC1->R.RESERVE_15 = __pivot[0] *10000000;
	TMC2->R.RESERVE_15 = __pivot[1] *10000000;
	TMC3->R.RESERVE_15 = __pivot[2] *10000000;
	TMC4->R.RESERVE_15 = __pivot[3] *10000000;
	//TMC5->R.RESERVE_15 = __pivot[4] *10000000;
	//TMC6->R.RESERVE_15 = __pivot[5] *10000000;
	TMC5->R.RESERVE_15 = __alpha *10000000;
	TMC6->R.RESERVE_15 = __beta *10000000;
	
	// Tstart
	TMC1->R.RESERVE_05 = __poseStart [0] *10000000;
	TMC2->R.RESERVE_05 = __poseStart [1] *10000000;
	TMC3->R.RESERVE_05 = __poseStart [2] *10000000;
	TMC4->R.RESERVE_05 = __poseStart [3] *10000000;
	TMC5->R.RESERVE_05 = __poseStart [4] *10000000;
	TMC6->R.RESERVE_05 = __poseStart [5] *10000000;
	
	// parameters
	TMC1->R.RESERVE_16 = __velocity *10000000;
	TMC2->R.RESERVE_16 = __scanRange *10000000;	
	TMC3->R.RESERVE_16 = __stepRange *10000000;	
	TMC4->R.RESERVE_16 = __lineSpacing *10000000;
	
	timeScale_scanMode_flag = ( __timeScale) | ( __scanMode << 4) | ( __useSimulateLight << 8);
	//std::cout << timeScale_scanMode_flag << std::endl;
	TMC5->R.RESERVE_16 = timeScale_scanMode_flag;
	

	thd = (( int(__thd0 *100) << 16)) | int( __thd1 *100);
	TMC6->R.RESERVE_16 = thd;
	
	lightSourceConfigCenter = ( int(__LSourceY) << 16) | int( __LSourceZ);
	TMC1->R.RESERVE_17 = lightSourceConfigCenter;
	
	lightSourceConfigRadius = ( int(__LRadA *100) << 16) | int( __LRadB *100);
	TMC2->R.RESERVE_17 = lightSourceConfigRadius;
	
	Stewart_GMT::Set_LightSource(&lightSource);

	
	
	Matrix3d ry = MathTool::rotY(__alpha);
	Matrix3d rx = MathTool::rotX(__beta);
	Vector3d PMtool{__pivot[0], __pivot[1], __pivot[2]};
	Matrix3d RMtool = ry * rx;
	TMtool = MathTool::rp2t(RMtool, PMtool);
}



void sinTrack()
{

}



extern "C" void GMT_N_Traj(void)
{		
	if(__iter == 1){
		// update stewart parameters
		updateStewartParameter();
	}
	// update stewart flags
	updateStewartFlags();

	
	VectorXd poseStart(6);
	poseStart (0) = __poseStart[0];
	poseStart (1) = __poseStart[1];
	poseStart (2) = __poseStart[2];
	poseStart (3) = __poseStart[3];
	poseStart (4) = __poseStart[4];
	poseStart (5) = __poseStart[5];
	
	Matrix4d Tstart;
	Tstart = MathTool::poseRPY2SE3(poseStart);
	
	if(__force_stop == 1)
	{
		stop_runing();
		return;
	}

	/* Spiral Track */
	if(__selected_traj_function == TRAJ_FUNCTION_FA_SPIRAL)
	{
		Vector3d pivot{__pivot[0], __pivot[1], __pivot[2]};
		enumTimeScaling timeScale = static_cast<enumTimeScaling>(__timeScale);
		
		//test
		Vector2d r;
		Vector2d u;
		r << __LSourceY, __LSourceZ;
		u << __LRadA, __LRadB;
		__thd0 = 0.75;
		__thd1 = 0.95;
		
		//out = Stewart_GMT::N_ToolSpiralTrackOnce(TMtool, Tstart, __scanRange, __lineSpacing, __velocity, __ts, __iter);	
		out = Stewart_GMT::N_ToolSpiralTrackOnce( TMtool, Tstart, __scanRange, __lineSpacing, __velocity, __ts, timeScale, __iter, __stage, r, u, __thd0, __thd1);
		__stage = out.next_stage;		
		//std::cout << out.message << std::endl;

	}
	
	/* Sin Track */
	if(__selected_traj_function == TRAJ_FUNCTION_FA_SIN)
	{
		enumTimeScaling timeScale = static_cast<enumTimeScaling>(__timeScale);
		enumSinScanMode scanMode = static_cast<enumSinScanMode>(__scanMode);
		
		// test
		Vector2d r;
		Vector2d u;
		r << __LSourceY, __LSourceZ;
		u << __LRadA, __LRadB;
		__thd0 = 0.75;
		__thd1 = 0.95;
		
		out = Stewart_GMT::N_ToolSinTrackOnece(
			TMtool,
			Tstart,
			__velocity,
			__ts,
			timeScale,
			__lineSpacing,
			__stepRange,
			__scanRange,
			scanMode,
			__stage,
			__iter, r, u, __thd0, __thd1
		);

		__stage = out.next_stage;			
	}
	
	/* MoveL */
	if(__selected_traj_function == TRAJ_FUNCTION_MOVELPP)
	{	
		static VectorXd poseEnd(6);
		static VectorXd poseStart_moveL(6);
		// test
		Vector2d r;
		Vector2d u;
		r << __LSourceY, __LSourceZ;
		u << __LRadA, __LRadB;

		if(__iter == 1){
			//updateStewartParameter();

			poseStart_moveL(0) = __poseCurrent[0];
			poseStart_moveL(1) = __poseCurrent[1];
			poseStart_moveL(2) = __poseCurrent[2];
			poseStart_moveL(3) = __poseCurrent[3];
			poseStart_moveL(4) = __poseCurrent[4];
			poseStart_moveL(5) = __poseCurrent[5];
			
			poseEnd(0) = __poseStart[0];
			poseEnd(1) = __poseStart[1];
			poseEnd(2) = __poseStart[2];
			poseEnd(3) = __poseStart[3];
			poseEnd(4) = __poseStart[4];
			poseEnd(5) = __poseStart[5];
			
			if(	__poseCurrent[0] == __poseStart[0] && __poseCurrent[1] == __poseStart[1] &&
					__poseCurrent[2] == __poseStart[2] && __poseCurrent[3] == __poseStart[3] &&
					__poseCurrent[4] == __poseStart[4] && __poseCurrent[5] == __poseStart[5] ){
				//out.is_finished = true;
				__return_message = RESPONSE_NORMAL_FINISHED;
				//std::cout << "__return_message1 :" << __return_message << std::endl;
				//std::cout << "start same as end" << std::endl;
				stop_runing();
				return;
			}
			
			//std::cout << poseStart_moveL << poseEnd << std::endl;

			//poseEnd << __poseStart[0], __poseStart[1], __poseStart[2], __poseStart[3], __poseStart[4], __poseStart[5];;
		}
		
		enumTimeScaling timeScale = static_cast<enumTimeScaling>(__timeScale);
		
		//std::cout << "move L" << std::endl;
		//std::cout << "out.trajp 1:" << out.trajp << std::endl;

		out = Stewart_GMT::N_moveL_PPOnce(poseStart_moveL, poseEnd, timeScale, __velocity, __ts, __iter, r, u);
		
		//std::cout << "poseStart_moveL :" << poseStart_moveL  << std::endl;
		//std::cout << "poseEnd :" << poseEnd << std::endl;
		//std::cout << "out.trajp 2:" << out.trajp << std::endl;
		//std::cout << "__iter :" << __iter << std::endl;

		//std::cout << "out.is_finished :" << out.is_finished << std::endl;

	}
	
	if( out.message_code > 3)
		__message_code_ = __message_code_;
	else
		__message_code_ = out.message_code;
	
	if (out.is_finished )
	{
		__return_message = RESPONSE_NORMAL_FINISHED;

		//std::cout << "__message_code_ :" << __message_code_ << std::endl;

		stop_runing();
		return;
	}
	
	if ( out.error )
	{
		__return_message = RESPONSE_ERROR_STOP;
		//std::cout << "__return_message2 :" << __return_message << std::endl;

		//std::cout << out.message << std::endl;
		stop_runing();
		__error = 1;
		return;
	}
	
	//std::cout << "__return_message3 :" << __return_message << std::endl;

	TMC1->R.RESERVE_06 = out.trajp(0) * 10000000;
	TMC2->R.RESERVE_06 = out.trajp(1) * 10000000;
	TMC3->R.RESERVE_06 = out.trajp(2) * 10000000;
	TMC4->R.RESERVE_06 = out.trajp(3) * 10000000;
	TMC5->R.RESERVE_06 = out.trajp(4) * 10000000;
	TMC6->R.RESERVE_06 = out.trajp(5) * 10000000;
	


	
	// VectorXd outLength(6);
	nik_result = Stewart_GMT::N_Stewart_IK(out.trajp);
	auto dg = nik_result.ls;
	VectorXd g;
	double _h0 = 21.2;
	MatrixXd tarr = nik_result.alpha.array().square() + SQUARE(_h0) - nik_result.ls.array().square();
	g = nik_result.alpha.array() - tarr.array().sqrt();
	
	TMC1->R.RESERVE_07 = g(0) * 10000000;
	TMC2->R.RESERVE_07 = g(1) * 10000000;
	TMC3->R.RESERVE_07 = g(2) * 10000000;
	TMC4->R.RESERVE_07 = g(3) * 10000000;
	TMC5->R.RESERVE_07 = g(4) * 10000000;
	TMC6->R.RESERVE_07 = g(5) * 10000000;
	
	setAllMotorLength(g);

	
	__iter = out.next_iter;
	// std::cout << "__iter :" <<__iter << "\t__stage :" << __stage << std::endl;
}


