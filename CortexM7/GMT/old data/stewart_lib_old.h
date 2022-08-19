#pragma once

#include <iostream>
#include <vector>

#define H755DEMO

#if defined(H755DEMO)
#include "arm_math.h"
#include "math.h"
#include "../GNU/cminpack-1.3.8/minpack.h"
#include "../GNU/eigen-3.4.0/Eigen/Dense"
/* Includes **************************************************************************************************/
#include "BSP_STM32H7.h"
#else
#include "eigen-3.3.9/Eigen/Dense"
#include "..\cminpack-1.3.8\cminpack.h"  
#include "..\cminpack-1.3.8\minpack.h" 
#endif

//#pragma comment(lib, " cminpack")

#define real __cminpack_real__

#define MathPI 3.14159265358979323846
#define cot(X) (tan(MathPI*0.5 -X))
#define SQUARE(X) pow(X,2)

#define Mat MatrixXd
#define MatList vector<MatrixXd>
#define pinv0(X) X.completeOrthogonalDecomposition().pseudoInverse()

typedef double(*LightSourceFunc)(void);


//#if defined(__WIN32__) || defined(WIN32) || defined (_WIN32) || defined (__linux__) //__unix__
//#define ERROR_MESSAGE(MSG) throw invalid_argument(MSG);
#if defined(UNDER_RTSS) || defined(NO_MESSAGE)
//#define DEBUG_PRINT_MESSAGE(STR) printf("%s",STR.c_str());
//#define ERROR_MESSAGE(MSG) printf("ERROR : %s",MSG.c_str());
#define DEBUG_PRINT_ERROR_MESSAGE(MSG) 
#define DEBUG_PRINT_MESSAGE(STR) 
#define DEBUG_PRINT_VALUE(VAR) 
#define DEBUG_SET_COUT_BOOL_TO_STRING
#define DEBUG_PRINT_FUNCTION_NAME
#define DEBUG_PRINT_FUNCTION_LINE
#define DEBUG_PRINT_FUNCTION_LINE
#define DEBUG_PRINT_CODE_LOCATION
#else 
#define DEBUG_PRINT_ERROR_MESSAGE(MSG) cout << endl << "ERROR:" << MSG << endl;
#define DEBUG_PRINT_MESSAGE(STR) cout << endl << STR << endl;
#define DEBUG_PRINT_VALUE(VAR) cout << endl << #VAR << endl << VAR << endl;
#define DEBUG_SET_COUT_BOOL_TO_STRING cout << std::boolalpha;
#define DEBUG_PRINT_FUNCTION_NAME std::cout << __func__ << std::endl;
#define DEBUG_PRINT_FUNCTION_LINE std::cout << __LINE__  << std::endl;
#define DEBUG_PRINT_FUNCTION_FILE std::cout << __FILE__   << std::endl;
#define DEBUG_PRINT_CODE_LOCATION std::cout << "at " << __FILE__ << " " << __func__ << "() line " << __LINE__ << std::endl;
#endif

#if defined(UNDER_RTSS) 
#define RELEASE_PRINT_VALUE(VAR) 
#define RELEASE_PRINT_MESSAGE(STR) 
#else 
#define RELEASE_PRINT_VALUE(VAR) cout << endl << #VAR << endl << VAR << endl;
#define RELEASE_PRINT_MESSAGE(STR) cout << endl << STR << endl;
#endif


using namespace std;
using namespace Eigen;




#pragma region enumeration

typedef enum
{
	Cubic,      // 3rd-order
	Quintic,    // 5th-order
	Trapezoid,      // Trapezoid
	Sigmoid       // Sigmoid
}enumTimeScaling;

typedef enum
{
	NotSupport,
	TypeF,
	TypeN
}enumStewartType;

typedef enum
{
	f1,
	f2
}enumSpiralEvalF;

typedef enum
{
	ToolPivotTraj_X,
	ToolPivotTraj_Y,
	ToolPivotTraj_Z,
	ToolPivotTraj_XYZ
}enumToolPivotTrajType;

typedef enum
{
	ToolM_WSandPivot,
	ToolM_Spiral
}enumToolMType;

typedef enum
{
	tool,
	base	
}enumBaseMode;

typedef enum
{
	vertical,
	horizontal,
	Both
}enumSinScanMode;

typedef enum
{
	FASin,
	FASpiral
}enumFAScanMode;

#pragma endregion enumeration

#pragma region structure

class FTypeParameter
{
public:
	FTypeParameter(Mat bb, Mat pp, double Lx)
	{
		_bb = bb;
		_pp = pp;
		_Lx = Lx;
	}

	//with default value
	FTypeParameter()
	{
		_bb = Mat(3, 6);
		_pp = Mat(3, 6);

		_bb.col(0) << 135.8414016786395061, 33.8690653839534812, 0;
		_bb.col(1) << -38.589229814379886, 134.5766374313646407, 0;
		_bb.col(2) << -97.2521718642596201, 100.7075720474111595, 0;
		_bb.col(3) << -97.2521718642596201, -100.7075720474111595, 0;
		_bb.col(4) << -38.589229814379886, -134.5766374313646407, 0;
		_bb.col(5) << 135.8414016786395061, -33.8690653839534812, 0;

		_pp.col(0) << 64.6148423047648329, 59.0014589135757969, 0;
		_pp.col(1) << 18.7893411271180298, 85.4588243542396911, 0;
		_pp.col(2) << -83.4041834318828627, 26.4573654406638942, 0;
		_pp.col(3) << -83.4041834318828627, -26.4573654406638942, 0;
		_pp.col(4) << 18.7893411271180298, -85.4588243542396911, 0;
		_pp.col(5) << 64.6148423047648329, -59.0014589135757969, 0;


		_Lx = 167.64;
	}

	void UpdateL(VectorXd newL)
	{
		_L = newL;
	}
	
	Mat _pp, _bb;
	VectorXd _L;
	double _Lx;
} ;

//TODO:??eigen matrixXd?
class pList {
public:

	vector< vector<double> > P;

	void add(double point[], int size = 6) {
		// vector<double> point = { 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 };
		vector<double> p;
		p.assign(size, 0); // set dimention to size, value is 0
		for (int i = 0; i < size; i++) p[i] = point[i];
		P.push_back(p);
	}

	inline void add(vector<double> p) { P.push_back(p); }
	inline void add(VectorXd p) {
		vector<double> vd(p.data(), p.data() + p.size());
		P.push_back(vd);
	}

	inline void append(pList P2)
	{
		P.reserve(P.size() + P2.P.size()); // preallocate memory
		P.insert(P.end(), P2.P.begin(), P2.P.end());
	}

	inline void clear() { P.clear(); }

	inline int getSize() { return P.size(); }

	void show(int i)
	{
		if (i < P.size())
		{
			vector<double> vd = P[i];

			DEBUG_PRINT_MESSAGE(
				i+1 << ":"
				<< vd[0] << ", "
				<< vd[1] << ", "
				<< vd[2] << ", "
				<< vd[3] << ", "
				<< vd[4] << ", "
				<< vd[5] << endl
			)
		}

	}

	void show()
	{
		int c = 0;
		for (vector<double> vd : P)
		{
			c++;
			show(c);
		}

	}

	vector<double> get(int i)
	{
		if (i < P.size()) 
			return P[i];
		else 
			return vector<double>();
	}

	vector<double> getRow(int i)
	{
		if (i >= P[0].size()) return vector<double>();

		int n = P.size();
		vector<double> row;
		row.reserve(n);

		for(int j=0;j<n;j++)
		{
			vector<double> pj = P[j];
			row.push_back(pj[i]);
		}

		return row;
	}

	void removeLast()
	{
		P.resize(P.size() - 1);
	}

	vector<double> end()
	{
		if(P.size()>0)return P[P.size() - 1];
		else return vector<double>();
		
	}

	pList() {};
	~pList() {
		// vector has its own destructor for this
	};
	pList(const pList & plist) {
		P = plist.P;
	};
};

class PivotInput
{
public:
	VectorXd Pstart;
	VectorXd Ppivot;
	VectorXd w;
	double theta;
	double tf;
	double ts;
	enumTimeScaling tscale;
};

class TrajOnceOutput
{
public:
	VectorXd trajp;
	int next_iter;//?????
	int n;// 2*n = ???????, n=???index
	bool is_finished;//??????????,????????????
	string message;
	bool error;
	int next_stage;//??fa???
	double Ldata;//?????????
	double LdataM;//???????
	VectorXd trajM;//????????????
	double normP2R;//???????
	int message_code;
	int fa_next_stage;//??fa???
	VectorXd fa_stage1_endp;//fa????????
	double fa_stage1_LdataM;//fa?????????
	VectorXd fa_stage1_trajM;//fa??????????????
	VectorXd fa_stage2_endp;//fa????????
	double fa_stage2_LdataM;//fa?????????
	VectorXd fa_stage2_trajM;//fa??????????????


	TrajOnceOutput(string error_message)
	{
		next_iter = -1;
		is_finished = false;
		error = true;
		message = error_message;
	}

	//TrajOnceOutput(VectorXd traj_i_in,int iter_in,int n_in)
	//{
	//	trajp = traj_i_in;
	//	next_iter = iter_in;
	//	n = n_in;
	//	error = false;

	//	if (next_iter >= n) is_finished = true;
	//	else is_finished = false;
	//}

	TrajOnceOutput(VectorXd traj_i_in, int next_iter_in)
	{
		trajp = traj_i_in;
		next_iter = next_iter_in;
		error = false;
		is_finished = false;
	}

	TrajOnceOutput(VectorXd traj_i_in, int next_iter_in, int next_stage_in)
	{
		trajp = traj_i_in;
		next_stage = next_stage_in;
		next_iter = next_iter_in;
		error = false;
		is_finished = false;
	}
	
	TrajOnceOutput()
	{
		is_finished = true;
		error = false;
	}

};

class NIKOutput 
{
public:
	VectorXd alpha;
	VectorXd ls;

	NIKOutput()
	{
	}

	NIKOutput(VectorXd alpha_in, VectorXd ls_in)
	{
		alpha = alpha_in;
		ls = ls_in;
	}
};

class SpiralTEvaluateParameter
{
public:
	double b;
	double rx;
	double n;
	double k;
	double v;

	void setsp1(double b_in, double rx_in, double n_in)
	{
		b = b_in;
		rx = rx_in;
		n = n_in;
	}

	void setsp2(double b_in, double k_in, double v_in)
	{
		b = b_in;
		k = k_in;
		v = v_in;
	}
};

class ToolWorkSpaceLoopPivotOutput
{
public:
	VectorXd limtplus;
	VectorXd limitminus;
	double toollimitplus;
	double toollimitminus;

	//ToolWorkSpaceLoopPivotOutput(Vector3d angleplus_in, Vector3d angleminus_in, double toolangleplus_in, double toolangleminus_in)
	//{
	//	angleplus = angleplus_in;
	//	angleminus = angleminus_in;
	//	toolangleplus = toolangleplus_in;
	//	toolangleminus = toolangleminus_in;
	//}
};

class ToolPivotTrajLoopOutput
{
public:
	pList trajplus;
	pList trajminus;
	double toollimitplus;
	double toollimitminus;

};

class toolTrans
{
public:
	Vector3d xrtool;
	Vector3d yrtool;
	Vector3d zrtool;
	VectorXd poseStart;
	Matrix4d TMtool;
};

//ws??????,????inputIllegal,false????????????
class WSOutput
{
public:
	//????????????????
	bool inputIllegal;
	//???????,?inputIllegal=false??????????
	VectorXd remain12;
	//???pose se3????,?inputIllegal=false??????????
	Matrix4d tstart;
	//?????????input
	VectorXd input;

	Matrix4d TMtool;

	   	 	   
};

class WSLimitTestOutput
{
public:
	double min;
	double max;
	bool inputIllegal;

	WSLimitTestOutput()
	{
		min = 0;
		max = 0;
		inputIllegal = false;
	}
};

class FADataPinOutput 
{
public:
	pList traj;
	VectorXd Ldata;
	vector<double> trajM;
	double LdataM;
	bool error;
	string message;
	double scan;
	double step;
	int message_code;
	double normP2R;

	FADataPinOutput()
	{
		
	}
	FADataPinOutput(string error_message)
	{
		error = true;
		message = error_message;
	}

	FADataPinOutput(pList traj_in, VectorXd Ldata_in)
	{
		traj = traj_in;
		Ldata = Ldata_in;
		error = false;
	}

	void append(pList traj_in)
	{
		traj.append(traj_in);
	}

	void append(pList traj_in, VectorXd Ldata_in)
	{
		traj.append(traj_in);

		VectorXd C(Ldata.rows() + Ldata_in.rows());
		C << Ldata, Ldata_in;
		Ldata = C;
	}

	void append(FADataPinOutput fadata)
	{
		append(fadata.traj, fadata.Ldata);
	}


	void show(int i)
	{
		if (i < traj.getSize() && i< Ldata.size())
		{
			vector<double> traji = traj.get(i);
			double Ldatai = Ldata[i];


			DEBUG_PRINT_MESSAGE(
				i+1 << ": traj "
				<< traji[0] << ", "
				<< traji[1] << ", "
				<< traji[2] << ", "
				<< traji[3] << ", "
				<< traji[4] << ", "
				<< traji[5] << ", "
				<< "  Ldata: "<< Ldatai << endl
			)
		}

	}

	void show(bool show_traj,bool show_max)
	{
		if (show_traj)
		{
			for (int i = 0; i < traj.getSize(); i++)
			{
				show(i);
			}
		}

		if (show_max )
		{
			if (trajM.size() >= 6)
			{

				DEBUG_PRINT_MESSAGE(
					"trajM "
					<< trajM[0] << ", "
					<< trajM[1] << ", "
					<< trajM[2] << ", "
					<< trajM[3] << ", "
					<< trajM[4] << ", "
					<< trajM[5] << ", "
					<< "  LdataM: " << LdataM << endl
				)
			}
			else
			{
				DEBUG_PRINT_ERROR_MESSAGE("trajM size < 6")
			}
		}

	}

};

class WeightAvarage
{
public:
	double preWeiSum;//??????
	VectorXd preAve;//???????

	WeightAvarage()
	{
		reset();
	}

	void reset()
	{
		preWeiSum = 0;
		preAve = VectorXd::Zero(6);
	}

	void update(double aw, VectorXd xm_6x1)
	{
		double weiSum = preWeiSum + aw;
		VectorXd wAver = (preWeiSum*preAve + aw * xm_6x1) / weiSum;
		preAve = wAver;
		preWeiSum = weiSum;
	}
};




#pragma endregion structure

class MathTool
{
public:

	//?????
	static double degree2Radian(double degree);

	//?????
	static double radian2Degree(double radian);

	//w(zyz) abg?  so3
	//???????
	static Mat ZYZ2SO3(Vector3d w);
	//pose xyzabg? zyz so3
	//???????
	static Mat poseZYZ2SO3(VectorXd Pose);
	//wToZYZSO3
	static Matrix4d poseZYZ2SE3(VectorXd pose);

	//???ZYZ2SO3??
	//???????????????,???????
	static Matrix3d ZYZ2SO3_2(Vector3d pose);




	//w(rpy=zyx) ?  so3
	static Mat RPY2SO3_2(Vector3d w);
	//pose (rpy=zyx)? so3
	static Mat poseRPY2SO3_2(VectorXd Pose);
	//pose (rpy=zyx)? se3
	static Matrix4d poseRPY2SE3(VectorXd pose);




	//????1*6???
	//?pose??4?5?6?? ????????
	static VectorXd degree2Radian6(VectorXd Pose_deg);
	//????1*12???
	//?pose??6~11?? ????????
	static VectorXd radian2Degree12(VectorXd Pose_rad);

	//??C++?????<random>??random_device,????????????
	//?? https://blog.gtwang.org/programming/cpp-random-number-generator-and-probability-distribution-tutorial/
	static vector<double> randomFactory(double min, double max, int how_many);
	static double getRandomNumber(double min, double max);


	static VectorXd mat2Vector(Mat in);


	static VectorXd sE3ToPoseAxisAngle(MatrixXd se3mat);


	static Vector3d sO3ToPoseAxisAngle(MatrixXd so3mat);


	static VectorXd sE3ToPoseRPY(MatrixXd se3mat);


	static Vector3d sO3ToPoseRPY(MatrixXd so3mat);


	static Matrix3d rotZ(double theta);
	static Matrix3d rotY(double theta);
	static Matrix3d rotX(double theta);


	static MatrixXd reshape(MatrixXd mat_in, int rows, int cols);


	static MatrixXd pinv(MatrixXd m);

	static Matrix3d vect3tomatrix3(Vector3d vin);

	static Mat MatrixExp6(Mat se3mat);

	static Mat so3ToVec(Mat so3Mat);

	static bool NearZero(double val);

	static double norm(Mat mat_in);

	static double AxisAng3(Mat expc3, Mat omghat);

	static Mat MatrixExp3(Mat so3mat);

	static double CubicTimeScaling(double Tf, double t);

	static double QuinticTimeScaling(double Tf, double t);

	static Matrix4d MatrixLog6(Mat T);

	static Matrix4d TransInv(Mat T);

	static double Trace(Mat M);

	static Matrix3d MatrixLog3(Mat R);

	static VectorXd SE3ToPoseZYZ(Mat T);

	static vector<double> vectorXDtoStdVector(VectorXd v);
	static VectorXd stdVectortoVectorXD(vector<double> v);

	static Matrix3d VecToso3(Mat omg);

	static Matrix4d rp2t(Matrix3d r, Vector3d p);
	static Matrix3d t2r(Matrix4d t);
	static Vector3d t2p(Matrix4d t);

	static Matrix3d AxisAngle2SO3(Vector3d w);

	//static vector<double> findAll(vector<double> source, double target);

private:
	static vector<double> random_numbers;

};

//TODO:?? header
class Stewart_GMT : private MathTool
{
public:
	//?????pose,??link??
	//?? :
	//VectorXd P(6) : ?????pose,1 * 6 double??
	//Mat pp(3, 6) : ?????,3 * 6 double??,??stewart??
	//Mat bb(3, 6) : ?????,3 * 6 double??,??stewart??
	//double h0 : ???link??(??),??stewart??
	//double g0 : ???link??(? )
	//?? :
	//NIKOutput.alpha : (? )
	//NIKOutput.ls : ???link??1 * 6 double??
	static NIKOutput N_Stewart_IK(VectorXd P, Mat pp, Mat bb, double h0, double g0);
	//?????pose,??????pp bb h0 g0,??link??
	//?? :
	//VectorXd P(6) : ?????pose,1 * 6 double??
	//?? :
	//NIKOutput.alpha : (? )
	//NIKOutput.ls : ???link??,1 * 6 double??
	static NIKOutput N_Stewart_IK(VectorXd P);
	//F??????????????
	//P:???pose pp:????? bb:?????
	static VectorXd F_Stewart_IK(VectorXd P, Mat pp, Mat bb);
	//P:???pose ??????pp bb
	static VectorXd F_Stewart_IK(VectorXd P);
	//N??????????????
	//x:???6?????????
	//??minpack?hybrid???matlab fsolve???????????? 
	//??1*6 ??pose
	static VectorXd N_Stewart_FK(VectorXd ini_pose);
	static VectorXd N_Stewart_FK();
	//F??????????????
	//x:???6?????????
	//??minpack?hybrid???matlab fsolve???????????? 
	static VectorXd F_Stewart_FK(FTypeParameter fpara, VectorXd ini_pose);
	//??????????
	static VectorXd F_Stewart_FK(FTypeParameter fpara);
	//P:???pose pp:????? bb:?????
	//??6*42 A??
	static Mat stewartJacobian(VectorXd P, Mat pp, Mat bb);
	

	//pp bb Lx ?? F?????
	static void set_F_Type_Stewart_DefaultParameter();
	//pp bb h0 g0 ?? N?????
	static void set_N_Type_Stewart_DefaultParameter();

	static void set_A12_DefaultParameter();
	//???tool?????????
	//??1*12?????????????
	static WSOutput N_ToolWS(VectorXd ToolPoseStart, Vector3d pivot, double alpha, double beta, enumBaseMode baseMode);
	static VectorXd F_PivotWorkSpace(VectorXd ToolPoseStart, Vector3d pivot, double alpha, double beta);
	//??1*6 link??,??????
	static bool checkL(VectorXd L);
	//??tool?????phome
	static void Reset_TposeStart();
	//???????????pivot?????delta
	static void SetWSdelta(double trans, double rot);


	//????????,??6*n double ??
	static pList moveL_Stewart(VectorXd pstart, VectorXd pend, double tf, double ts, enumTimeScaling timescaling);
	//pivot????????,??tool????
	static pList pivotTraj(VectorXd pstart, Vector3d pivot, Vector3d w, double theta, double tf, double ts, enumTimeScaling timescaling);
	static pList pivotTraj(PivotInput input);
	//pivot????????,??tool????,???????
	static TrajOnceOutput pivotTrajOnce(VectorXd pstart, Vector3d pivot, Vector3d w, double theta, double tf, double ts, enumTimeScaling timescaling, int iter);
	//????pivot??(??),??tool????
	static pList pivotMotion(VectorXd pstart, Vector3d pivot, Vector3d angle, double tf, double ts, enumTimeScaling timescaling);
	//????pivot??(??????),??tool????
	static pList pivotMotionN(VectorXd pstart, Vector3d pivot, Vector3d angle, double tf, double ts, enumTimeScaling timescaling);
	//????pivot??(??????),??tool????
	static pList pivotMotionP(VectorXd pstart, Vector3d pivot, Vector3d angle, double tf, double ts, enumTimeScaling timescaling);
	//???xyz?????????????
	static pList freeRun(VectorXd pstart, Vector3d angle, double tf, double ts, enumTimeScaling timescaling);
	//?tool?????pivot??,?????z+ z- y+ y- x+ x-
	static pList N_ToolPivotTraj(Vector3d pivot, double alpha, double beta, Matrix4d Tstart, double tf, double ts, enumTimeScaling tscale, enumToolPivotTrajType pivotTrajType);
	//?tool?????pivot??,?????x+ x-
	//spiral????,??6*n??
	static pList N_ToolSpiralTrack(Matrix4d TMtool, Matrix4d Tstart, double scanRange, double lineSpacing, double velocity, double ts, enumTimeScaling tscal);
	static TrajOnceOutput N_ToolSpiralTrackOnce(Matrix4d TMtool, Matrix4d Tstart, double scanRange, double lineSpacing, double velocity, double ts, enumTimeScaling tscal, int iter, int stage, Vector2d r, Vector2d u, double thd0, double thd1);
	//static pList N_FA_SpiralTrack(Vector3d pivot, double alpha, double beta, Matrix4d Tstart, double scanRange, double lineSpacing, double velocity, double ts, enumTimeScaling tscal);
	static FADataPinOutput N_FA_SpiralTrack_DataPin(Matrix4d TMtool, Matrix4d Tstart, double scanRange, double lineSpacing, double velocity, double ts, enumTimeScaling tscal, VectorXd remain12, Vector2d r, Vector2d u, double thd0, double thd1);


	static pList N_ToolSinTrack(Matrix4d TMtool, Matrix4d Tstart, double velocity, double ts, enumTimeScaling tscal, double lineSpacing, double stepRange, double scanRange, enumSinScanMode scanMode, VectorXd remain12);
	//????input????,??????
	static TrajOnceOutput N_ToolSinTrackOnece(Matrix4d TMtool, Matrix4d Tstart, double velocity, double ts, enumTimeScaling tscal, double lineSpacing, double stepRange, double scanRange, enumSinScanMode scanMode, int stage, int iter, Vector2d r, Vector2d u, double thd0, double thd1);
	static FADataPinOutput N_FA_SinTrack_DataPin(Matrix4d TMtool, Matrix4d Tstart, double velocity, double ts, enumTimeScaling tscal, double lineSpacing, double stepRange, double scanRange, enumSinScanMode scanMode, VectorXd remain12, Vector2d r, Vector2d u, double thd0, double thd1);


	static pList N_moveL_PP(VectorXd Pstart, VectorXd Pend, enumTimeScaling timescaling, double velocity, double ts);
	static TrajOnceOutput N_moveL_PPOnce(VectorXd Pstart, VectorXd Pend, enumTimeScaling timescaling, double velocity, double ts, int iter, Vector2d r, Vector2d u);
	static FADataPinOutput N_FA_moveL_PP_Data(Matrix4d TMtool, Matrix4d Tstart, VectorXd Pend, enumTimeScaling timescaling, double velocity, double ts, VectorXd remain12, Vector2d r, Vector2d u);

	static WSOutput A12_WS(VectorXd ToolPoseWorking);
	static pList A12_SpiralTrack(Matrix4d Tstart, double scanRange, double lineSpacing, double velocity, double ts, enumTimeScaling tscal);
	static pList A12_SinTrack(Matrix4d Tstart, double velocity, double ts, enumTimeScaling tscal, double lineSpacing, double stepRange, double scanRange, enumSinScanMode scanMode);
	static pList A12_moveL_PP(VectorXd Pstart, VectorXd Pend, enumTimeScaling timescaling, double velocity, double ts);

	//??????????
	static void Set_LightSource();
	//????????,??????
	//??
	//double light()
	//{
	//	return 0.55f;
	//}
	//	Stewart_GMT::Set_LightSource(&light);
	//	double light2 = Stewart_GMT::GetPowerMeterData_Wang(y, z, r1, u1);
	static void Set_LightSource(LightSourceFunc source);
	//????????,y z:?? r:????? u:????
	static double GetPowerMeterData_Wang(double y, double z, Vector2d r, Vector2d u);

	//?FA?????,??FAScanMode=spiral????scanRange??stepRange
	//????Set_LightSource(LightSourceFunc source)??????,r u ???
	static TrajOnceOutput N_FA_TrackOnce(
		Matrix4d TMtool, Matrix4d Tstart, double velocity, double ts, enumTimeScaling tscal,
		double lineSpacing_firstScan, double stepRange_firstScan, double scanRange_firstScan, enumFAScanMode firstScan, 
		double lineSpacing_secondScan, double stepRange2_secondScan, double scanRange2_secondScan, enumFAScanMode secondScan,
		enumSinScanMode scanMode, double thd0, double thd1, Vector2d r, Vector2d u,
		int iter, int stage, int fa_stage
	);



	
	static VectorXd pivotLimit(VectorXd pstart, Vector3d pivot, Vector3d w, double theta, double tf, double ts, enumTimeScaling timescaling);
	 
	//pp: ????? 
	static void setPP(Mat pp);
	static Mat getPP();
	//bb: ?????
	static void setBB(Mat bb);
	static Mat getBB();
	//L: ?? FK?
	static void setL(VectorXd L);
	static VectorXd getL();
	//H: N????
	static void setH(double h0);
	//G: N????
	static void setG(double g0);
	static double getG();
	//Lx: ?????
	static void setLx(double Lx);
	static double getLx();

	static void setFKDebugMessage(bool show);



	static pList SpiralTrack_v1(VectorXd PStart, double t, double Ts, double b, double N);

	static VectorXd F_WorkSpace1(VectorXd Pose);//?? ?????
	static VectorXd N_WorkSpace1(VectorXd Pose);//?? ?????
	static VectorXd F_WorkSpace(VectorXd Pose);
	static VectorXd N_WorkSpace(VectorXd Pose);
	
	//screw????,??6*n??
	static pList ScrewTrack(VectorXd Pstart, double t, double Ts, double b, double zb, double N);

//TODO:?????????????????
#define WS_LIMIT_X_PLUS 0
#define WS_LIMIT_X_MINUS 1
#define WS_LIMIT_Y_PLUS 2
#define WS_LIMIT_Y_MINUS 3
#define WS_LIMIT_Z_PLUS 4
#define WS_LIMIT_Z_MINUS 5
#define WS_LIMIT_PHI_PLUS 6
#define WS_LIMIT_PHI_MINUS 7
#define WS_LIMIT_THETA_PLUS 8
#define WS_LIMIT_THETA_MINUS 9
#define WS_LIMIT_PSI_PLUS 10
#define WS_LIMIT_PSI_MINUS 11
	


private:

	//N??????????????
	//n:????? x:????? fvec:N_myfun?? iflag:?"1"?????hybrid??
	//??hybrid(=matlab fsolve)?????????
	static void N_myfun(const int *n, const double *x, double *fvec, int *iflag);
	//F??????????????
	//n:????? x:????? fvec:N_myfun?? iflag:?"1"?????hybrid??
	//??hybrid(=matlab fsolve)?????????
	static void F_myfun(const int *n, const double *x, double *fvec, int *iflag);

	//??tool??????????
	static WSOutput WorkSpacePivot(VectorXd ToolPoseWorking, Vector3d pivot, double alpha, double beta, enumStewartType stewart_type, enumBaseMode baseMode);
	//??tool??????????pose?????
	static toolTrans ToolCalculateM(VectorXd Pstart, Vector3d Ppivot, double alpha, double beta, enumBaseMode baseMode);
	//tool?????????????
	//????????????
	static WSLimitTestOutput ToolWorkSpaceLoop(VectorXd Pose, enumStewartType stewart_type, VectorXd w, double limit_plus, double limit_minus, double interval);
	//tool???????pivot??????
	//????????pivot????
	static WSLimitTestOutput ToolWorkSpaceLoopPivot(VectorXd Pstart, VectorXd Ppivot, VectorXd w, enumStewartType stewart_type, double limit_plus, double limit_minus, double interval);
	//?????pivot????????,??pivot?????(??)
	static VectorXd PivotTrajForCheck(VectorXd Pstart, VectorXd pivot, VectorXd w, double theta);
	//??link??????
	static bool illegal_lenth(VectorXd PTest, enumStewartType stewart_type);
	static bool illegal_lenth(vector<double> PTest, enumStewartType stewart_type);
	//12??
	static bool illegal_lenthA12(VectorXd PTest);

	//???tool?????pivot??
	static pList ToolPivotTrajCal(Vector3d pivot, double alpha, double beta, Matrix4d Tstart, double tf, double ts, enumTimeScaling tscale, enumStewartType stewart_type, enumToolPivotTrajType pivotTrajType);

	//newton_evaluate_function:???????
	//*p:???????????? ?:myclass input; &input = *p;
	static double newton(double x0, double tolx, int maxiter, double(*newton_evaluate_function)(double t, void *p), void *p);
	static double spiral_time_eval_f1(double t, void *p);
	static double spiral_time_eval_f2(double t, void *p);

	static double GetPowerMeterData(double y, double z, double y1, double y2, double z1, double z2, Vector2d r, Vector2d u, Vector3d xyz, Vector3d toolTransly, Vector3d toolTranslz);

	


	static VectorXd WorkSpace(VectorXd Pose, enumStewartType stewart_type);
	static VectorXd WorkSpaceLoop(VectorXd Pose, enumStewartType stewart_type, VectorXd w, double limit_plus,double limit_minus, double interval);
	static ToolPivotTrajLoopOutput ToolPivotTrajloop(VectorXd Pstart, VectorXd pivot, VectorXd w, double tf, double ts, enumTimeScaling tscale, enumStewartType stewart_type, double limit_plus, double limit_minus, double interval);


	//12???????
	static WSOutput a12_ws(VectorXd ToolPoseWorking);



	static bool check_stewart_type_and_set_parameter(enumStewartType stype);

	//static double *_pp, *_bb, *_L;
	static Mat _pp;
	static Mat _bb;
	static VectorXd _L;
	static double _h0, _g0;
	static double _Lx;
	static double _Llow;
	static double _Lupp;
	static double _work_space_delta_trans;
	static double _work_space_delta_rot;
	static VectorXd _phome;
	static Matrix4d _TposeStart;
	static bool _firstRun;
	static WSOutput _LastLegalResult;
	static bool _lightsource_real;
	static LightSourceFunc _light_source;
	static int _message_code;

	// [0] = x+
	// [1] = x- 
	// [2] = y+ 
	// [3] = y- 
	// [4] = z+ 
	// [5] = z- 
	// [6] = phi+ 
	// [7] = phi- 
	// [8] = theta+ 
	// [9] = theta- 
	// [10] = psi+
	// [11] = psi-
	static vector<double> _work_space_test_limit;

	static bool _is_show_fk_debug;
};
