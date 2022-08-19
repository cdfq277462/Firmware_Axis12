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

#define MESSAGE_SCAN_FIND_MAX 		1
#define MESSAGE_SCAN_FIND_AVARAGE 2
#define MESSAGE_SCAN_FAIL 				3

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
	TypeN,
	TypeA12
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

typedef enum
{
	ConstV,//���t
	ConstA //�����t��
}enumSpiralTimeEvaluate;


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

//TODO:���eigen matrixXd?
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
	int next_iter;//����y���I
	int n;// 2*n = ����y���I�ƶq, n=����Iindex
	bool is_finished;//�O�_���̫�@�ӭy���I�A�i�@�������y�񵲧�������
	string message;
	bool error;
	int next_stage;//����fa�p�y�{
	double Ldata;//��e�y���I���G�j�p
	double LdataM;//���{���G�̤j��
	VectorXd trajM;//���{���G�̤j�ȹ����y���I
	double normP2R;//�ȼ��������ͮ�
	int message_code;
	int fa_next_stage;//����fa�j�y�{
	VectorXd fa_stage1_endp;//fa�Ĥ@���q�y����I
	double fa_stage1_LdataM;//fa�Ĥ@���q���G�̤j��
	VectorXd fa_stage1_trajM;//fa�Ĥ@���q���G�̤j�ȹ����y���I
	VectorXd fa_stage2_endp;//fa�ĤG���q�y����I
	double fa_stage2_LdataM;//fa�ĤG���q���G�̤j��
	VectorXd fa_stage2_trajM;//fa�ĤG���q���G�̤j�ȹ����y���I


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
		trajp;
		next_iter=0;//����y���I
		n=0;// 2*n = ����y���I�ƶq, n=����Iindex
		is_finished=false;//�O�_���̫�@�ӭy���I�A�i�@�������y�񵲧�������
		message;
		error=false;
		next_stage=0;//����fa�p�y�{
		Ldata=0;//��e�y���I���G�j�p
		LdataM=0;//���{���G�̤j��
		trajM;//���{���G�̤j�ȹ����y���I
		normP2R=0;//�ȼ��������ͮ�
		message_code= MESSAGE_SCAN_FAIL;
		fa_next_stage=0;//����fa�j�y�{
		fa_stage1_endp;//fa�Ĥ@���q�y����I
		fa_stage1_LdataM=0;//fa�Ĥ@���q���G�̤j��
		fa_stage1_trajM;//fa�Ĥ@���q���G�̤j�ȹ����y���I
		fa_stage2_endp;//fa�ĤG���q�y����I
		fa_stage2_LdataM=0;//fa�ĤG���q���G�̤j��
		fa_stage2_trajM;//fa�ĤG���q���G�̤j�ȹ����y���I
	}
	
	static TrajOnceOutput Finished()
	{
		TrajOnceOutput out;
		out.is_finished = true;
		out.error = false;
		return out;
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

	void setsp2(double b_in, double k_in)
	{
		b = b_in;
		k = k_in;
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

//ws�p�������X�A�`�N�ˬdinputIllegal�Afalse�ɷ|�^�ǳ̫�@�����`���G
class WSOutput
{
public:
	//�����p���J�O�_�W�X���x���ʽd��
	bool inputIllegal;
	//�u�@�Ŷ��Ѿl�q�A�YinputIllegal=false�h�^�ǳ̫�@�����`��
	VectorXd remain12;
	//�W���xpose se3�ഫ�x�}�A�YinputIllegal=false�h�^�ǳ̫�@�����`��
	Matrix4d tstart;
	//�^�ǳ̫�@�����`��input
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
	double preWeiSum;//�e�@���v���M
	VectorXd preAve;//�e�@���v�����G

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



#define MESSAGE_FA_MAX_FOUND 4000001
#define MESSAGE_FA_BETWEEN_THD0_THD1 4000002
#define MESSAGE_FA_BELOW_THD0 4000003
#define MESSAGE_FA_SET_THRESHOLD 4000004

class LDataLogic
{
private:
	WeightAvarage avg;
	VectorXd PMax;
	double LDataMax;
	int messageCode;
	double thd0;
	double thd1;

public:
	LDataLogic()
	{
		Reset();
		messageCode = MESSAGE_FA_SET_THRESHOLD;
	}

	//������k�s
	void Reset()
	{
		avg.reset();
		PMax = VectorXd::Zero(6);
		LDataMax = 0;
		messageCode = MESSAGE_FA_BELOW_THD0;
	}

	//�`�N�P�ɷ|������k�s
	void SetThreshold(double thd0, double thd1)
	{
		this->thd0 = thd0;
		this->thd1 = thd1;
		Reset();
	}

	void Updata(double LData, VectorXd P)
	{
		if (messageCode == MESSAGE_FA_SET_THRESHOLD) return;

		if (LData > LDataMax)
		{
			LDataMax = LData;
			PMax = P;
		}

		if (LData >= thd1)
		{
			messageCode = MESSAGE_FA_MAX_FOUND;
		}
		else if (LData >= thd0)
		{
			if (messageCode != MESSAGE_FA_MAX_FOUND)
			{
				messageCode = MESSAGE_FA_BETWEEN_THD0_THD1;
			}
			avg.update(LData, P);

		}
		else if (messageCode != MESSAGE_FA_MAX_FOUND
			&& messageCode != MESSAGE_FA_BETWEEN_THD0_THD1)
		{
			messageCode = MESSAGE_FA_BELOW_THD0;
		}
	}

	double GetLDataMax()
	{
		return LDataMax;
	}

	VectorXd GetPMax()
	{
		return PMax;
	}

	VectorXd GetPAverage()
	{
		return avg.preAve;
	}

	int GetMessageCode()
	{
		return messageCode;
	}

};


#pragma endregion structure

class MathTool
{
public:

	//�����੷��
	static double degree2Radian(double degree);

	//�����ਤ��
	static double radian2Degree(double radian);

	//w(zyz) abg��  so3
	//��@�ϥήi�}��
	static Mat ZYZ2SO3(Vector3d w);
	//pose xyzabg�� zyz so3
	//��@�ϥήi�}��
	static Mat poseZYZ2SO3(VectorXd Pose);
	//wToZYZSO3
	static Matrix4d poseZYZ2SE3(VectorXd pose);

	//���G�MZYZ2SO3�ۦP
	//�t�O�u�b���{�L�{�ϥίx�}���k�A�ӫe�̥ήi�}��
	static Matrix3d ZYZ2SO3_2(Vector3d pose);




	//w(rpy=zyx) ��  so3
	static Mat RPY2SO3_2(Vector3d w);
	//pose (rpy=zyx)�� so3
	static Mat poseRPY2SO3_2(VectorXd Pose);
	//pose (rpy=zyx)�� se3
	static Matrix4d poseRPY2SE3(VectorXd pose);




	//�u��B�z1*6���x�}
	//�Npose����4�B5�B6���q �q�����੷�ת��
	static VectorXd degree2Radian6(VectorXd Pose_deg);
	//�u��B�z1*12���x�}
	//�Npose����6~11���q �q�����ਤ�ת��
	static VectorXd radian2Degree12(VectorXd Pose_rad);

	//�ϥ�C++�зǨ禡�w<random>����random_device�A�귽�������A�X�j�q�I�s?
	//�Ѧ� https://blog.gtwang.org/programming/cpp-random-number-generator-and-probability-distribution-tutorial/
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

//TODO:��z header
class Stewart_GMT : private MathTool
{
public:
	//��J�W���xpose�A�f��link����
	//��J :
	//VectorXd P(6) : �W���x��epose�A1 * 6 double�V�q
	//Mat pp(3, 6) : �W���x�ѼơA3 * 6 double�x�}�A�Ѩ�stewart�W��
	//Mat bb(3, 6) : �U���x�ѼơA3 * 6 double�x�}�A�Ѩ�stewart�W��
	//double h0 : �W�b�qlink����(�T�w)�A�Ѩ�stewart�W��
	//double g0 : �U�b�qlink����(? )
	//��X :
	//NIKOutput.alpha : (? )
	//NIKOutput.ls : �U�b�qlink����1 * 6 double�V�q
	static NIKOutput N_Stewart_IK(VectorXd P, Mat pp, Mat bb, double h0, double g0);
	//��J�W���xpose�A�ϥΤw�]�w��pp bb h0 g0�A�f��link����
	//��J :
	//VectorXd P(6) : �W���x��epose�A1 * 6 double�V�q
	//��X :
	//NIKOutput.alpha : (? )
	//NIKOutput.ls : �U�b�qlink���סA1 * 6 double�V�q
	static NIKOutput N_Stewart_IK(VectorXd P);
	//F�t�C���@�q�����c���b���x�ϥ�
	//P:�W���xpose pp:�U���x�Ѽ� bb:�W���x�Ѽ�
	static VectorXd F_Stewart_IK(VectorXd P, Mat pp, Mat bb);
	//P:�W���xpose �ϥΤw�]�w��pp bb
	static VectorXd F_Stewart_IK(VectorXd P);
	//N�t�C����q�����c���b���x�ϥ�
	//x:�j�p��6���}����l�q���V�q
	//�ϥ�minpack��hybrid�ӹ�{matlab fsolve�ۦP�ѫD�u���p�ߤ�{�\�� 
	//��^1*6 ���xpose
	static VectorXd N_Stewart_FK(VectorXd ini_pose);
	static VectorXd N_Stewart_FK();
	//F�t�C���@�q�����c���b���x�ϥ�
	//x:�j�p��6���}����l�q���V�q
	//�ϥ�minpack��hybrid�ӹ�{matlab fsolve�ۦP�ѫD�u���p�ߤ�{�\�� 
	static VectorXd F_Stewart_FK(FTypeParameter fpara, VectorXd ini_pose);
	//�ϥιw�]���q���_�l��
	static VectorXd F_Stewart_FK(FTypeParameter fpara);
	//P:�W���xpose pp:�U���x�Ѽ� bb:�W���x�Ѽ�
	//��^6*42 A�x�}
	static Mat stewartJacobian(VectorXd P, Mat pp, Mat bb);
	

	//pp bb Lx �a�J F�t�C�w�]��
	static void set_F_Type_Stewart_DefaultParameter();
	//pp bb h0 g0 �a�J N�t�C�w�]��
	static void set_N_Type_Stewart_DefaultParameter();

	static void set_A12_DefaultParameter();
	//�p�⦳tool�y���ഫ���u�@�Ŷ�
	//��^1*12�U�b���t��V�B�ʷ����Ѿl�q
	static WSOutput N_ToolWS(VectorXd ToolPoseStart, Vector3d pivot, double alpha, double beta, enumBaseMode baseMode);
	static VectorXd F_PivotWorkSpace(VectorXd ToolPoseStart, Vector3d pivot, double alpha, double beta);
	//��J1*6 link���סA��^�O�_�X�k
	static bool checkL(VectorXd L);
	//���]tool�ಾ�x�}��phome
	static void Reset_TposeStart();
	//�]�w�u�@�Ŷ��p�⥭���Mpivot�j��ϥΪ�delta
	static void SetWSdelta(double trans, double rot);


	//���u�B�ʭy���ơA��^6*n double �y��
	static pList moveL_Stewart(VectorXd pstart, VectorXd pend, double tf, double ts, enumTimeScaling timescaling);
	//pivot�y���ƭ�l�����A�S��tool�y���ഫ
	static pList pivotTraj(VectorXd pstart, Vector3d pivot, Vector3d w, double theta, double tf, double ts, enumTimeScaling timescaling);
	static pList pivotTraj(PivotInput input);
	//pivot�y���ƭ�l�����A�S��tool�y���ഫ�A�u�p����I�y��
	static TrajOnceOutput pivotTrajOnce(VectorXd pstart, Vector3d pivot, Vector3d w, double theta, double tf, double ts, enumTimeScaling timescaling, int iter);
	//¶�T�b��pivot�B��(�Ӧ^)�A�S��tool�y���ഫ
	static pList pivotMotion(VectorXd pstart, Vector3d pivot, Vector3d angle, double tf, double ts, enumTimeScaling timescaling);
	//¶�T�b��pivot�B��(�ȥ����פ�V)�A�S��tool�y���ഫ
	static pList pivotMotionN(VectorXd pstart, Vector3d pivot, Vector3d angle, double tf, double ts, enumTimeScaling timescaling);
	//¶�T�b��pivot�B��(�ȭt���פ�V)�A�S��tool�y���ഫ
	static pList pivotMotionP(VectorXd pstart, Vector3d pivot, Vector3d angle, double tf, double ts, enumTimeScaling timescaling);
	//��^�uxyz�U�b���t��V�����B����y��
	static pList freeRun(VectorXd pstart, Vector3d angle, double tf, double ts, enumTimeScaling timescaling);
	//��tool�y���ഫ��pivot�y��A��b���Ǭ�z+ z- y+ y- x+ x-
	static pList N_ToolPivotTraj(Vector3d pivot, double alpha, double beta, Matrix4d Tstart, double tf, double ts, enumTimeScaling tscale, enumToolPivotTrajType pivotTrajType);
	//��tool�y���ഫ��pivot�y��A��b���Ǭ�x+ x-
	//spiral�y���ơA��^6*n�y��
	static pList N_ToolSpiralTrack(Matrix4d TMtool, Matrix4d Tstart, double scanRange, double lineSpacing, double velocity, double ts, enumTimeScaling tscal);
	static TrajOnceOutput N_ToolSpiralTrackOnce(Matrix4d TMtool, Matrix4d Tstart, double scanRange, double lineSpacing, double velocity, double ts, enumTimeScaling tscal, int iter, int stage, Vector2d r, Vector2d u, double thd0, double thd1, enumSpiralTimeEvaluate spiraltimeEva);
	//static pList N_FA_SpiralTrack(Vector3d pivot, double alpha, double beta, Matrix4d Tstart, double scanRange, double lineSpacing, double velocity, double ts, enumTimeScaling tscal);
	static FADataPinOutput N_FA_SpiralTrack_DataPin(Matrix4d TMtool, Matrix4d Tstart, double scanRange, double lineSpacing, double velocity, double ts, enumTimeScaling tscal, VectorXd remain12, Vector2d r, Vector2d u, double thd0, double thd1, enumSpiralTimeEvaluate spiraltimeEva);


	static pList N_ToolSinTrack(Matrix4d TMtool, Matrix4d Tstart, double velocity, double ts, enumTimeScaling tscal, double lineSpacing, double stepRange, double scanRange, enumSinScanMode scanMode, VectorXd remain12);
	//�ثe�S��input��������A�ѼƤ��൹��
	static TrajOnceOutput N_ToolSinTrackOnece(Matrix4d TMtool, Matrix4d Tstart, double velocity, double ts, enumTimeScaling tscal, double lineSpacing, double stepRange, double scanRange, enumSinScanMode scanMode, int stage, int iter, Vector2d r, Vector2d u, double thd0, double thd1);
	static FADataPinOutput N_FA_SinTrack_DataPin(Matrix4d TMtool, Matrix4d Tstart, double velocity, double ts, enumTimeScaling tscal, double lineSpacing, double stepRange, double scanRange, enumSinScanMode scanMode, VectorXd remain12, Vector2d r, Vector2d u, double thd0, double thd1);


	static pList N_moveL_PP(VectorXd Pstart, VectorXd Pend, enumTimeScaling timescaling, double velocity, double ts);
	static TrajOnceOutput N_moveL_PPOnce(VectorXd Pstart, VectorXd Pend, enumTimeScaling timescaling, double velocity, double ts, int iter, Vector2d r, Vector2d u);
	static FADataPinOutput N_FA_moveL_PP_Data(Matrix4d TMtool, Matrix4d Tstart, VectorXd Pend, enumTimeScaling timescaling, double velocity, double ts, VectorXd remain12, Vector2d r, Vector2d u);

	static WSOutput A12_WS(VectorXd ToolPoseWorking);
	static TrajOnceOutput A12_FA_SpiralTrackOnce(Matrix4d Tstart, double scanRange, double lineSpacing, double velocity, double ts, enumTimeScaling tscal, int iter, int stage, Vector2d r, Vector2d u, double thd0, double thd1, enumSpiralTimeEvaluate spiraltimeEva);
	static FADataPinOutput A12_FA_SpiralTrack_DataPin(Matrix4d Tstart, double scanRange, double lineSpacing, double velocity, double ts, enumTimeScaling tscal, VectorXd remain12, Vector2d r, Vector2d u, double thd0, double thd1, enumSpiralTimeEvaluate spiraltimeEva);
	static TrajOnceOutput A12_FA_SinTrackOnece(Matrix4d Tstart, double velocity, double ts, enumTimeScaling tscal, double lineSpacing, double stepRange, double scanRange, enumSinScanMode scanMode, int stage, int iter, Vector2d r, Vector2d u, double thd0, double thd1);
	static FADataPinOutput A12_FA_SinTrack_DataPin(Matrix4d Tstart, double velocity, double ts, enumTimeScaling tscal, double lineSpacing, double stepRange, double scanRange, enumSinScanMode scanMode, VectorXd remain12, Vector2d r, Vector2d u, double thd0, double thd1);
	static TrajOnceOutput A12_moveL_PPOnce(VectorXd Pstart, VectorXd Pend, enumTimeScaling timescaling, double velocity, double ts, int iter, Vector2d r, Vector2d u);
	static FADataPinOutput A12_moveL_PP_Data(Matrix4d Tstart, VectorXd Pend, enumTimeScaling timescaling, double velocity, double ts, VectorXd remain12, Vector2d r, Vector2d u);

	//�]�w�����ϥμ�������
	static void Set_LightSource();
	//��J�����ӷ���ơA�ϥίu�����
	//�d��
	//double light()
	//{
	//	return 0.55f;
	//}
	//	Stewart_GMT::Set_LightSource(&light);
	//	double light2 = Stewart_GMT::GetPowerMeterData_Wang(y, z, r1, u1);
	static void Set_LightSource(LightSourceFunc source);
	//���o�����������G�Ay z:�y�� r:���������I u:�����b�|
	static double GetPowerMeterData_Wang(double y, double z, Vector2d r, Vector2d u);

	//��FA���Ϊ��y��A�p�GFAScanMode=spiral���ܷ|��scanRange�L��stepRange
	//�p�G�ϥ�Set_LightSource(LightSourceFunc source)�]�w�u������Ar u �|�L��
	static TrajOnceOutput N_FA_TrackOnce(
		Matrix4d TMtool, Matrix4d Tstart, double velocity, double ts, enumTimeScaling tscal,
		double lineSpacing_firstScan, double stepRange_firstScan, double scanRange_firstScan, enumFAScanMode firstScan, 
		double lineSpacing_secondScan, double stepRange2_secondScan, double scanRange2_secondScan, enumFAScanMode secondScan,
		enumSinScanMode scanMode, enumSpiralTimeEvaluate spiraltimeEva, 
		double thd0, double thd1, Vector2d r, Vector2d u,
		int iter, int stage, int fa_stage
	);



	
	static VectorXd pivotLimit(VectorXd pstart, Vector3d pivot, Vector3d w, double theta, double tf, double ts, enumTimeScaling timescaling);
	 
	//pp: �W���x�Ѽ� 
	static void setPP(Mat pp);
	static Mat getPP();
	//bb: �U���x�Ѽ�
	static void setBB(Mat bb);
	static Mat getBB();
	//L: �}�� FK��
	static void setL(VectorXd L);
	static VectorXd getL();
	//H: N�t�C�}��
	static void setH(double h0);
	//G: N�t�C�}��
	static void setG(double g0);
	static double getG();
	//Lx: �����ˬd��
	static void setLx(double Lx);
	static double getLx();

	static void setFKDebugMessage(bool show);



	static pList SpiralTrack_v1(VectorXd PStart, double t, double Ts, double b, double N);

	static VectorXd F_WorkSpace1(VectorXd Pose);//�쪩 �d�����ҥ�
	static VectorXd N_WorkSpace1(VectorXd Pose);//�쪩 �d�����ҥ�
	static VectorXd F_WorkSpace(VectorXd Pose);
	static VectorXd N_WorkSpace(VectorXd Pose);
	
	//screw�y���ơA��^6*n�y��
	static pList ScrewTrack(VectorXd Pstart, double t, double Ts, double b, double zb, double N);

//TODO:�ˬd�Ҧ��ϥάɭ��ˬd���a��N������
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

	//N�t�C����q�����c���b���x�ϥ�
	//n:�����Ƽƶq x:�����ƦV�q fvec:N_myfun��X iflag:��"1"�ɥi�H�פ�hybrid�p��
	//����hybrid(=matlab fsolve)�ѫD�u���p�ߤ�{��
	static void N_myfun(const int *n, const double *x, double *fvec, int *iflag);
	//F�t�C���@�q�����c���b���x�ϥ�
	//n:�����Ƽƶq x:�����ƦV�q fvec:N_myfun��X iflag:��"1"�ɥi�H�פ�hybrid�p��
	//����hybrid(=matlab fsolve)�ѫD�u���p�ߤ�{��
	static void F_myfun(const int *n, const double *x, double *fvec, int *iflag);

	//�p��tool�y���ഫ�U���u�@�Ŷ�
	static WSOutput WorkSpacePivot(VectorXd ToolPoseWorking, Vector3d pivot, double alpha, double beta, enumStewartType stewart_type, enumBaseMode baseMode);
	//��^tool�y���ഫ�L�᪺�W���xpose���ഫ�x�}
	static toolTrans ToolCalculateM(VectorXd Pstart, Vector3d Ppivot, double alpha, double beta, enumBaseMode baseMode);
	//tool�u�@�Ŷ��p�⤤�����j��p��
	//��^��b���t��V��������
	static WSLimitTestOutput ToolWorkSpaceLoop(VectorXd Pose, enumStewartType stewart_type, VectorXd w, double limit_plus, double limit_minus, double interval);
	//tool�u�@�Ŷ��p�⤤pivot�y��j��p��
	//��^��b���t����pivot�B�ʷ���
	static WSLimitTestOutput ToolWorkSpaceLoopPivot(VectorXd Pstart, VectorXd Ppivot, VectorXd w, enumStewartType stewart_type, double limit_plus, double limit_minus, double interval);
	//���ɮį��pivot�y����²�ƪ����A��^pivot�y�����I(���I)
	static VectorXd PivotTrajForCheck(VectorXd Pstart, VectorXd pivot, VectorXd w, double theta);
	//�ˬdlink���׬O�_�X�k
	static bool illegal_lenth(VectorXd PTest, enumStewartType stewart_type);
	static bool illegal_lenth(vector<double> PTest, enumStewartType stewart_type);
	//12�b��
	static bool A12CheckRange(VectorXd P, double range);

	//�p�⦳tool�y���ഫ��pivot�y��
	static pList ToolPivotTrajCal(Vector3d pivot, double alpha, double beta, Matrix4d Tstart, double tf, double ts, enumTimeScaling tscale, enumStewartType stewart_type, enumToolPivotTrajType pivotTrajType);

	//newton_evaluate_function:�����禡������
	//*p:�����禡�Ϊ��ѼƵ��c���� ��:myclass input; &input = *p;
	static double newton(double x0, double tolx, int maxiter, double(*newton_evaluate_function)(double t, void *p), void *p);
	static double spiral_time_eval_f1(double t, void *p);
	static double spiral_time_eval_f2(double t, void *p);

	static double GetPowerMeterData(double y, double z, double y1, double y2, double z1, double z2, Vector2d r, Vector2d u, Vector3d xyz, Vector3d toolTransly, Vector3d toolTranslz);

	


	static VectorXd WorkSpace(VectorXd Pose, enumStewartType stewart_type);
	static VectorXd WorkSpaceLoop(VectorXd Pose, enumStewartType stewart_type, VectorXd w, double limit_plus,double limit_minus, double interval);
	static ToolPivotTrajLoopOutput ToolPivotTrajloop(VectorXd Pstart, VectorXd pivot, VectorXd w, double tf, double ts, enumTimeScaling tscale, enumStewartType stewart_type, double limit_plus, double limit_minus, double interval);


	//12�b�Ϊ��u�@�Ŷ�
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
