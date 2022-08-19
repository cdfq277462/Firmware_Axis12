#pragma once

#include <chrono>
#include <ctime>

#include "Trajctory.h"

#pragma region define

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

#define TOOL_TRAJCTORY_END_T 1


#pragma endregion

using namespace GMT;


namespace GMT
{
#pragma region enumeration
	
	typedef enum
	{
		StewartTypeNotInitialed,
		TypeF,
		TypeN,
		TypeA12
	}enumStewartType;

	typedef enum//���~�T��
	{
		Normal, //�@��T��
		Warning,  // ��L���~
		Error //�|�y��crash�����~
	}enumMessageLevel;
	
	typedef enum
	{
		tool,
		base
	}enumWSBaseMode;

#pragma endregion enumeration


#pragma region structure
	//�µ��c��

	struct MessageRecord
	{
		time_t timeStamp;
		enumMessageLevel level;
		int messageCode;
	};
	
	struct IKOutput
	{
		VectorXd alpha;
		VectorXd Ls;
		int messageCode;
		VectorXd g;
		VectorXd LL;
		enumStewartType stewart;
	};
	struct FKOutput
	{
		VectorXd P;
		int messageCode;
	};
		
	struct toolTrans
	{
	public:
		Vector3d xrtool;
		Vector3d yrtool;
		Vector3d zrtool;
		VectorXd poseStart;
		Matrix4d TMtool;
	};

	//ws�p�������X�A�`�N�ˬdinputIllegal�Afalse�ɷ|�^�ǳ̫�@�����`���G
	struct WSOutput
	{
	public:
		//�����p���J�O�_�W�X���x���ʽd��
		int messageCode;
		//�u�@�Ŷ��Ѿl�q�A�YinputIllegal=false�h�^�ǳ̫�@�����`��
		VectorXd remain12;
		//�W���xpose se3�ഫ�x�}�A�YinputIllegal=false�h�^�ǳ̫�@�����`��
		Matrix4d Tstart;
		//�^�ǳ̫�@�����`��input
		VectorXd input;

		Matrix4d TMtool;


	};

	struct WSLimitTestOutput
	{
	public:
		double min;
		double max;
		bool inputIllegal;
		bool scanSuccess;

		WSLimitTestOutput()
		{
			min = 0;
			max = 0;
			inputIllegal = false;
		}
	};

	//��X�y�񵲪G�ο��~�T��(�p�G��)
	struct ToolTrajctoryResultP :TrajctoryResultP
	{	/*	
		�H�U�~�Ӧ�TrajctoryResultP
		VectorXd trajP;//�y���I�A6x1�x�}
		int messageCode = -1;//��X�T���w�d���
		int kNext;
		�H�W�~�Ӧ�TrajctoryResultP	
		*/ 
		int stageNext;
			   
	};
	
	struct StewartParameters
	{
		enumStewartType type;
		MatrixXd pp;
		MatrixXd bb;
	};

#pragma endregion structure
	
#pragma region functional class
	
	class MessageRecorder
	{
		vector<MessageRecord> _records;
		int _recorderLimit;

	public:
		MessageRecorder(int limit);
		void Add(enumMessageLevel level, int message);
		const vector<MessageRecord> GetRecords();
		const int GetCounts();
		void Clear();
	};
	
	//TODO:���eigen matrixXd?
	//���Ȯɧ⤶���令eigen vector�A���ŦA�^�Ӿ�ӧﱼ
	class PList {
	public:

		vector< vector<double> > P;
		int messageCode;

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

		inline void append(PList P2)
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
					i + 1 << ":"
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

		VectorXd get(int i)
		{
			if (i < P.size())
				return MathTool::stdVectortoVectorXD(P[i]);
			else
				return VectorXd();
		}

		//vector<double> getRow(int i)
		//{
		//	if (i >= P[0].size()) return vector<double>();

		//	int n = P.size();
		//	vector<double> row;
		//	row.reserve(n);

		//	for (int j = 0; j < n; j++)
		//	{
		//		vector<double> pj = P[j];
		//		row.push_back(pj[i]);
		//	}

		//	return row;
		//}

		void removeLast()
		{
			P.resize(P.size() - 1);
		}

		VectorXd end()
		{
			
			if (P.size() > 0)return MathTool::stdVectortoVectorXD(P[P.size() - 1]);
			else return VectorXd();
		}

		PList()
		{};
		~PList() {
			// vector has its own destructor for this
		};
		//pList(const pList & plist) {
		//	P = plist.P;
		//};
	};

#pragma endregion functional class

	   


	class StewartBasic :
		protected Trajctory
	{
		
#pragma region �غc

	public:
		//�����w�����غc(�L�k�ϥ�)
		StewartBasic();
		//���w�����غc
		StewartBasic(enumStewartType type);
		//
		void SetStewartType(enumStewartType type);
		//���o�ثe�]�w������
		const enumStewartType GetStewartType();
		//�]�w�y�����W�v
		void SetTrajctoryTs(double ts);
		//�]�w�ثe�]�w���y�����W�v
		double GetTs();
		//�]�w�y��ɶ��Y��
		void SetTrajctoryTimeScaling(enumTimeScaling timeScale);
		//�]�w�ثe�]�w���y��ɶ��Y��
		enumTimeScaling GetTimeScaling();

		const StewartParameters GetSetting();
			   
	private:
		//Stewart����
		bool CheckStewartType();

#pragma region �u�@�Ŷ�&�u�@�d���ˬd

	public:
		//***********************�u�@�Ŷ�����****************************************
		//�`�NGetStewartType() SetToolCoordinate()
		WSOutput ToolWS(VectorXd PToolStart_1x6, enumWSBaseMode baseMode);
		//�O�l���|�p��WS�A���O�l�]�w�ΡA�Y�b�����w�p��Lws�h�|�۰ʰO�����γ]�w
		void SetToolTrans(Matrix4d Tstart, Matrix4d TMTool, VectorXd wsRemain12_1x12);
		//�]�w�u��y��
		void SetToolCoordinate(Vector3d pivot, double alpha, double beta);
		//�]�w�u�@�Ŷ��p�⥭���Mpivot�j��ϥΪ�delta
		void SetWSdelta(double trans, double rot);
		//���]tool�ಾ�x�}��phome
		void ResetTposeStart();
		static bool CheckL(VectorXd L_1x6 , double lLow, double lUpp);
		//�ˬdlink 6x1�O�_�X�k
		//�ϥΤ��ذѼƳ]�w�ˬd
		bool CheckL(VectorXd L_1x6);
		//�ˬdPose 6x1�O�_�X�k
		//�ϥΤ��ذѼƳ]�w�ˬd
		bool CheckP(VectorXd P_1x6);

	private:
		double _ts; 
		enumTimeScaling _timeScale;
		static Mat _pp;
		static Mat _bb;
		enumStewartType _stewartType;
		static VectorXd _L_1x6;
		VectorXd _initGuess_1x6;

		minpack_func_nn _fkCallback;
		double _Lx;
		double _Llow;
		double _Lupp;
		Vector3d _pivot;
		double _alpha;
		double _beta;
		double _workSpaceDeltaTrans;
		double _workSpaceDeltaRot;
		VectorXd _PHome_1x6;
		Matrix4d _TposeStart;
		Matrix4d _TMTool;
		VectorXd _WSRemain12_1x12;
		bool _firstRun;
		WSOutput _WSLastLegalResult;
		// [0] = x+	// [1] = x- 
		// [2] = y+	// [3] = y- 
		// [4] = z+	// [5] = z- 
		// [6] = phi+ 	// [7] = phi- 
		// [8] = theta+ 	// [9] = theta- 
		// [10] = psi+	// [11] = psi-
		vector<double> _workSpaceTestLimit;

		//FK��Jlink��pose
		FKOutput CminpackHybrdFK(VectorXd L, VectorXd ini_pose);

		//�u�@�Ŷ�
		WSOutput WorkSpacePivot(VectorXd PToolDelta, enumWSBaseMode baseMode);
		//��^tool�y���ഫ�L�᪺�W���xpose���ഫ�x�}
		toolTrans ToolCalculateM(VectorXd PToolDelta, enumWSBaseMode baseMode);	//tool�u�@�Ŷ��p�⤤�����j��p��
		//��^��b���t��V��������
		WSLimitTestOutput ToolWSTrans(VectorXd P, Vector3d axis3, double limit_plus, double limit_minus);
		//tool�u�@�Ŷ��p�⤤pivot�y��j��p��
		//��^��b���t����pivot�B�ʷ���
		WSLimitTestOutput ToolWSPivot(VectorXd P, VectorXd axis3, double limit_plus, double limit_minus);
		//���ɮį��pivot�y����²�ƪ����A��^pivot�y�����I(���I)
		VectorXd PivotTrajForCheck(VectorXd Pstart, VectorXd pivot, VectorXd w, double theta);

#pragma endregion

#pragma region �y����
	public:
		//*******************���]�t�O�_�W�X�����i�B��d��P�_���y����*********************************

		//�p��Tstart�MTend�������u�ʲ���
		//k�����p�� ceil(t / ts) + 1
		//Matrix4d Tstart, //�y��_�I�A4x4�x�}
		//Matrix4d Tend, //�y����I�A4x4�x�}
		//enumTimeScaling timeScaling, //�y��ɶ��Y��
		//double t, //�y���`�ɶ�
		//double ts, //�y���I�ɶ�����
		//int k //���w�p���k�ӭy���I(1 base)
		TrajctoryResultP ToolMoveLPP(
			Matrix4d Tstart, //�y��_�I�A4x4�x�}
			Matrix4d Tend, //�y����I�A4x4�x�}
			double speed, //
			int k //���w�p���k�ӭy���I(1 base)
		);
		PList ToolMoveLPP(
			Matrix4d Tstart, //�y��_�I�A4x4�x�}
			Matrix4d Tend, //�y����I�A4x4�x�}
			double speed //
		);

		TrajctoryResultP ToolMoveLPP(
			VectorXd Pstart, //�y��_�I�A1x6
			VectorXd Pend, //�y����I�A1x6
			double speed, //
			int k //���w�p���k�ӭy���I(1 base)
		);
		PList ToolMoveLPP(
			VectorXd Pstart, //�y��_�I�A1x6
			VectorXd Pend, //�y����I�A1x6
			double speed //
		);


		//�p�����ۭy��A
		ToolTrajctoryResultP ToolSpiral(
			Matrix4d Tstart, 
			double scanRange, 
			double lineSpacing,
			//Vector3d firstDirection,
			//Vector3d secondDirection,
			double speedSpiral, //Spiral�y��t��
			double speedMoveL, //Spiral��moveL�t��
			enumSpiralTEvaluate spiraltimeEva,
			int k, 
			int stage
		);
		
		//�p�����ۭy��A
		PList ToolSpiral(
			Matrix4d Tstart,
			double scanRange,
			double lineSpacing,
			//Vector3d firstDirection,
			//Vector3d secondDirection,
			double speedSpiral, //Spiral�y��t��
			double speedMoveL, //Spiral��moveL�t��
			enumSpiralTEvaluate spiraltimeEva,
			bool backToStart
		);

		ToolTrajctoryResultP ToolSin(
			Matrix4d Tstart, //�y��_�I�A4x4�x�}
			double scanRange, //
			double stepRange,
			double lineSpacing, //�A���FlineSpacing�ٷ|��velocity�v�T
			//Vector3d firstDirection,
			//Vector3d secondDirection,
			double speedSin, //sin�y��t��
			double speedMoveL, //sin�e��moveL�t��
			double frequency, //
			enumSinScanOption sinMode, //
			int k, //���w�p���k�ӭy���I(1 base)
			int stage
		);

		PList ToolSin(
			Matrix4d Tstart, //�y��_�I�A4x4�x�}
			double scanRange, //
			double stepRange,
			double lineSpacing, //�A���FlineSpacing�ٷ|��velocity�v�T
			//Vector3d firstDirection,
			//Vector3d secondDirection,
			double speedSin, //sin�y��t��
			double speedMoveL, //sin�e��moveL�t��
			double frequency, //
			enumSinScanOption sinMode,
			bool backToStart
		);
	
	protected:
		Matrix4d _TTrajctoryEnd;
		
	private:
		TrajctoryResultP CalculateSinStart(VectorXd P_1x6, double scan, double step, enumSinScanOption mode, Vector3d toolTransY, Vector3d toolTransZ);

#pragma endregion 

#pragma region N type

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
		static IKOutput N_IK(VectorXd P, Mat pp, Mat bb, double h0, double g0);
		//��J�W���xpose�A�ϥΤw�]�w��pp bb h0 g0�A�f��link����
		//��J :
		//VectorXd P(6) : �W���x��epose�A1 * 6 double�V�q
		//��X :
		//NIKOutput.alpha : (? )
		//NIKOutput.ls : �U�b�qlink���סA1 * 6 double�V�q
		IKOutput N_IK(VectorXd P);
		//N�t�C����q�����c���b���x�ϥ�
		//x:�j�p��6���}����l�q���V�q
		//�ϥ�minpack��hybrid�ӹ�{matlab fsolve�ۦP�ѫD�u���p�ߤ�{�\�� 
		//��^1*6 ���xpose
		FKOutput N_FK(VectorXd L, VectorXd initGuess);
		FKOutput N_FK(VectorXd L);

	private:
		static double _h0, _g0;	

		//pp bb h0 g0 �a�J N�t�C�w�]��
		void SetStewartNTypeDefaultParameters();
		//N�t�C����q�����c���b���x�ϥ�
		//n:�����Ƽƶq x:�����ƦV�q fvec:N_myfun��X iflag:��"1"�ɥi�H�פ�hybrid�p��
		//����hybrid(=matlab fsolve)�ѫD�u���p�ߤ�{��
		static void N_FKCallback(const int *n, const double *x, double *fvec, int *iflag);//TODO:�����D���S����k����static


#pragma endregion N type

#pragma region F type

	public:

		//F�t�C���@�q�����c���b���x�ϥ�
		//P:�W���xpose pp:�U���x�Ѽ� bb:�W���x�Ѽ�
		static IKOutput F_IK(VectorXd P, Mat pp, Mat bb);
		//P:�W���xpose �ϥΤw�]�w��pp bb
		IKOutput F_IK(VectorXd P);

		FKOutput F_FK(VectorXd L, VectorXd initGuess);
		FKOutput F_FK(VectorXd L);
		
	private:
		void SetStewartFTypeDefaultParameters();
		//F�t�C���@�q�����c���b���x�ϥ�
	//n:�����Ƽƶq x:�����ƦV�q fvec:N_myfun��X iflag:��"1"�ɥi�H�פ�hybrid�p��
	//����hybrid(=matlab fsolve)�ѫD�u���p�ߤ�{��
		static void F_FKCallback(const int *n, const double *x, double *fvec, int *iflag);//TODO:�����D���S����k����static

#pragma endregion F type
			   
#pragma region 12�b

	public:

	private:
		void SetStewartA12DefaultParameters();
		WSOutput A12_ToolWS(VectorXd PToolDelta, enumWSBaseMode baseMode);
		bool A12_checkP(VectorXd P);

#pragma endregion 12�b



	};

}