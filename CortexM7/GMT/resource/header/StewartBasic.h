#pragma once

#include <chrono>
#include <ctime>

#include "Trajctory.h"

#pragma region define

//TODO:檢查所有使用界限檢查的地方代換巨集
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

	typedef enum//錯誤訊息
	{
		Normal, //一般訊息
		Warning,  // 其他錯誤
		Error //會造成crash的錯誤
	}enumMessageLevel;
	
	typedef enum
	{
		tool,
		base
	}enumWSBaseMode;

#pragma endregion enumeration


#pragma region structure
	//純結構類

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

	//ws計算相關輸出，注意檢查inputIllegal，false時會回傳最後一次正常結果
	struct WSOutput
	{
	public:
		//本次計算輸入是否超出平台活動範圍
		int messageCode;
		//工作空間剩餘量，若inputIllegal=false則回傳最後一次正常的
		VectorXd remain12;
		//上平台pose se3轉換矩陣，若inputIllegal=false則回傳最後一次正常的
		Matrix4d Tstart;
		//回傳最後一次正常的input
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

	//輸出軌跡結果及錯誤訊息(如果有)
	struct ToolTrajctoryResultP :TrajctoryResultP
	{	/*	
		以下繼承自TrajctoryResultP
		VectorXd trajP;//軌跡點，6x1矩陣
		int messageCode = -1;//輸出訊息預留欄位
		int kNext;
		以上繼承自TrajctoryResultP	
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
	
	//TODO:改用eigen matrixXd?
	//先暫時把介面改成eigen vector，有空再回來整個改掉
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
		
#pragma region 建構

	public:
		//不指定機型建構(無法使用)
		StewartBasic();
		//指定機型建構
		StewartBasic(enumStewartType type);
		//
		void SetStewartType(enumStewartType type);
		//取得目前設定的機型
		const enumStewartType GetStewartType();
		//設定軌跡函數頻率
		void SetTrajctoryTs(double ts);
		//設定目前設定的軌跡函數頻率
		double GetTs();
		//設定軌跡時間係數
		void SetTrajctoryTimeScaling(enumTimeScaling timeScale);
		//設定目前設定的軌跡時間係數
		enumTimeScaling GetTimeScaling();

		const StewartParameters GetSetting();
			   
	private:
		//Stewart類型
		bool CheckStewartType();

#pragma region 工作空間&工作範圍檢查

	public:
		//***********************工作空間相關****************************************
		//注意GetStewartType() SetToolCoordinate()
		WSOutput ToolWS(VectorXd PToolStart_1x6, enumWSBaseMode baseMode);
		//板子不會計算WS，給板子設定用，若在本機已計算過ws則會自動記錄不用設定
		void SetToolTrans(Matrix4d Tstart, Matrix4d TMTool, VectorXd wsRemain12_1x12);
		//設定工具座標
		void SetToolCoordinate(Vector3d pivot, double alpha, double beta);
		//設定工作空間計算平移和pivot迴圈使用的delta
		void SetWSdelta(double trans, double rot);
		//重設tool轉移矩陣為phome
		void ResetTposeStart();
		static bool CheckL(VectorXd L_1x6 , double lLow, double lUpp);
		//檢查link 6x1是否合法
		//使用內建參數設定檢查
		bool CheckL(VectorXd L_1x6);
		//檢查Pose 6x1是否合法
		//使用內建參數設定檢查
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

		//FK輸入link算pose
		FKOutput CminpackHybrdFK(VectorXd L, VectorXd ini_pose);

		//工作空間
		WSOutput WorkSpacePivot(VectorXd PToolDelta, enumWSBaseMode baseMode);
		//返回tool座標轉換過後的上平台pose及轉換矩陣
		toolTrans ToolCalculateM(VectorXd PToolDelta, enumWSBaseMode baseMode);	//tool工作空間計算中平移迴圈計算
		//返回單軸正負方向平移極限
		WSLimitTestOutput ToolWSTrans(VectorXd P, Vector3d axis3, double limit_plus, double limit_minus);
		//tool工作空間計算中pivot軌跡迴圈計算
		//返回單軸正負角度pivot運動極限
		WSLimitTestOutput ToolWSPivot(VectorXd P, VectorXd axis3, double limit_plus, double limit_minus);
		//提升效能用pivot軌跡函數簡化版本，返回pivot軌跡折返點(端點)
		VectorXd PivotTrajForCheck(VectorXd Pstart, VectorXd pivot, VectorXd w, double theta);

#pragma endregion

#pragma region 軌跡函數
	public:
		//*******************有包含是否超出機器可運行範圍判斷的軌跡函數*********************************

		//計算Tstart和Tend之間的線性移動
		//k必須小於 ceil(t / ts) + 1
		//Matrix4d Tstart, //軌跡起點，4x4矩陣
		//Matrix4d Tend, //軌跡終點，4x4矩陣
		//enumTimeScaling timeScaling, //軌跡時間係數
		//double t, //軌跡總時間
		//double ts, //軌跡點時間間格
		//int k //指定計算第k個軌跡點(1 base)
		TrajctoryResultP ToolMoveLPP(
			Matrix4d Tstart, //軌跡起點，4x4矩陣
			Matrix4d Tend, //軌跡終點，4x4矩陣
			double speed, //
			int k //指定計算第k個軌跡點(1 base)
		);
		PList ToolMoveLPP(
			Matrix4d Tstart, //軌跡起點，4x4矩陣
			Matrix4d Tend, //軌跡終點，4x4矩陣
			double speed //
		);

		TrajctoryResultP ToolMoveLPP(
			VectorXd Pstart, //軌跡起點，1x6
			VectorXd Pend, //軌跡終點，1x6
			double speed, //
			int k //指定計算第k個軌跡點(1 base)
		);
		PList ToolMoveLPP(
			VectorXd Pstart, //軌跡起點，1x6
			VectorXd Pend, //軌跡終點，1x6
			double speed //
		);


		//計算螺旋軌跡，
		ToolTrajctoryResultP ToolSpiral(
			Matrix4d Tstart, 
			double scanRange, 
			double lineSpacing,
			//Vector3d firstDirection,
			//Vector3d secondDirection,
			double speedSpiral, //Spiral軌跡速度
			double speedMoveL, //Spiral後moveL速度
			enumSpiralTEvaluate spiraltimeEva,
			int k, 
			int stage
		);
		
		//計算螺旋軌跡，
		PList ToolSpiral(
			Matrix4d Tstart,
			double scanRange,
			double lineSpacing,
			//Vector3d firstDirection,
			//Vector3d secondDirection,
			double speedSpiral, //Spiral軌跡速度
			double speedMoveL, //Spiral後moveL速度
			enumSpiralTEvaluate spiraltimeEva,
			bool backToStart
		);

		ToolTrajctoryResultP ToolSin(
			Matrix4d Tstart, //軌跡起點，4x4矩陣
			double scanRange, //
			double stepRange,
			double lineSpacing, //，除了lineSpacing還會受velocity影響
			//Vector3d firstDirection,
			//Vector3d secondDirection,
			double speedSin, //sin軌跡速度
			double speedMoveL, //sin前後moveL速度
			double frequency, //
			enumSinScanOption sinMode, //
			int k, //指定計算第k個軌跡點(1 base)
			int stage
		);

		PList ToolSin(
			Matrix4d Tstart, //軌跡起點，4x4矩陣
			double scanRange, //
			double stepRange,
			double lineSpacing, //，除了lineSpacing還會受velocity影響
			//Vector3d firstDirection,
			//Vector3d secondDirection,
			double speedSin, //sin軌跡速度
			double speedMoveL, //sin前後moveL速度
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
		//輸入上平台pose，逆推link長度
		//輸入 :
		//VectorXd P(6) : 上平台當前pose，1 * 6 double向量
		//Mat pp(3, 6) : 上平台參數，3 * 6 double矩陣，參見stewart規格
		//Mat bb(3, 6) : 下平台參數，3 * 6 double矩陣，參見stewart規格
		//double h0 : 上半段link長度(固定)，參見stewart規格
		//double g0 : 下半段link長度(? )
		//輸出 :
		//NIKOutput.alpha : (? )
		//NIKOutput.ls : 下半段link長度1 * 6 double向量
		static IKOutput N_IK(VectorXd P, Mat pp, Mat bb, double h0, double g0);
		//輸入上平台pose，使用已設定的pp bb h0 g0，逆推link長度
		//輸入 :
		//VectorXd P(6) : 上平台當前pose，1 * 6 double向量
		//輸出 :
		//NIKOutput.alpha : (? )
		//NIKOutput.ls : 下半段link長度，1 * 6 double向量
		IKOutput N_IK(VectorXd P);
		//N系列為兩段式結構六軸平台使用
		//x:大小為6的腳長初始猜測向量
		//使用minpack的hybrid來實現matlab fsolve相同解非線性聯立方程功能 
		//返回1*6 平台pose
		FKOutput N_FK(VectorXd L, VectorXd initGuess);
		FKOutput N_FK(VectorXd L);

	private:
		static double _h0, _g0;	

		//pp bb h0 g0 帶入 N系列預設值
		void SetStewartNTypeDefaultParameters();
		//N系列為兩段式結構六軸平台使用
		//n:未知數數量 x:未知數向量 fvec:N_myfun輸出 iflag:給"1"時可以終止hybrid計算
		//提供hybrid(=matlab fsolve)解非線性聯立方程用
		static void N_FKCallback(const int *n, const double *x, double *fvec, int *iflag);//TODO:不知道有沒有辦法不用static


#pragma endregion N type

#pragma region F type

	public:

		//F系列為一段式結構六軸平台使用
		//P:上平台pose pp:下平台參數 bb:上平台參數
		static IKOutput F_IK(VectorXd P, Mat pp, Mat bb);
		//P:上平台pose 使用已設定的pp bb
		IKOutput F_IK(VectorXd P);

		FKOutput F_FK(VectorXd L, VectorXd initGuess);
		FKOutput F_FK(VectorXd L);
		
	private:
		void SetStewartFTypeDefaultParameters();
		//F系列為一段式結構六軸平台使用
	//n:未知數數量 x:未知數向量 fvec:N_myfun輸出 iflag:給"1"時可以終止hybrid計算
	//提供hybrid(=matlab fsolve)解非線性聯立方程用
		static void F_FKCallback(const int *n, const double *x, double *fvec, int *iflag);//TODO:不知道有沒有辦法不用static

#pragma endregion F type
			   
#pragma region 12軸

	public:

	private:
		void SetStewartA12DefaultParameters();
		WSOutput A12_ToolWS(VectorXd PToolDelta, enumWSBaseMode baseMode);
		bool A12_checkP(VectorXd P);

#pragma endregion 12軸



	};

}