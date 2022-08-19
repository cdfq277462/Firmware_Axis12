#pragma once

#include "StewartBasic.h"


#pragma region define



#define FA_MOVEL_T 1

using LightSourceFunc = double(*)(void);//c++ 11語法，等效於 typedef double(*LightSourceFunc)(void);


#pragma endregion


using namespace GMT;


namespace GMT
{


#pragma region enumeration

	typedef enum
	{
		LightSourceNonuse,
		LightFromSimulation,
		LightFromReal
	}enumLightSourceType;

	typedef enum
	{
		FAEndTrajctoryEnd,
		FAEndToStart,
		FAEndToMaxLight,
		FAEndToWeightCenter
	}enumFAEndPointOption;


#pragma endregion enumeration


#pragma region structure

	//輸出軌跡結果及錯誤訊息(如果有)
	struct FATrajctoryResultP :ToolTrajctoryResultP
	{	/*
		以下繼承自TrajctoryResultP
		VectorXd trajP;//軌跡點，6x1矩陣
		int messageCode = -1;//輸出訊息預留欄位
		int kNext;
		int stageNext;
		以上繼承自ToolTrajctoryResultP
		*/
		double lightDataCurr;

		void GetData(TrajctoryResultP trajP)
		{
			this->trajP = trajP.trajP;
			this->messageCode = trajP.messageCode;
			this->kNext = trajP.kNext;
		}

		void GetData(ToolTrajctoryResultP toolP)
		{
			this->trajP = toolP.trajP;
			this->messageCode = toolP.messageCode;
			this->kNext = toolP.kNext;
			this->stageNext = toolP.stageNext;
		}

	};

#pragma endregion structure


#pragma region functional class

	class WeightAvarage
	{
	public:
		double preWeiSum;//前一次權重和
		VectorXd preAve;//前一次權重結果

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

		//把紀錄歸零
		void Reset()
		{
			avg.reset();
			PMax = VectorXd::Zero(6);
			LDataMax = 0;
			messageCode = MESSAGE_FA_BELOW_THD0;
		}

		//注意同時會把紀錄歸零
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

		void PrintResult()
		{
			DEBUG_PRINT_VALUE(messageCode);
			DEBUG_PRINT_VALUE(LDataMax);
			DEBUG_PRINT_VALUE(PMax);
			if (messageCode == MESSAGE_FA_MAX_FOUND)// >=thd1
			{
				DEBUG_PRINT_MESSAGE("max found");
			}
			if (messageCode == MESSAGE_FA_BETWEEN_THD0_THD1)// thd1 >  x >= thd0
			{
				DEBUG_PRINT_MESSAGE("average found");
				DEBUG_PRINT_VALUE(avg.preAve);
			}
			if (messageCode == MESSAGE_FA_BELOW_THD0) // <thd0
			{
				DEBUG_PRINT_MESSAGE("not found");
			}
		}

		void PrintResultD(Vector2d r)
		{

			if (messageCode == MESSAGE_FA_MAX_FOUND)// >=thd1
			{
				double maxy = PMax[1];
				double maxz = PMax[2];
				Vector2d max = { maxy,maxz };
				DEBUG_PRINT_VALUE((r - max).norm());
			}
			if (messageCode == MESSAGE_FA_BETWEEN_THD0_THD1)// thd1 >  x >= thd0
			{
				double avgy = PMax[1];
				double avgz = PMax[2];
				Vector2d avg = { avgy,avgz };
				DEBUG_PRINT_VALUE((r - avg).norm());
			}
		}

	};

	class PLList : public PList
	{
	public:
		vector<double> lLdata;

		void GetData(PList pl)
		{
			this->P = pl.P;
			this->messageCode = pl.messageCode;
		}

		inline void add(vector<double> P, double Ldata) 
		{
			this->P.push_back(P);
			lLdata.push_back(Ldata);
		}

		inline void add(VectorXd p, double Ldata) 
		{
			vector<double> vd(p.data(), p.data() + p.size());
			add(vd, Ldata);
		}

		inline void add(FATrajctoryResultP Pfa)
		{
			VectorXd P = Pfa.trajP;
			vector<double> vd(P.data(), P.data() + P.size());
			add(vd, Pfa.lightDataCurr);
		}

		void show(int i)
		{
			if (i < P.size())
			{
				vector<double> vd = P[i];

				DEBUG_PRINT_MESSAGE(
					i + 1 << ":P = "
					<< vd[0] << ", "
					<< vd[1] << ", "
					<< vd[2] << ", "
					<< vd[3] << ", "
					<< vd[4] << ", "
					<< vd[5] << "  Ldata = "
					<< lLdata[i] << endl
				);
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

		
		
	};

#pragma endregion

	class StewartFA :
		public StewartBasic
	{
	public:


#pragma region 建構
		
		StewartFA();
		StewartFA(enumStewartType);
		
#pragma endregion

#pragma region 光源處理

		//不使用光源(等同於tool系列結果?)
		void SetLightSourceOff();
		//設定光源使用模擬光源
		void SetFALightSource(Vector2d center, Vector2d radius);
		//輸入光源來源函數，使用真實光源
		//範例
		//double light()
		//{
		//	return 0.55f;
		//}
		//	Stewart_GMT::Set_LightSource(&light);
		//	double light2 = Stewart_GMT::GetPowerMeterData_Wang(y, z, r1, u1);
		void SetFALightSource(LightSourceFunc source);
		double GetLightData(VectorXd P);
		void SetFAThreshold(double thd0, double thd1);
		void ClearLdataRecord();
		const LDataLogic GetLdataRecord();

	private:
		enumLightSourceType _lightSourceType;
		Vector2d _simulationLightSourceCenter;
		Vector2d _simulationLightSourceRadius;
		LightSourceFunc _realLightSource;
		LDataLogic _LdataRecord;



		
#pragma endregion

#pragma region 軌跡函數


	public:
		//計算Tstart和Tend之間的線性移動
		//k必須小於 ceil(t / ts) + 1
		//Matrix4d Tstart, //軌跡起點，4x4矩陣
		//Matrix4d Tend, //軌跡終點，4x4矩陣
		//enumTimeScaling timeScaling, //軌跡時間係數
		//double t, //軌跡總時間
		//double ts, //軌跡點時間間格
		//int k //指定計算第k個軌跡點(1 base)
		FATrajctoryResultP FAMoveLPP(
			Matrix4d Tstart, //軌跡起點，4x4矩陣
			Matrix4d Tend, //軌跡終點，4x4矩陣
			double speed, //
			int k, //指定計算第k個軌跡點(1 base)
			VectorXd PCurrent_6x1 //目前的點座標，用來計算光源資訊
		);

		PLList FAMoveLPP(
			Matrix4d Tstart, //軌跡起點，4x4矩陣
			Matrix4d Tend, //軌跡終點，4x4矩陣
			double speed //
		);

		FATrajctoryResultP FAMoveLPP(
			VectorXd Pstart, //軌跡起點，1x6
			VectorXd Pend, //軌跡終點，1x6
			double speed, //
			int k, //指定計算第k個軌跡點(1 base)
			VectorXd PCurrent_6x1 //目前的點座標，用來計算光源資訊
		);
		PLList FAMoveLPP(
			VectorXd Pstart, //軌跡起點，1x6
			VectorXd Pend, //軌跡終點，1x6
			double speed //
		);

		//計算螺旋軌跡，
		FATrajctoryResultP FASpiral(
			Matrix4d Tstart,
			double scanRange,
			double lineSpacing,
			//Vector3d firstDirection,
			//Vector3d secondDirection,
			double speedSpiral, //Spiral軌跡速度
			double speedMoveL, //Spiral後moveL速度
			enumSpiralTEvaluate spiraltimeEva,
			int k,
			int stage,
			VectorXd PCurrent_6x1, //目前的點座標，用來計算光源資訊
			enumFAEndPointOption endOption
		);

		//計算螺旋軌跡，
		PLList FASpiral(
			Matrix4d Tstart,
			double scanRange,
			double lineSpacing,
			//Vector3d firstDirection,
			//Vector3d secondDirection,
			double speedSpiral, //Spiral軌跡速度
			double speedMoveL, //Spiral後moveL速度
			enumSpiralTEvaluate spiraltimeEva,
			enumFAEndPointOption endOption
		);

		FATrajctoryResultP FASin(
			Matrix4d Tstart, //軌跡起點，4x4矩陣
			double scanRange, //
			double stepRange,
			double lineSpacing, //，除了lineSpacing還會受velocity影響
			//Vector3d firstDirection,
			//Vector3d secondDirection,
			double speedSin, //sin軌跡速度
			double speedMoveL, //sin前後moveL速度
			double frequency,
			enumSinScanOption sinMode, //
			int k, //指定計算第k個軌跡點(1 base)
			int stage,
			VectorXd PCurrent_6x1, //目前的點座標，用來計算光源資訊
			enumFAEndPointOption endOption
		);

		PLList FASin(
			Matrix4d Tstart, //軌跡起點，4x4矩陣
			double scanRange, //
			double stepRange,
			double lineSpacing, //，除了lineSpacing還會受velocity影響
			//Vector3d firstDirection,
			//Vector3d secondDirection,
			double speedSin, //sin軌跡速度
			double speedMoveL, //sin前後moveL速度
			double frequency,
			enumSinScanOption sinMode,
			enumFAEndPointOption endOption
		);

		private:
			ToolTrajctoryResultP FATrajctoryEndOperation(
				Matrix4d Tstart, //軌跡起點，4x4矩陣
				VectorXd PCurrent_6x1, //目前的點座標，用來計算光源資訊
				int k,
				enumFAEndPointOption endOption,
				double speed,
				int stageCode
			);
#pragma endregion


	};
}