#pragma once

#include "StewartBasic.h"


#pragma region define



#define FA_MOVEL_T 1

using LightSourceFunc = double(*)(void);//c++ 11�y�k�A���ĩ� typedef double(*LightSourceFunc)(void);


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

	//��X�y�񵲪G�ο��~�T��(�p�G��)
	struct FATrajctoryResultP :ToolTrajctoryResultP
	{	/*
		�H�U�~�Ӧ�TrajctoryResultP
		VectorXd trajP;//�y���I�A6x1�x�}
		int messageCode = -1;//��X�T���w�d���
		int kNext;
		int stageNext;
		�H�W�~�Ӧ�ToolTrajctoryResultP
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


#pragma region �غc
		
		StewartFA();
		StewartFA(enumStewartType);
		
#pragma endregion

#pragma region �����B�z

		//���ϥΥ���(���P��tool�t�C���G?)
		void SetLightSourceOff();
		//�]�w�����ϥμ�������
		void SetFALightSource(Vector2d center, Vector2d radius);
		//��J�����ӷ���ơA�ϥίu�����
		//�d��
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

#pragma region �y����


	public:
		//�p��Tstart�MTend�������u�ʲ���
		//k�����p�� ceil(t / ts) + 1
		//Matrix4d Tstart, //�y��_�I�A4x4�x�}
		//Matrix4d Tend, //�y����I�A4x4�x�}
		//enumTimeScaling timeScaling, //�y��ɶ��Y��
		//double t, //�y���`�ɶ�
		//double ts, //�y���I�ɶ�����
		//int k //���w�p���k�ӭy���I(1 base)
		FATrajctoryResultP FAMoveLPP(
			Matrix4d Tstart, //�y��_�I�A4x4�x�}
			Matrix4d Tend, //�y����I�A4x4�x�}
			double speed, //
			int k, //���w�p���k�ӭy���I(1 base)
			VectorXd PCurrent_6x1 //�ثe���I�y�СA�Ψӭp�������T
		);

		PLList FAMoveLPP(
			Matrix4d Tstart, //�y��_�I�A4x4�x�}
			Matrix4d Tend, //�y����I�A4x4�x�}
			double speed //
		);

		FATrajctoryResultP FAMoveLPP(
			VectorXd Pstart, //�y��_�I�A1x6
			VectorXd Pend, //�y����I�A1x6
			double speed, //
			int k, //���w�p���k�ӭy���I(1 base)
			VectorXd PCurrent_6x1 //�ثe���I�y�СA�Ψӭp�������T
		);
		PLList FAMoveLPP(
			VectorXd Pstart, //�y��_�I�A1x6
			VectorXd Pend, //�y����I�A1x6
			double speed //
		);

		//�p�����ۭy��A
		FATrajctoryResultP FASpiral(
			Matrix4d Tstart,
			double scanRange,
			double lineSpacing,
			//Vector3d firstDirection,
			//Vector3d secondDirection,
			double speedSpiral, //Spiral�y��t��
			double speedMoveL, //Spiral��moveL�t��
			enumSpiralTEvaluate spiraltimeEva,
			int k,
			int stage,
			VectorXd PCurrent_6x1, //�ثe���I�y�СA�Ψӭp�������T
			enumFAEndPointOption endOption
		);

		//�p�����ۭy��A
		PLList FASpiral(
			Matrix4d Tstart,
			double scanRange,
			double lineSpacing,
			//Vector3d firstDirection,
			//Vector3d secondDirection,
			double speedSpiral, //Spiral�y��t��
			double speedMoveL, //Spiral��moveL�t��
			enumSpiralTEvaluate spiraltimeEva,
			enumFAEndPointOption endOption
		);

		FATrajctoryResultP FASin(
			Matrix4d Tstart, //�y��_�I�A4x4�x�}
			double scanRange, //
			double stepRange,
			double lineSpacing, //�A���FlineSpacing�ٷ|��velocity�v�T
			//Vector3d firstDirection,
			//Vector3d secondDirection,
			double speedSin, //sin�y��t��
			double speedMoveL, //sin�e��moveL�t��
			double frequency,
			enumSinScanOption sinMode, //
			int k, //���w�p���k�ӭy���I(1 base)
			int stage,
			VectorXd PCurrent_6x1, //�ثe���I�y�СA�Ψӭp�������T
			enumFAEndPointOption endOption
		);

		PLList FASin(
			Matrix4d Tstart, //�y��_�I�A4x4�x�}
			double scanRange, //
			double stepRange,
			double lineSpacing, //�A���FlineSpacing�ٷ|��velocity�v�T
			//Vector3d firstDirection,
			//Vector3d secondDirection,
			double speedSin, //sin�y��t��
			double speedMoveL, //sin�e��moveL�t��
			double frequency,
			enumSinScanOption sinMode,
			enumFAEndPointOption endOption
		);

		private:
			ToolTrajctoryResultP FATrajctoryEndOperation(
				Matrix4d Tstart, //�y��_�I�A4x4�x�}
				VectorXd PCurrent_6x1, //�ثe���I�y�СA�Ψӭp�������T
				int k,
				enumFAEndPointOption endOption,
				double speed,
				int stageCode
			);
#pragma endregion


	};
}