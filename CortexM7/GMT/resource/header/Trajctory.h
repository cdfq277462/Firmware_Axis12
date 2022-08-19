#pragma once

#include "MathTool.h"


#pragma region define

#define TimeScalingCoefficientCubic 3.f/2.f
#define TimeScalingCoefficientQuintic 15.f/8.f
#define SpiralSpeedCoefficient 1e-4
#define SIN_FREQUENCY_COEFFICIENT 0.1

typedef double(*SpiralNewtonEvaluateFunction)(double t, void *p);


#pragma endregion


namespace GMT
{

#pragma region enumeration

	
	typedef enum//指定spiral計算評估T的方式
	{
		ConstV,//等速
		ConstA //等角速度
	}enumSpiralTEvaluate;

	typedef enum
	{
		vertical,
		horizontal,
		Both
	}enumSinScanOption;

#pragma endregion

	//輸出軌跡結果及錯誤訊息(如果有)
	struct TrajctoryResultT
	{
		Matrix4d trajT;//軌跡點，4x4矩陣
		int messageCode = -1;//輸出訊息預留欄位
	};	
	//輸出軌跡結果及錯誤訊息(如果有)
	struct TrajctoryResultP
	{
		VectorXd trajP;//軌跡點，6x1矩陣
		int messageCode = -1;//輸出訊息預留欄位
		int kNext;
	};

	struct SpiralTEvaluateParameter
	{
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


	//軌跡函數集合，繼承MathTool
	class Trajctory :
		protected MathTool
	{
	public:
		//計算Tstart和Tend之間的線性移動
		//k必須小於 ceil(t / ts) + 1
		//Matrix4d Tstart, //軌跡起點，4x4矩陣
		//Matrix4d Tend, //軌跡終點，4x4矩陣
		//enumTimeScaling timeScaling, //軌跡時間係數
		//double t, //軌跡總時間
		//double ts, //軌跡點時間間格
		//int k //指定計算第k個軌跡點(1 base)
		static TrajctoryResultP MoveLPPK(
			Matrix4d Tstart, //軌跡起點，4x4矩陣
			Matrix4d Tend, //軌跡終點，4x4矩陣
			enumTimeScaling timeScaling, //軌跡時間係數
			double t, //軌跡總時間
			double ts, //軌跡點時間間格
			int k //指定計算第k個軌跡點(1 base)
		);

		//計算Tstart為起點的螺旋軌跡
		//Matrix4d Tstart, //軌跡起點，4x4矩陣
		//double scanRange, //掃描範圍限制，以Tstart為中心
		//double lineSpacing, //軌跡圈與圈之間的寬度，除了lineSpacing還會受velocity影響
		//double velocity, //軌跡速度，影響總點數和圈之間寬度
		//double ts, //軌跡點時間間格
		//enumTimeScaling tscal, //軌跡時間係數
		//enumSpiralTEvaluate spiralTEva, //指定時間細數評估方式
		//int k //指定計算第k個軌跡點(1 base)
		static TrajctoryResultP SpiralK(
			Matrix4d Tstart, //軌跡起點，4x4矩陣
			double scanRange, //掃描範圍限制，以Tstart為中心
			double lineSpacing, //軌跡圈與圈之間的寬度，除了lineSpacing還會受velocity影響
			Vector3d firstDirection,
			Vector3d secondDirection,
			double velocity, //軌跡速度，影響總點數和圈之間寬度
			double ts, //軌跡點時間間格
			enumTimeScaling tscal, //軌跡時間係數
			enumSpiralTEvaluate spiralTEva, //指定時間細數評估方式
			int k //指定計算第k個軌跡點(1 base)
		);

		static TrajctoryResultP SinK(
			Matrix4d Tstart, //軌跡起點，4x4矩陣
			double scanRange, //
			double stepRange,
			double lineSpacing, //，除了lineSpacing還會受velocity影響
			Vector3d firstDirection,
			Vector3d secondDirection,
			double velocity, //軌跡速度
			double ts, //軌跡點時間間格
			enumTimeScaling tscal, //軌跡時間係數
			double frequency,
			enumSinScanOption sinMode, //
			int k //指定計算第k個軌跡點(1 base)
		);

	private:

		static double newton(double x0, double tolx, int maxiter, SpiralNewtonEvaluateFunction spiralEva, void *p);//TODO:計算式須修正結束條件，參考newton.m
		static double spiral_time_eval_f1(double t, void *p);
		static double spiral_time_eval_f2(double t, void *p);

	};




}
