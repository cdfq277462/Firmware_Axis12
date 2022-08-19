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

	
	typedef enum//���wspiral�p�����T���覡
	{
		ConstV,//���t
		ConstA //�����t��
	}enumSpiralTEvaluate;

	typedef enum
	{
		vertical,
		horizontal,
		Both
	}enumSinScanOption;

#pragma endregion

	//��X�y�񵲪G�ο��~�T��(�p�G��)
	struct TrajctoryResultT
	{
		Matrix4d trajT;//�y���I�A4x4�x�}
		int messageCode = -1;//��X�T���w�d���
	};	
	//��X�y�񵲪G�ο��~�T��(�p�G��)
	struct TrajctoryResultP
	{
		VectorXd trajP;//�y���I�A6x1�x�}
		int messageCode = -1;//��X�T���w�d���
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


	//�y���ƶ��X�A�~��MathTool
	class Trajctory :
		protected MathTool
	{
	public:
		//�p��Tstart�MTend�������u�ʲ���
		//k�����p�� ceil(t / ts) + 1
		//Matrix4d Tstart, //�y��_�I�A4x4�x�}
		//Matrix4d Tend, //�y����I�A4x4�x�}
		//enumTimeScaling timeScaling, //�y��ɶ��Y��
		//double t, //�y���`�ɶ�
		//double ts, //�y���I�ɶ�����
		//int k //���w�p���k�ӭy���I(1 base)
		static TrajctoryResultP MoveLPPK(
			Matrix4d Tstart, //�y��_�I�A4x4�x�}
			Matrix4d Tend, //�y����I�A4x4�x�}
			enumTimeScaling timeScaling, //�y��ɶ��Y��
			double t, //�y���`�ɶ�
			double ts, //�y���I�ɶ�����
			int k //���w�p���k�ӭy���I(1 base)
		);

		//�p��Tstart���_�I�����ۭy��
		//Matrix4d Tstart, //�y��_�I�A4x4�x�}
		//double scanRange, //���y�d�򭭨�A�HTstart������
		//double lineSpacing, //�y���P�餧�����e�סA���FlineSpacing�ٷ|��velocity�v�T
		//double velocity, //�y��t�סA�v�T�`�I�ƩM�餧���e��
		//double ts, //�y���I�ɶ�����
		//enumTimeScaling tscal, //�y��ɶ��Y��
		//enumSpiralTEvaluate spiralTEva, //���w�ɶ��ӼƵ����覡
		//int k //���w�p���k�ӭy���I(1 base)
		static TrajctoryResultP SpiralK(
			Matrix4d Tstart, //�y��_�I�A4x4�x�}
			double scanRange, //���y�d�򭭨�A�HTstart������
			double lineSpacing, //�y���P�餧�����e�סA���FlineSpacing�ٷ|��velocity�v�T
			Vector3d firstDirection,
			Vector3d secondDirection,
			double velocity, //�y��t�סA�v�T�`�I�ƩM�餧���e��
			double ts, //�y���I�ɶ�����
			enumTimeScaling tscal, //�y��ɶ��Y��
			enumSpiralTEvaluate spiralTEva, //���w�ɶ��ӼƵ����覡
			int k //���w�p���k�ӭy���I(1 base)
		);

		static TrajctoryResultP SinK(
			Matrix4d Tstart, //�y��_�I�A4x4�x�}
			double scanRange, //
			double stepRange,
			double lineSpacing, //�A���FlineSpacing�ٷ|��velocity�v�T
			Vector3d firstDirection,
			Vector3d secondDirection,
			double velocity, //�y��t��
			double ts, //�y���I�ɶ�����
			enumTimeScaling tscal, //�y��ɶ��Y��
			double frequency,
			enumSinScanOption sinMode, //
			int k //���w�p���k�ӭy���I(1 base)
		);

	private:

		static double newton(double x0, double tolx, int maxiter, SpiralNewtonEvaluateFunction spiralEva, void *p);//TODO:�p�⦡���ץ���������A�Ѧ�newton.m
		static double spiral_time_eval_f1(double t, void *p);
		static double spiral_time_eval_f2(double t, void *p);

	};




}
