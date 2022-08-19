#include "..\header\Trajctory.h"

using namespace GMT;

TrajctoryResultP GMT::Trajctory::MoveLPPK(
	Matrix4d Tstart, Matrix4d Tend, 
	enumTimeScaling timeScaling, double t, double ts, int k)
{
	TrajctoryResultP out;
	int n = ceil(t / ts) + 1;
	
	if (isnan((double)n) || n < 0)//TODO:補充檢查說明
	{
		DEBUG_PRINT_CODE_LOCATION;
		DEBUG_PRINT_ERROR_MESSAGE("n = nan");
		DEBUG_PRINT_VALUE(t);
		DEBUG_PRINT_VALUE(ts);
				
		out.messageCode = MESSAGE_TRAJCTORY_N_IS_NAN;
		return out;
	}
	
	if (k > n) //TODO:補充檢查說明
	{
		DEBUG_PRINT_CODE_LOCATION;
		DEBUG_PRINT_ERROR_MESSAGE("k out of range");
		DEBUG_PRINT_VALUE(n);
		DEBUG_PRINT_VALUE(k);

		out.messageCode = MESSAGE_TRAJCTORY_K_OUT_OF_RANGE;
		return out;
	}


	
	Matrix3d rstart = t2r(Tstart);
	Vector3d pstart = t2p(Tstart);
	Matrix3d rend = t2r(Tend);
	Vector3d pend = t2p(Tend);

	Mat tmat = MatrixLog3(rstart.adjoint()*rend);
	//DEBUG_PRINT_VALUE(tmat);
	double s;
	if (timeScaling == enumTimeScaling::Cubic)
	{
		s = CubicTimeScaling(t, ts*(k - 1));
	}
	else
		s = QuinticTimeScaling(t, ts*(k - 1));

	Matrix3d ri = rstart * MatrixExp3(tmat*s);
	Vector3d pi = pstart + s * (pend - pstart);
	Matrix4d Ttraji = rp2t(ri, pi);

	out.trajP = sE3ToPoseRPY(Ttraji);
	if (k == n)
	{
		out.messageCode = MESSAGE_TRAJCTORY_NORMAL_END;
		out.kNext = k;
	}
	else
	{
		out.messageCode = MESSAGE_OK;
		out.kNext = k + 1;
	}

	return out;
}

TrajctoryResultP GMT::Trajctory::SpiralK(
	Matrix4d Tstart, double scanRange, double lineSpacing,	Vector3d firstDirection,	Vector3d secondDirection,
	double velocity, double ts, enumTimeScaling tscal, enumSpiralTEvaluate spiralTEva, int k)
{
	VectorXd Pstart = sE3ToPoseRPY(Tstart);	

	TrajctoryResultP out;

	double b = lineSpacing / (2 * MathPI);
	double nr = scanRange / lineSpacing;
	double tf = 2 * MathPI * nr;
	double tsv = ts * velocity;
	int K = ceil(tf / tsv) + 1;
	if (spiralTEva == ConstV) K *= 10;
		
	if (k > K || k == 0)
	{
		DEBUG_PRINT_CODE_LOCATION;
		DEBUG_PRINT_ERROR_MESSAGE("k out of range");
		DEBUG_PRINT_VALUE(K);
		DEBUG_PRINT_VALUE(k);

		out.messageCode = MESSAGE_TRAJCTORY_K_OUT_OF_RANGE;
		return out;
	}

	double t;
	if (spiralTEva == ConstV)
	{
		double x0 = 0.1;
		SpiralTEvaluateParameter sp2;
		sp2.setsp2(b, k);
		t = newton(x0, 1e-5, 50, &spiral_time_eval_f2, &sp2);
		if (t > tsv)
		{
			t = tsv;
		}

	}
	else if (spiralTEva == ConstA)
	{
		t = tsv;
	}
	else
	{
		DEBUG_PRINT_CODE_LOCATION;
		DEBUG_PRINT_ERROR_MESSAGE("spiral evaluate type unknow");
		DEBUG_PRINT_VALUE(spiralTEva);
		DEBUG_PRINT_VALUE(k);

		out.messageCode = MESSAGE_TRAJCTORY_SPIRAL_OPTION_TYPE_UNKNOW;
		return out;
	}

	double theta = (k - 1)*t;
	double r = b * theta;
	double firstCoef = r * cos(theta);
	double secondCoef = r * sin(theta);

	Vector3d trans = firstCoef * firstDirection + secondCoef * secondDirection;
	VectorXd deltaP(6);
	deltaP << trans, 0, 0, 0;
	VectorXd P = deltaP + Pstart;

	double d = deltaP.norm();
	out.trajP = P;
	if (d > scanRange)//
	{
		out.messageCode = MESSAGE_TRAJCTORY_SPIRAL_P_OUT_OF_SCAN_RANGE;
		out.kNext = k;
		return out;
	}

	if (k == K)
	{
		out.messageCode = MESSAGE_TRAJCTORY_NORMAL_END;
		out.kNext = k;
		return out;
	}
	   	  	
	out.messageCode = MESSAGE_OK;
	out.kNext = k + 1;
	return out;
}

TrajctoryResultP GMT::Trajctory::SinK(
	Matrix4d Tstart, double scanRange, double stepRange, double lineSpacing,	Vector3d firstDirection,	Vector3d secondDirection,
	double velocity, double ts, enumTimeScaling tscal, double frequency, enumSinScanOption sinMode, int k)
{
	
	VectorXd Pstart = sE3ToPoseRPY(Tstart);
	TrajctoryResultP out;
	
	double step_range = stepRange;// *0.5;
	double scan_range = scanRange;// *0.5;

	double nr = step_range / lineSpacing;
	double tf = (2 * MathPI) * nr * (SIN_FREQUENCY_COEFFICIENT*frequency);
	double tsv = ts * velocity;

	int K = ceil(tf / tsv) + 1;
	int M = 2 * K + 1;
	
	Vector3d translv;
	Vector3d translh;
	//TODO::方向確認
	if (sinMode == horizontal || sinMode == Both)//TODO:加入both的實作
	{
		 step_range = scanRange;// *0.5;
		 scan_range = stepRange;// *0.5;
		translv = secondDirection;
		translh = firstDirection;
	}
	else if (sinMode == vertical)
	{
		 step_range = stepRange;// *0.5;
		 scan_range = scanRange;// *0.5;
		translv = firstDirection;
		translh = secondDirection;
	}

	   
	if (k > M)
	{
		DEBUG_PRINT_CODE_LOCATION;
		DEBUG_PRINT_ERROR_MESSAGE("k out of range");
		DEBUG_PRINT_VALUE(M);
		DEBUG_PRINT_VALUE(k);

		out.messageCode = MESSAGE_TRAJCTORY_K_OUT_OF_RANGE;
		return out;
	}
	else
	{
		Vector3d X = step_range * (((double)k - 1) / (double)K - 1)*translh;//要強制轉型(double)k不然int和double一起算會出錯?
		Vector3d Y = scan_range * cos(((double)k - 1)*tsv)*translv;
		//Z = 0.0*toolTranslz;

		Vector3d trans = X + Y;
		VectorXd P(6);
		P << trans, 0, 0, 0;
		P = P + Pstart;

		out.trajP = P;
		if (k == M)
		{
			out.messageCode = MESSAGE_TRAJCTORY_NORMAL_END;
			out.kNext = k;
			return out;
		}

		//Matrix4d T = poseRPY2SE3(P);

		out.messageCode = MESSAGE_OK;
		out.kNext = k + 1;
		return out;
	}

}

#pragma region private

double GMT::Trajctory::newton(double x0, double tolx, int maxiter, SpiralNewtonEvaluateFunction spiralEva, void *p)
{
	double h = 1e-4;
	double h2 = 2 * h;
	double tolfun = std::numeric_limits<double>::epsilon();
	
	double fx = spiralEva(x0, p);

	double xx = x0;
	for (int i = 0; i < maxiter; i++)
	{
		double dfdx = (spiralEva(xx + h, p) - spiralEva(xx - h, p)) / h2;
		double dx = -fx / dfdx;
		xx += dx;
		fx = spiralEva(xx, p);

		if (abs(fx) < tolfun || abs(dx) < maxiter) break;
	}

	return xx;
}

double GMT::Trajctory::spiral_time_eval_f1(double t, void *p)
{
	SpiralTEvaluateParameter *input = (SpiralTEvaluateParameter *)p;
	double b = input->b;
	double rx = input->rx;
	double n = input->n;

	return (b / 2)*(log(t + sqrt(t*t + 1)) + t * sqrt(t*t + 1)) - 2 * MathPI*rx*n;
}

double GMT::Trajctory::spiral_time_eval_f2(double t, void *p)
{
	SpiralTEvaluateParameter *input = (SpiralTEvaluateParameter *)p;
	double b = input->b;
	double k = input->k;

	return (b / 2)*(log((t*k + sqrt(t*t*k*k + 1)) / (t*(k - 1) + sqrt(t*t*SQUARE(k - 1) + 1)))
		+ t * k*sqrt(t*t*k*k + 1) - t * (k - 1)*sqrt(t*t*SQUARE(k - 1) + 1));
}


#pragma endregion private
