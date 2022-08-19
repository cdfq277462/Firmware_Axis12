#include "..\header\StewartBasic.h"



#pragma region generic

GMT::StewartBasic::StewartBasic()
{
}


void GMT::StewartBasic::SetStewartType(enumStewartType type)
{
	_PHome_1x6 = VectorXd(6);
	_initGuess_1x6 = VectorXd(6);

	switch (type)
	{
	case TypeN:
		SetStewartNTypeDefaultParameters();
		break;
	case TypeF:
		SetStewartFTypeDefaultParameters();
		break;
	case TypeA12:
		SetStewartA12DefaultParameters();
		break;
	default:
		DEBUG_PRINT_CODE_LOCATION;
		DEBUG_PRINT_ERROR_MESSAGE("Stewart type not initialel or unknow");
		DEBUG_PRINT_VALUE(type);
		break;
	};

	ResetTposeStart();
}

GMT::StewartBasic::StewartBasic(enumStewartType type)
{
	SetStewartType(type);
}

bool GMT::StewartBasic::CheckStewartType()
{
	if (_stewartType == TypeN
		|| _stewartType == TypeN
		|| _stewartType == TypeA12
		)
	{
		return true;
	}
	else if (_stewartType == StewartTypeNotInitialed)
	{
		DEBUG_PRINT_CODE_LOCATION;
		DEBUG_PRINT_ERROR_MESSAGE("Stewart type not initialed");
	}
	else
	{
		DEBUG_PRINT_CODE_LOCATION;
		DEBUG_PRINT_ERROR_MESSAGE("Stewart type unknow");
	}

	return false;
}

const enumStewartType GMT::StewartBasic::GetStewartType()
{
	return _stewartType;
}

void GMT::StewartBasic::SetTrajctoryTs(double ts)
{
	_ts = ts;
}

double GMT::StewartBasic::GetTs()
{
	return _ts;
}

void GMT::StewartBasic::SetTrajctoryTimeScaling(enumTimeScaling timeScale)
{
	_timeScale = timeScale;
}

enumTimeScaling GMT::StewartBasic::GetTimeScaling()
{
	return _timeScale;
}

const StewartParameters GMT::StewartBasic::GetSetting()
{
	StewartParameters para;
	para.type = _stewartType;
	para.bb = _bb;
	para.pp = _pp;

	return para;
}


WSOutput GMT::StewartBasic::ToolWS(VectorXd PToolStart, enumWSBaseMode baseMode)
{

		if (_firstRun)
		{
			ResetTposeStart();
			_firstRun = false;
		}
		//DEBUG_PRINT_VALUE(_TposeStart)

		VectorXd delta = PToolStart - _WSLastLegalResult.input;
		//DEBUG_PRINT_VALUE(delta)

		WSOutput newResult;
		if (_stewartType == TypeN
			|| _stewartType == TypeF)
		{
			newResult = WorkSpacePivot(delta, baseMode);
		}
		else if(_stewartType == TypeA12)
		{
			newResult = A12_ToolWS(delta, baseMode);
		}
		else
		{
			newResult.messageCode = MESSAGE_TOOL_STEWART_TYPE_ERROR;
			return newResult;
		}

		if (newResult.messageCode == MESSAGE_INPUT_ILLEGAL)
		{
			_TposeStart = _WSLastLegalResult.Tstart;
			newResult = _WSLastLegalResult;
			newResult.messageCode = MESSAGE_INPUT_ILLEGAL;
			return newResult;
		}
		else
		{
			newResult.input = PToolStart;
			_WSLastLegalResult = newResult;
		}

		return newResult;
}

void GMT::StewartBasic::SetToolTrans(Matrix4d Tstart, Matrix4d TMTool, VectorXd wsRemain12_1x12)
{
	_TposeStart = Tstart;
	_TMTool = TMTool;
	_WSRemain12_1x12 = wsRemain12_1x12;
}

void GMT::StewartBasic::SetToolCoordinate(Vector3d pivot, double alpha, double beta)
{
	if (_stewartType == TypeA12) return;//12軸沒有工具座標，防呆

	_pivot = pivot;
	_alpha = alpha;
	_beta = beta;
}

void GMT::StewartBasic::SetWSdelta(double trans, double rot)
{
	_workSpaceDeltaTrans = trans;
	_workSpaceDeltaRot = rot;
}

bool GMT::StewartBasic::CheckL(VectorXd L, double lLow, double lUpp)
{
	if (L.size() < 6) return false;
	
	auto illegal_lenth = (L.array() < lLow || L.array() > lUpp);
	
	if (illegal_lenth.any()) {
		//DEBUG_PRINT_VALUE(LL)
		return false;
	}
	else return true;
}

bool GMT::StewartBasic::CheckL(VectorXd L)
{
	return CheckL(L, _Llow, _Lupp);
}

bool GMT::StewartBasic::CheckP(VectorXd P)
{
	if (_stewartType == TypeA12)
	{
		return A12_checkP(P);
	}

	IKOutput result;
	if (_stewartType == TypeF)
	{
		result = F_IK(P);
	}

	else if (_stewartType == TypeN) {
		result = N_IK(P);

		Mat tarr = result.alpha.array().square() + SQUARE(_h0) - result.Ls.array().square();
		auto illegal_lenth = (tarr.array() < 0);
		if (illegal_lenth.any()) return false;

		result.LL = result.alpha.array() - tarr.array().sqrt();
	}

	else {
		DEBUG_PRINT_MESSAGE("type unknow")
			return false;
	}

	return CheckL(result.LL);
}



TrajctoryResultP GMT::StewartBasic::ToolMoveLPP(VectorXd Pstart, VectorXd Pend, double speed, int k)
{
	Matrix4d Tstart = poseRPY2SE3(Pstart);
	Matrix4d Tend = poseRPY2SE3(Pend);

	return ToolMoveLPP(Tstart, Tend, speed,  k);
}


TrajctoryResultP GMT::StewartBasic::ToolMoveLPP(Matrix4d Tstart, Matrix4d Tend, double speed, int k)
{
	TrajctoryResultP out;
	VectorXd Pstart = sE3ToPoseRPY(Tstart);
	if (!CheckP(Pstart))
	{
		out.messageCode = MESSAGE_TOOL_TRAJCTORY_START_OUT_OF_RANGE;
		return out;
	}
	VectorXd Pend = sE3ToPoseRPY(Tend);
	if (!CheckP(Pend))
	{
		out.messageCode = MESSAGE_TOOL_TRAJCTORY_END_OUT_OF_RANGE;
		return out;
	}

	VectorXd Lstart(6);
	VectorXd Lend(6);

	if (_stewartType == TypeA12)
	{
		Lstart = Pstart;
		Lend = Pend;
	}
	else if (_stewartType == TypeF)
	{
		Lstart = F_IK(Pstart).LL;
		Lend = F_IK(Pend).LL;
	}
	else if (_stewartType == TypeN)
	{
		Lstart = N_IK(Pstart).g;
		Lend = N_IK(Pend).g;
	}

	double coef;
	if (_timeScale == enumTimeScaling::Cubic)
	{
		coef = TimeScalingCoefficientCubic;
	}
	else
		coef = TimeScalingCoefficientQuintic;

	double t = coef * ((Lstart - Lend).lpNorm<Infinity>() / speed);

	//DEBUG_PRINT_CODE_LOCATION;
	//DEBUG_PRINT_VALUE(N_IK(Pstart).dg);
	//DEBUG_PRINT_VALUE(N_IK(Pend).dg);
	//DEBUG_PRINT_VALUE(Lstart);
	//DEBUG_PRINT_VALUE(Lend);

	return MoveLPPK(Tstart, Tend, _timeScale, t, _ts, k);

}


PList GMT::StewartBasic::ToolMoveLPP(VectorXd Pstart, VectorXd Pend, double speed)
{
	Matrix4d Tstart = poseRPY2SE3(Pstart);
	Matrix4d Tend = poseRPY2SE3(Pend);

	return ToolMoveLPP(Tstart, Tend, speed);
}

PList GMT::StewartBasic::ToolMoveLPP(Matrix4d Tstart, Matrix4d Tend, double speed)
{
	PList out;

	int k = 1;
	out.messageCode = MESSAGE_OK;
	while (true)
	{
		TrajctoryResultP outk = ToolMoveLPP(Tstart, Tend, speed, k);

		if (outk.messageCode != MESSAGE_OK)
		{
			out.messageCode = outk.messageCode;
			break;
		}
		else
		{
			out.add(outk.trajP);
		}

		k++;
	}

	return out;
}

ToolTrajctoryResultP GMT::StewartBasic::ToolSpiral(Matrix4d Tstart, double scanRange, double lineSpacing,
	double speedSpiral, //Spiral軌跡速度
	double speedMoveL, //Spiral後moveL速度
	enumSpiralTEvaluate spiraltimeEva, int k, int stage)
{
	ToolTrajctoryResultP out;

	VectorXd Pstart = sE3ToPoseRPY(Tstart);
	if (!CheckP(Pstart))
	{
		out.messageCode = MESSAGE_TOOL_TRAJCTORY_START_OUT_OF_RANGE;
		return out;
	}

	//TODO?:加入檢查scanRange是否超出可運作範圍

	int currentStage = stage;
	int currentK = k;
	if (currentStage == SPIRAL_STAGE_START_SPIRAL)// start -> spiral
	{
		//VectorXd Pstart = sE3ToPoseRPY(Tstart);
		Matrix4d TtoolStart = Tstart * _TMTool;

		Matrix3d toolTrans = t2r(TtoolStart);
		Vector3d toolTranslz = toolTrans.col(2);
		Vector3d toolTransly = toolTrans.col(1);

		TrajctoryResultP spiralk = SpiralK(
			Tstart, scanRange, lineSpacing,
			toolTransly,
			toolTranslz,
			speedSpiral, _ts, _timeScale, spiraltimeEva, currentK
		);

		if (!CheckP(spiralk.trajP))//純軌跡函數算出來的點不會有機器資訊，需檢查點是否超過機器可運行範圍
		{
			out.messageCode = MESSAGE_TOOL_TRAJCTORY_P_OUT_OF_MACHINE_WORKING_RANGE;
			return out;
		}
		
		if (spiralk.messageCode == MESSAGE_TRAJCTORY_SPIRAL_P_OUT_OF_SCAN_RANGE 
			|| spiralk.messageCode == MESSAGE_TRAJCTORY_NORMAL_END) //目前spiral沒有好的結束偵測條件，超出scan範圍和k=M都當作結束處理
		{
			currentStage = SPIRAL_STAGE_SPIRAL_END_BACK_TO_START;
			currentK = 1;
		}
		else 
		{
			_TTrajctoryEnd = poseRPY2SE3(spiralk.trajP);
			out.messageCode = spiralk.messageCode;
			out.trajP = spiralk.trajP;
			out.kNext = spiralk.kNext;
			out.stageNext = SPIRAL_STAGE_START_SPIRAL;
			return out;
		}
	}

	if (currentStage == SPIRAL_STAGE_SPIRAL_END_BACK_TO_START)// spiral end -> start
	{
		TrajctoryResultP moveLk = ToolMoveLPP(
			_TTrajctoryEnd, //軌跡起點，4x4矩陣
			Tstart, //軌跡終點，4x4矩陣
			speedMoveL, //
			currentK //指定計算第k個軌跡點(1 base)
		);
		
		out.messageCode = moveLk.messageCode;
		out.trajP = moveLk.trajP;
		out.kNext = moveLk.kNext;
		out.stageNext = SPIRAL_STAGE_SPIRAL_END_BACK_TO_START;
		return out;	

	}
	
	out.messageCode = MESSAGE_TOOL_STAGE_ERROR;
	return out;
}

PList GMT::StewartBasic::ToolSpiral(Matrix4d Tstart, double scanRange, double lineSpacing,
	double speedSpiral, //Spiral軌跡速度
	double speedMoveL, //Spiral後moveL速度
	enumSpiralTEvaluate spiraltimeEva, bool backToStart)
{
	PList out;

	int k = 1;
	int stage = 1;
	out.messageCode = MESSAGE_OK;
	while (true)
	{
		ToolTrajctoryResultP outk = ToolSpiral(
			Tstart,
			scanRange,
			lineSpacing,
			 speedSpiral, //Spiral軌跡速度
			 speedMoveL, //Spiral後moveL速度
			spiraltimeEva,
			k,
			stage
		);

		if (outk.messageCode != MESSAGE_OK)
		{
			out.messageCode = outk.messageCode;
			break;
		}

		if (!backToStart 
			&& outk.stageNext == SPIRAL_STAGE_SPIRAL_END_BACK_TO_START)
		{
			_TTrajctoryEnd = poseRPY2SE3(outk.trajP);
			break;
		}
		
		out.add(outk.trajP);

		k = outk.kNext;
		stage = outk.stageNext;
	}

	return out;
}

ToolTrajctoryResultP GMT::StewartBasic::ToolSin(Matrix4d Tstart, double scanRange, double stepRange, double lineSpacing,
	double speedSin, //sin軌跡速度
	double speedMoveL, //sin前後moveL速度
	double frequency, enumSinScanOption sinMode, int k, int stage)
{
	ToolTrajctoryResultP out;

	VectorXd Pstart = sE3ToPoseRPY(Tstart);
	if (!CheckP(Pstart))
	{
		out.messageCode = MESSAGE_TOOL_TRAJCTORY_START_OUT_OF_RANGE;
		return out;
	}

	//TODO?:加入檢查scanRange是否超出可運作範圍
	Matrix4d TtoolStart = Tstart * _TMTool;
	Matrix3d toolTrans = t2r(TtoolStart);
	Vector3d toolTranslx = toolTrans.col(0);
	Vector3d toolTransly = toolTrans.col(1);
	Vector3d toolTranslz = toolTrans.col(2);
	Vector3d firstDirection;
	Vector3d secondDirection;
	if (_stewartType == TypeA12)
	{
		firstDirection = toolTranslz;
		secondDirection = toolTransly;
	}
	else if (_stewartType == TypeF
		|| _stewartType == TypeN)
	{
		firstDirection = toolTranslx;
		secondDirection = toolTransly;
	}
	else
	{
		out.messageCode = MESSAGE_TOOL_STEWART_TYPE_ERROR;
		return out;
	}

		

	int currentStage = stage;
	int currentK = k;

	if (currentStage == SIN_STAGE_START_TO_SIN)// spiral end -> start
	{			   

		//TrajctoryResultP sinStart = CalculateSinStart(Pstart, stepRange, scanRange, sinMode, toolTransly, toolTranslz);
		//if (sinStart.messageCode != MESSAGE_OK)
		//{
		//	out.messageCode = sinStart.messageCode;
		//	return out;
		//}
		TrajctoryResultP sinStart = SinK(
			Tstart,
			scanRange,
			stepRange,
			lineSpacing,
			firstDirection,
			secondDirection,
			speedSin,
			_ts,
			_timeScale,
			frequency,
			sinMode,
			1
		);
		if (!CheckP(sinStart.trajP))//純軌跡函數算出來的點不會有機器資訊，需檢查點是否超過機器可運行範圍
		{
			out.messageCode = MESSAGE_TOOL_SIN_OUT_OF_RANGE;
			return out;
		}


		Matrix4d TsinStart = poseRPY2SE3(sinStart.trajP);
		TrajctoryResultP moveLk = ToolMoveLPP(
			TtoolStart, //軌跡起點，4x4矩陣
			TsinStart, //軌跡終點，4x4矩陣
			speedMoveL, //
			currentK //指定計算第k個軌跡點(1 base)
		);

		if (!CheckP(moveLk.trajP))//純軌跡函數算出來的點不會有機器資訊，需檢查點是否超過機器可運行範圍
		{
			out.messageCode = MESSAGE_TOOL_TRAJCTORY_P_OUT_OF_MACHINE_WORKING_RANGE;
			return out;
		}

		if (moveLk.messageCode == MESSAGE_TRAJCTORY_NORMAL_END)//如果跑完moveL，切換stage進入sin階段
		{
			currentStage = SIN_STAGE_SIN;
			currentK = 1;
		}
		else //其餘狀況正常和不正常都直接回傳
		{
			out.messageCode = moveLk.messageCode;
			out.trajP = moveLk.trajP;
			out.kNext = moveLk.kNext;
			out.stageNext = currentStage;
			return out;
		}

	}

	if (currentStage == SIN_STAGE_SIN)// sin -> start
	{
		//VectorXd Pstart = sE3ToPoseRPY(Tstart);


		TrajctoryResultP sink = SinK(
			 Tstart, 
			scanRange, 
			stepRange, 
			lineSpacing,
			firstDirection,
			secondDirection,
			speedSin,
			_ts, 
			_timeScale,
			frequency,
			sinMode, 
			currentK
		);
		
		if (!CheckP(sink.trajP))//純軌跡函數算出來的點不會有機器資訊，需檢查點是否超過機器可運行範圍
		{
			out.messageCode = MESSAGE_TOOL_TRAJCTORY_P_OUT_OF_MACHINE_WORKING_RANGE;
			return out;
		}

		if (sink.messageCode == MESSAGE_TRAJCTORY_NORMAL_END)//如果跑完sin，切換stage進入sin之後的moveL階段
		{
			currentStage = SIN_STAGE_SIN_END_TO_START;
			currentK = 1;
		}
		else //其餘狀況正常和不正常都直接回傳
		{
			_TTrajctoryEnd = poseRPY2SE3(sink.trajP);//儲存最後的軌跡點給下一階段moveL用
			out.messageCode = sink.messageCode;
			out.trajP = sink.trajP;
			out.kNext = sink.kNext;
			out.stageNext = currentStage;
			return out;
		}
	}

	if (currentStage == SIN_STAGE_SIN_END_TO_START)// sin end -> start
	{
		TrajctoryResultP moveLk = ToolMoveLPP(
			_TTrajctoryEnd, //軌跡起點，4x4矩陣
			Tstart, //軌跡終點，4x4矩陣
			speedMoveL, //
			currentK //指定計算第k個軌跡點(1 base)
		);

		out.messageCode = moveLk.messageCode;
		out.trajP = moveLk.trajP;
		out.kNext = moveLk.kNext;
		out.stageNext = SIN_STAGE_SIN_END_TO_START;
		return out;

	}

	out.messageCode = MESSAGE_TOOL_STAGE_ERROR;
	return out;
}

PList GMT::StewartBasic::ToolSin(Matrix4d Tstart, double scanRange, double stepRange, double lineSpacing,
	double speedSin, //sin軌跡速度
	double speedMoveL, //sin前後moveL速度
	double frequency, enumSinScanOption sinMode, bool backToStart)
{

	PList out;

	int k = 1;
	int stage = SIN_STAGE_START_TO_SIN;
	out.messageCode = MESSAGE_OK;
	while (true)
	{
		ToolTrajctoryResultP outk = ToolSin(
			Tstart,
			scanRange,
			stepRange,
			lineSpacing,
			 speedSin, //sin軌跡速度
			 speedMoveL, //sin前後moveL速度
			frequency,
			sinMode,
			k,
			stage
		);

		if (outk.messageCode != MESSAGE_OK)
		{
			out.messageCode = outk.messageCode;
			break;
		}

		if (!backToStart
			&& outk.stageNext == SIN_STAGE_SIN_END_TO_START)
		{
			_TTrajctoryEnd = poseRPY2SE3(outk.trajP);
			break;
		}

		out.add(outk.trajP);

		k = outk.kNext;
		stage = outk.stageNext;
	}

	return out;
}



Mat StewartBasic::_pp(3, 6);
Mat StewartBasic::_bb(3, 6);
VectorXd StewartBasic::_L_1x6(6);




FKOutput GMT::StewartBasic::CminpackHybrdFK(VectorXd L, VectorXd ini_pose)
{
	_L_1x6 = L;//存給FKCallback用

	auto x = ini_pose;
	//DEBUG_PRINT_VALUE(x);

	//minpack_func_nn aaa = &N_FKCallback;
	
	//以下從範例引用來的，別問我內容這是神馬妖魔鬼怪
#pragma region hybrid1範例

	double *fvec;  //output array of length n which contains the functions evaluated at the output x. 
	int iflag;     //...not quite sure on this one
	int info;      //integer output variable. If the user has terminated execution, info is set to the (negative) value of iflag.  Otherwise RTFM
	int lwa;       //length of work array
	int n = 6;     //number of unknowns (x1, x2)
	double tol = 1.5e-8;
	double *wa;    //work array of length lwa
	//double *x;     //array of length n. On input x must contain an initial estimate of the solution vector. On output x contains the final estimate of the solution vector. 

	lwa = (n * (3 * n + 13)) / 2;
	fvec = new double[n];  //mathematical output
	wa = new double[lwa];
	//	x = new double[n];     //mathematical input

	iflag = 1;
	_fkCallback(&n, x.data(), fvec, &iflag);//_fkCallback 在set stewart parameter裡面設定
		
#pragma endregion
#pragma region hybrid1 裡面找來的


	/* Initialized data */

	const real factor = 6.226e-6;

	/* System generated locals */
	int i__1;

	/* Local variables */
	int j, ml, lr, mu, mode, nfev;
	real xtol;
	int index;
	real epsfcn;
	int maxfev, nprint;

	//--fvec;
	//--x;
	//--wa;

	/* Function Body */
	info = 0;

	/*     check the input parameters for errors. */

	if (n <= 0 || tol < 0. || lwa < n * (n * 3 + 13) / 2) {
		/* goto L20; */
		FKOutput out;
		out.P = x;
		out.messageCode = MESSAGE_INPUT_ILLEGAL;
		return out;
	}

	/*     call hybrd. */
	double *x_ptr = x.data();

	maxfev = (n + 1) * 200;
	xtol = tol;
	ml = n - 1;
	mu = n - 1;
	epsfcn = 0.;
	mode = 2;
	i__1 = n;
	for (j = 0; j < i__1; ++j) {
		wa[j] = 1.;
		/* L10: */
	}
	nprint = 0;
	lr = n * (n + 1) / 2;
	index = n * 6 + lr;
	__minpack_func__(hybrd)(_fkCallback, &n, x_ptr, &fvec[0], &xtol, &maxfev, &ml, &mu, &epsfcn, &
		wa[0], &mode, &factor, &nprint, &info, &nfev, &wa[index], &n, &
		wa[n * 6], &lr, &wa[n], &wa[(n << 1)], &wa[n * 3
		], &wa[(n << 2)], &wa[n * 5]);
	if (info == 5) {
		info = 4;
	}

#pragma endregion

	//if (_is_show_fk_debug)
	//{
	//	DEBUG_PRINT_MESSAGE("  Returned value of INFO = " << info)
	//		DEBUG_PRINT_VALUE(x)
	//		DEBUG_PRINT_MESSAGE("  F(X)" << endl
	//			<< fvec[0] << endl
	//			<< fvec[1] << endl
	//			<< fvec[2] << endl
	//			<< fvec[3] << endl
	//			<< fvec[4] << endl
	//			<< fvec[5] << endl
	//			<< endl
	//		)
	//}

	delete[] fvec;
	delete[] wa;

	FKOutput out;
	out.P = x;
	out.messageCode = MESSAGE_OK;
	return out;
}

WSOutput GMT::StewartBasic::WorkSpacePivot(VectorXd PToolDelta, enumWSBaseMode baseMode)
{

	WSOutput out;

	toolTrans m = ToolCalculateM(PToolDelta, baseMode);
	out.Tstart = _TposeStart;
	out.TMtool = m.TMtool;

	WSLimitTestOutput limit_posi_x = ToolWSTrans(m.poseStart, m.xrtool, _workSpaceTestLimit[WS_LIMIT_X_PLUS], _workSpaceTestLimit[WS_LIMIT_X_MINUS]);
	WSLimitTestOutput limit_posi_y = ToolWSTrans(m.poseStart, m.yrtool, _workSpaceTestLimit[WS_LIMIT_Y_PLUS], _workSpaceTestLimit[WS_LIMIT_Y_MINUS]);
	WSLimitTestOutput limit_posi_z = ToolWSTrans(m.poseStart, m.zrtool, _workSpaceTestLimit[WS_LIMIT_Z_PLUS], _workSpaceTestLimit[WS_LIMIT_Z_MINUS]);

	WSLimitTestOutput limit_angle_z = ToolWSPivot(m.poseStart, m.zrtool, _workSpaceTestLimit[WS_LIMIT_PHI_PLUS], _workSpaceTestLimit[WS_LIMIT_PHI_MINUS]);
	WSLimitTestOutput limit_angle_y = ToolWSPivot(m.poseStart, m.yrtool, _workSpaceTestLimit[WS_LIMIT_THETA_PLUS], _workSpaceTestLimit[WS_LIMIT_THETA_MINUS]);
	WSLimitTestOutput limit_angle_x = ToolWSPivot(m.poseStart, m.xrtool, _workSpaceTestLimit[WS_LIMIT_PSI_PLUS], _workSpaceTestLimit[WS_LIMIT_PSI_MINUS]);

	VectorXd RemValue12(12);
	RemValue12 << limit_posi_x.max, limit_posi_x.min,
		limit_posi_y.max, limit_posi_y.min,
		limit_posi_z.max, limit_posi_z.min,
		limit_angle_z.max, limit_angle_z.min,
		limit_angle_y.max, limit_angle_y.min,
		limit_angle_x.max, limit_angle_x.min;

	_WSRemain12_1x12 = RemValue12;
	out.remain12 = RemValue12;

	bool scanSuccess =
		limit_posi_x.scanSuccess
		&& limit_posi_y.scanSuccess
		&& limit_posi_z.scanSuccess
		&& limit_angle_z.scanSuccess
		&& limit_angle_y.scanSuccess
		&& limit_angle_x.scanSuccess;
	
	bool inputIllegal =
		limit_posi_x.inputIllegal
		|| limit_posi_y.inputIllegal
		|| limit_posi_z.inputIllegal
		|| limit_angle_z.inputIllegal
		|| limit_angle_y.inputIllegal
		|| limit_angle_x.inputIllegal;

	if (inputIllegal)
		out.messageCode = MESSAGE_INPUT_ILLEGAL;
	else if (!scanSuccess)
		out.messageCode = MESSAGE_TOOL_WS_SCAN_FAIL;
	else
		out.messageCode = MESSAGE_OK;

	return out;

}

toolTrans GMT::StewartBasic::ToolCalculateM(VectorXd PToolDelta, enumWSBaseMode baseMode)
{
	toolTrans out;

	Matrix3d ry = rotY(_alpha);
	Matrix3d rx = rotX(_beta);
	Vector3d PMtool = _pivot;
	Matrix3d RMtool = ry * rx;
	Matrix4d TMtool = rp2t(RMtool, PMtool);
	out.TMtool = TMtool;

	Matrix4d TtoolPoseWorking = poseRPY2SE3(PToolDelta);
	Vector3d MposiWork;
	if (baseMode == base) MposiWork = t2p(TtoolPoseWorking);
	else if (baseMode == tool)MposiWork = RMtool * t2p(TtoolPoseWorking);
	else {
		DEBUG_PRINT_ERROR_MESSAGE("base mode unknow")
			return out;
	}
	Matrix3d ToolOrietWork = t2r(TtoolPoseWorking);
	VectorXd w = sO3ToPoseAxisAngle(ToolOrietWork);
	VectorXd wm = RMtool * w;

	double theta = norm(wm);
	Matrix4d trotStart;
	if (NearZero(theta))
	{
		trotStart = Matrix4d::Identity();
	}
	else
	{
		VectorXd w3 = wm / theta;
		Vector3d r = _pivot;
		Matrix3d wx = vect3tomatrix3(w3);
		Matrix4d s;
		s << wx, -wx * r, 0, 0, 0, 0;
		trotStart = MatrixExp6(s*theta);
	}

	Matrix4d THMwork = rp2t(Matrix3d::Identity(), MposiWork);
	Matrix4d Twork = THMwork * MatrixExp6(MatrixLog6(THMwork.inverse()*trotStart));
	_TposeStart = _TposeStart * THMwork*Twork;
	//Matrix4d Tstart = _TposeStart;
	out.poseStart = sE3ToPoseRPY(_TposeStart);
	//DEBUG_PRINT_VALUE(_TposeStart);
	//DEBUG_PRINT_VALUE(t_out.poseStart);

	out.xrtool = RMtool.col(0);
	out.yrtool = RMtool.col(1);
	out.zrtool = RMtool.col(2);

	return out;
}

WSLimitTestOutput GMT::StewartBasic::ToolWSTrans(VectorXd P, Vector3d axis3, double limit_plus, double limit_minus)
{

	VectorXd PTest(6);
	WSLimitTestOutput out;
	VectorXd axis6(6);
	axis6 << axis3, 0, 0, 0;

	bool plusSuccess = false;
	bool minusSuccess = false;

	for (double i = 0; i < limit_plus; i += _workSpaceDeltaTrans)
	{
		PTest << P + axis6 * i;

		if (!CheckP(PTest))
		{
			if (i == 0) {
				DEBUG_PRINT_ERROR_MESSAGE("illegal pose")
					DEBUG_PRINT_VALUE(PTest)
					out.inputIllegal = true;
			}
			else
			{
				out.max = i - _workSpaceDeltaTrans;
				plusSuccess = true;
			}

			break;
		}
	}

	for (double i = 0; i > limit_minus; i -= _workSpaceDeltaTrans)
	{
		PTest << P + axis6 * i;

		if (!CheckP(PTest))
		{
			if (i == 0) {
				DEBUG_PRINT_ERROR_MESSAGE("illegal pose")
					DEBUG_PRINT_VALUE(PTest)
					out.inputIllegal = true;
			}
			else 
			{
				out.min = i + _workSpaceDeltaTrans;
				minusSuccess = true;
			}
			break;
		}
	}
	out.scanSuccess = plusSuccess && minusSuccess;

	return out;

}

WSLimitTestOutput GMT::StewartBasic::ToolWSPivot(VectorXd P, VectorXd axis3, double limit_plus, double limit_minus)
{

	WSLimitTestOutput out;

	bool plusSuccess = false;
	bool minusSuccess = false;

	//clock_t pos_loop_start = clock();
	bool posi_break = false;
	//DEBUG_PRINT_MESSAGE("pos limit")
	for (double posi_angle_test = 0; posi_angle_test < limit_plus; posi_angle_test += _workSpaceDeltaRot)
	{
		double theta = posi_angle_test;

		//clock_t pivot_start = clock();
		VectorXd trajn = PivotTrajForCheck(P, _pivot, axis3, posi_angle_test);

		if (!CheckP(trajn))
		{
			if (posi_angle_test == 0) {
				DEBUG_PRINT_ERROR_MESSAGE("illegal pose")
					DEBUG_PRINT_VALUE(posi_angle_test)
					DEBUG_PRINT_VALUE(trajn)
					out.inputIllegal = true;
			}
			else {
				out.max = posi_angle_test - _workSpaceDeltaRot;
				plusSuccess = true;
			}

			posi_break = true;
			break;
		}
	}

	//RELEASE_PRINT_MESSAGE("positive loop用時:" << clock() - pos_loop_start << "(ms)" << endl)


	//clock_t nag_loop_start = clock();
	//DEBUG_PRINT_MESSAGE("nag limit")
	bool nag_break = false;

	for (double nag_angle_test = 0; nag_angle_test > limit_minus; nag_angle_test -= _workSpaceDeltaRot)
	{
		double theta = nag_angle_test;
		VectorXd trajn = PivotTrajForCheck(P, _pivot, axis3, nag_angle_test);

		if (!CheckP(trajn))
		{
			if (nag_angle_test == 0) {
				DEBUG_PRINT_ERROR_MESSAGE("illegal pose")
					DEBUG_PRINT_VALUE(nag_angle_test)
					out.inputIllegal = true;
			}
			else {
				out.min = nag_angle_test + _workSpaceDeltaRot;
				minusSuccess = true;
			}

			nag_break = true;
			break;
		}

		//}

		//if (nag_break) break;


		//loop_counter++;
	}
	//RELEASE_PRINT_MESSAGE("nagtive loop用時:" << clock() - nag_loop_start << "(ms)" << endl)
	//RELEASE_PRINT_VALUE(loop_counter)
	out.scanSuccess = plusSuccess && minusSuccess;

	return out;
}

VectorXd GMT::StewartBasic::PivotTrajForCheck(VectorXd Pstart, VectorXd pivot, VectorXd w, double theta)
{
	Matrix4d xstart = poseRPY2SE3(Pstart);
	Vector3d r = pivot;

	w = w / norm(w);
	Matrix3d wx = vect3tomatrix3(w);
	Matrix3d rx = vect3tomatrix3(r);
	Matrix4d s;
	s << wx, rx*w, 0, 0, 0, 0;
	Matrix4d xstart_to_end = MatrixExp6(s*theta);
	Matrix4d traj_end = xstart * xstart_to_end;
	VectorXd ptraj_end = sE3ToPoseRPY(traj_end);

	return ptraj_end;
	
}

void GMT::StewartBasic::ResetTposeStart()
{
	_TposeStart = poseRPY2SE3(_PHome_1x6);
	_TMTool = Matrix4d::Identity();
	_WSLastLegalResult.Tstart = poseRPY2SE3(_PHome_1x6);
	//_LastLegalResult.lastLegalInput = (VectorXd(6) << 0, 0, 0, 0, 0, 0).finished();
	_WSLastLegalResult.input = VectorXd::Zero(6);
	_WSLastLegalResult.remain12 = VectorXd::Zero(12);
}

TrajctoryResultP GMT::StewartBasic::CalculateSinStart(VectorXd P_6x1, double scan_range, double step_range, enumSinScanOption scanMode, Vector3d toolTransY, Vector3d toolTransZ)
{
	double dz = min(abs(_WSRemain12_1x12[WS_LIMIT_Z_PLUS]), abs(_WSRemain12_1x12[WS_LIMIT_Z_MINUS]));
	double dy = min(abs(_WSRemain12_1x12[WS_LIMIT_Y_PLUS]), abs(_WSRemain12_1x12[WS_LIMIT_Y_MINUS]));
	Vector3d translv;
	Vector3d translh;
	double scanLimit;
	double stepLimit;
	if (scanMode == vertical || scanMode == Both)//TODO:加入both的實作
	{
		scanLimit = abs(dz);
		stepLimit = abs(dy);
		translv = toolTransZ;
		translh = toolTransY;
	}
	else if (scanMode == horizontal)
	{
		scanLimit = abs(dy);
		stepLimit = abs(dz);
		translv = toolTransY;
		translh = toolTransZ;
	}

	if (scan_range > scanLimit)
	{
		TrajctoryResultP out;
		out.messageCode = MESSAGE_TOOL_SCAN_OUT_OF_RANGE;
		DEBUG_PRINT_VALUE(scanLimit);
		DEBUG_PRINT_VALUE(scan_range);
		return out;
	}
	if (step_range > stepLimit)
	{
		TrajctoryResultP out;
		out.messageCode = MESSAGE_TOOL_STEP_OUT_OF_RANGE;
		DEBUG_PRINT_VALUE(stepLimit);
		DEBUG_PRINT_VALUE(step_range);
		return out;
	}

	double scan = scan_range;
	double step = step_range;

	double hs = step * (-1);
	Vector3d H1 = hs * translh;
	double vs = scan;
	Vector3d V1 = vs * translv;
	//double x = 0.0;
	//Vector3d X1 = x * toolTranslx;
	//DEBUG_PRINT_VALUE(X1)
	// DEBUG_PRINT_VALUE(H1)
	// DEBUG_PRINT_VALUE(V1)

	Vector3d trans_3x1 = H1 + V1;
	VectorXd trans_6x1(6);
	trans_6x1 << trans_3x1, 0, 0, 0;
	VectorXd Pend = P_6x1 + trans_6x1;

	TrajctoryResultP out;
	out.trajP = Pend;
	out.messageCode = MESSAGE_OK;
	return out;
}


#pragma endregion generic

#pragma region N type





void GMT::StewartBasic::SetStewartNTypeDefaultParameters()
{

#if 0U
	_bb.col(0) << 27.79095418441866, -25.6223118691413, 0;
	_bb.col(1) << 8.294095890154637, -36.87882825368669, 0;
	_bb.col(2) << -36.0850500745733, -11.25651638454532, 0;
	_bb.col(3) << -36.0850500745733, 11.25651638454532, 0;
	_bb.col(4) << 8.294095890154637, 36.87882825368669, 0;
	_bb.col(5) << 27.79095418441866, 25.6223118691413, 0;

	_pp.col(0) << 25.20360654775701, -8.3474976481494, 0;
	_pp.col(1) << -5.372658252550269, -26.00071236142009, 0;
	_pp.col(2) << -19.83094829520674, -17.65321471327068, 0;
	_pp.col(3) << -19.83094829520674, 17.65321471327068, 0;
	_pp.col(4) << -5.372658252550269, 26.00071236142009, 0;
	_pp.col(5) << 25.20360654775701, 8.3474976481494, 0;
#else
	_bb.col(5) << 27.79095418441866, -25.6223118691413, 0;
	_bb.col(4) << 8.294095890154637, -36.87882825368669, 0;
	_bb.col(3) << -36.0850500745733, -11.25651638454532, 0;
	_bb.col(2) << -36.0850500745733, 11.25651638454532, 0;
	_bb.col(1) << 8.294095890154637, 36.87882825368669, 0;
	_bb.col(0) << 27.79095418441866, 25.6223118691413, 0;

	_pp.col(5) << 25.20360654775701, -8.3474976481494, 0;
	_pp.col(4) << -5.372658252550269, -26.00071236142009, 0;
	_pp.col(3) << -19.83094829520674, -17.65321471327068, 0;
	_pp.col(2) << -19.83094829520674, 17.65321471327068, 0;
	_pp.col(1) << -5.372658252550269, 26.00071236142009, 0;
	_pp.col(0) << 25.20360654775701, 8.3474976481494, 0;
#endif
	_h0 = 21.2;
	_g0 = 49.41;
	_Lupp = 58.5;
	_Llow = 50.5;
	_workSpaceDeltaTrans = 0.001;
	_workSpaceDeltaRot = 0.0001;

	_PHome_1x6 << 0, 0, 66.4236, 0, 0, 0;
	_initGuess_1x6 << 0, 0, 75, 0, 0, 0;

	_workSpaceTestLimit.clear();
	_workSpaceTestLimit.reserve(12);
	//x
	_workSpaceTestLimit.push_back(50);
	_workSpaceTestLimit.push_back(-50);
	//y
	_workSpaceTestLimit.push_back(40);
	_workSpaceTestLimit.push_back(-40);
	//z
	_workSpaceTestLimit.push_back(40);
	_workSpaceTestLimit.push_back(-40);
	//phi
	_workSpaceTestLimit.push_back(0.5);
	_workSpaceTestLimit.push_back(-0.5);
	//theta
	_workSpaceTestLimit.push_back(0.5);
	_workSpaceTestLimit.push_back(-0.5);
	//psi
	_workSpaceTestLimit.push_back(0.5);
	_workSpaceTestLimit.push_back(-0.5);

	_stewartType = TypeN;
	_fkCallback = N_FKCallback;
}

IKOutput GMT::StewartBasic::N_IK(VectorXd P, Mat pp, Mat bb, double h0, double g0)
{
	Vector3d bd = P.segment(0, 3);

	Mat R = MathTool::poseRPY2SO3_2(P);

	Vector3d bb1 = bb.col(0);
	Vector3d bb2 = bb.col(1);
	Vector3d bb3 = bb.col(2);
	Vector3d bb4 = bb.col(3);
	Vector3d bb5 = bb.col(4);
	Vector3d bb6 = bb.col(5);

	Vector3d pp1 = pp.col(0);
	Vector3d pp2 = pp.col(1);
	Vector3d pp3 = pp.col(2);
	Vector3d pp4 = pp.col(3);
	Vector3d pp5 = pp.col(4);
	Vector3d pp6 = pp.col(5);

	Vector3d Bx1 = bd - bb1;
	Vector3d Bx2 = bd - bb2;
	Vector3d Bx3 = bd - bb3;
	Vector3d Bx4 = bd - bb4;
	Vector3d Bx5 = bd - bb5;
	Vector3d Bx6 = bd - bb6;

	Vector3d Bp1 = R * pp1;
	Vector3d Bp2 = R * pp2;
	Vector3d Bp3 = R * pp3;
	Vector3d Bp4 = R * pp4;
	Vector3d Bp5 = R * pp5;
	Vector3d Bp6 = R * pp6;

	Vector3d Lv1 = Bx1 + Bp1;
	Vector3d Lv2 = Bx2 + Bp2;
	Vector3d Lv3 = Bx3 + Bp3;
	Vector3d Lv4 = Bx4 + Bp4;
	Vector3d Lv5 = Bx5 + Bp5;
	Vector3d Lv6 = Bx6 + Bp6;

	double lz1 = Lv1(2);
	double lz2 = Lv2(2);
	double lz3 = Lv3(2);
	double lz4 = Lv4(2);
	double lz5 = Lv5(2);
	double lz6 = Lv6(2);
	VectorXd alpha(6);
	alpha << lz1, lz2, lz3, lz4, lz5, lz6;

	double l1 = sqrt(SQUARE(Bx1(0) + Bp1(0)) + SQUARE(Bx1(1) + Bp1(1)) + SQUARE(Bx1(2) + Bp1(2)));
	double l2 = sqrt(SQUARE(Bx2(0) + Bp2(0)) + SQUARE(Bx2(1) + Bp2(1)) + SQUARE(Bx2(2) + Bp2(2)));
	double l3 = sqrt(SQUARE(Bx3(0) + Bp3(0)) + SQUARE(Bx3(1) + Bp3(1)) + SQUARE(Bx3(2) + Bp3(2)));
	double l4 = sqrt(SQUARE(Bx4(0) + Bp4(0)) + SQUARE(Bx4(1) + Bp4(1)) + SQUARE(Bx4(2) + Bp4(2)));
	double l5 = sqrt(SQUARE(Bx5(0) + Bp5(0)) + SQUARE(Bx5(1) + Bp5(1)) + SQUARE(Bx5(2) + Bp5(2)));
	double l6 = sqrt(SQUARE(Bx6(0) + Bp6(0)) + SQUARE(Bx6(1) + Bp6(1)) + SQUARE(Bx6(2) + Bp6(2)));

	VectorXd LL(6);
	LL << l1, l2, l3, l4, l5, l6;

	IKOutput out;
	out.stewart = TypeN;
	out.alpha = alpha;
	out.Ls = LL;
	
	Mat tarr = alpha.array().square() + h0 * h0 - LL.array().square();
	auto illegal_lenth = (tarr.array() < 0);
	if (illegal_lenth.any()) {
		DEBUG_PRINT_ERROR_MESSAGE("Pose cannot be reached");
		DEBUG_PRINT_ERROR_MESSAGE("IK fail !!!");
		out.messageCode = MESSAGE_TOOL_N_IK_FAIL_POSE_CANT_BE_REACHED;
	}
	else
	{
		out.messageCode = MESSAGE_OK;
	}

	VectorXd tv = tarr.array().sqrt();
	VectorXd g = alpha - tv;
	//VectorXd dg = g.array() - g0;
	out.g = g;

	return out;

}

IKOutput GMT::StewartBasic::N_IK(VectorXd P)
{
	return N_IK(P, _pp, _bb, _h0, _g0);
}

FKOutput GMT::StewartBasic::N_FK(VectorXd L, VectorXd initGuess)
{
	return CminpackHybrdFK(L, initGuess);
}

FKOutput GMT::StewartBasic::N_FK(VectorXd L)
{
	return CminpackHybrdFK( L, _initGuess_1x6);
}


double StewartBasic::_h0 = 0;
double StewartBasic::_g0 = 0;


void GMT::StewartBasic::N_FKCallback(const int * n, const double * x, double * fvec, int * iflag)
{
#pragma region
	//if (_is_show_fk_debug)
	   // cout << "x in : " << x[0] << ", " << x[1] << ", " << x[2] << ", " << x[3] << ", " << x[4] << ", " << x[5] << endl;

	VectorXd P(6);
	P << x[0], x[1], x[2], x[3], x[4], x[5];
	Mat R = MathTool::poseRPY2SO3_2(P);
	Vector3d X = P.segment(0, 3);

	Vector3d bb1 = _bb.col(0);
	Vector3d bb2 = _bb.col(1);
	Vector3d bb3 = _bb.col(2);
	Vector3d bb4 = _bb.col(3);
	Vector3d bb5 = _bb.col(4);
	Vector3d bb6 = _bb.col(5);

	Vector3d pp1 = _pp.col(0);
	Vector3d pp2 = _pp.col(1);
	Vector3d pp3 = _pp.col(2);
	Vector3d pp4 = _pp.col(3);
	Vector3d pp5 = _pp.col(4);
	Vector3d pp6 = _pp.col(5);

	Vector3d Bp1 = R * pp1;
	Vector3d Bp2 = R * pp2;
	Vector3d Bp3 = R * pp3;
	Vector3d Bp4 = R * pp4;
	Vector3d Bp5 = R * pp5;
	Vector3d Bp6 = R * pp6;

	double l1 = _L_1x6(0);
	double l2 = _L_1x6(1);
	double l3 = _L_1x6(2);
	double l4 = _L_1x6(3);
	double l5 = _L_1x6(4);
	double l6 = _L_1x6(5);

#pragma endregion

	double h02 = SQUARE(_h0);
	fvec[0] = SQUARE(norm(X + R * pp1 - bb1)) - (h02 - SQUARE(l1)) - 2 * l1*(x[2] + Bp1(2) - bb1(2));
	fvec[1] = SQUARE(norm(X + R * pp2 - bb2)) - (h02 - SQUARE(l2)) - 2 * l2*(x[2] + Bp2(2) - bb2(2));
	fvec[2] = SQUARE(norm(X + R * pp3 - bb3)) - (h02 - SQUARE(l3)) - 2 * l3*(x[2] + Bp3(2) - bb3(2));
	fvec[3] = SQUARE(norm(X + R * pp4 - bb4)) - (h02 - SQUARE(l4)) - 2 * l4*(x[2] + Bp4(2) - bb4(2));
	fvec[4] = SQUARE(norm(X + R * pp5 - bb5)) - (h02 - SQUARE(l5)) - 2 * l5*(x[2] + Bp5(2) - bb5(2));
	fvec[5] = SQUARE(norm(X + R * pp6 - bb6)) - (h02 - SQUARE(l6)) - 2 * l6*(x[2] + Bp6(2) - bb6(2));

	//if (_is_show_fk_debug)
	   // cout << "f: " << fvec[0] << ", " << fvec[1] << ", " << fvec[2] << ", " << fvec[3] << ", " << fvec[4] << ", " << fvec[5] << endl << endl;

	return;
}

IKOutput GMT::StewartBasic::F_IK(VectorXd P, Mat pp, Mat bb)
{

	Vector3d bd = P.segment(0, 3);

	Mat R = MathTool::poseRPY2SO3_2(P);

	Vector3d bb1 = bb.col(0);
	Vector3d bb2 = bb.col(1);
	Vector3d bb3 = bb.col(2);
	Vector3d bb4 = bb.col(3);
	Vector3d bb5 = bb.col(4);
	Vector3d bb6 = bb.col(5);

	Vector3d pp1 = pp.col(0);
	Vector3d pp2 = pp.col(1);
	Vector3d pp3 = pp.col(2);
	Vector3d pp4 = pp.col(3);
	Vector3d pp5 = pp.col(4);
	Vector3d pp6 = pp.col(5);

	Vector3d Bx1 = bd - bb1;
	Vector3d Bx2 = bd - bb2;
	Vector3d Bx3 = bd - bb3;
	Vector3d Bx4 = bd - bb4;
	Vector3d Bx5 = bd - bb5;
	Vector3d Bx6 = bd - bb6;

	Vector3d BP1 = R * pp1;
	Vector3d BP2 = R * pp2;
	Vector3d BP3 = R * pp3;
	Vector3d BP4 = R * pp4;
	Vector3d BP5 = R * pp5;
	Vector3d BP6 = R * pp6;

	double l1 = sqrt(pow(Bx1(0) + BP1(0), 2) + pow(Bx1(1) + BP1(1), 2) + pow(Bx1(2) + BP1(2), 2));
	double l2 = sqrt(pow(Bx2(0) + BP2(0), 2) + pow(Bx2(1) + BP2(1), 2) + pow(Bx2(2) + BP2(2), 2));
	double l3 = sqrt(pow(Bx3(0) + BP3(0), 2) + pow(Bx3(1) + BP3(1), 2) + pow(Bx3(2) + BP3(2), 2));
	double l4 = sqrt(pow(Bx4(0) + BP4(0), 2) + pow(Bx4(1) + BP4(1), 2) + pow(Bx4(2) + BP4(2), 2));
	double l5 = sqrt(pow(Bx5(0) + BP5(0), 2) + pow(Bx5(1) + BP5(1), 2) + pow(Bx5(2) + BP5(2), 2));
	double l6 = sqrt(pow(Bx6(0) + BP6(0), 2) + pow(Bx6(1) + BP6(1), 2) + pow(Bx6(2) + BP6(2), 2));

	VectorXd L(6);
	L << l1, l2, l3, l4, l5, l6;

	IKOutput out;
	out.stewart = TypeF;
	out.LL = L;
	return out;
}

IKOutput GMT::StewartBasic::F_IK(VectorXd P)
{
	return F_IK(P, _pp, _bb);
}

FKOutput GMT::StewartBasic::F_FK(VectorXd L, VectorXd initGuess)
{
	return CminpackHybrdFK(L, initGuess);
}

FKOutput GMT::StewartBasic::F_FK(VectorXd L)
{
	return CminpackHybrdFK(L, _initGuess_1x6);
}

void GMT::StewartBasic::SetStewartFTypeDefaultParameters()
{
	_bb.col(0) << 97.25217186425962, -100.70757204741115, 0;
	_bb.col(1) << 38.58922981437988, -134.57663743136464, 0;
	_bb.col(2) << -135.8414016786395, -33.86906538395348, 0;
	_bb.col(3) << -135.8414016786395, 33.86906538395348, 0;
	_bb.col(4) << 38.58922981437988, 134.57663743136464, 0;
	_bb.col(5) << 97.25217186425962, 100.70757204741115, 0;

	_pp.col(0) << 83.40418343188286, -26.4573654406638, -20.3;
	_pp.col(1) << -18.78934112711802, -85.4588243542396, -20.3;
	_pp.col(2) << -64.6148423047648, -59.00145891357579, -20.3;
	_pp.col(3) << -64.6148423047648, 59.00145891357579, -20.3;
	_pp.col(4) << -18.78934112711802, 85.4588243542396, -20.3;
	_pp.col(5) << 83.40418343188286, 26.4573654406638, -20.3;


	_Lx = 167.5;
	_Lupp = 181;
	_Llow = 164;
	_workSpaceDeltaTrans = 0.001;
	_workSpaceDeltaRot = 0.0001;

	_PHome_1x6 << 0, 0, 155.085, 0, 0, 0;
	_initGuess_1x6 << 0, 0, 168.32, 0, 0, 0;

	_workSpaceTestLimit.clear();
	_workSpaceTestLimit.reserve(12);
	//x
	_workSpaceTestLimit.push_back(50);
	_workSpaceTestLimit.push_back(-50);
	//y
	_workSpaceTestLimit.push_back(40);
	_workSpaceTestLimit.push_back(-40);
	//z
	_workSpaceTestLimit.push_back(40);
	_workSpaceTestLimit.push_back(-40);
	//phi
	_workSpaceTestLimit.push_back(0.25);
	_workSpaceTestLimit.push_back(-0.25);
	//theta
	_workSpaceTestLimit.push_back(0.5);
	_workSpaceTestLimit.push_back(-0.5);
	//psi
	_workSpaceTestLimit.push_back(0.5);
	_workSpaceTestLimit.push_back(-0.5);
	
	_stewartType = TypeF;
	_fkCallback = F_FKCallback;

}

void GMT::StewartBasic::F_FKCallback(const int * n, const double * x, double * fvec, int * iflag)
{
#pragma region
	//if (_is_show_fk_debug)
	   // cout << "x in : " << x[0] << ", " << x[1] << ", " << x[2] << ", " << x[3] << ", " << x[4] << ", " << x[5] << endl;

	//syms x1 x2 x3 a b g


	VectorXd P(6);
	P << x[0], x[1], x[2], x[3], x[4], x[5];
	Mat R = MathTool::poseRPY2SO3_2(P);
	Vector3d X = P.segment(0, 3);


	Vector3d bb1 = _bb.col(0);
	Vector3d bb2 = _bb.col(1);
	Vector3d bb3 = _bb.col(2);
	Vector3d bb4 = _bb.col(3);
	Vector3d bb5 = _bb.col(4);
	Vector3d bb6 = _bb.col(5);

	Vector3d pp1 = _pp.col(0);
	Vector3d pp2 = _pp.col(1);
	Vector3d pp3 = _pp.col(2);
	Vector3d pp4 = _pp.col(3);
	Vector3d pp5 = _pp.col(4);
	Vector3d pp6 = _pp.col(5);

	Vector3d Bp1 = R * pp1;
	Vector3d Bp2 = R * pp2;
	Vector3d Bp3 = R * pp3;
	Vector3d Bp4 = R * pp4;
	Vector3d Bp5 = R * pp5;
	Vector3d Bp6 = R * pp6;

	double l1 = _L_1x6(0);
	double l2 = _L_1x6(1);
	double l3 = _L_1x6(2);
	double l4 = _L_1x6(3);
	double l5 = _L_1x6(4);
	double l6 = _L_1x6(5);

#pragma endregion

	fvec[0] = SQUARE(MathTool::norm(X + R * pp1 - bb1)) - SQUARE(l1);
	fvec[1] = SQUARE(MathTool::norm(X + R * pp2 - bb2)) - SQUARE(l2);
	fvec[2] = SQUARE(MathTool::norm(X + R * pp3 - bb3)) - SQUARE(l3);
	fvec[3] = SQUARE(MathTool::norm(X + R * pp4 - bb4)) - SQUARE(l4);
	fvec[4] = SQUARE(MathTool::norm(X + R * pp5 - bb5)) - SQUARE(l5);
	fvec[5] = SQUARE(MathTool::norm(X + R * pp6 - bb6)) - SQUARE(l6);

	//if (_is_show_fk_debug)
	   // cout << "f: " << fvec[0] << ", " << fvec[1] << ", " << fvec[2] << ", " << fvec[3] << ", " << fvec[4] << ", " << fvec[5] << endl << endl;

	return;
}

void GMT::StewartBasic::SetStewartA12DefaultParameters()
{
	_PHome_1x6 << 0, 0, 0, 0, 0, 0;

	_workSpaceTestLimit.clear();
	_workSpaceTestLimit.reserve(12);
	//x
	_workSpaceTestLimit.push_back(25.88);
	_workSpaceTestLimit.push_back(-25.67);
	//y
	_workSpaceTestLimit.push_back(20.31);
	_workSpaceTestLimit.push_back(-30.11);
	//z
	_workSpaceTestLimit.push_back(3.97);
	_workSpaceTestLimit.push_back(-6.33);
	//phi
	_workSpaceTestLimit.push_back(MathTool::degree2Radian(6.54));
	_workSpaceTestLimit.push_back(MathTool::degree2Radian(-6.1));
	//theta
	_workSpaceTestLimit.push_back(MathTool::degree2Radian(5.3));
	_workSpaceTestLimit.push_back(MathTool::degree2Radian(-4.89));
	//psi
	_workSpaceTestLimit.push_back(MathTool::degree2Radian(7.18));
	_workSpaceTestLimit.push_back(MathTool::degree2Radian(-7.66));

	_stewartType = TypeA12;

	_pivot = Vector3d{ 0,0,0 };
	_alpha = 0;
	_beta = 0;
}

WSOutput GMT::StewartBasic::A12_ToolWS(VectorXd PToolDelta, enumWSBaseMode baseMode)
{

	WSOutput out;

	//DEBUG_PRINT_CODE_LOCATION
	//DEBUG_PRINT_VALUE(ToolPoseWorking)
	   // DEBUG_PRINT_VALUE(_TposeStart)

	Matrix4d TtoolPoseWorking = poseRPY2SE3(PToolDelta);
	_TposeStart = _TposeStart * TtoolPoseWorking;
	out.Tstart = _TposeStart;
	//DEBUG_PRINT_VALUE(_TposeStart)
	   // DEBUG_PRINT_VALUE(out.tstart)
	Matrix3d rstart = t2r(_TposeStart);
	Vector3d pstart = t2p(_TposeStart);
	Vector3d ptp = sO3ToPoseRPY(rstart);
	double phi = ptp(0);
	double theta = ptp(1);
	double psi = ptp(2);

	double ax = pstart(0);
	double ay = pstart(1);
	double az = pstart(2);

	double pdx = _workSpaceTestLimit[0] - ax;
	double mdx = _workSpaceTestLimit[1] - ax;
	double pdy = _workSpaceTestLimit[2] - ay;
	double mdy = _workSpaceTestLimit[3] - ay;
	double pdz = _workSpaceTestLimit[4] - az;
	double mdz = _workSpaceTestLimit[5] - az;
	double pdrz = _workSpaceTestLimit[6] - phi;
	double mdrz = _workSpaceTestLimit[7] - phi;
	double pdry = _workSpaceTestLimit[8] - theta;
	double mdry = _workSpaceTestLimit[9] - theta;
	double pdrx = _workSpaceTestLimit[10] - psi;
	double mdrx = _workSpaceTestLimit[11] - psi;

	VectorXd pd(6);
	pd << pdx, pdy, pdz, pdrz, pdry, pdrx;
	VectorXd md(6);
	md << mdx, mdy, mdz, mdrz, mdry, mdrx;

	//DEBUG_PRINT_CODE_LOCATION
	//DEBUG_PRINT_VALUE(pd)
	   // DEBUG_PRINT_VALUE(md)

	auto illegalp = (pd.array() < 0);
	auto illegalm = (md.array() > 0);
	if (illegalp.any() || illegalm.any())
	{
		out.messageCode = MESSAGE_INPUT_ILLEGAL;
		return out;
	}

	VectorXd RemValue12(12);
	RemValue12 << pdx, mdx,
		pdy, mdy,
		pdz, mdz,
		pdrz, mdrz,
		pdry, mdry,
		pdrx, mdrx;

	_WSRemain12_1x12 = RemValue12;
	out.remain12 = RemValue12;
	out.messageCode = MESSAGE_OK;
	out.TMtool = Matrix4d::Zero();

	return out;
}

bool GMT::StewartBasic::A12_checkP(VectorXd P)
{
	// 0 base
	double phi = P(3);
	double theta = P(4);
	double psi = P(5);

	double ax = P(0);
	double ay = P(1);
	double az = P(2);

	double pdx = _workSpaceTestLimit[0] - ax;
	double mdx = _workSpaceTestLimit[1] - ax;
	double pdy = _workSpaceTestLimit[2] - ay;
	double mdy = _workSpaceTestLimit[3] - ay;
	double pdz = _workSpaceTestLimit[4] - az;
	double mdz = _workSpaceTestLimit[5] - az;
	double pdrz = _workSpaceTestLimit[6] - phi;
	double mdrz = _workSpaceTestLimit[7] - phi;
	double pdry = _workSpaceTestLimit[8] - theta;
	double mdry = _workSpaceTestLimit[9] - theta;
	double pdrx = _workSpaceTestLimit[10] - psi;
	double mdrx = _workSpaceTestLimit[11] - psi;

	VectorXd pd(6);
	pd << pdx, pdy, pdz, pdrz, pdry, pdrx;
	VectorXd md(6);
	md << mdx, mdy, mdz, mdrz, mdry, mdrx;

	//DEBUG_PRINT_CODE_LOCATION
	//DEBUG_PRINT_VALUE(pd)
	   // DEBUG_PRINT_VALUE(md)

	auto illegalp = (pd.array() < 0);
	auto illegalm = (md.array() > 0);
	if (illegalp.any() || illegalm.any())
		return false;
	else
		return true;
}




#pragma endregion N type


#pragma region other class

GMT::MessageRecorder::MessageRecorder(int limit)
{
	_recorderLimit = limit;
}

void GMT::MessageRecorder::Add(enumMessageLevel level, int message)
{
	if (_records.size() > _recorderLimit)
	{
		DEBUG_PRINT_ERROR_MESSAGE("recorder limit riched");
		return;
	}

	MessageRecord record;
	record.timeStamp = chrono::system_clock::to_time_t(chrono::system_clock::now());
	record.level = level;
	record.messageCode = message;
	_records.push_back(record);
}

const vector<MessageRecord> GMT::MessageRecorder::GetRecords()
{
	return _records;
}

const int GMT::MessageRecorder::GetCounts()
{
	return _records.size();
}

void GMT::MessageRecorder::Clear()
{
	_records.clear();
}



#pragma endregion other class