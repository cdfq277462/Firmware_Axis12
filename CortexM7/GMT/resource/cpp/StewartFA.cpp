#include "..\header\StewartFA.h"

GMT::StewartFA::StewartFA()
{
	SetLightSourceOff();
	ClearLdataRecord();
}

GMT::StewartFA::StewartFA(enumStewartType type)
	:StewartBasic(type)//㊣basic篶Α盿诀竟把计磅︽FA篶
{
	StewartFA();
}

void GMT::StewartFA::SetLightSourceOff()
{
	_lightSourceType = LightSourceNonuse;
}

void GMT::StewartFA::SetFALightSource(Vector2d center, Vector2d radius)
{
	_lightSourceType = LightFromSimulation;
	_simulationLightSourceCenter = center;
	_simulationLightSourceRadius = radius;

}

void GMT::StewartFA::SetFALightSource(LightSourceFunc source)
{
	_lightSourceType = LightFromReal;
	_realLightSource = source;
}

void GMT::StewartFA::ClearLdataRecord()
{
	_LdataRecord.Reset();
}

const LDataLogic GMT::StewartFA::GetLdataRecord()
{
	return _LdataRecord;
}

double GMT::StewartFA::GetLightData(VectorXd P)
{
	if (_lightSourceType == LightSourceNonuse) return MESSAGE_FA_LIGHT_SOURCE_NONUSE;

	if (_lightSourceType == LightFromReal)
	{
		double value = _realLightSource();
		if (value > 1)
		{
			DEBUG_PRINT_ERROR_MESSAGE("light value >1");
			DEBUG_PRINT_VALUE(value);
		}

		return value;
	}

	if (_lightSourceType == LightFromSimulation)
	{

		double y = P(1);
		double z = P(2);

		//double y1 = yzRange[0]; //y max
		//double y2 = yzRange[1];	//y min
		//double z1 = yzRange[2];	//z max
		//double z2 = yzRange[3];	//z min
		double cy = _simulationLightSourceCenter(0);// *y1 + (1 - _r(0)) * y2;
		double cz = _simulationLightSourceCenter(1);// * z1 + (1 - _r(1)) * z2;

		double s1 = _simulationLightSourceRadius(0);
		double s2 = _simulationLightSourceRadius(1);

		return exp(-(SQUARE(y - cy) / SQUARE(s1) + SQUARE(z - cz) / SQUARE(s2)));
	}

	return MESSAGE_FA_LIGHT_SOURCE_ERROR;
}

void GMT::StewartFA::SetFAThreshold(double thd0, double thd1)
{
	_LdataRecord.SetThreshold(thd0, thd1);
}


FATrajctoryResultP GMT::StewartFA::FAMoveLPP(VectorXd Pstart, VectorXd Pend, double speed, int k, VectorXd PCurrent_6x1)
{
	Matrix4d Tstart = poseRPY2SE3(Pstart);
	Matrix4d Tend = poseRPY2SE3(Pend);

	return FAMoveLPP(Tstart, Tend, speed,  k,  PCurrent_6x1);
}

FATrajctoryResultP GMT::StewartFA::FAMoveLPP(Matrix4d Tstart, Matrix4d Tend, double speed, int k, VectorXd PCurrent_6x1)
{
	TrajctoryResultP toolMoveL = ToolMoveLPP(
		Tstart,
		Tend,
		speed,
		k
	);
	   
	FATrajctoryResultP faMoveL;
	faMoveL.GetData(toolMoveL);

	if (_lightSourceType != LightSourceNonuse)
	{
		double ldata = GetLightData(PCurrent_6x1);
		faMoveL.lightDataCurr = ldata;
		_LdataRecord.Updata(ldata, PCurrent_6x1);
	}

	return faMoveL;
}


PLList GMT::StewartFA::FAMoveLPP(VectorXd Pstart, VectorXd Pend, double speed)
{
	Matrix4d Tstart = poseRPY2SE3(Pstart);
	Matrix4d Tend = poseRPY2SE3(Pend);

	return FAMoveLPP(Tstart, Tend, speed);
}


PLList GMT::StewartFA::FAMoveLPP(Matrix4d Tstart, Matrix4d Tend, double speed)
{
	PLList out;

	int k = 1;
	int stage = 1;
	VectorXd PCurrent_6x1 = sE3ToPoseRPY(Tstart);
	while (true)
	{
		FATrajctoryResultP faMoveLk = FAMoveLPP(
			Tstart,
			Tend,
			speed,
			k,
			PCurrent_6x1
		);

		if (faMoveLk.messageCode != MESSAGE_OK)
		{
			out.messageCode = faMoveLk.messageCode;
			break;
		}
		
		out.add(faMoveLk);

		k = faMoveLk.kNext;
		PCurrent_6x1 = faMoveLk.trajP;
	}

	return out;
}

FATrajctoryResultP GMT::StewartFA::FASpiral(Matrix4d Tstart, double scanRange, double lineSpacing,
	double speedSpiral, //Spiral瓂格硉
	double speedMoveL, //SpiralmoveL硉
	enumSpiralTEvaluate spiraltimeEva, int k, int stage, VectorXd PCurrent_6x1, enumFAEndPointOption endOption)
{
	ToolTrajctoryResultP toolSpiralP;

	if (stage == SPIRAL_STAGE_START_SPIRAL)
	{
		toolSpiralP = ToolSpiral(
			Tstart,
			scanRange,
			lineSpacing,
			 speedSpiral, //Spiral瓂格硉
			 speedMoveL, //SpiralmoveL硉
			spiraltimeEva,
			k,
			stage
		);
	}
	else if (stage == SPIRAL_STAGE_SPIRAL_END_BACK_TO_START)
	{
		toolSpiralP = FATrajctoryEndOperation(Tstart, PCurrent_6x1, k, endOption, speedMoveL, stage);
	}
	else
	{
		FATrajctoryResultP errorOut;
		errorOut.messageCode = MESSAGE_TOOL_STAGE_ERROR;
		return errorOut;
	}

	FATrajctoryResultP faEndMoveLk;
	faEndMoveLk.GetData(toolSpiralP);

	if (_lightSourceType != LightSourceNonuse)
	{
		double ldata = GetLightData(PCurrent_6x1);
		faEndMoveLk.lightDataCurr = ldata;
		_LdataRecord.Updata(ldata, PCurrent_6x1);
	}

	return faEndMoveLk;

}

PLList GMT::StewartFA::FASpiral(Matrix4d Tstart, double scanRange, double lineSpacing,
	double speedSpiral, //Spiral瓂格硉
	double speedMoveL, //SpiralmoveL硉 
	enumSpiralTEvaluate spiraltimeEva, enumFAEndPointOption endOption)
{
	PLList out;

	int k = 1;
	int stage = SPIRAL_STAGE_START_SPIRAL;
	VectorXd PCurrent_6x1 = sE3ToPoseRPY(Tstart);
	while (true)
	{
		FATrajctoryResultP faSpiralk = FASpiral(
			Tstart,
			scanRange,
			lineSpacing,
			 speedSpiral, //Spiral瓂格硉
			 speedMoveL, //SpiralmoveL硉
			spiraltimeEva,
			k,
			stage,
			PCurrent_6x1,
			endOption
		);

		if (faSpiralk.messageCode != MESSAGE_OK)
		{
			out.messageCode = faSpiralk.messageCode;
			break;
		}

		out.add(faSpiralk);

		k = faSpiralk.kNext;
		stage = faSpiralk.stageNext;
		PCurrent_6x1 = faSpiralk.trajP;
		//DEBUG_PRINT_CODE_LOCATION;
		//DEBUG_PRINT_VALUE(PCurrent_6x1);
		//DEBUG_PRINT_VALUE(faSpiralk.lightDataCurr);
	}

	return out;
}

FATrajctoryResultP GMT::StewartFA::FASin(Matrix4d Tstart, double scanRange, double stepRange, double lineSpacing,
	double speedSin, //sin瓂格硉
	double speedMoveL, //sin玡moveL硉
	double frequency, enumSinScanOption sinMode, int k, int stage, VectorXd PCurrent_6x1, enumFAEndPointOption endOption)
{

	ToolTrajctoryResultP toolSinP;
	if (stage == SIN_STAGE_START_TO_SIN
		|| stage == SIN_STAGE_SIN)
	{
		toolSinP = ToolSin(
			Tstart, //瓂格癬翴4x4痻皚
			scanRange, //
			stepRange,
			lineSpacing, //埃lineSpacing临穦velocity紇臫
			 speedSin, //sin瓂格硉
			 speedMoveL, //sin玡moveL硉
			frequency,
			sinMode, //
			k, //﹚璸衡材k瓂格翴(1 base)
			stage
		);
	}
	else if (stage == SIN_STAGE_SIN_END_TO_START)
	{
		toolSinP = FATrajctoryEndOperation(Tstart, PCurrent_6x1, k, endOption, speedMoveL, stage);
	}
	else
	{
		FATrajctoryResultP errorOut;
		errorOut.messageCode = MESSAGE_TOOL_STAGE_ERROR;
		return errorOut;
	}

	FATrajctoryResultP faEndMoveLk;
	faEndMoveLk.GetData(toolSinP);

	if (_lightSourceType != LightSourceNonuse)
	{
		double ldata = GetLightData(PCurrent_6x1);
		faEndMoveLk.lightDataCurr = ldata;
		_LdataRecord.Updata(ldata, PCurrent_6x1);
	}

	return faEndMoveLk;
}

PLList GMT::StewartFA::FASin(Matrix4d Tstart, double scanRange, double stepRange, double lineSpacing,
	double speedSin, //sin瓂格硉
	double speedMoveL, //sin玡moveL硉
	double frequency, enumSinScanOption sinMode, enumFAEndPointOption endOption)
{
	PLList out;

	int k = 1;
	int stage = SIN_STAGE_START_TO_SIN;
	VectorXd PCurrent_6x1 = sE3ToPoseRPY(Tstart);
	while (true)
	{
		FATrajctoryResultP faSink = FASin(
			Tstart,
			scanRange,
			stepRange, 
			lineSpacing,
			speedSin, //sin瓂格硉
			speedMoveL, //sin玡moveL硉
			frequency,
			sinMode,
			k,
			stage,
			PCurrent_6x1,
			endOption
		);

		if (faSink.messageCode != MESSAGE_OK)
		{
			out.messageCode = faSink.messageCode;
			break;
		}

		out.add(faSink);

		k = faSink.kNext;
		stage = faSink.stageNext;
		PCurrent_6x1 = faSink.trajP;
	}

	return out;
}

ToolTrajctoryResultP GMT::StewartFA::FATrajctoryEndOperation(Matrix4d Tstart, VectorXd PCurrent_6x1, int k, enumFAEndPointOption endOption, double speed, int stageCode)
{
	if (endOption == FAEndTrajctoryEnd)
	{
		ToolTrajctoryResultP endP;
		endP.messageCode = MESSAGE_TRAJCTORY_NORMAL_END;
		return endP;
	}
	else
	{
		Matrix4d TmoveLTo;
		if (endOption == FAEndToMaxLight)
		{
			TmoveLTo = poseRPY2SE3(_LdataRecord.GetPMax());
		}
		else if (endOption == FAEndToWeightCenter)
		{
			TmoveLTo = poseRPY2SE3(_LdataRecord.GetPAverage());
		}
		else if (endOption == FAEndToStart)
		{
			TmoveLTo = Tstart;
		}
		
		ToolTrajctoryResultP endMoveLk = FAMoveLPP(
			_TTrajctoryEnd,
			TmoveLTo,
			speed,
			k,
			PCurrent_6x1
		);
		endMoveLk.stageNext = stageCode;
		return endMoveLk;
	}

}
