#include "trajectory_generator.h"
#include <cstring>
#include <Aris_Plan.h>
#include <Robot_Base.h>


using namespace std;

void DecodeMsg(const Aris::Core::MSG &msg, std::string &cmd, std::map<std::string,std::string> &params)
{
	char content[500];

	std::int32_t size=0;
	std::int32_t beginPos=0;

	msg.PasteStruct(size);
	beginPos+=4;
	msg.PasteAt(content,size,beginPos);
	cmd.assign(content);
	beginPos+=size;

	std::int32_t paramNum;
	msg.PasteAt(&paramNum,4,beginPos);
	beginPos+=4;

	for(int i=0;i<paramNum;++i)
	{
		std::string cmdd,param;

		msg.PasteAt(&size,4,beginPos);
		beginPos+=4;
		msg.PasteAt(content,size,beginPos);
		cmdd.assign(content);
		beginPos+=size;

		msg.PasteAt(&size,4,beginPos);
		beginPos+=4;
		msg.PasteAt(content,size,beginPos);
		param.assign(content);
		beginPos+=size;

		params.insert(std::make_pair(cmdd,param));
	}

}
void GenerateCmdMsg(const std::string &cmd, const std::map<std::string,std::string> &params, Aris::Core::MSG &msg)
{
	if(cmd=="en")
	{
		MOTOR_PARAM robotState;
		robotState.cmdID=ENABLE;

		for(auto &i:params)
		{
			if(i.first=="all")
			{
				int motors[18]={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17};
				std::memcpy(robotState.motorID,motors,sizeof(motors));
				robotState.motorNum=18;
			}
			else if(i.first=="left")
			{
				int motors[9]={0,1,2,6,7,8,12,13,14};
				std::memcpy(robotState.motorID,motors,sizeof(motors));
				robotState.motorNum=9;
			}
			else if(i.first=="right")
			{
				int motors[9]={3,4,5,9,10,11,15,16,17};
				std::memcpy(robotState.motorID,motors,sizeof(motors));
				robotState.motorNum=9;
			}
			else if(i.first=="motor")
			{
				int motors[1]={stoi(i.second)};
				std::memcpy(robotState.motorID,motors,sizeof(motors));
				robotState.motorNum=1;
			}
		}

		msg.CopyStruct(robotState);
		return;
	}

	if(cmd=="ds")
	{
		MOTOR_PARAM robotState;
		robotState.cmdID=DISABLE;

		for(auto &i:params)
		{
			if(i.first=="all")
			{
				int motors[18]={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17};
				std::memcpy(robotState.motorID,motors,sizeof(motors));
				robotState.motorNum=18;
			}
			else if(i.first=="left")
			{
				int motors[9]={0,1,2,6,7,8,12,13,14};
				std::memcpy(robotState.motorID,motors,sizeof(motors));
				robotState.motorNum=9;
			}
			else if(i.first=="right")
			{
				int motors[9]={3,4,5,9,10,11,15,16,17};
				std::memcpy(robotState.motorID,motors,sizeof(motors));
				robotState.motorNum=9;
			}
			else if(i.first=="motor")
			{
				int motors[1]={stoi(i.second)};
				std::memcpy(robotState.motorID,motors,sizeof(motors));
				robotState.motorNum=1;
			}
		}

		msg.CopyStruct(robotState);
		return;
	}

	if(cmd=="hm")
	{
		MOTOR_PARAM robotState;
		robotState.cmdID=HOME;

		for(auto &i:params)
		{
			if(i.first=="all")
			{
				int motors[18]={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17};
				std::memcpy(robotState.motorID,motors,sizeof(motors));
				robotState.motorNum=18;
			}
			else if(i.first=="left")
			{
				int motors[9]={0,1,2,6,7,8,12,13,14};
				std::memcpy(robotState.motorID,motors,sizeof(motors));
				robotState.motorNum=9;
			}
			else if(i.first=="right")
			{
				int motors[9]={3,4,5,9,10,11,15,16,17};
				std::memcpy(robotState.motorID,motors,sizeof(motors));
				robotState.motorNum=9;
			}
			else if(i.first=="motor")
			{
				int motors[1]={stoi(i.second)};
				std::memcpy(robotState.motorID,motors,sizeof(motors));
				robotState.motorNum=1;
			}
		}

		msg.CopyStruct(robotState);
		return;
	}

	if(cmd=="ro")
	{
		MOTOR_PARAM robotState;
		robotState.cmdID=RESET_ORIGIN;
		msg.CopyStruct(robotState);
		return;
	}


	if(cmd=="bg")
	{

	}


	if(cmd=="wk")
	{
		Robots::WALK_PARAM  param;

		for(auto &i:params)
		{
			if(i.first=="totalCount")
			{
				param.totalCount=std::stoi(i.second);
			}
			else if(i.first=="n")
			{
				param.n=stoi(i.second);
			}
			else if(i.first=="walkDirection")
			{
				param.walkDirection=stoi(i.second);
			}
			else if(i.first=="upDirection")
			{
				param.upDirection=stoi(i.second);
			}
			else if(i.first=="distance")
			{
				param.d=stod(i.second);
			}
			else if(i.first=="height")
			{
				param.h=stod(i.second);
			}
			else if(i.first=="alpha")
			{
				param.alpha=stod(i.second);
			}
			else if(i.first=="beta")
			{
				param.beta=stod(i.second);
			}
		}


		param.cmdID=WALK;
		msg.CopyStruct(param);
		return;
	}
}

int HEXBOT_HOME_OFFSETS_RESOLVER[18]=
{
		-15849882+349000,-16354509+349000,-16354509+349000,
		-15849882+349000,-16354509+349000,-16354509+349000,
		-15849882+349000,-16354509+349000,-16354509+349000,
		-16354509+349000,-15849882+349000,-16354509+349000,
		-15849882+349000,-16354509+349000,-16354509+349000,
		-16354509+349000,-16354509+349000,-15849882+349000,
};
const double meter2count = 1/0.01*3.5*65536;

const int MapAbsToPhy[18]
{
	10,11,9,
	12,14,13,
	17,15,16,
	6,8,7,
	3,5,4,
	0,2,1
};

const int MapPhyToAbs[18]
{
	15,17,16,
	12,14,13,
	9,11,10,
	2,0,1,
	3,5,4,
	7,8,6
};

void inline p2a(const int *phy, int *abs, int num = 18)
{
	for(int i=0;i<num;++i)
	{
		abs[i]=MapPhyToAbs[phy[i]];
	}
}
void inline a2p(const int *abs, int *phy, int num = 18)
{
	for(int i=0;i<num;++i)
	{
		phy[i]=MapAbsToPhy[abs[i]];
	}
}


int walk(Robots::ROBOT_BASE *pRobot, const Robots::GAIT_PARAM_BASE *pParam)
{
	static double lastPee[18];
	static double lastPbody[6];

	const Robots::WALK_PARAM *pWP=static_cast<const Robots::WALK_PARAM *>(pParam);


	if(pParam->count < pWP->totalCount)
	{
		walkAcc(pRobot,pParam);
	}
	else
	{
		Robots::WALK_PARAM param2=*pWP;
		param2.count=pWP->count - pWP->totalCount;

		memcpy(param2.beginPee,lastPee,sizeof(lastPee));
		memcpy(param2.beginBodyPE,lastPbody,sizeof(lastPbody));

		walkDec(pRobot,&param2);
	}

	if(pParam->count%100==0)
	{
		double pEE[18];
		double pBody[18];
		pRobot->GetPee(pEE,"G");
				pRobot->GetBodyPe(pBody,"313");



		rt_printf("%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n"
					,pEE[0],pEE[1],pEE[2],pEE[3],pEE[4],pEE[5],pEE[6],pEE[7],pEE[8]
						,pEE[9],pEE[10],pEE[11],pEE[12],pEE[13],pEE[14],pEE[15],pEE[16],pEE[17]);
				//rt_printf("%f %f %f %f %f %f\n"
					//			,pBody[0],pBody[1],pBody[2],pBody[3],pBody[4],pBody[5]);
	}


	if(pParam->count==pWP->totalCount-1)
	{
		pRobot->GetPee(lastPee,"G");
		pRobot->GetBodyPe(lastPbody,"313");

		double *pEE=lastPee;
		double *pBody=lastPbody;

		/*rt_printf("%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n"
				,pEE[0],pEE[1],pEE[2],pEE[3],pEE[4],pEE[5],pEE[6],pEE[7],pEE[8]
				,pEE[9],pEE[10],pEE[11],pEE[12],pEE[13],pEE[14],pEE[15],pEE[16],pEE[17]);
		rt_printf("%f %f %f %f %f %f\n"
						,pBody[0],pBody[1],pBody[2],pBody[3],pBody[4],pBody[5]);*/
	}

	//rt_printf("value is:%d",2* pWP->totalCount - pWP->count-1);


	return 2* pWP->totalCount - pWP->count-1;


}



int home(Robots::ROBOT_BASE *pRobot, const Robots::GAIT_PARAM_BASE *pParam, Aris::RT_CONTROL::CMachineData &data)
{
	double homeIn[18]=
	{
			0.675784824916295,0.697784816196987,0.697784816196987,
			0.675784824916295,0.697784816196987,0.697784816196987,
			0.675784824916295,0.697784816196987,0.697784816196987,
			0.675784824916295,0.697784816196987,0.697784816196987,
			0.675784824916295,0.697784816196987,0.697784816196987,
			0.675784824916295,0.697784816196987,0.697784816196987,};

	bool isAllHomed=true;

	auto param=static_cast<const MOTOR_PARAM *>(pParam);

	int id[18];
	a2p(param->motorID,id,param->motorNum);

	for(int i=0;i< param->motorNum;++i)
	{
		if(data.isMotorHomed[id[i]])
		{
			data.motorsCommands[id[i]]=Aris::RT_CONTROL::EMCMD_RUNNING;
			data.commandData[id[i]].Position=-HEXBOT_HOME_OFFSETS_RESOLVER[id[i]];
		}
		else
		{
			data.motorsCommands[id[i]]=Aris::RT_CONTROL::EMCMD_GOHOME;
			data.commandData[id[i]].Position=-HEXBOT_HOME_OFFSETS_RESOLVER[id[i]];
			isAllHomed=false;
		}
	}


	if(isAllHomed)
	{
		double pBody[6]{0,0,0,0,0,0},vBody[6]{0};
		double vEE[18]{0};
		pRobot->SetPin(homeIn,pBody);
		pRobot->SetVee(vEE,vBody);

		return 0;
	}
	else
	{
		return -1;
	}
};
int enable(Robots::ROBOT_BASE *pRobot, const Robots::GAIT_PARAM_BASE *pParam, Aris::RT_CONTROL::CMachineData &data)
{
	static Aris::RT_CONTROL::CMachineData lastCmdData;

	auto param=static_cast<const MOTOR_PARAM *>(pParam);

	bool isAllRunning=true;

	int id[18];
	a2p(param->motorID,id,param->motorNum);

	for (int i=0;i< param->motorNum;++i)
	{
		if(data.motorsStates[id[i]]==Aris::RT_CONTROL::EMSTAT_RUNNING)
		{
			data.motorsCommands[id[i]]=Aris::RT_CONTROL::EMCMD_RUNNING;
			data.commandData[id[i]]=lastCmdData.commandData[id[i]];
		}
		else if(data.motorsStates[id[i]]==Aris::RT_CONTROL::EMSTAT_ENABLED)
		{
			data.motorsCommands[id[i]]=Aris::RT_CONTROL::EMCMD_RUNNING;
			data.commandData[id[i]]=data.feedbackData[param->motorID[i]];
			lastCmdData.commandData[id[i]]=data.feedbackData[id[i]];
			isAllRunning=false;
		}
		else
		{
			data.motorsCommands[id[i]]=Aris::RT_CONTROL::EMCMD_ENABLE;
			isAllRunning=false;
		}
	}

	if(isAllRunning)
	{
		return 0;
	}
	else
	{
		return -1;
	}
};
int disable(Robots::ROBOT_BASE *pRobot, const Robots::GAIT_PARAM_BASE *pParam, Aris::RT_CONTROL::CMachineData &data)
{
	auto param=static_cast<const MOTOR_PARAM *>(pParam);

	int id[18];
	a2p(param->motorID,id,param->motorNum);


	bool isAllDisabled=true;
	for (int i=0;i< param->motorNum;++i)
	{
		if(data.motorsStates[id[i]]!=Aris::RT_CONTROL::EMSTAT_STOPPED)
		{
			data.motorsCommands[id[i]]=Aris::RT_CONTROL::EMCMD_STOP;
			isAllDisabled=false;
		}
	}

	if(isAllDisabled)
	{
		return 0;
	}
	else
	{
		return -1;
	}
}
int resetOrigin(Robots::ROBOT_BASE *pRobot, const Robots::GAIT_PARAM_BASE *pParam, Aris::RT_CONTROL::CMachineData &data)
{
	double pEE[18],pBody[6]{0},vEE[18],vBody[6]{0};
	pRobot->GetPee(pEE,"B");
	pRobot->GetVee(vEE,"B");

	pRobot->SetPee(pEE,pBody,"G");
	pRobot->SetVee(vEE,vBody,"G");

	return 0;
}
int runGait(Robots::ROBOT_BASE *pRobot, const Robots::GAIT_PARAM_BASE *pParam, Aris::RT_CONTROL::CMachineData &data)
{
	int ret=0;
	double pIn[18];
	switch(pParam->cmdID)
	{
	case WALK:
		ret = walk(pRobot,pParam);
		break;
	case BEGIN:
		//ret=begin(pRobot,pParam);
		break;
	default:
		break;
	}

	pRobot->GetPin(pIn);

	for(int i=0;i<18;++i)
	{
		data.motorsCommands[MapAbsToPhy[i]]=Aris::RT_CONTROL::EMCMD_RUNNING;
		data.commandData[MapAbsToPhy[i]].Position=pIn[i]*meter2count;
	}


	return ret;
}



Robots::ROBOT_III robot;
int execute_cmd(int count,char *cmd, Aris::RT_CONTROL::CMachineData &data)
{
	static double pBody[6]{0},vBody[6]{0},pEE[18]{0},vEE[18]{0};

	int ret;

	Robots::GAIT_PARAM_BASE *pParam=reinterpret_cast<Robots::GAIT_PARAM_BASE *>(cmd);
	pParam->count=count;

	memcpy(pParam->beginPee,pEE,sizeof(pEE));
	memcpy(pParam->beginVee,vEE,sizeof(vEE));
	memcpy(pParam->beginBodyPE,pBody,sizeof(pBody));
	memcpy(pParam->beginBodyVel,pBody,sizeof(vBody));

	switch (pParam->cmdID)
	{
	case ENABLE:
	{
		ret = enable(&robot, pParam , data);
		break;
	}
	case DISABLE:
	{
		ret = disable(&robot, pParam ,data);
		break;
	}
	case HOME:
	{
		ret = home(&robot, pParam, data);
		break;
	}
	case RESET_ORIGIN:
		ret = resetOrigin(&robot, pParam,data);
		break;
	default:
		ret = runGait(&robot, pParam,data);
		break;

	}

	if(ret==0)
	{
		robot.GetBodyPe(pBody);
		robot.GetPee(pEE);
		robot.GetBodyVel(vBody);
		robot.GetVee(vEE);


		rt_printf("%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n"
				,pEE[0],pEE[1],pEE[2],pEE[3],pEE[4],pEE[5],pEE[6],pEE[7],pEE[8]
				,pEE[9],pEE[10],pEE[11],pEE[12],pEE[13],pEE[14],pEE[15],pEE[16],pEE[17]);
		rt_printf("%f %f %f %f %f %f\n"
						,pBody[0],pBody[1],pBody[2],pBody[3],pBody[4],pBody[5]);
	}


	return ret;
}

int tg(Aris::RT_CONTROL::CMachineData &data,Aris::Core::RT_MSG &recvMsg,Aris::Core::RT_MSG &sendMsg)
{
	static double pBodyPE[6]{0},pEE[18]{0};

	static const int cmdSize=8192;

	static char cmdQueue[10][cmdSize];

	static int currentCmd=0;
	static int cmdNum=0;

	static int count = 0;

	static Aris::RT_CONTROL::CMachineData lastCmdData=data,lastStateData=data;
	static Aris::RT_CONTROL::CMachineData stateData, cmdData;

	stateData=data;
	cmdData=data;

	switch (recvMsg.GetMsgID())
	{
	case 0:
		recvMsg.Paste(cmdQueue[(currentCmd+cmdNum)%10]);
		++cmdNum;
		break;
	default:
		break;
	}


	if(cmdNum>0)
	{
		if(execute_cmd(count,cmdQueue[currentCmd],cmdData)==0)
		{
			count = 0;
			currentCmd= (currentCmd+1)%10;
			cmdNum--;
			rt_printf("cmd finished\n");
		}
		else
		{
			count++;
		}

		if(count%1000==0)
		{
			rt_printf("the cmd is:%d in count:%d\n",cmdData.motorsCommands[0],count);
		}
	}
	else
	{
		cmdData=lastCmdData;
	}


	data=cmdData;

	lastStateData=stateData;
	lastCmdData=cmdData;

	return 0;
}
