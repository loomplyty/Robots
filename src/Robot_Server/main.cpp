#include <iostream>
#include <cstring>
#include <iomanip> 
#include <bitset>
#include <cstring>
#include <map>
#include <string>

#include <stdlib.h>

#include <Aris_Core.h>
#include <Aris_Message.h>
#include <Aris_Control.h>

using namespace Aris::Core;

#include "trajectory_generator.h"


extern int HEXBOT_HOME_OFFSETS_RESOLVER[18];




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
	}


	if(pParam->count==pWP->totalCount-1)
	{
		pRobot->GetPee(lastPee,"G");
		pRobot->GetBodyPe(lastPbody,"313");

		double *pEE=lastPee;
		double *pBody=lastPbody;
	}
	return 2* pWP->totalCount - pWP->count-1;
}





int copyClient()
{
	const int TASK_NAME_LEN =1024;

	std::int32_t count = 0;
	std::int32_t nIndex = 0;
	char path[TASK_NAME_LEN] = {0};
	char cParam[100] = {0};
	char *proName = path;
	std::int32_t tmp_len;

	pid_t pId = getpid();
	sprintf(cParam,"/proc/%d/exe",pId);
	count = readlink(cParam, path, TASK_NAME_LEN);

	if (count < 0 || count >= TASK_NAME_LEN)
    {
        throw std::logic_error("Current System Not Surport Proc.\n");
    }
	else
	{
		nIndex = count - 1;

		for( ; nIndex >= 0; nIndex--)
		{
			if( path[nIndex] == '/' )//筛选出进程名
		    {
				nIndex++;
				proName += nIndex;
				break;
		    }
		}
	}

	std::string pwd(path,nIndex);


	Aris::Core::DOCUMENT doc;

	/*get all cmd names*/
	if(doc.LoadFile("/usr/Robots/CMakeDemo/Robot_III/resource/client.xml")!=0)
		throw std::logic_error("failed to read client.xml");

	auto pCmds=doc.FirstChildElement("Commands");

	if(pCmds==nullptr)
		throw std::logic_error("invalid client.xml");

	for (auto pChild = pCmds->FirstChildElement();
		pChild != nullptr;
		pChild = pChild->NextSiblingElement())
	{
		std::string fullpath=std::string("cp ")+pwd+std::string("Client ")+pwd+pChild->Name();

		//cout<<fullpath<<endl;
		system(fullpath.c_str());

	}

	return 0;
};


Aris::RT_CONTROL::ACTUATION cs;
int main()
{
	robot.LoadXml("/usr/Robots/resource/HexapodIII/HexapodIII.xml");

	robot.AddGait(Robots::adjust);
	robot.AddGait(walk);





	Aris::RT_CONTROL::CSysInitParameters initParam;

	initParam.motorNum=18;
	initParam.homeMode=-1;
	initParam.homeTorqueLimit=950;
	initParam.homeHighSpeed=280000;
	initParam.homeLowSpeed=160000;
	initParam.homeOffsets=HEXBOT_HOME_OFFSETS_RESOLVER;



	cs.SetTrajectoryGenerator(tg);
	cs.SysInit(initParam);
	cs.SysInitCommunication();
	cs.SysStart();

	/**/
	copyClient();



	Aris::Core::CONN control_interface;
	control_interface.SetOnReceivedConnection([](Aris::Core::CONN *pConn,const char *pRemoteIP,int remotePort)
	{
		std::cout << "control client received:" << std::endl;
		std::cout << "    remote ip is:" << pRemoteIP << std::endl;
		std::cout << std::endl;
		return 0;
	});
	control_interface.SetOnReceiveRequest([&cs](Aris::Core::CONN *pConn, Aris::Core::MSG &msg)
	{
		std::cout<<"received request"<<std::endl;

		/*now decode msg to cmd and params*/
		std::string cmd;
		std::map<std::string,std::string> params;
		DecodeMsg(msg,cmd,params);

		/*decode finished, now generate command msg*/
		GenerateCmdMsg(cmd,params,msg);

		std::cout<<cmd<<std::endl;
		for(auto &i:params)
		{
			std::cout<<i.first<<":"<<i.second<<std::endl;
		}

		msg.SetMsgID(0);

		cs.NRT_PostMsg(msg);

		return MSG();
	});
	control_interface.SetOnReceivedData([&cs](Aris::Core::CONN *pConn, Aris::Core::MSG &msg)
	{
		std::cout<<"received data"<<std::endl;
		cs.NRT_PostMsg(msg);
		return 0;
	});
	control_interface.SetOnLoseConnection([](Aris::Core::CONN *pConn)
	{
		std::cout << "control_interface lost" << std::endl;

		while(true)
		{
			try
			{
				pConn->StartServer("5866");
				break;
			}
			catch(CONN::START_SERVER_ERROR &e)
			{
				std::cout <<e.what()<<std::endl<<"will restart in 5s"<<std::endl;
				usleep(5000000);
			}
		}

		return 0;
	});


	/*start server*/
	while(true)
	{
		try
		{
			control_interface.StartServer("5866");
			break;
		}
		catch(CONN::START_SERVER_ERROR &e)
		{
			std::cout <<e.what()<<std::endl<<"will restart in 5s"<<std::endl;
			usleep(5000000);
		}
	}


	/**/
	std::cout<<"finished"<<std::endl;


	Aris::Core::RunMsgLoop();

	return 0;
}
