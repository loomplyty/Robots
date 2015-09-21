#include <Platform.h>
#ifdef PLATFORM_IS_WINDOWS
#define rt_printf printf
#define _SCL_SECURE_NO_WARNINGS
#endif
#ifdef PLATFORM_IS_LINUX
#include <Aris_Control.h>
#endif

#include "Robot_Server.h"
#include <cstring>
#include <Aris_Core.h>
#include <Aris_Plan.h>
#include <Robot_Base.h>


using namespace std;

namespace Robots
{
    //const double meter2count = 1 / 0.01*3.5 * 65536;

	void ROBOT_SERVER::LoadXml(const char *fileName)
	{
		/*open xml file*/
		Aris::Core::DOCUMENT doc;

		if (doc.LoadFile(fileName) != 0)
		{
			throw std::logic_error((std::string("could not open file:") + std::string(fileName)));
		}


		/*load connection param*/
		auto pConnEle = doc.RootElement()->FirstChildElement("Server")->FirstChildElement("Connection");
		ip = pConnEle->Attribute("IP");
		port = pConnEle->Attribute("Port");

		
		/*load home parameters and map*/
		auto pContEle = doc.RootElement()->FirstChildElement("Server")->FirstChildElement("Control");
		Aris::DynKer::CALCULATOR c;
		auto mat = c.CalculateExpression(pContEle->FirstChildElement("HomeEE")->GetText());
		std::copy_n(mat.Data(), 18, homeEE);

		std::string mapPhy2AbsText{ pContEle->FirstChildElement("MapPhy2Abs")->GetText() };
		std::stringstream stream(mapPhy2AbsText);

		for (int i = 0; i < 18; ++i)
		{
			std::string word;

			while (stream >> word)
			{
				std::stringstream intStream{ word };
				if (intStream >> mapPhy2Abs[i])
					break;
			}
		}

		for (int i = 0; i < 18; ++i)
		{
			mapAbs2Phy[i] = std::find(mapPhy2Abs, mapPhy2Abs + 18, i) - mapPhy2Abs;
		}



		std::string docName{ doc.RootElement()->Name() };

		pRobot->LoadXml(fileName);

		double pe[6]{ 0 };
		pRobot->SetPee(homeEE, pe, "B");
		pRobot->GetPin(homeIn);



		for (int i = 0; i < 18; ++i)
		{
			homeCount[i] = -static_cast<int>(homeIn[mapPhy2Abs[i]] * meter2count);
		}

		std::cout<<"abs to phy:"<<std::endl;
				for(int i=0;i<18;++i)cout<<mapAbs2Phy[i]<<std::endl;

		std::cout<<"home count:"<<std::endl;
				for(int i=0;i<18;++i)cout<<homeCount[i]<<std::endl;


		/*copy client*/
#ifdef PLATFORM_IS_LINUX
		const int TASK_NAME_LEN = 1024;

		std::int32_t count = 0;
		std::int32_t nIndex = 0;
		char path[TASK_NAME_LEN] = { 0 };
		char cParam[100] = { 0 };
		char *proName = path;
		std::int32_t tmp_len;

		pid_t pId = getpid();
		sprintf(cParam, "/proc/%d/exe", pId);
		count = readlink(cParam, path, TASK_NAME_LEN);

		if (count < 0 || count >= TASK_NAME_LEN)
		{
			throw std::logic_error("Current System Not Surport Proc.\n");
		}
		else
		{
			nIndex = count - 1;
			
			for (; nIndex >= 0; nIndex--)
			{
				if (path[nIndex] == '/')//ɸѡ��������
				{
					nIndex++;
					proName += nIndex;
					break;
				}
			}
		}

		std::string pwd(path, nIndex);

		auto pCmds = doc.RootElement()->FirstChildElement("Server")->FirstChildElement("Commands");
		
		if (pCmds == nullptr)
			throw std::logic_error("invalid client.xml");

		for (auto pChild = pCmds->FirstChildElement();
		pChild != nullptr;
		pChild = pChild->NextSiblingElement())
		{
			std::string fullpath = std::string("cp ") + pwd + std::string("Client ") + pwd + pChild->Name();
			auto ret=system(fullpath.c_str());
		}
#endif
	}
	void ROBOT_SERVER::AddGait(std::string cmdName, GAIT_FUNC gaitFunc, PARSE_FUNC parseFunc)
	{
		if (mapName2ID.find(cmdName) == mapName2ID.end())
		{
			allGaits.push_back(gaitFunc);
			allParsers.push_back(parseFunc);

			mapName2ID.insert(std::make_pair(cmdName, allGaits.size() - 1));

			std::cout << cmdName << ":" << mapName2ID.at(cmdName) << std::endl;

		}
	};


/*	void ROBOT_SERVER::AddOnlineGait(std::string cmdName, GAIT_ONLINE_FUNC gaitFunc, PARSE_FUNC parseFunc)
	{
	 	if (mapName2ID.find(cmdName) == mapName2ID.end())
		{
			allOnlineGaits.push_back(gaitFunc);
			allOnlineParsers.push_back(parseFunc);

			mapName2ID.insert(std::make_pair(cmdName, allGaits.size() - 1));

			std::cout << cmdName << ":" << mapName2ID.at(cmdName) << std::endl;

		}
	};
*/
	void ROBOT_SERVER::Start()
	{
#ifdef PLATFORM_IS_LINUX
		Aris::RT_CONTROL::CSysInitParameters initParam;

		initParam.motorNum = 18;
		initParam.homeMode = -1;
        initParam.homeTorqueLimit = 0;
		initParam.homeHighSpeed = 280000;
		initParam.homeLowSpeed = 160000;
		initParam.homeOffsets = homeCount;

		cs.SetTrajectoryGenerator(tg);
		cs.SysInit(initParam);
		cs.SysInitCommunication();
		cs.SysStart();
#endif

		server.SetOnReceivedConnection([](Aris::Core::CONN *pConn, const char *pRemoteIP, int remotePort)
		{
			std::cout << "control client received:" << std::endl;
			std::cout << "    remote ip is:" << pRemoteIP << std::endl;
			std::cout << std::endl;
			return 0;
		});
		server.SetOnReceiveRequest([this](Aris::Core::CONN *pConn, Aris::Core::MSG &msg)
		{
			std::cout << "received request" << std::endl;

			this->ExecuteMsg(msg);

			return Aris::Core::MSG();
		});
		server.SetOnLoseConnection([this](Aris::Core::CONN *pConn)
		{
			std::cout << "control interface lost" << std::endl;

			while (true)
			{
				try
				{
					pConn->StartServer(this->port.c_str());
					break;
				}
				catch (Aris::Core::CONN::START_SERVER_ERROR &e)
				{
					std::cout << e.what() << std::endl << "will restart in 5s" << std::endl;
#ifdef PLATFORM_IS_LINUX
					usleep(5000000);
#endif
				}
			}

			return 0;
		});

		while (true)
		{
			try
			{
				server.StartServer(port.c_str());
				break;
			}
			catch (Aris::Core::CONN::START_SERVER_ERROR &e)
			{
				std::cout << e.what() << std::endl << "will restart in 5s" << std::endl;
#ifdef PLATFORM_IS_LINUX
				usleep(5000000);
#endif
			}
		}
	}

	void ROBOT_SERVER::DecodeMsg(const Aris::Core::MSG &msg, std::string &cmd, std::map<std::string, std::string> &params)
	{
		char content[500];

		std::int32_t size = 0;
		std::int32_t beginPos = 0;

		msg.PasteStruct(size);
		beginPos += 4;
		msg.PasteAt(content, size, beginPos);
		cmd.assign(content);
		beginPos += size;

		std::int32_t paramNum;
		msg.PasteAt(&paramNum, 4, beginPos);
		beginPos += 4;

		for (int i = 0; i<paramNum; ++i)
		{
			std::string cmdd, param;

			msg.PasteAt(&size, 4, beginPos);
			beginPos += 4;
			msg.PasteAt(content, size, beginPos);
			cmdd.assign(content);
			beginPos += size;

			msg.PasteAt(&size, 4, beginPos);
			beginPos += 4;
			msg.PasteAt(content, size, beginPos);
			param.assign(content);
			beginPos += size;

			params.insert(std::make_pair(cmdd, param));
		}


	}
	void ROBOT_SERVER::GenerateCmdMsg(const std::string &cmd, const std::map<std::string, std::string> &params, Aris::Core::MSG &msg)
	{
		if (cmd == "en")
		{
			Robots::GAIT_PARAM_BASE robotState;
			robotState.cmdType = ENABLE;

			for (auto &i : params)
			{
				if (i.first == "all")
				{
					int motors[18] = { 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17 };
					std::memcpy(robotState.motorID, motors, sizeof(motors));
					robotState.motorNum = 18;
				}
				else if (i.first == "first")
				{
					robotState.motorNum = 9;
					int motors[9] = { 0,1,2,6,7,8,12,13,14 };
					std::memcpy(robotState.motorID, motors, sizeof(motors));
				}
				else if (i.first == "second")
				{
					robotState.motorNum = 9;
					int motors[9] = { 3,4,5,9,10,11,15,16,17 };
					std::memcpy(robotState.motorID, motors, sizeof(motors));
				}
				else if (i.first == "motor")
				{
					int motors[1] = { stoi(i.second) };
					std::memcpy(robotState.motorID, motors, sizeof(motors));
					robotState.motorNum = 1;
				}
			}

			msg.CopyStruct(robotState);
			return;
		}

		if (cmd == "ds")
		{
			Robots::GAIT_PARAM_BASE robotState;
			robotState.cmdType = DISABLE;

			for (auto &i : params)
			{
				if (i.first == "all")
				{
					int motors[18] = { 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17 };
					std::memcpy(robotState.motorID, motors, sizeof(motors));
					robotState.motorNum = 18;
				}
				else if (i.first == "first")
				{
					int motors[9] = { 0,1,2,6,7,8,12,13,14 };
					std::memcpy(robotState.motorID, motors, sizeof(motors));
					robotState.motorNum = 9;
				}
				else if (i.first == "second")
				{
					int motors[9] = { 3,4,5,9,10,11,15,16,17 };
					std::memcpy(robotState.motorID, motors, sizeof(motors));
					robotState.motorNum = 9;
				}
				else if (i.first == "motor")
				{
					int motors[1] = { stoi(i.second) };
					std::memcpy(robotState.motorID, motors, sizeof(motors));
					robotState.motorNum = 1;
				}
			}

			msg.CopyStruct(robotState);
			return;
		}

		if (cmd == "hm")
		{
			Robots::GAIT_PARAM_BASE robotState;
			robotState.cmdType = HOME;

			for (auto &i : params)
			{
				if (i.first == "all")
				{
					int motors[18] = { 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17 };
					std::memcpy(robotState.motorID, motors, sizeof(motors));
					robotState.motorNum = 18;

					robotState.legNum = 6;
					int legs[6] = { 0,1,2,3,4,5 };
					std::memcpy(robotState.legID, legs, sizeof(legs));
				}
				else if (i.first == "first")
				{
					int motors[9] = { 0,1,2,6,7,8,12,13,14 };
					std::memcpy(robotState.motorID, motors, sizeof(motors));
					robotState.motorNum = 9;

					robotState.legNum = 3;
					int legs[3] = { 0,2,4 };
					std::memcpy(robotState.legID, legs, sizeof(legs));
				}
				else if (i.first == "second")
				{
					int motors[9] = { 3,4,5,9,10,11,15,16,17 };
					std::memcpy(robotState.motorID, motors, sizeof(motors));
					robotState.motorNum = 9;

					robotState.legNum = 3;
					int legs[3] = { 1,3,5 };
					std::memcpy(robotState.legID, legs, sizeof(legs));
				}
				else if (i.first == "motor")
				{
					int motors[1] = { stoi(i.second) };
					std::memcpy(robotState.motorID, motors, sizeof(motors));
					robotState.motorNum = 1;

					robotState.legNum = 6;
					int legs[6] = { 0,1,2,3,4,5 };
					std::memcpy(robotState.legID, legs, sizeof(legs));
				}
			}

			msg.CopyStruct(robotState);
			return;
		}

		if (cmd == "ro")
		{
			Robots::GAIT_PARAM_BASE robotState;
			robotState.cmdType = RESET_ORIGIN;
			msg.CopyStruct(robotState);
			return;
		}

		auto cmdPair = this->mapName2ID.find(cmd);

		if (cmdPair != this->mapName2ID.end())
		{
			msg = this->allParsers.at(cmdPair->second).operator()(cmd, params);
			reinterpret_cast<GAIT_PARAM_BASE *>(msg.GetDataAddress())->cmdType=RUN_GAIT;
			reinterpret_cast<GAIT_PARAM_BASE *>(msg.GetDataAddress())->cmdID=cmdPair->second;

		}
		else
		{
			cout << "cmd not found" << endl;
		}
	}
	void ROBOT_SERVER::ExecuteMsg(const Aris::Core::MSG &msg)
	{
		std::string cmd;
		std::map<std::string, std::string> params;
		DecodeMsg(msg, cmd, params);

		Aris::Core::MSG cmdMsg;
        GenerateCmdMsg(cmd, params, cmdMsg);

		cmdMsg.SetMsgID(0);

#ifdef PLATFORM_IS_LINUX
		cs.NRT_PostMsg(cmdMsg);
#endif
	}

	int ROBOT_SERVER::home(Robots::ROBOT_BASE *pRobot, const Robots::GAIT_PARAM_BASE *param, Aris::RT_CONTROL::CMachineData &data)
	{
		bool isAllHomed = true;

		int id[18];
		a2p(param->motorID, id, param->motorNum);

		if(param->count%1000==0)
		rt_printf("motor %d is homing",id[0]);

		for (int i = 0; i< param->motorNum; ++i)
		{
			if (data.isMotorHomed[id[i]])
			{
				data.motorsCommands[id[i]] = Aris::RT_CONTROL::EMCMD_RUNNING;
				data.commandData[id[i]].Position = -homeCount[id[i]];
			}
			else
			{
				data.motorsCommands[id[i]] = Aris::RT_CONTROL::EMCMD_GOHOME;
				data.commandData[id[i]].Position = -homeCount[id[i]];
				isAllHomed = false;

				if (param->count % 1000 == 0)
				{
                    rt_printf("motor %d not homed, physical id is:\n", id[i]);
					rt_printf("motor %d not homed, absolute id is:\n", param->motorID[i]);
				}
			}
		}


		if (isAllHomed)
		{
			double pBody[6]{ 0,0,0,0,0,0 }, vBody[6]{ 0 };
			double vEE[18]{ 0 };

			pRobot->SetPin(nullptr, pBody);

			for (int i = 0; i < param->legNum; ++i)
			{
				rt_printf("leg:%d\n", param->legID[i]);
				pRobot->pLegs[param->legID[i]]->SetPee(&homeEE[param->legID[i] * 3],"B");
			}
			//pRobot->SetPin(homeIn,pBody);
			pRobot->SetVee(vEE, vBody);

			return 0;
		}
		else
		{


			return -1;
		}
	};
	int ROBOT_SERVER::enable(Robots::ROBOT_BASE *pRobot, const Robots::GAIT_PARAM_BASE *param, Aris::RT_CONTROL::CMachineData &data)
	{
		static Aris::RT_CONTROL::CMachineData lastCmdData;

		bool isAllRunning = true;

		int id[18];
		a2p(param->motorID, id, param->motorNum);

		for (int i = 0; i< param->motorNum; ++i)
		{
			if (data.motorsStates[id[i]] == Aris::RT_CONTROL::EMSTAT_RUNNING)
			{
				data.motorsCommands[id[i]] = Aris::RT_CONTROL::EMCMD_RUNNING;
				data.commandData[id[i]] = lastCmdData.commandData[id[i]];
			}
			else if (data.motorsStates[id[i]] == Aris::RT_CONTROL::EMSTAT_ENABLED)
			{
				data.motorsCommands[id[i]] = Aris::RT_CONTROL::EMCMD_RUNNING;
				data.commandData[id[i]] = data.feedbackData[id[i]];
				lastCmdData.commandData[id[i]] = data.feedbackData[id[i]];
				isAllRunning = false;
			}
			else
			{
				data.motorsCommands[id[i]] = Aris::RT_CONTROL::EMCMD_ENABLE;
				isAllRunning = false;
			}
		}

		if (isAllRunning)
		{
			return 0;
		}
		else
		{
			return -1;
		}
	};
	int ROBOT_SERVER::disable(Robots::ROBOT_BASE *pRobot, const Robots::GAIT_PARAM_BASE *param, Aris::RT_CONTROL::CMachineData &data)
	{
		int id[18];
		a2p(param->motorID, id, param->motorNum);


		bool isAllDisabled = true;
		for (int i = 0; i< param->motorNum; ++i)
		{
			if (data.motorsStates[id[i]] != Aris::RT_CONTROL::EMSTAT_STOPPED)
			{
				data.motorsCommands[id[i]] = Aris::RT_CONTROL::EMCMD_STOP;
				isAllDisabled = false;
			}
		}

		if (isAllDisabled)
		{
			return 0;
		}
		else
		{
			return -1;
		}
	}
	int ROBOT_SERVER::resetOrigin(Robots::ROBOT_BASE *pRobot, const Robots::GAIT_PARAM_BASE *param, Aris::RT_CONTROL::CMachineData &data)
	{
		double pEE[18], pBody[6]{ 0 }, vEE[18], vBody[6]{ 0 };
		pRobot->GetPee(pEE, "B");
		pRobot->GetVee(vEE, "B");

		pRobot->SetPee(pEE, pBody, "G");
		pRobot->SetVee(vEE, vBody, "G");

		return 0;
	}
	int ROBOT_SERVER::runGait(Robots::ROBOT_BASE *pRobot, const Robots::GAIT_PARAM_BASE *pParam, Aris::RT_CONTROL::CMachineData &data)
	{

		int ret = 0;
		double pIn[18], pEE_B[18];

		pRobot->TransformCoordinatePee(pParam->beginBodyPE,"G",pParam->beginPee,"B",pEE_B);

		ret = this->allGaits.at(pParam->cmdID).operator()(pRobot,pParam); //pRobot legs and body altered

        pRobot->GetPin(pIn);

/********************position mode *************/
        //position mode,writing down positions to machinedata

		int id[18];
		a2p(pParam->motorID, id, pParam->motorNum);

		for (int i = 0; i<pParam->motorNum; ++i)
		{
			data.motorsCommands[id[i]] = Aris::RT_CONTROL::EMCMD_RUNNING;
			data.commandData[id[i]].Position = static_cast<int>(pIn[pParam->motorID[i]] * meter2count);
		}

		for(int i=0;i<6;++i)
		{
			//auto found=std::find(pParam->legID,pParam->legID+pParam->legNum,i);
			if((std::find(pParam->legID,pParam->legID+pParam->legNum,i))==(pParam->legID+pParam->legNum))
			{
				if(pParam->count%1000==0)
				rt_printf("%d leg not found\n",i);

				pRobot->pLegs[i]->SetPee(pEE_B+i*3,"B");
			}
		}
        //force verification
        Robots::ROBOT_III* robotIII=dynamic_cast<Robots::ROBOT_III*>(pRobot);

        //need set already pos and vel in rungait functions, so that here we can use fastdyn
        robotIII->FastDyn();
        robotIII->SetFixedFeet("000000");
        //robotIII.setactivemotion("01010100")//18
    //    Robots::ROBOT_III::FastDyn();


      /*  static double fIN[18]{0},model_fIN[18]{0};
        for(int i=0;i<18;i++)
        {
            fIN[i]=pParam->pActuationData->feedbackData[i].Torque*Robots::current2torque;
            pRobot
            model_fIN[]


        }*/

/*************************torque mode***********************/



		return ret;
	}

	int ROBOT_SERVER::execute_cmd(int count, char *cmd, Aris::RT_CONTROL::CMachineData &data)
	{
        static double pBody[6]{ 0 }, vBody[6]{ 0 }, pEE[18]{ 0 }, vEE[18]{ 0 };

		int ret;

		Robots::GAIT_PARAM_BASE *pParam = reinterpret_cast<Robots::GAIT_PARAM_BASE *>(cmd);
		pParam->count = count;
		pParam->pActuationData = &data;

		memcpy(pParam->beginPee, pEE, sizeof(pEE));
		memcpy(pParam->beginVee, vEE, sizeof(vEE));
		memcpy(pParam->beginBodyPE, pBody, sizeof(pBody));
		memcpy(pParam->beginBodyVel, pBody, sizeof(vBody));


		switch (pParam->cmdType)
		{
		case ENABLE:
			ret = enable(pRobot.get(), pParam, data);
			break;
		case DISABLE:
			ret = disable(pRobot.get(), pParam, data);
			break;
		case HOME:
			ret = home(pRobot.get(), pParam, data);
			break;
		case RESET_ORIGIN:
			ret = resetOrigin(pRobot.get(), pParam, data);
			break;
		case RUN_GAIT:
            ret = runGait(pRobot.get(), pParam, data);
			break;
		default:
			rt_printf("unknown cmd type\n");
			break;

		}

		if (ret == 0)
		{
			pRobot->GetBodyPe(pBody);
			pRobot->GetPee(pEE);
			pRobot->GetBodyVel(vBody);
            pRobot->GetVee(vEE);


			rt_printf("%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n"
				, pEE[0], pEE[1], pEE[2], pEE[3], pEE[4], pEE[5], pEE[6], pEE[7], pEE[8]
				, pEE[9], pEE[10], pEE[11], pEE[12], pEE[13], pEE[14], pEE[15], pEE[16], pEE[17]);
			rt_printf("%f %f %f %f %f %f\n"
				, pBody[0], pBody[1], pBody[2], pBody[3], pBody[4], pBody[5]);
		}

		return ret;
	}

	int ROBOT_SERVER::tg(Aris::RT_CONTROL::CMachineData &data, Aris::Core::RT_MSG &recvMsg, Aris::Core::RT_MSG &sendMsg)
	{
        // in tg recvMsg.GetMsgID()=0 if it is a validate message;and,
		static double pBodyPE[6]{ 0 }, pEE[18]{ 0 };

		static const int cmdSize = 8192;

		static char cmdQueue[50][cmdSize];

		static int currentCmd = 0;
		static int cmdNum = 0;

		static int count = 0;

		static Aris::RT_CONTROL::CMachineData lastCmdData = data, lastStateData = data;
		static Aris::RT_CONTROL::CMachineData stateData, cmdData;

		stateData = data;
		cmdData = data;



		switch (recvMsg.GetMsgID())
		{
		case 0:
			recvMsg.Paste(cmdQueue[(currentCmd + cmdNum) % 10]);
			++cmdNum;
            rt_printf("recvMsg datalength %d\n",recvMsg.GetLength());

			break;
		default:
			break;
		}


		if (cmdNum>0)
		{
			if (Robots::ROBOT_SERVER::GetInstance()->execute_cmd(count, cmdQueue[currentCmd], cmdData) == 0)///cmdData should be data
			{
				count = 0;
				currentCmd = (currentCmd + 1) % 10;
				cmdNum--;
				rt_printf("cmd finished\n");
			}
			else
			{
				count++;
			}

			if (count % 1000 == 0)
			{
				rt_printf("the cmd is:%d in count:%d\n", cmdData.motorsCommands[0], count);
			}
		}
		else
		{
			cmdData = lastCmdData;
		}

		static bool firstError=true;

		for (int i = 0; i<18; ++i)
		{
			if (lastCmdData.motorsCommands[i] == Aris::RT_CONTROL::EMCMD_RUNNING)
			{
				if (cmdData.motorsCommands[i] == Aris::RT_CONTROL::EMCMD_RUNNING)
				{
					if (std::abs(lastCmdData.commandData[i].Position - cmdData.commandData[i].Position)>20000)
					{
						if(firstError)
						{
							rt_printf("data %d not continuous\n",i);
							rt_printf("last:%d, now:%d\n",lastCmdData.commandData[i].Position
									, cmdData.commandData[i].Position);

							rt_printf("data not continuous in count:%d\n",count);

							auto pR=GetInstance()->pRobot.get();
							double pEE[18];
							double pBody[6];
							pR->GetPee(pEE);
							pR->GetBodyPe(pBody);
                        /*	rt_printf("%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n"
											, pEE[0], pEE[1], pEE[2], pEE[3], pEE[4], pEE[5], pEE[6], pEE[7], pEE[8]
											, pEE[9], pEE[10], pEE[11], pEE[12], pEE[13], pEE[14], pEE[15], pEE[16], pEE[17]);
										rt_printf("%f %f %f %f %f %f\n"
											, pBody[0], pBody[1], pBody[2], pBody[3], pBody[4], pBody[5]);


							for(int i=0;i<18;++i)
							{
								rt_printf("%d %d\n",lastCmdData.commandData[i].Position,cmdData.commandData[i].Position);
							}
                            */
							firstError=false;
						}


						data = lastCmdData;
						return 0;
					}
				}
			}
		}


		if(!firstError)
		{
			data = lastCmdData;
			return 0;
		}


		data = cmdData;

		lastStateData = stateData;
		lastCmdData = cmdData;

		return 0;
	}















}








