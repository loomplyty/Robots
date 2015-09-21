#include <Platform.h>
#ifdef PLATFORM_IS_WINDOWS
#define rt_printf printf
#define _SCL_SECURE_NO_WARNINGS
#include <windows.h>
#undef CM_NONE
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
	class NODE
	{
	public:
		NODE* AddChildGroup(const char *Name);
		NODE* AddChildUnique(const char *Name);
		NODE* AddChildParam(const char *Name);
		NODE* FindChild(const char *Name)
		{
			auto result = find_if(children.begin(), children.end(), [Name](std::unique_ptr<NODE> &node)
			{
				bool is = !strcmp(node->name.c_str(), Name);

				return is;
			});

			if (result != children.end())
			{
				return result->get();
			}
			else
			{
				return nullptr;
			}
		};

		bool IsTaken() { return isTaken; };
		void Take();
		void Reset()
		{
			this->isTaken = false;
			for (auto &child : children)
			{
				child->Reset();
			}
		};

	public:
		NODE(NODE*father, const char *Name) :name(Name) { this->father = father; };
		virtual ~NODE() {};

	private:
		string name;
		NODE* father;
		std::vector<std::unique_ptr<NODE> > children;

		bool isTaken{ false };

		friend void addAllParams(Aris::Core::ELEMENT *pEle, NODE *pNode, map<string, NODE *> &allParams, map<char, string>& shortNames);
		friend void addAllDefault(NODE *pNode, map<string, string> &params);
	};
	class ROOT_NODE :public NODE
	{
	public:
		ROOT_NODE(const char *Name) :NODE(nullptr, Name) {};

	private:
		NODE *pDefault;

		friend void addAllParams(Aris::Core::ELEMENT *pEle, NODE *pNode, map<string, NODE *> &allParams, map<char, string>& shortNames);
		friend void addAllDefault(NODE *pNode, map<string, string> &params);
	};
	class GROUP_NODE :public NODE
	{
	public:
		GROUP_NODE(NODE*father, const char *Name) :NODE(father, Name) {};
	};
	class UNIQUE_NODE :public NODE
	{
	public:
		UNIQUE_NODE(NODE*father, const char *Name) :NODE(father, Name) {};

	private:
		NODE *pDefault;

		friend void addAllParams(Aris::Core::ELEMENT *pEle, NODE *pNode, map<string, NODE *> &allParams, map<char, string>& shortNames);
		friend void addAllDefault(NODE *pNode, map<string, string> &params);
	};
	class PARAM_NODE :public NODE
	{
	public:
		PARAM_NODE(NODE*father, const char *Name) :NODE(father, Name) {};
	private:
		std::string type;
		std::string defaultValue;
		std::string minValue, maxValue;

		friend void addAllParams(Aris::Core::ELEMENT *pEle, NODE *pNode, map<string, NODE *> &allParams, map<char, string>& shortNames);
		friend void addAllDefault(NODE *pNode, map<string, string> &params);
	};

	NODE* NODE::AddChildGroup(const char *Name)
	{
		this->children.push_back(std::unique_ptr<NODE>(new GROUP_NODE(this, Name)));
		return children.back().get();
	};
	NODE* NODE::AddChildUnique(const char *Name)
	{
		this->children.push_back(std::unique_ptr<NODE>(new UNIQUE_NODE(this, Name)));
		return children.back().get();
	}
	NODE* NODE::AddChildParam(const char *Name)
	{
		this->children.push_back(std::unique_ptr<NODE>(new PARAM_NODE(this, Name)));
		return children.back().get();
	}
	void NODE::Take()
	{
		if (dynamic_cast<ROOT_NODE*>(this))
		{
			if (this->isTaken)
			{
				throw std::logic_error(std::string("Param ") + this->name + " has been inputed twice");
			}
			else
			{
				this->isTaken = true;
				return;
			}
		}
		else if (dynamic_cast<GROUP_NODE*>(this))
		{
			if (this->isTaken)
			{
				return;
			}
			else
			{
				this->isTaken = true;
				father->Take();
			}
		}
		else
		{
			if (this->isTaken)
			{
				throw std::logic_error(std::string("Param ") + this->name + " has been inputed twice");
			}
			else
			{
				this->isTaken = true;
				father->Take();
			}
		}
	}

	void addAllParams(Aris::Core::ELEMENT *pEle, NODE *pNode, map<string, NODE *> &allParams, map<char, string>& shortNames)
	{
		/*add all children*/
		for (auto pChild = pEle->FirstChildElement();pChild != nullptr;	pChild = pChild->NextSiblingElement())
		{
			/*check if children already has this value*/
			if (pNode->FindChild(pChild->Name()))
			{
				throw logic_error(std::string("XML file has error: node \"") + pChild->Name() + "\" already exist");
			}

			/*set all children*/
			if (pChild->Attribute("type", "group"))
			{
				addAllParams(pChild, pNode->AddChildGroup(pChild->Name()), allParams, shortNames);
			}
			else if (pChild->Attribute("type", "unique"))
			{
				addAllParams(pChild, pNode->AddChildUnique(pChild->Name()), allParams, shortNames);
			}
			else
			{
				/*now the pChild is a param_node*/
				NODE * insertNode;

				if (allParams.find(string(pChild->Name())) != allParams.end())
				{
					throw std::logic_error(std::string("XML file has error: node \"") + pChild->Name() + "\" already exist");
				}
				else
				{
					insertNode = pNode->AddChildParam(pChild->Name());
					allParams.insert(pair<string, NODE *>(string(pChild->Name()), insertNode));
				}

				/*set abbreviation*/
				if (pChild->Attribute("abbreviation"))
				{
					if (shortNames.find(*pChild->Attribute("abbreviation")) != shortNames.end())
					{
						throw std::logic_error(std::string("XML file has error: abbreviations \"")+ pChild->Attribute("abbreviation") + "\" already exist");
					}
					else
					{
						char abbr = *pChild->Attribute("abbreviation");
						shortNames.insert(pair<char, string>(abbr, string(pChild->Name())));
					}
				}

				/*set values*/
				if (pChild->Attribute("type"))
				{
					dynamic_cast<PARAM_NODE*>(insertNode)->type = string(pChild->Attribute("type"));
				}
				else
				{
					dynamic_cast<PARAM_NODE*>(insertNode)->type = "";
				}

				if (pChild->Attribute("default"))
				{
					dynamic_cast<PARAM_NODE*>(insertNode)->defaultValue = string(pChild->Attribute("default"));
				}
				else
				{
					dynamic_cast<PARAM_NODE*>(insertNode)->defaultValue = "";
				}

				if (pChild->Attribute("maxValue"))
				{
					dynamic_cast<PARAM_NODE*>(insertNode)->maxValue = string(pChild->Attribute("maxValue"));
				}
				else
				{
					dynamic_cast<PARAM_NODE*>(insertNode)->maxValue = "";
				}

				if (pChild->Attribute("minValue"))
				{
					dynamic_cast<PARAM_NODE*>(insertNode)->minValue = string(pChild->Attribute("minValue"));
				}
				else
				{
					dynamic_cast<PARAM_NODE*>(insertNode)->minValue = "";
				}
			}
		}

		/*set all values*/
		if (dynamic_cast<ROOT_NODE*>(pNode))
		{
			if (pEle->Attribute("default"))
			{
				if (pNode->FindChild(pEle->Attribute("default")))
				{
					dynamic_cast<ROOT_NODE*>(pNode)->pDefault = pNode->FindChild(pEle->Attribute("default"));
				}
				else
				{
					throw logic_error(std::string("XML file has error: \"") + pNode->name + "\" can't find default param");
				}
			}
			else
			{
				dynamic_cast<ROOT_NODE*>(pNode)->pDefault = nullptr;
			}
		}

		if (dynamic_cast<UNIQUE_NODE*>(pNode))
		{
			if (pEle->Attribute("default"))
			{
				if (pNode->FindChild(pEle->Attribute("default")))
				{
					dynamic_cast<UNIQUE_NODE*>(pNode)->pDefault = pNode->FindChild(pEle->Attribute("default"));
				}
				else
				{
					throw logic_error(std::string("XML file has error: \"") + pNode->name + "\" can't find default param");
				}
			}
			else
			{
				if (pNode->children.empty())
				{
					throw logic_error(std::string("XML file has error: unique node \"") + pNode->name + "\" must have more than 1 child");
				}
				else
				{
					dynamic_cast<UNIQUE_NODE*>(pNode)->pDefault = nullptr;
				}
			}
		}
	}
	void addAllDefault(NODE *pNode, map<string, string> &params)
	{
		if (pNode->isTaken)
		{
			if (dynamic_cast<ROOT_NODE*>(pNode))
			{
				auto found = find_if(pNode->children.begin(), pNode->children.end(), [](unique_ptr<NODE> &a)
				{
					return a->isTaken;
				});

				addAllDefault(found->get(), params);
			}

			if (dynamic_cast<UNIQUE_NODE*>(pNode))
			{
				auto found = find_if(pNode->children.begin(), pNode->children.end(), [](unique_ptr<NODE> &a)
				{
					return a->isTaken;
				});

				addAllDefault(found->get(), params);
			}

			if (dynamic_cast<GROUP_NODE*>(pNode))
			{
				for (auto &i : pNode->children)
					addAllDefault(i.get(), params);
			}

			if (dynamic_cast<PARAM_NODE*>(pNode))
			{
				if (params.at(pNode->name) == "")
				{
					params.at(pNode->name) = dynamic_cast<PARAM_NODE*>(pNode)->defaultValue;
				}
				return;
			}
		}
		else
		{
			if (dynamic_cast<ROOT_NODE*>(pNode))
			{
				if (!pNode->children.empty())
				{
					if ((dynamic_cast<ROOT_NODE*>(pNode)->pDefault))
					{
						addAllDefault(dynamic_cast<ROOT_NODE*>(pNode)->pDefault, params);
					}
					else
					{
						throw std::logic_error(std::string("cmd \"") + pNode->name + "\" has no default param");
					}
				}

				pNode->isTaken = true;
			}

			if (dynamic_cast<UNIQUE_NODE*>(pNode))
			{
				if (!pNode->children.empty())
				{
					if (dynamic_cast<UNIQUE_NODE*>(pNode)->pDefault)
					{
						addAllDefault(dynamic_cast<UNIQUE_NODE*>(pNode)->pDefault, params);
					}
					else
					{
						throw std::logic_error(std::string("param \"") + pNode->name + "\" has no default sub-param");
					}
				}

				pNode->isTaken = true;
			}

			if (dynamic_cast<GROUP_NODE*>(pNode))
			{
				for (auto &i : pNode->children)
				{
					addAllDefault(i.get(), params);
				}
					

				pNode->isTaken = true;
			}

			if (dynamic_cast<PARAM_NODE*>(pNode))
			{
				params.insert(make_pair(pNode->name, dynamic_cast<PARAM_NODE*>(pNode)->defaultValue));
				pNode->isTaken = true;
			}
		}
	}

	struct COMMAND_STRUCT
	{
		std::unique_ptr<ROOT_NODE> root;
		map<string, NODE *> allParams{};
		map<char, string> shortNames{};

		COMMAND_STRUCT(const std::string &name) 
			:root(new ROOT_NODE(name.c_str()))
		{
		}
	};
	std::map<std::string, std::unique_ptr<COMMAND_STRUCT> > mapCmd;

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
		std::string homeCurStr(pContEle->Attribute("homeCur"));
		homeCur = std::stoi(homeCurStr);
		meter2count = c.CalculateExpression(pContEle->Attribute("meter2count")).ToDouble();

		/*construct mapPhy2Abs and mapAbs2Phy*/
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

		/*begin to copy client and insert cmd nodes*/
		const int TASK_NAME_LEN = 1024;
		char path_char[TASK_NAME_LEN] = { 0 };
#ifdef PLATFORM_IS_WINDOWS
		GetModuleFileName(NULL, path_char, TASK_NAME_LEN);
		std::string path(path_char);
		path = path.substr(0, path.rfind('\\'));
#endif
#ifdef PLATFORM_IS_LINUX
		char cParam[100] = { 0 };
		sprintf(cParam, "/proc/%d/exe", getpid());
		auto count = readlink(cParam, path_char, TASK_NAME_LEN);
		std::string path(path_char);
		path = path.substr(0, path.rfind('/'));
#endif

		auto pCmds = doc.RootElement()->FirstChildElement("Server")->FirstChildElement("Commands");

		if (pCmds == nullptr)
			throw std::logic_error("invalid client.xml");


		mapCmd.clear();
		for (auto pChild = pCmds->FirstChildElement(); pChild != nullptr; pChild = pChild->NextSiblingElement())
		{
#ifdef PLATFORM_IS_WINDOWS
			std::string fullpath = std::string("copy ") + path + std::string("\\Client.exe ") + path + "\\" + pChild->Name() + ".exe";
#endif
#ifdef PLATFORM_IS_LINUX
			std::string fullpath = std::string("cp ") + path + std::string("/Client ") + path + "/" + pChild->Name();
#endif
			auto ret = system(fullpath.c_str());

			if (mapCmd.find(pChild->Name())!=mapCmd.end())
				throw std::logic_error(std::string("command ")+ pChild->Name() +" is already existed, please rename it");

			mapCmd.insert(std::make_pair(std::string(pChild->Name()), std::unique_ptr<COMMAND_STRUCT>(new COMMAND_STRUCT(pChild->Name()))));
			addAllParams(pChild, mapCmd.at(pChild->Name())->root.get(), mapCmd.at(pChild->Name())->allParams, mapCmd.at(pChild->Name())->shortNames);
		}
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
        //initParam.homeTorqueLimit = 0;
        initParam.homeTorqueLimit = homeCur;
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
			Aris::Core::log(std::string("received connection, the ip is: ") + pRemoteIP +"\n");
			return 0;
		});
		server.SetOnReceiveRequest([this](Aris::Core::CONN *pConn, Aris::Core::MSG &msg)
		{
			Aris::Core::MSG ret;
            this->(msg,ret);

			return ret;
		});
		server.SetOnLoseConnection([this](Aris::Core::CONN *pConn)
		{
			Aris::Core::log("lost connection\n");
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

	void ROBOT_SERVER::ExecuteMsg(const Aris::Core::MSG &msg, Aris::Core::MSG &retError)
	{
		std::string cmd;
		std::map<std::string, std::string> params;
		try
		{
			DecodeMsg(msg, cmd, params);
		}
		catch (std::exception &e)
		{
			retError.Copy(e.what());
			return;
		}



		Aris::Core::MSG cmdMsg;
		GenerateCmdMsg(cmd, params, cmdMsg);

		cmdMsg.SetMsgID(0);

#ifdef PLATFORM_IS_LINUX
		cs.NRT_PostMsg(cmdMsg);
#endif
	}
	void ROBOT_SERVER::DecodeMsg(const Aris::Core::MSG &msg, std::string &cmd, std::map<std::string, std::string> &params)
	{
		std::vector<std::string> paramVector;
		int paramNum{0};

		/*将msg转换成cmd和一系列参数，不过这里的参数为原生字符串，既包括名称也包含值，例如“-heigt=0.5”*/
		if (msg.GetDataAddress()[msg.GetLength() - 1] == '\0')
		{
			std::string input{ msg.GetDataAddress() };
			std::stringstream inputStream{ input };
			std::string word;

			if (!(inputStream >> cmd))
			{
				throw std::logic_error(Aris::Core::log("invalid message from client, please at least contain a word\n"));
			};
			Aris::Core::log(std::string("received command string:")+msg.GetDataAddress()+"\n");

			while (inputStream >> word)
			{
				paramVector.push_back(word);
				++paramNum;
			}
		}
		else
		{
			throw std::logic_error(Aris::Core::log("invalid message from client, please be sure that the command message end with char \'\\0\'\n"));
		}

		if (mapCmd.find(cmd) != mapCmd.end())
		{
			mapCmd.at(cmd)->root->Reset();
		}
		else
		{
			throw std::logic_error(Aris::Core::log(std::string("invalid command name, server does not have command \"") + cmd + "\"\n"));
		}
		

		for (int i = 0; i<paramNum; ++i)
		{
			string str{ paramVector[i] };
			string paramName, paramValue;
			if (str.find("=") == string::npos)
			{
				paramName = str;
				paramValue = "";
			}
			else
			{
				paramName.assign(str, 0, str.find("="));
				paramValue.assign(str, str.find("=") + 1, str.size() - str.find("="));
			}

			if (paramName.size() == 0)
				throw logic_error("invalid param: what the hell, param should not start with '='");

			/*not start with '-'*/
			if (paramName.data()[0] != '-')
			{
				if (paramValue != "")
				{
					throw logic_error("invalid param: only param start with - or -- can be assigned a value\n");
				}

				for (auto c : paramName)
				{
					if (mapCmd.at(cmd)->shortNames.find(c) != mapCmd.at(cmd)->shortNames.end())
					{
						params.insert(make_pair(mapCmd.at(cmd)->shortNames.at(c), paramValue));
						mapCmd.at(cmd)->allParams.at(mapCmd.at(cmd)->shortNames.at(c))->Take();
					}
					else
					{
						throw logic_error(std::string("invalid param: param \"") + c + "\" is not a abbreviation of any valid param" );
					}
				}

				continue;
			}

			/*all following part start with at least one '-'*/
			if (paramName.size() == 1)
			{
				throw logic_error("invalid param: symbol \"-\" must be followed by an abbreviation of param");
			}

			/*start with '-', but only one '-'*/
			if (paramName.data()[1] != '-')
			{
				if (paramName.size() != 2)
				{
					throw std::logic_error("invalid param: param start with single '-' must be an abbreviation");
				}

				char c = paramName.data()[1];

				if (mapCmd.at(cmd)->shortNames.find(c) != mapCmd.at(cmd)->shortNames.end())
				{
					params.insert(make_pair(mapCmd.at(cmd)->shortNames.at(c), paramValue));
					mapCmd.at(cmd)->allParams.at(mapCmd.at(cmd)->shortNames.at(c))->Take();
				}
				else
				{
					throw logic_error(std::string("invalid param: param \"") + c + "\" is not a abbreviation of any valid param");
				}

				continue;
			}
			else
			{
				/*start with '--'*/
				if (paramName.size()<3)
				{
					throw std::logic_error("invalid param: symbol \"--\" must be followed by a full name of param");
				}

				string str = paramName;
				paramName.assign(str, 2, str.size() - 2);

				if (mapCmd.at(cmd)->allParams.find(paramName) != mapCmd.at(cmd)->allParams.end())
				{
					params.insert(make_pair(paramName, paramValue));
					mapCmd.at(cmd)->allParams.at(paramName)->Take();
				}
				else
				{
					throw logic_error(std::string("invalid param: param \"") + paramName + "\" is not a valid param");
				}

				

				continue;
			}
		}

		addAllDefault(mapCmd.at(cmd)->root.get(), params);

		cout << cmd << endl;

		int paramPrintLength;
		if (params.empty())
		{
			paramPrintLength = 2;
		}
		else
		{
			paramPrintLength = std::max_element(params.begin(), params.end(), [](decltype(*params.begin()) a, decltype(*params.begin()) b)
			{
				return a.first.length() < b.first.length();
			})->first.length() + 2;
		}

		int maxParamNameLength{ 0 };
		for (auto &i : params)
		{
			cout << std::string(paramPrintLength-i.first.length(),' ') << i.first << " : " << i.second << endl;
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

			if (msg.GetLength() < sizeof(GAIT_PARAM_BASE))
			{
				throw std::logic_error(std::string("parse function of command \"") + cmdPair->first + "\" failed: because it returned invalid msg\n");
			}

			reinterpret_cast<GAIT_PARAM_BASE *>(msg.GetDataAddress())->cmdType=RUN_GAIT;
			reinterpret_cast<GAIT_PARAM_BASE *>(msg.GetDataAddress())->cmdID=cmdPair->second;

		}
		else
		{
			throw std::logic_error(std::string("command \"") + cmdPair->first + "\" does not have gait function, please AddGait() first\n");
		}
	}

	int ROBOT_SERVER::home(Robots::ROBOT_BASE *pRobot, const Robots::GAIT_PARAM_BASE *param, Aris::RT_CONTROL::CMachineData &data)
	{
		bool isAllHomed = true;

		int id[18];
		a2p(param->motorID, id, param->motorNum);

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

					rt_printf("motor not homed, physical id: %d, absolute id: %d\n", id[i], param->motorID[i]);
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
				rt_printf("leg %d is homed\n", param->legID[i]);
				pRobot->pLegs[param->legID[i]]->SetPee(&homeEE[param->legID[i] * 3], "B");
			}
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
		pRobot->TransformCoordinatePee(pParam->beginBodyPE, "G", pParam->beginPee, "B", pEE_B);

        ret = this->allGaits.at(pParam->cmdID).operator()(pRobot,pParam); //pRobot legs and body altered

        pRobot->GetPin(pIn);

/********************position mode *************/
        //position mode,writing down positions to machinedata


		int id[18];
		a2p(pParam->motorID, id, pParam->motorNum);

		/*向下写入输入位置*/
		for (int i = 0; i<pParam->motorNum; ++i)
		{
			data.motorsCommands[id[i]] = Aris::RT_CONTROL::EMCMD_RUNNING;
			data.commandData[id[i]].Position = static_cast<int>(pIn[pParam->motorID[i]] * meter2count);
		}

		/*寻找不运动的腿，并将其设到初始位置*/
		for (int i = 0; i<6; ++i)
		{
			if ((std::find(pParam->legID, pParam->legID + pParam->legNum, i)) == (pParam->legID + pParam->legNum))
			{
				pRobot->pLegs[i]->SetPee(pEE_B + i * 3, "B");
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
			ret = 0;
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
        //? static double pBodyPE[6]{ 0 }, pEE[18]{ 0 };

        static const int cmdSize{8192};

		static char cmdQueue[50][cmdSize];

		static int currentCmd{ 0 };
		static int cmdNum{ 0 };
		static int count{ 0 };

		static Aris::RT_CONTROL::CMachineData lastCmdData=data , lastStateData=data ;
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
				rt_printf("the server is in count: %d\n", count);
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
		}
		else
		{
			data = cmdData;
			lastStateData = stateData;
			lastCmdData = cmdData;
		}


		

		return 0;
	}
}








