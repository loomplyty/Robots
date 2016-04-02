#include <Platform.h>

#include <iostream>
#include <list>
#include <vector>
#include <map>
#include <cstring>
#include <string>
#include <memory>
#include <algorithm>
#include <cstdint>

#include <Aris_Socket.h>
#include <Aris_XML.h>
#include <Aris_Core.h>

using namespace std;

namespace Robots
{
	int SendRequest(int argc, char *argv[], const char *xmlFileName)
	{
		/*需要去除命令名的路径和扩展名*/
		std::string cmdName(argv[0]);
		
#ifdef PLATFORM_IS_WINDOWS
		if (cmdName.rfind('\\'))
		{
			cmdName = cmdName.substr(cmdName.rfind('\\') + 1, cmdName.npos);
		}
#endif
#ifdef PLATFORM_IS_LINUX
		if (cmdName.rfind('/'))
		{
			cmdName = cmdName.substr(cmdName.rfind('/') + 1, cmdName.npos);
		}
#endif
		
		if (cmdName.rfind('.'))
		{
			cmdName = cmdName.substr(0, cmdName.rfind('.'));
		}

		/*添加命令的所有参数*/
		for (int i = 1; i < argc; ++i)
		{
			cmdName = cmdName + " " + argv[i];
		}



		/*构造msg，这里需要先copy命令名称，然后依次copy各个参数*/
		Aris::Core::MSG msg;
		msg.Copy(cmdName.c_str());
		


		/*连接并发送msg*/
		Aris::Core::DOCUMENT doc;

		if (doc.LoadFile(xmlFileName) != 0)
			throw std::logic_error("failed to read configuration xml file");

		std::string ip = doc.RootElement()->FirstChildElement("Server")->FirstChildElement("Connection")->Attribute("IP");
		std::string port = doc.RootElement()->FirstChildElement("Server")->FirstChildElement("Connection")->Attribute("Port");

		Aris::Core::CONN conn;

		conn.Connect(ip.c_str(), port.c_str());
		Aris::Core::MSG ret = conn.SendRequest(msg);

		/*错误处理*/
		if (ret.GetLength() > 0)
		{
			cout << "cmd has fault, please regard to following information:" << endl;
			cout << "    " << ret.GetDataAddress() << std::endl;
		}
		else
		{
			cout << "send command successful" << std::endl;
		}

		return 0;
	}
}

