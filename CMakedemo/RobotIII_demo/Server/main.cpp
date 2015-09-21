#include <iostream>
#include <cstring>
#include <iomanip> 
#include <bitset>
#include <cstring>
#include <map>
#include <string>

using namespace std;

#include <stdlib.h>

#include <Platform.h>
#include <Aris_Core.h>
#include <Aris_Message.h>
#include <Robot_Server.h>
#include <HexapodIII.h>

using namespace Aris::Core;

int fast(Robots::ROBOT_BASE * pRobot, const Robots::GAIT_PARAM_BASE * pParam)
{
	auto pRealRobot = dynamic_cast<Robots::ROBOT_III *>(pRobot);
	pRealRobot->FastDyn();
	pRealRobot->pLF->pSf->Activate();


}

int main()
{
	auto rs = Robots::ROBOT_SERVER::GetInstance();
	rs->CreateRobot<Robots::ROBOT_III>();

#ifdef PLATFORM_IS_LINUX
	rs->LoadXml("/usr/Robots/resource/HexapodIII/HexapodIII.xml");
#endif
#ifdef PLATFORM_IS_WINDOWS
	rs->LoadXml("C:\\Robots\\resource\\HexapodIII\\HexapodIII.xml");
#endif

	rs->AddGait("wk", Robots::walk, Robots::parseWalk);
	rs->AddGait("ad", Robots::adjust, Robots::parseAdjust);
	rs->Start();

	Aris::Core::RunMsgLoop();

	return 0;
}
