#include <Platform.h>
#include <Robot_Client.h>

using namespace std;

int main(int argc, char *argv[])
{
#ifdef PLATFORM_IS_LINUX
	Robots::SendRequest(argc, argv, "/usr/Robots/resource/HexapodIII/HexapodIII.xml");
#endif
#ifdef PLATFORM_IS_WINDOWS
	Robots::SendRequest(argc, argv, "C:\\Robots\\resource\\HexapodIII\\HexapodIII.xml");
#endif

	return 0;
}
