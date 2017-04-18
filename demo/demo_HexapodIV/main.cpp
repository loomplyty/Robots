

#include <iostream>

#include <aris.h>

#include "Robot_Type_II.h"
#include "Robot_Type_I.h"
#include "Robot_Gait.h"

double homeEE[18] =
{ 
-0.318791579531186,   -0.719675656557493,   -0.500049789146799,
-0.413084678293599,   -0.719675656557493,    0,
-0.318791579531187,   -0.719675656557493,    0.498125689146798,
0.318791579531186,   -0.719675656557493,   -0.500049789146799,
0.413084678293599,   -0.719675656557493,    0,
0.318791579531187,   -0.719675656557493,    0.498125689146798,
};

double firstEE[18] =
{
	-0.3,-0.75,-0.65,
	-0.45,-0.75,0,
	-0.3,-0.75,0.65,
	0.3,-0.75,-0.65,
	0.45,-0.75,0,
	0.3,-0.75,0.65,
};

double beginEE[18]
{
	-0.317036,-0.85,-0.670592,
	-0.45,-0.85,0,
	-0.3,-0.85,0.65,
	0.3,-0.85,-0.65,
	0.45,-0.85,0,
	0.3,-0.85,0.65,
};

double beginPe[6]{ 0.002996, -0.030742, -0.020592, 0.043633, 0, 0.043633 };

Robots::RobotTypeI rbt;

int main()
{
#ifdef WIN32
	rbt.loadXml("C:\\Robots\\resource\\Robot_Type_I\\Robot_VIII\\Robot_VIII.xml");
#endif
#ifdef UNIX
	rbt.loadXml("/usr/Robots/resource/Robot_Type_I/HexapodVIII.xml");
#endif

	rbt.SetPeb(beginPe);
	rbt.SetPee(beginEE);

	Robots::WalkParam param;
	param.totalCount = 3000;
	param.alpha = 2.7489;
	param.d = 0.26105;
	param.h = 0.05;
	param.n = 1;




	std::cout << "finished" << std::endl;

	char aaa;
	std::cin>>aaa;
	return 0;
}

