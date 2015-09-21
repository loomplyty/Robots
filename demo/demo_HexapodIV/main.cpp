#include <Platform.h>

#ifdef PLATFORM_IS_WINDOWS
#define _CRT_SECURE_NO_WARNINGS
#define _SCL_SECURE_NO_WARNINGS
#endif

#include "HexapodIV.h"
#include "HexapodIII.h"
#include "Robot_Gait.h"
#include <Aris_Plan.h>

using namespace std;
using namespace Aris::DynKer;
using namespace Aris::Plan;
using namespace Robots;

const int totalCount = 500;

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
	-0.3,-0.85,-0.65,
	-0.45,-0.85,0,
	-0.3,-0.85,0.65,
	0.3,-0.85,-0.65,
	0.45,-0.85,0,
	0.3,-0.85,0.65,
};

//Robots::ROBOT_IV rbt;
Robots::ROBOT_III rbt;



int main()
{
	const int totalCount = 3000;
	double pEE_Mat[totalCount][18], pIn_Mat[totalCount][18], pBodyEp_Mat[totalCount][6];
	
	
#ifdef PLATFORM_IS_WINDOWS
	rbt.LoadXml("C:\\Robots\\resource\\HexapodIII\\HexapodIII.xml");
#endif
#ifdef PLATFORM_IS_LINUX
	rbt.LoadXml("/usr/Robots/resource/HexapodIII/HexapodIII.xml");
#endif

	double beginPE[6]{ 0,0,-0.25,PI/2,0.3,-PI/2 };

	rbt.SetPee(beginEE, beginPE, "B");

	Robots::WALK_PARAM param;
	param.alpha = 0.3;
	param.beta = 0.5;
	param.h = 0.05;
	param.d = 0.8;
	param.totalCount = 500;
	param.n = 1;
	param.upDirection = 2;
	param.walkDirection = -3;

	//std::copy_n(beginPee, 18, param.beginPee);
	//std::copy_n(beginPE, 6, param.beginBodyPE);
	//std::fill_n(param.beginBodyPE, 6, 0);
	rbt.GetPee(param.beginPee);
	rbt.GetBodyPe(param.beginBodyPE);

	








	param.count = 0;
	
	while (true)
	{
		int ret = walk2(&rbt, &param);
		rbt.GetPee(pEE_Mat[param.count],"G");
		rbt.GetPin(pIn_Mat[param.count]);
		rbt.GetBodyPe(pBodyEp_Mat[param.count]);
		param.count++;

		if (ret == 0)
			break;
	}

	dlmwrite("C:\\Users\\yang\\Desktop\\pIn_Mat.txt", *pIn_Mat, param.count, 18);
	dlmwrite("C:\\Users\\yang\\Desktop\\pEE_Mat.txt", *pEE_Mat, param.count, 18);
	dlmwrite("C:\\Users\\yang\\Desktop\\pBody_Mat.txt", *pBodyEp_Mat, param.count, 6);
	


	//Robots::ADJUST_PARAM adParam;

	//adParam.periodNum = 2;
	//adParam.periodCount[0] = 1000;
	//adParam.periodCount[1] = 1500;

	//std::copy_n(homeEE, 18, adParam.beginPee);
	//std::fill_n(adParam.beginBodyPE, 6, 0);
	//std::copy_n(firstEE, 18, adParam.targetPee[0]);
	//std::fill_n(adParam.targetBodyPE[0], 6, 0);
	//std::copy_n(beginEE, 18, adParam.targetPee[1]);
	//std::fill_n(adParam.targetBodyPE[1], 6, 0);

	//adParam.count = 0;

	//adjust(&rbt, &adParam);

	//while (true)
	//{
	//	int ret = adjust(&rbt, &adParam);
	//	rbt.GetPee(pEE_Mat[adParam.count], "G");
	//	rbt.GetPin(pIn_Mat[adParam.count]);
	//	rbt.GetBodyPe(pBodyEp_Mat[adParam.count]);
	//	adParam.count++;

	//	if (ret == 0)
	//		break;
	//}

	//dlmwrite("C:\\Users\\yang\\Desktop\\pIn_Mat.txt", *pIn_Mat, adParam.count, 18);
	//dlmwrite("C:\\Users\\yang\\Desktop\\pEE_Mat.txt", *pEE_Mat, adParam.count, 18);
	//dlmwrite("C:\\Users\\yang\\Desktop\\pBody_Mat.txt", *pBodyEp_Mat, adParam.count, 6);
	


	double pBody[6], pEE[3];

	param.count = 499;
	walk(&rbt, &param);

	


	rbt.GetBodyPe(pBody);
	dsp(pBody, 6, 1);

	param.count = 1499;
	walk(&rbt, &param);

	
	rbt.GetBodyPe(pBody);
	dsp(pBody, 6, 1);

	rbt.pLM->GetPee(pEE, "G");
	dsp(pEE, 3, 1);


	double pm1[16], pm2[16], pm3[16], pm_end[16];

	double pe[6] = { 0.1,0.2,0.3,0,0,0 };

	s_pe2pm(pe, pm1);
	s_pe2pm(pe, pm2);
	s_pe2pm(pe, pm3);

	s_pm_dot_pm(pm1, pm2, pm3, pm_end);

	dsp(pm_end,4,4);

	cout << "finished" << endl;


	char aaa;
	cin>>aaa;
	return 0;
}

