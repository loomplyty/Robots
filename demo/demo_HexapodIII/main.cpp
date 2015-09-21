#include <Platform.h>

#ifdef PLATFORM_IS_WINDOWS
#define _CRT_SECURE_NO_WARNINGS
#define _SCL_SECURE_NO_WARNINGS
#endif

#include <iostream>
#include <iomanip>
#include <cmath>

#include <Aris_Core.h>
#include <Aris_Socket.h>
#include <Aris_DynKer.h>
#include <Aris_DynModel.h>
#include <Aris_ExpCal.h>

#include "Robot_Server.h"

#include <map>

#include "Robot_Gait.h"
#include "HexapodIII.h"
#include "HexapodIV.h"

using namespace std;
using namespace Robots;

//using namespace Aris::Core;
using namespace Aris::DynKer;

ROBOT_III rbt;

#include<ctime>

int main()
{

#ifdef PLATFORM_IS_WINDOWS
	rbt.LoadXml("C:\\Robots\\resource\\HexapodIII\\HexapodIII.xml");
#endif
#ifdef PLATFORM_IS_LINUX
	rbt.LoadXml("/usr/Robots/resource/HexapodIII/HexapodIII.xml");
#endif

	//double pEE_G[18] =
	//{
	//	-0.4, -0.75, -0.7,
	//	-0.5, -0.75, 0,
	//	-0.4, -0.75, 0.7,
	//	 0.4, -0.75, -0.7,
	//	 0.5, -0.75, 0,
	//	 0.4, -0.75, 0.7
	//};
	//double bodyPE[6]{ 0,0,0,0,0,0 };
	//rbt.SetPee(pEE_G, bodyPE);
	//rbt.SetFixFeet("101010");
	//rbt.SetActiveMotion("011111011111011111");
	///**/
	//Robots::ADJUST_PARAM param;
	//std::copy_n(pEE_G, 18, &param.targetPee[0][0]);
	//std::copy_n(bodyPE, 6, &param.targetBodyPE[0][0]);

	//param.targetBodyPE[0][1] += 0.1;
	//param.targetPee[0][15] += 0.1;

	//rbt.SimByAdams("C:\\Users\\yang\\Desktop\\test", Robots::adjust, &param, 10);
	//rbt.SimByAdamsResultAt(155);
	//double fIn[18];
	//rbt.GetFinDyn(fIn);
	//dsp(fIn, 18, 1);

	//param.targetBodyPE[0][1] += 0.1;
	//param.targetPee[0][15] += 0.1;
	//rbt.SimByAdams("C:\\Users\\yang\\Desktop\\test1", Robots::adjust, &param, 10);


	
	/*Compute inverse position kinematics in Ground coordinates*/
	double pIn[18], vIn[18], aIn[18], fIn[18];
	double pEE_G[18] =
	{
		-0.4, -0.75, -0.7,
		-0.5, -0.75, 0,
		-0.4, -0.75, 0.7,
		 0.4, -0.75, -0.7,
		 0.5, -0.75, 0,
		 0.4, -0.75, 0.7
	};
	double vEE_G[18] =
	{
		0, 0, 0,
		10, 10, 10,
		0, 0, 0,
		0, 0, 0,
		0, 0, 0,
		0, 0, 0
	};
	double aEE_G[18] =
	{
		0, 0, 0,
		20, 10, -10,
		0, 0, 0,
		0, 0, 0,
		0, 0, 0,
		0, 0, 0
	};

	double bodyPE[6]{ 0.01,0.02,0.03,0.04,0.05,0.06 };
	//double bodyPE[6]{ 0 };

	double bodyVel[6]{ 0,0,0,0,0,0 };
	//double bodyVel[6]{ 0.9,0.8,0.7,0.6,0.5,0.4 };

	double bodyAcc[6]{ 0,0,0,0,0,0 };
	//double bodyAcc[6]{ 0 };

	rbt.SetFixFeet("101010");
	rbt.SetActiveMotion("011111011111011111");

	rbt.SetPee(pEE_G, bodyPE, "G");
	rbt.SetVee(vEE_G, bodyVel, "G");
	rbt.SetAee(aEE_G, bodyAcc, "G");
	rbt.GetPin(pIn);
	rbt.GetVin(vIn);
	rbt.GetAin(aIn);

	clock_t  clockBegin, clockEnd;
	clockBegin = clock();

	for (int i = 0; i < 10000; ++i)
	{
		rbt.SetPin(pIn);
		rbt.SetVin(vIn);
		rbt.SetAin(aIn);
		rbt.SetPee(pEE_G, bodyPE, "G");
		rbt.SetVee(vEE_G, bodyVel, "G");
		rbt.SetAee(aEE_G, bodyAcc, "G");
		rbt.FastDyn();
	}

	clockEnd = clock();
	std::cout << "consumed time is:" << double(clockEnd - clockBegin)/ CLOCKS_PER_SEC << std::endl;
	rbt.GetFinDyn(fIn);
	dsp(fIn, 18, 1);
	
	rbt.Dyn();

	rbt.ForEachMotion([](Aris::DynKer::MOTION_BASE *mot)
	{
		cout << mot->GetMotFceDyn() << endl;
	});




	char aaa;
	cin >> aaa;

	
	return 0;
}

