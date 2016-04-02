

#include <iostream>
#include "Robot_Type_I.h"

using namespace std;
using namespace Robots;
using namespace aris::dynamic;


int main()
{

	RobotTypeI rbt;

#ifdef WIN32
	rbt.loadXml("C:\\Robots\\resource\\Robot_Type_I\\Robot_III\\Robot_III.xml");
#endif
#ifdef UNIX
	rbt.loadXml("/usr/Robots/resource/Robot_Type_I/Robot_III/Robot_III.xml");
#endif


	/*Compute inverse position kinematics in Ground coordinates*/
	
	
	/*
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

	//double bodyVel[6]{ 0,0,0,0,0,0 };
	double bodyVel[6]{ 0.9,0.8,0.7,0.6,0.5,0.4 };

	double bodyAcc[6]{ 0.2,0.2,0.2,0.2,0.2,0.2 };
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

	for (int i = 0; i < 1; ++i)
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
	
	rbt.dyn();

	rbt.ForEachMotion([](aris::dynamic::Motion *mot)
	{
		cout << mot->GetMotFceDyn() << endl;
	});


	double zeros[18]{0};
	std::cout << "*****************  position  ********************" << std::endl;
	double pEE[18];
	rbt.GetPee(pEE,"B");
	dsp(pEE, 6, 3);
	rbt.GetPee(pEE, &rbt.body());
	dsp(pEE, 6, 3);

	std::cout << "-----------" << std::endl;
	rbt.SetPee(zeros, nullptr, "G");

	rbt.SetPee(pEE, &rbt.body());
	rbt.GetPee(pEE, &rbt.body());
	dsp(pEE, 6, 3);


	std::cout << "*****************  velocity  ********************" << std::endl;
	double vEE[18];
	rbt.GetVee(vEE, "B");
	dsp(vEE, 6, 3);
	rbt.GetVee(vEE, &rbt.body());
	dsp(vEE, 6, 3);

	std::cout << "-----------" << std::endl;
	rbt.SetVee(zeros, nullptr, "G");

	rbt.SetVee(vEE, &rbt.Ground());
	rbt.GetVee(vEE, "G");
	dsp(vEE, 6, 3);

	std::cout << "*****************  accleration  ********************" << std::endl;
	double aEE[3];
	rbt.pLM->GetAee(aEE, "B");
	dsp(aEE, 3, 1);
	rbt.pLM->GetAee(aEE, &rbt.body());
	dsp(aEE, 3, 1);

	std::cout << "-----------" << std::endl;
	rbt.pLM->SetAee(zeros, "G");

	rbt.pLM->SetAee(aEE, &rbt.body());
	rbt.pLM->GetAee(aEE, "B");
	dsp(aEE, 3, 1);

	std::cout << "*****************  fEE_sta  ********************" << std::endl;
	double fEE[3]{0,1,0.2};
	rbt.pLM->SetFeeSta(fEE, "L");
	//dsp(fEE, 3, 1);
	rbt.pLM->GetFeeSta(fEE, &rbt.body());
	dsp(fEE, 3, 1);
	rbt.pLM->GetFeeSta(fEE, "B");
	dsp(fEE, 3, 1);

	std::cout << "-----------" << std::endl;
	rbt.pLM->SetFeeSta(zeros, "G");

	rbt.pLM->SetFeeSta(fEE, &rbt.body());
	rbt.pLM->GetFeeSta(fEE, "B");
	dsp(fEE, 3, 1);

	std::cout << "*****************  Jacobian  ********************" << std::endl;
	double J[9];
	rbt.pLM->GetJfd(J, &rbt.body());
	dsp(J, 3, 3);
	rbt.pLM->GetJfd(J, "B");
	dsp(J, 3, 3);

	std::cout << "-----------" << std::endl;
	rbt.pLM->GetJfi(J, &rbt.body());
	dsp(J, 3, 3);
	rbt.pLM->GetJfi(J, "B");
	dsp(J, 3, 3);


	std::cout << "*****************  Jacobian  ********************" << std::endl;
	double dJ[9];
	rbt.pLM->GetDifJfd(dJ, &rbt.Ground());
	dsp(dJ, 3, 3);
	rbt.pLM->GetDifJfd(dJ, "G");
	dsp(dJ, 3, 3);

	std::cout << "-----------" << std::endl;
	rbt.pLM->GetDifJfi(dJ, &rbt.Ground());
	dsp(dJ, 3, 3);
	rbt.pLM->GetDifJfi(dJ, "G");
	dsp(dJ, 3, 3);

	std::cout << "*****************  cv  ********************" << std::endl;
	double c[3];
	rbt.pLM->GetCvd(c, &rbt.body());
	dsp(c, 3, 1);
	rbt.pLM->GetCvd(c, "B");
	dsp(c, 3, 1);

	std::cout << "-----------" << std::endl;
	rbt.pLM->GetCvi(c, &rbt.Ground());
	dsp(c, 3, 1);
	rbt.pLM->GetCvi(c, "G");
	dsp(c, 3, 1);

	std::cout << "-----------" << std::endl;
	rbt.pLM->GetCai(c, &rbt.Ground());
	dsp(c, 3, 1);
	rbt.pLM->GetCai(c, "G");
	dsp(c, 3, 1);

	std::cout << "-----------" << std::endl;
	rbt.pLM->GetCad(c, &rbt.Ground());
	dsp(c, 3, 1);
	rbt.pLM->GetCad(c, "G");
	dsp(c, 3, 1);
	*/


	char aaa;
	cin >> aaa;
	
	
	return 0;
}

