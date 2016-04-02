#include <Platform.h>

#include <iostream>
#include <iomanip>
#include <cmath>
#include <Aris_Core.h>
#include <Aris_Socket.h>
#include <Aris_DynKer.h>
#include <Aris_ExpCal.h>
#include <Aris_Plan.h>
#include <Robot_Type_I.h>

using namespace std;
using namespace Aris::Core;
using namespace Aris::DynKer;
using namespace Aris::Plan;

Robots::ROBOT_TYPE_I robot;

double eePosIni[6][3] =
{ { -0.3, -0.85, -0.6 }
, { -0.45, -0.85, 0 }
, { -0.3, -0.85, 0.6 }
, { 0.3, -0.85, -0.5 }
, { 0.45, -0.85, 0 }
, { 0.3, -0.85, 0.5 } };


#define NUM 900

double stepH = 0.04;
double stepD = 1.1;
double totalTime = NUM;
double v = stepD / totalTime * 1000 / 2;
int leg_index = 2;

void b_const(double s_in, double *b_out)
{
	b_out[0] = eePosIni[leg_index][0];
	b_out[1] = stepH * sin(s_in) + eePosIni[leg_index][1];
	b_out[2] = -stepD / 2 * cos(s_in) + eePosIni[leg_index][2] + stepD / 4;
}
void g_const(double s_in, double *g_out)
{
	g_out[0] = 0;
	g_out[1] = stepH*cos(s_in);
	g_out[2] = stepD / 2 * sin(s_in);
}
void h_const(double s_in, double *h_out)
{
	h_out[0] = 0;
	h_out[1] = -stepH*sin(s_in);
	h_out[2] = stepD / 2 * cos(s_in);
}

void GetEveryThing(Aris::Plan::FAST_PATH::DATA & data)
{
	double bodyPe[6]{ 0,0,v*data.time,0,0,0 }, bodyVel[6]{ 0,0,v,0,0,0 }, bodyAcc[6]{ 0 };
	double bodyPm[16];
	s_pe2pm(bodyPe, bodyPm);
	robot.pBody->SetPm(bodyPm);
	robot.pBody->SetVel(bodyVel);
	robot.pBody->SetAcc(bodyAcc);

	double pEE[3]{ 0 }, vEE[3]{ 0 };
	
	b_const(data.s, pEE);
	g_const(data.s, data.g);
	h_const(data.s, data.h);
	
	s_daxpy(3, data.ds, data.g, 1, vEE, 1);

	robot.pLegs[leg_index]->SetPee(pEE, "G");
	robot.pLegs[leg_index]->SetVee(vEE, "G");

	robot.pLegs[leg_index]->GetJvi(data.Ji, "G");
	robot.pLegs[leg_index]->GetDifJvi(data.dJi, "G");
	robot.pLegs[leg_index]->GetCvi(data.Cv, "G");
	robot.pLegs[leg_index]->GetCai(data.Ca, "G");
}


int main()
{
#ifdef PLATFORM_IS_WINDOWS
	robot.LoadXml("C:\\Robots\\resource\\Robot_Type_I\\Robot_VIII.xml");
#endif
#ifdef PLATFORM_IS_LINUX
	robot.LoadXml("/usr/Robots/resource/Robot_Type_I/Robot_III.xml");
#endif
	
	FAST_PATH tg;

	tg.SetMotorLimit(std::vector<FAST_PATH::MOTOR_LIMIT> {3, { 0.9,-0.9,3.2,-3.2 } });
	tg.SetBeginNode({ 0.0, 0.0, 0.0, 0.0, true });
	tg.SetEndNode({ totalTime / 1000.0, PI, 0.0, 0.0, true });
	tg.SetFunction(GetEveryThing);
	tg.Run();

	double pEE_all[NUM][3];
	double pIn_all[NUM][3];

	for (int i = 0; i < NUM; ++i)
	{
		b_const(tg.Result().at(i), pEE_all[i]);

		double pe[6]{ 0, 0, v * (i+1)*0.001, 0, 0, 0 };
		double pm[16];
		s_pe2pm(pe, pm);
		robot.pBody->SetPm(pm);
		robot.pLegs[leg_index]->SetPee(pEE_all[i]);
		robot.pLegs[leg_index]->GetPin(pIn_all[i]);
	}
	Aris::DynKer::dlmwrite("C:\\Users\\yang\\Desktop\\pIn.txt", *pIn_all, NUM, 3);
	Aris::DynKer::dlmwrite("C:\\Users\\yang\\Desktop\\pEE.txt", *pEE_all, NUM, 3);

//#define NUM 1900
//	double s[NUM];
//	double ds[NUM];
//	double dds[NUM];
//
//
//
//	for (auto p = tg.list.begin(); p != tg.list.end(); ++p)
//	{
//		static auto i = 0;
//
//
//		s[i] = p->s;
//		ds[i] = p->ds;
//		dds[i] = p->dds;
//		++i;
//	}
//	Aris::DynKer::dlmwrite("C:\\Users\\yang\\Desktop\\s.txt", s, tg.list.size(), 1);
//	Aris::DynKer::dlmwrite("C:\\Users\\yang\\Desktop\\ds.txt", ds, tg.list.size(), 1);
//	Aris::DynKer::dlmwrite("C:\\Users\\yang\\Desktop\\dds.txt", dds, tg.list.size(), 1);

	std::cout << "size:" << tg.list.size();

	char aaa;
	std::cin >> aaa;
	return 0;
}

