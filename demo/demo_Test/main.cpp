#include <iostream>
#include <iomanip>
#include <cmath>
#include <Aris_Core.h>
#include <Aris_Socket.h>
#include <Aris_DynKer.h>
#include <Aris_ExpCal.h>

using namespace std;
using namespace Aris::Core;
using namespace Aris::DynKer;


int main()
{
	// -x（机器人坐标系）轴转   RPY：1.83144	0.0982242 - 3.07334



	double pm_G02G[4][4] =
	{
		{ -1, 0, 0, 0 },
		{ 0, 0, 1, 0 },
		{ 0, 1, 0, 0 },
		{ 0, 0, 0, 1 }
	};

	//double pe[6] = { -3.07334, 0.0982242, 1.83144, 0, 0, 0 };

	//double pe[6] = { -3.019166, - 0.051400, 2.312598, 0, 0, 0 };//x-
	//double pe[6] = { 2.413440 , 0.003196 , -3.096247,   0, 0, 0 };//x+
	//double pe[6] = { 3.091502 , 0.582818 , 3.111017, 0, 0, 0 };//z-
	double pe[6] = { 2.950758 -0.528866 -3.099109, 0, 0, 0 };//z+

	//double pe[6] = { -3.07334, 0.0982242, 1.83144, 0, 0, 0 };//x- z-
	//double pe[6] = { -3.07334, 0.0982242, 1.83144, 0, 0, 0 };//z+ x+


	double loc = pe[0];
	pe[0] = pe[2];
	pe[2] = loc;

	dsp(pe, 6, 1);

	double pm_I2G0[4][4];
	s_pe2pm(pe, *pm_I2G0, "321");
	dsp(*pm_I2G0, 4, 4);

	double pm_R2I[4][4] =
	{
		{ 1, 0, 0, 0 },
		{ 0, 0, 1, 0 },
		{ 0, -1, 0, 0 },
		{ 0, 0, 0, 1 }
	};


	double pm_R2G0[4][4];

	double pm_R2G[4][4];

	s_pm_dot_pm(*pm_I2G0, *pm_R2I, *pm_R2G0);
	s_pm_dot_pm(*pm_G02G, *pm_R2G0, *pm_R2G);
	dsp(*pm_R2G, 4, 4);

	double pe_changed[6];
	s_pm2pe(*pm_R2G, pe_changed, "321");

	dsp(pe_changed, 6, 1);






	char aaa;
	cin >> aaa;
	return 0;
}

