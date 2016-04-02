#include <Aris_Plan.h>
#include <Aris_Thread.h>
#include <Aris_DynKer.h>

#include <cmath>
#include <iostream>

#include "Hexapod_Robot.h"

using namespace std;

using namespace Hexapod_Robot;
using namespace Aris::DynKer;
using namespace Aris::Plan;

Hexapod_Robot::ROBOT robot;
Aris::Plan::TRAJECTORY_GENERATOR<3, 3> generator;

#define PERIOD 1850


namespace HexapodRobotPlan
{
	double stepH = 0.04, stepD = 1.1;
	double v = stepD / 2 * 1000 / PERIOD;

	int leg_index;

	double final_s[6][PERIOD];

	double data[PERIOD * 4][18];

	double eePosBeforeMoveConst[6][3] =
	{ { -0.3, -0.85, -0.925 }
	, { -0.45, -0.85, -0.275 }
	, { -0.3, -0.85, 0.375 }
	, { 0.3, -0.85, -0.925 }
	, { 0.45, -0.85, -0.275 }
	, { 0.3, -0.85, 0.375 } };

	double eePosAfterMoveConst[6][3] =
	{ { -0.3, -0.85, -0.375 }
	, { -0.45, -0.85, 0.275 }
	, { -0.3, -0.85, 0.925 }
	, { 0.3, -0.85, -0.375 }
	, { 0.45, -0.85, 0.275 }
	, { 0.3, -0.85, 0.925 } };

	double eePosIni[6][3] =
	{ { -0.3, -0.85, -0.65 }
	, { -0.45, -0.85, 0 }
	, { -0.3, -0.85, 0.65 }
	, { 0.3, -0.85, -0.65 }
	, { 0.45, -0.85, 0 }
	, { 0.3, -0.85, 0.65 } };

	double zR1[6] = { 0.045, 0.05, 0.025, 0.045, 0.05, 0.018 };
	double zR2[6] = { stepD / 2, stepD / 2, stepD / 2, stepD / 2, stepD / 2, stepD / 2, };
	double zR3[6] = { 0.018, 0.0285, 0.05, 0.018, 0.0285, 0.038 };
	double zT2[6] = { 1, 1, 1, 1, 1, 1 };
	double zT1[6] = {
		zR1[0] / zR2[0] * PI*zT2[0],
		zR1[1] / zR2[1] * PI*zT2[1],
		zR1[2] / zR2[2] * PI*zT2[2],
		zR1[3] / zR2[3] * PI*zT2[3],
		zR1[4] / zR2[4] * PI*zT2[4],
		zR1[5] / zR2[5] * PI*zT2[5] };
	double zT3[6] = {
		zR3[0] / zR2[0] * PI*zT2[0],
		zR3[1] / zR2[1] * PI*zT2[1],
		zR3[2] / zR2[2] * PI*zT2[2],
		zR3[3] / zR2[3] * PI*zT2[3],
		zR3[4] / zR2[4] * PI*zT2[4],
		zR3[5] / zR2[5] * PI*zT2[5] };
	double zBT[6][4] = {
		{ 0, zT1[0], zT1[0] + zT2[0], zT1[0] + zT2[0] + zT3[0] },
		{ 0, zT1[1], zT1[1] + zT2[1], zT1[1] + zT2[1] + zT3[1] },
		{ 0, zT1[2], zT1[2] + zT2[2], zT1[2] + zT2[2] + zT3[2] },
		{ 0, zT1[3], zT1[3] + zT2[3], zT1[3] + zT2[3] + zT3[3] },
		{ 0, zT1[4], zT1[4] + zT2[4], zT1[4] + zT2[4] + zT3[4] },
		{ 0, zT1[5], zT1[5] + zT2[5], zT1[5] + zT2[5] + zT3[5] },
	};

	double yT1[6] = { zBT[0][3] / 2 - 0.3, zBT[1][3] / 2 - 0.4, 0.28
		, zBT[3][3] / 2 - 0.3, zBT[4][3] / 2 - 0.4, 0.25 };
	double yT3[6] = { zBT[0][3] / 2 + 0.3, zBT[1][3] / 2 + 0.4, 0.3
		, zBT[3][3] / 2 + 0.3, zBT[4][3] / 2 + 0.4, zBT[5][3] / 2 };
	double yT2[6] = {
		zBT[0][3] - yT1[0] - yT3[0],
		zBT[1][3] - yT1[1] - yT3[1],
		zBT[2][3] - yT1[2] - yT3[2],
		zBT[3][3] - yT1[3] - yT3[3],
		zBT[4][3] - yT1[4] - yT3[4],
		zBT[5][3] - yT1[5] - yT3[5] };

	double yBT[6][4] = {
		{ 0, yT1[0], yT1[0] + yT2[0], yT1[0] + yT2[0] + yT3[0] },
		{ 0, yT1[1], yT1[1] + yT2[1], yT1[1] + yT2[1] + yT3[1] },
		{ 0, yT1[2], yT1[2] + yT2[2], yT1[2] + yT2[2] + yT3[2] },
		{ 0, yT1[3], yT1[3] + yT2[3], yT1[3] + yT2[3] + yT3[3] },
		{ 0, yT1[4], yT1[4] + yT2[4], yT1[4] + yT2[4] + yT3[4] },
		{ 0, yT1[5], yT1[5] + yT2[5], yT1[5] + yT2[5] + yT3[5] },
	};


	void b_const(double s_in, double *b_out)
	{
		b_out[0] = eePosIni[leg_index][0];

		double ratio;

		if (s_in < yBT[leg_index][1])
		{
			ratio = PI / yT1[leg_index];
			b_out[1] = stepH *(1 - cos((s_in - yBT[leg_index][0])*ratio)) / 2 + eePosIni[leg_index][1];
		}
		else if (s_in < yBT[leg_index][2])
		{
			b_out[1] = stepH + eePosIni[leg_index][1];
		}
		else
		{
			ratio = PI / yT3[leg_index];
			b_out[1] = stepH *(1 - cos((s_in - yBT[leg_index][2])*ratio + PI)) / 2 + eePosIni[leg_index][1];
		}




		if (s_in < zBT[leg_index][1])
		{
			ratio = PI / zT1[leg_index];
			b_out[2] = -zR1[leg_index] * sin((s_in - zBT[leg_index][0])*ratio) - stepD / 4 + eePosIni[leg_index][2];
		}
		else if (s_in < zBT[leg_index][2])
		{
			b_out[2] = zR2[leg_index] * (s_in - zBT[leg_index][1]) / zT2[leg_index] - stepD / 4 + eePosIni[leg_index][2];
		}
		else
		{
			ratio = PI / zT3[leg_index];
			b_out[2] = -zR3[leg_index] * sin((s_in - zBT[leg_index][2])*ratio + PI) + eePosIni[leg_index][2] + stepD / 4;
		}
	}
	void g_const(double s_in, double *g_out)
	{
		g_out[0] = 0;

		double ratio;

		if (s_in < yBT[leg_index][1])
		{
			ratio = PI / yT1[leg_index];
			g_out[1] = stepH *sin((s_in - yBT[leg_index][0])*ratio) / 2 * ratio;
		}
		else if (s_in < yBT[leg_index][2])
		{
			g_out[1] = 0;
		}
		else
		{
			ratio = PI / yT3[leg_index];
			g_out[1] = stepH *sin((s_in - yBT[leg_index][2])*ratio + PI) / 2 * ratio;
		}




		if (s_in < zBT[leg_index][1])
		{
			ratio = PI / zT1[leg_index];
			g_out[2] = -zR1[leg_index] * cos((s_in - zBT[leg_index][0])*ratio)*ratio;
		}
		else if (s_in < zBT[leg_index][2])
		{
			g_out[2] = zR2[leg_index] / zT2[leg_index];
		}
		else
		{
			ratio = PI / zT3[leg_index];
			g_out[2] = -zR3[leg_index] * cos((s_in - zBT[leg_index][2])*ratio + PI)*ratio;
		}
	}
	void h_const(double s_in, double *h_out)
	{
		h_out[0] = 0;

		double ratio = 2 * PI / zBT[leg_index][3];
		h_out[1] = stepH *cos(s_in *ratio) / 2 * ratio* ratio;

		if (s_in < yBT[leg_index][1])
		{
			ratio = PI / yT1[leg_index];
			h_out[1] = stepH *cos((s_in - yBT[leg_index][0])*ratio) / 2 * ratio* ratio;
		}
		else if (s_in < yBT[leg_index][2])
		{
			h_out[1] = 0;
		}
		else
		{
			ratio = PI / yT3[leg_index];
			h_out[1] = stepH *cos((s_in - yBT[leg_index][2])*ratio + PI) / 2 * ratio* ratio;
		}


		if (s_in < zBT[leg_index][1])
		{
			ratio = PI / zT1[leg_index];
			h_out[2] = zR1[leg_index] * sin((s_in - zBT[leg_index][0])*ratio)*ratio*ratio;
		}
		else if (s_in < zBT[leg_index][2])
		{
			h_out[2] = 0;
		}
		else
		{
			ratio = PI / zT3[leg_index];
			h_out[2] = zR3[leg_index] * sin((s_in - zBT[leg_index][2])*ratio + PI)*ratio*ratio;
		}
	}

	void b_acc(double s_in, double *b_out)
	{
		b_out[0] = eePosIni[leg_index][0];
		b_out[1] = stepH * sin(s_in) + eePosIni[leg_index][1];
		b_out[2] = -stepD / 4 * cos(s_in) + eePosIni[leg_index][2] + stepD / 4;
	}
	void g_acc(double s_in, double *g_out)
	{
		g_out[0] = 0;
		g_out[1] = stepH*cos(s_in);
		g_out[2] = stepD / 4 * sin(s_in);
	}
	void h_acc(double s_in, double *h_out)
	{
		h_out[0] = 0;
		h_out[1] = -stepH*sin(s_in);
		h_out[2] = stepD / 4 * cos(s_in);
	}

	void b_dec(double s_in, double *b_out)
	{
		b_out[0] = eePosBeforeMoveConst[leg_index][0];
		b_out[1] = stepH * sin(s_in) + eePosBeforeMoveConst[leg_index][1];
		b_out[2] = -stepD / 4 * cos(s_in) + eePosBeforeMoveConst[leg_index][2] + stepD / 4;
	}
	void g_dec(double s_in, double *g_out)
	{
		g_out[0] = 0;
		g_out[1] = stepH*cos(s_in);
		g_out[2] = stepD / 4 * sin(s_in);
	}
	void h_dec(double s_in, double *h_out)
	{
		h_out[0] = 0;
		h_out[1] = -stepH*sin(s_in);
		h_out[2] = stepD / 4 * cos(s_in);
	}

	int GetEveryThing(double t_in, double s_in, double dotS_in
		, double* jacInv_out, double* cV_out, double* cA_out
		, double* g_out, double* h_out)
	{
		double bodyEp[6], bodyVel[6], bodyAcc[6], bodyPm[4][4];

		double pEE[3], vEE[3];

		/*Set Body Pm and vel*/
		memset(bodyEp, 0, sizeof(bodyEp));
		memset(bodyVel, 0, sizeof(bodyVel));
		memset(bodyAcc, 0, sizeof(bodyAcc));

		s_ep2pm(bodyEp, *bodyPm);

		robot.pBody->SetPm(*bodyPm);
		robot.pBody->SetVel(bodyVel);
		robot.pBody->SetAcc(bodyAcc);

		/*Set Leg Pos */
		b_const(s_in, pEE);
		g_const(s_in, g_out);
		h_const(s_in, h_out);

		vEE[0] = g_out[0] * dotS_in;
		vEE[1] = g_out[1] * dotS_in;
		vEE[2] = g_out[2] * dotS_in;

		robot.pLegs[leg_index]->SetPee(pEE, "B");
		robot.pLegs[leg_index]->SetVee(vEE, "B");

		robot.pLegs[leg_index]->GetVelJacInv(0, cV_out, "B");
		robot.pLegs[leg_index]->GetAccJacInv(jacInv_out, cA_out, "B");

		return 0;
	};

	int Plan_Output_Acc()
	{
		double pEE[18], bodyEp[6];
		for (int i = 0; i < PERIOD; ++i)
		{
			memset(bodyEp, 0, sizeof(bodyEp));
			bodyEp[5] = acc_even(PERIOD, i + 1)*stepD / 4;

			for (int j = 0; j < 6; ++j)
			{
				double b[3];

				if (j % 2 != 0)
				{
					leg_index = j;


					if (j == 5)
					{
						int nb = 400;
						int nn = 1450;
						int ne = PERIOD;

						int n1 = nb;
						int n2 = nn - nb;
						int n3 = PERIOD - nn;

						double d2 = PI / (1 / 1.5*n1 / n2 + 1 + 1 / 1.5*n3 / n2);
						double d1 = 1 / 1.5*n1 / n2 * d2;
						double d3 = 1 / 1.5*n3 / n2 * d2;


						if (i < nb)
						{
							b_acc(acc_down(n1, i + 1)*d1, b);
						}
						else if (i < nn)
						{
							b_acc(even(n2, i + 1 - nb)*d2 + d1, b);
						}
						else
						{
							b_acc(dec_down(n3, i + 1 - nn)*d3 + d1 + d2, b);
						}
					}
					else
					{
						if (i < PERIOD / 2)
						{
							b_acc(acc_down(PERIOD / 2, i + 1)*PI / 2, b);
						}
						else
						{
							b_acc(dec_down(PERIOD / 2, i + 1 - PERIOD / 2)*PI / 2 + PI / 2, b);
						}
					}




					memcpy(pEE + j * 3, b, 3 * sizeof(double));
				}
				else
				{
					memcpy(pEE + j * 3, eePosIni[j], 3 * sizeof(double));
				}


			}

			if (i == 0)
			{
				//dsp(pEE, 6, 3);
			}

			if (i == PERIOD - 1)
			{
				//dsp(pEE, 6, 3);
			}


			robot.SetPee(pEE, bodyEp, "G");
			robot.GetPin(data[i]);
		}

		dlmwrite("fast_back_acc.txt", *data, PERIOD, 18);

		return 0;
	}

	int Plan_Output_Dec()
	{
		double pEE[18], bodyEp[6];
		for (int i = 0; i < PERIOD; ++i)
		{
			memset(bodyEp, 0, sizeof(bodyEp));
			bodyEp[5] = dec_even(PERIOD, i + 1)*stepD / 4;

			for (int j = 0; j < 6; ++j)
			{
				double b[3];

				if (j % 2 == 0)
				{
					leg_index = j;


					if (j == 0)
					{
						int nb = 400;
						int nn = 1450;
						int ne = PERIOD;

						int n1 = nb;
						int n2 = nn - nb;
						int n3 = PERIOD - nn;

						double d2 = PI / (1 / 1.5*n1 / n2 + 1 + 1 / 1.5*n3 / n2);
						double d1 = 1 / 1.5*n1 / n2 * d2;
						double d3 = 1 / 1.5*n3 / n2 * d2;


						if (i < nb)
						{
							b_dec(acc_down(n1, i + 1)*d1, b);
						}
						else if (i < nn)
						{
							b_dec(even(n2, i + 1 - nb)*d2 + d1, b);
						}
						else
						{
							b_dec(dec_down(n3, i + 1 - nn)*d3 + d1 + d2, b);
						}
					}
					else
					{
						if (i < PERIOD / 2)
						{
							b_dec(acc_down(PERIOD / 2, i + 1)*PI / 2, b);
						}
						else
						{
							b_dec(dec_down(PERIOD / 2, i + 1 - PERIOD / 2)*PI / 2 + PI / 2, b);
						}
					}

					memcpy(pEE + j * 3, b, 3 * sizeof(double));
				}
				else
				{
					memcpy(pEE + j * 3, eePosAfterMoveConst[j], 3 * sizeof(double));
				}
			}

			robot.SetPee(pEE, bodyEp, "G");
			robot.GetPin(data[i]);

			if (i == 0)
			{
				dsp(pEE, 6, 3);
			}

		}

		dlmwrite("fast_back_dec.txt", *data, PERIOD, 18);

		return 0;
	}

	void Plan_Prepare(int i)
	{
		leg_index = i;


		/*double zBT_loc[4] = { 0, zT1[i], zT1[i] + zT2[i], zT1[i] + zT2[i] + zT3[i] };
		memcpy(zBT[i], zBT_loc, sizeof(zBT_loc));

		double yBT_loc[4] = { 0, yT1[i], yT1[i] + yT2[i], yT1[i] + yT2[i] + yT3[i] };
		memcpy(yBT[i], yBT_loc, sizeof(yBT_loc));*/


		double VelMax[3] = { 0.23, 0.23, 0.23 };
		double VelMin[3] = { -0.23, -0.23, -0.23 };
		double AccMax[3] = { 0.8, 0.8, 0.8 };
		double AccMin[3] = { -0.8, -0.8, -0.8 };

		memcpy(generator.inVelMax, VelMax, sizeof(VelMax));
		memcpy(generator.inVelMin, VelMin, sizeof(VelMin));
		memcpy(generator.inAccMax, AccMax, sizeof(AccMax));
		memcpy(generator.inAccMin, AccMin, sizeof(AccMin));

		generator.dt = 0.001;

		generator.beginS = 0;
		generator.beginDotS = v / PI / zR1[i] * zT1[i];
		generator.endS = zBT[i][3];
		generator.endDotS = v / PI / zR3[i] * zT3[i];

		generator.maxDotS = 100;

		generator.GetEveryThing = GetEveryThing;

		generator.totalNum = PERIOD;






	}

	int Plan_Output_Const()
	{

		for (int i = 0; i < 6; ++i)
		{
			Plan_Prepare(i);
			generator.Run();
			memcpy(final_s[i], generator.final_s, sizeof(double)*PERIOD);

			if (generator.realTotalNum>PERIOD)
				cout << "leg " << i << " actually failed with real number: " << generator.realTotalNum;
		}

		double b[3];
		double pEE[18], bodyEp[6];
		for (int i = 0; i < PERIOD; ++i)
		{
			memset(bodyEp, 0, sizeof(bodyEp));

			for (int j = 0; j < 6; ++j)
			{
				if (j % 2 == 0)
				{
					leg_index = j;
					b_const(final_s[j][i], b);
					memcpy(pEE + j * 3, b, 3 * sizeof(double));
				}
				else
				{
					memcpy(pEE + j * 3, eePosAfterMoveConst[j], 3 * sizeof(double));
					pEE[j * 3 + 2] -= v*0.001*(i + 1);
				}
			}
			robot.SetPee(pEE, bodyEp, "B");
			robot.GetPin(data[i]);
		}

		for (int i = 0; i < PERIOD; ++i)
		{
			memset(bodyEp, 0, sizeof(bodyEp));

			for (int j = 0; j < 6; ++j)
			{
				if (j % 2 != 0)
				{
					leg_index = j;
					b_const(final_s[j][i], b);
					memcpy(pEE + j * 3, b, 3 * sizeof(double));
				}
				else
				{
					memcpy(pEE + j * 3, eePosAfterMoveConst[j], 3 * sizeof(double));
					pEE[j * 3 + 2] -= v*0.001*(i + 1);
				}
			}

			robot.SetPee(pEE, bodyEp, "B");
			robot.GetPin(data[i + PERIOD]);
		}

		dlmwrite("fast_back_const.txt", *data, 2 * PERIOD, 18);

		dlmwrite("s.txt", generator.s, generator.realTotalNum, 1);
		dlmwrite("ds.txt", generator.ds, generator.realTotalNum, 1);



		return 0;
	}

	int Plan_Output_Const_Pos_G()
	{

		for (int i = 2; i < 3; ++i)
		{
			Plan_Prepare(i);
			generator.Run();
			memcpy(final_s[i], generator.final_s, sizeof(double)*PERIOD);

			if (generator.realTotalNum>PERIOD)
				cout << "leg " << i << " actually failed with real number: " << generator.realTotalNum;
		}

		double b[3];
		double pEE[18], bodyEp[6];
		for (int i = 0; i < PERIOD; ++i)
		{
			memset(bodyEp, 0, sizeof(bodyEp));
			bodyEp[5] = v*0.001*(i + 1);

			for (int j = 0; j < 6; ++j)
			{
				if (j % 2 == 0)
				{
					leg_index = j;
					b_const(final_s[j][i], b);
					memcpy(pEE + j * 3, b, 3 * sizeof(double));
				}
				else
				{
					memcpy(pEE + j * 3, eePosAfterMoveConst[j], 3 * sizeof(double));
					pEE[j * 3 + 2] -= v*0.001*(i + 1);
				}
			}
			robot.SetPee(pEE, bodyEp, "B");
			robot.GetPee(data[i], "G");
		}

		for (int i = 0; i < PERIOD; ++i)
		{
			memset(bodyEp, 0, sizeof(bodyEp));
			bodyEp[5] = v*0.001*(i + 1) + v*PERIOD*0.001;

			for (int j = 0; j < 6; ++j)
			{
				if (j % 2 != 0)
				{
					leg_index = j;
					b_const(final_s[j][i], b);
					memcpy(pEE + j * 3, b, 3 * sizeof(double));
				}
				else
				{
					memcpy(pEE + j * 3, eePosAfterMoveConst[j], 3 * sizeof(double));
					pEE[j * 3 + 2] -= v*0.001*(i + 1);
				}
			}

			robot.SetPee(pEE, bodyEp, "B");
			robot.GetPee(data[i + PERIOD], "G");
		}

		dlmwrite("fast_back_const.txt", *data, 2 * PERIOD, 18);

		dlmwrite("s.txt", generator.s, generator.realTotalNum, 1);
		dlmwrite("ds.txt", generator.ds, generator.realTotalNum, 1);

		dlmwrite("max_ds.txt", generator.maxDs, generator.realTotalNum, 1);

		/*double testData[10000][3];

		for (int i = 1; i <= 10000; ++i)
		{
		::b_const(i*zBT[3] / 10000, testData[i - 1]);
		}
		dlmwrite("data.txt", testData[0], 10000, 3);

		for (int i = 1; i <= 10000; ++i)
		{
		::g_const(i*zBT[3] / 10000, testData[i - 1]);
		}
		dlmwrite("data1.txt", testData[0], 10000, 3);

		for (int i = 1; i <= 10000; ++i)
		{
		::h_const(i*zBT[3] / 10000, testData[i - 1]);
		}
		dlmwrite("data2.txt", testData[0], 10000, 3);*/


		return 0;
	}
}

