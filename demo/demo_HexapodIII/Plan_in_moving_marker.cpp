#include "Plan_in_moving_marker.h"

#include <iostream>

using namespace std;

using namespace Hexapod_Robot;
using namespace Aris::DynKer;
using namespace Aris::Plan;

#define PERIOD 1200

namespace PlanInOtherMarker
{
	double stepH = 0.04, stepD = 0.4;
	double v = stepD/PERIOD/2*1000;

	double eePosIni[6][3] =
	{ { -0.3, -0.85, -0.65 }
	, { -0.45, -0.85, 0 }
	, { -0.3, -0.85, 0.65 }
	, { 0.3, -0.85, -0.65 }
	, { 0.45, -0.85, 0 }
	, { 0.3, -0.85, 0.65 } };

	int leg_index;

	double final_s[6][PERIOD];

	double data[PERIOD * 4][18];

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
		b_out[0] = eePosIni[leg_index][0];
		b_out[1] = stepH * sin(s_in) + eePosIni[leg_index][1];
		b_out[2] = -stepD / 4 * cos(s_in) + eePosIni[leg_index][2];
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

		bodyVel[2] = v;
		bodyEp[5] = v*t_in;
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

		robot.pLegs[leg_index]->SetPee(pEE, "G");
		robot.pLegs[leg_index]->SetVee(vEE, "G");

		robot.pLegs[leg_index]->GetVelJacInv(0, cV_out, "G");
		robot.pLegs[leg_index]->GetAccJacInv(jacInv_out, cA_out, "G");

		return 0;
	};

	void Plan_Prepare(int i)
	{
		leg_index = i;

		double VelMax[3] = { 0.23, 0.23, 0.23 };
		double VelMin[3] = { -0.23, -0.23, -0.23 };
		double AccMax[3] = { 1.2, 1.2, 1.2 };
		double AccMin[3] = { -1.2, -1.2, -1.2 };

		memcpy(generator.inVelMax, VelMax, sizeof(VelMax));
		memcpy(generator.inVelMin, VelMin, sizeof(VelMin));
		memcpy(generator.inAccMax, AccMax, sizeof(AccMax));
		memcpy(generator.inAccMin, AccMin, sizeof(AccMin));

		generator.dt = 0.001;

		generator.beginS = 0;
		generator.beginDotS = 0;
		generator.endS = PI;
		generator.endDotS = 0;

		generator.maxDotS = 10;

		generator.GetEveryThing = GetEveryThing;

		generator.totalNum = PERIOD;
	}

	int Plan_Output_Const()
	{
		for (int i = 0; i < 3; ++i)
		{
			Plan_Prepare(i);
			generator.RunInMovingMarker();
			memcpy(final_s[i], generator.final_s, sizeof(double)*PERIOD);
			cout << "Real number needed of leg " << i << " is: " << generator.realTotalNum << endl;
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
					double loc[3] = { eePosIni[j][0], eePosIni[j][1], eePosIni[j][2] + stepD / 4 };
					memcpy(pEE + j * 3, loc, 3 * sizeof(double));
				}
			}
			robot.SetPee(pEE, bodyEp, "G");
			robot.GetPin(data[i]);
		}

		for (int i = 0; i < PERIOD; ++i)
		{
			memset(bodyEp, 0, sizeof(bodyEp));
			bodyEp[5] = v*0.001*(i + 1);

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
					double loc[3] = { eePosIni[j][0], eePosIni[j][1], eePosIni[j][2] + stepD / 4 };
					memcpy(pEE + j * 3, loc, 3 * sizeof(double));
				}
			}
			robot.SetPee(pEE, bodyEp, "G");
			robot.GetPin(data[i + PERIOD]);
		}

		dlmwrite("fast_back_const.txt", *data, 2 * PERIOD, 18);

		dlmwrite("s.txt", generator.final_s, PERIOD, 1);
		dlmwrite("ds.txt", generator.ds, PERIOD, 1);

		return 0;
	}

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
					double loc[3] = { eePosIni[j][0], eePosIni[j][1], eePosIni[j][2] + stepD / 4 };
					memcpy(pEE + j * 3, loc, 3 * sizeof(double));
				}
			}

			robot.SetPee(pEE, bodyEp, "G");
			robot.GetPin(data[i]);
		}

		

		dlmwrite("fast_back_dec.txt", *data, PERIOD, 18);

		return 0;
	}

	int Plan_Output_Ds()
	{
		Plan_Prepare(1);

		generator.FindInterval();

		dlmwrite("max_ds.txt", generator.maxDs, PERIOD, 1);

		return 0;
	}
}


