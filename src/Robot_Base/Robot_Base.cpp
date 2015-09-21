#include <Platform.h>

#ifdef PLATFORM_IS_WINDOWS
#define _CRT_SECURE_NO_WARNINGS
#define _SCL_SECURE_NO_WARNINGS
#endif


#include <complex>
#include <cmath>
#include <ctime>
#include <iostream>

#include <Aris_DynKer.h>

#include "Robot_Base.h"

using namespace Aris::DynKer;
using namespace std;

namespace Robots
{
	LEG_BASE::LEG_BASE(ROBOT_BASE* pRobot)
		: pRobot(pRobot)
	{
	}

	void LEG_BASE::GetPee(double *pEE, const char *RelativeCoodinate) const
	{
		switch (*RelativeCoodinate)
		{
		case 'L':
			std::copy_n(this->pEE, 3, pEE);
			break;
		case 'B':
		case 'M':
			s_pm_dot_pnt(pBasePrtPm, this->pEE, pEE);
			break;
		case 'G':
		case 'O':
		default:
			s_pm_dot_pnt(pBasePm, this->pEE, pEE);
			break;
		}
	}
	void LEG_BASE::SetPee(const double *pEE, const char *RelativeCoodinate)
	{
		s_pm_dot_pm(pRobot->pBodyPm, pBasePrtPm, pBasePm);

		switch (*RelativeCoodinate)
		{
		case 'L':
			std::copy_n(pEE, 3, this->pEE);
			break;
		case 'B':
		case 'M':
			s_inv_pm_dot_pnt(pBasePrtPm, pEE, this->pEE);
			break;
		case 'G':
		case 'O':
		default:
			s_inv_pm_dot_pnt(pBasePm, pEE, this->pEE);
			break;
		}

		calculate_from_pEE();
		calculate_jac();
	}
	void LEG_BASE::GetVee(double *vEE, const char *RelativeCoodinate) const
	{
		switch (*RelativeCoodinate)
		{
		case 'L':
			std::copy_n(this->vEE, 3, vEE);
			break;
		case 'B':
		case 'M':
			s_pm_dot_v3(pBasePrtPm, this->vEE, vEE);
			//s_dgemm(3, 1, 3, 1, pBasePrtPm, 4, this->vEE, 1, 0, vEE, 1);
			break;
		case 'G':
		case 'O':
		default:
			s_pv2pv(pBasePm, pRobot->pBodyVel, this->pEE, this->vEE, vEE);
			break;
		}
	}
	void LEG_BASE::SetVee(const double *vEE, const char *RelativeCoodinate)
	{
		double pnt[3];

		switch (*RelativeCoodinate)
		{
		case 'L':
			std::copy_n(vEE, 3, this->vEE);
			break;
		case 'B':
		case 'M':
			s_inv_pm_dot_v3(pBasePrtPm, vEE, this->vEE);
			break;
		case 'G':
		case 'O':
		default:
			GetPee(pnt, "G");
			s_inv_pv2pv(pBasePm, pRobot->pBodyVel, pnt, vEE, this->vEE);
			break;
		}

		calculate_from_vEE();
		calculate_diff_jac();
	}
	void LEG_BASE::GetAee(double *aEE, const char *RelativeCoodinate) const
	{
		switch (*RelativeCoodinate)
		{
		case 'L':
			std::copy_n(this->aEE, 3, aEE);
			break;
		case 'B':
		case 'M':
			s_pa2pa(pBasePrtPm, 0, 0, this->pEE, this->vEE, this->aEE, aEE);
			break;
		case 'G':
		case 'O':
		default:
			s_pa2pa(pBasePm, pRobot->pBodyVel, pRobot->pBodyAcc, this->pEE, this->vEE, this->aEE, aEE);
			break;
		}
	}
	void LEG_BASE::SetAee(const double *aEE, const char *RelativeCoodinate)
	{
		double pv[3], pnt[3];

		switch (*RelativeCoodinate)
		{
		case 'L':
			std::copy_n(aEE, 3, this->aEE);
			break;
		case 'B':
		case 'M':
			s_inv_pm_dot_v3(pBasePrtPm, aEE, this->aEE);
			break;
		case 'G':
		case 'O':
		default:
			GetPee(pnt, "G");
			GetVee(pv, "G");
			s_inv_pa2pa(pBasePm, pRobot->pBodyVel, pRobot->pBodyAcc, pnt, pv, aEE, this->aEE);
			break;
		}

		calculate_from_aEE();
	}
	void LEG_BASE::GetFeeSta(double *fEE_sta, const char *RelativeCoodinate) const
	{
		switch (*RelativeCoodinate)
		{
		case 'L':
			std::copy_n(this->fEE_sta, 3, fEE_sta);
			break;
		case 'B':
		case 'M':
			s_pm_dot_v3(pBasePrtPm, this->fEE_sta, fEE_sta);
			break;
		case 'G':
		case 'O':
		default:
			s_pm_dot_v3(pBasePm, this->fEE_sta, fEE_sta);
			break;
		}
	}
	void LEG_BASE::SetFeeSta(const double *fEE_sta, const char *RelativeCoodinate)
	{
		switch (*RelativeCoodinate)
		{
		case 'L':
			std::copy_n(fEE_sta, 3, this->fEE_sta);
			break;
		case 'B':
		case 'M':
			s_inv_pm_dot_v3(pBasePrtPm, fEE_sta, this->fEE_sta);
			break;
		case 'G':
		case 'O':
		default:
			s_inv_pm_dot_v3(pBasePm, fEE_sta, this->fEE_sta);
			break;
		}

		s_dgemmTN(3, 1, 3, 1, *Jvd, 3, this->fEE_sta, 1, 0, this->fIn_sta, 1);
	}
	void LEG_BASE::GetPin(double *pIn) const
	{
		std::copy_n(this->pIn, 3, pIn);
	}
	void LEG_BASE::SetPin(const double *pIn)
	{
		s_pm_dot_pm(pRobot->pBodyPm, pBasePrtPm, pBasePm);

		std::copy_n(pIn, 3, this->pIn);
		calculate_from_pIn();
		calculate_jac();
	}
	void LEG_BASE::GetVin(double *vIn) const
	{
		std::copy_n(this->vIn, 3, vIn);
	}
	void LEG_BASE::SetVin(const double *vIn)
	{
		std::copy_n(vIn, 3, this->vIn);

		calculate_from_vIn();
		calculate_diff_jac();
	}
	void LEG_BASE::GetAin(double *aIn) const
	{
		std::copy_n(this->aIn, 3, aIn);
	}
	void LEG_BASE::SetAin(const double *aIn)
	{
		std::copy_n(aIn, 3, this->aIn);
		calculate_from_aIn();
	}
	void LEG_BASE::GetFinSta(double *fIn_sta) const
	{
		std::copy_n(this->fIn_sta, 3, fIn_sta);
	}
	void LEG_BASE::SetFinSta(const double *fIn_sta)
	{
		std::copy_n(fIn_sta, 3, this->fIn_sta);
		s_dgemmTN(3, 1, 3, 1, *Jvi, 3, fIn_sta, 1, 0, fEE_sta, 1);
	}

	void LEG_BASE::GetJfd(double *jac, const char *RelativeCoodinate) const
	{
		/*力雅克比是速度雅克比转置的逆*/
		double locJac[3][3];

		GetJvi(*locJac, RelativeCoodinate);
		s_transpose(3, 3, *locJac, 3, jac, 3);
	}
	void LEG_BASE::GetJfi(double *jac, const char *RelativeCoodinate) const
	{
		/*力雅克比是速度雅克比转置的逆*/
		double locJac[3][3];

		GetJvd(*locJac, RelativeCoodinate);
		s_transpose(3, 3, *locJac, 3, jac, 3);
	}
	void LEG_BASE::GetJvd(double *jac, const char *RelativeCoodinate) const
	{
		switch (*RelativeCoodinate)
		{
		case 'L':
			std::copy_n(&Jvd[0][0], 9, jac);
			break;
		case 'M':
		case 'B':
			s_dgemm(3, 3, 3, 1, pBasePrtPm, 4, *Jvd, 3, 0, jac, 3);
			break;
		case 'G':
		case 'O':
		default:
			s_dgemm(3, 3, 3, 1, pBasePm, 4, *Jvd, 3, 0, jac, 3);
			break;
		}

	}
	void LEG_BASE::GetJvi(double *jac, const char *RelativeCoodinate) const
	{
		switch (*RelativeCoodinate)
		{
		case 'L':
			std::copy_n(&Jvi[0][0], 9, jac);
			break;
		case 'M':
		case 'B':
			s_dgemmNT(3, 3, 3, 1, *Jvi, 3, pBasePrtPm, 4, 0, jac, 3);
			break;
		case 'G':
		case 'O':
		default:
			s_dgemmNT(3, 3, 3, 1, *Jvi, 3, pBasePm, 4, 0, jac, 3);
			break;
		}
	}
	void LEG_BASE::GetDifJfd(double *dJac, const char *RelativeCoodinate) const
	{
		/*力雅克比是速度雅克比转置的逆*/
		double locJac[3][3];

		GetDifJvi(*locJac, RelativeCoodinate);
		s_transpose(3, 3, *locJac, 3, dJac, 3);
	}
	void LEG_BASE::GetDifJfi(double *dJac, const char *RelativeCoodinate) const
	{
		/*力雅克比是速度雅克比转置的逆*/
		double locJac[3][3];

		GetJvd(*locJac, RelativeCoodinate);
		s_transpose(3, 3, *locJac, 3, dJac, 3);
	}
	void LEG_BASE::GetDifJvd(double *dJac, const char *RelativeCoodinate) const
	{
		switch (*RelativeCoodinate)
		{
		case 'L':
			std::copy_n(&vJvd[0][0], 9, dJac);
			break;
		case 'M':
		case 'B':
			s_dgemm(3, 3, 3, 1, pBasePrtPm, 4, *vJvd, 3, 0, dJac, 3);
			break;
		case 'G':
		case 'O':
		default:
			{
				double dR[4][4];
				s_v_cro_pm(pRobot->pBodyVel, pBasePm, *dR);
				
				s_dgemm(3, 3, 3, 1, *dR, 4, *Jvd, 3, 0, dJac, 3);
				s_dgemm(3, 3, 3, 1, pBasePm, 4, *vJvd, 3, 1, dJac, 3);
				break;
			}
		}
	}
	void LEG_BASE::GetDifJvi(double *dJac, const char *RelativeCoodinate) const
	{
		switch (*RelativeCoodinate)
		{
		case 'L':
			std::copy_n(&vJvi[0][0], 9, dJac);
			break;
		case 'M':
		case 'B':
			s_dgemmNT(3, 3, 3, 1, *vJvi, 3, pBasePrtPm, 4, 0, dJac, 3);
			break;
		case 'G':
		case 'O':
		default:
			{
				double dR[4][4];
				s_v_cro_pm(pRobot->pBodyVel, pBasePm, *dR);

				s_dgemmNT(3, 3, 3, 1, *vJvi, 3, pBasePm, 4, 0, dJac, 3);
				s_dgemmNT(3, 3, 3, 1, *Jvi, 3, *dR, 4, 1, dJac, 3);
				
				break;
			}
		}
	}
	void LEG_BASE::GetCvd(double *c, const char *RelativeCoodinate) const
	{
		switch (*RelativeCoodinate)
		{
		case 'L':
			std::fill_n(c, 3, 0);
			break;
		case 'M':
		case 'B':
			std::fill_n(c, 3, 0);
			break;
		case 'G':
		case 'O':
		default:
			s_pv2pv(pBasePm, pRobot->pBodyVel, this->pEE, 0, c);
		}

	}
	void LEG_BASE::GetCvi(double *c, const char *RelativeCoodinate) const
	{
		switch (*RelativeCoodinate)
		{
		case 'L':
			std::fill_n(c, 3, 0);
			break;
		case 'M':
		case 'B':
			std::fill_n(c, 3, 0);
			break;
		case 'G':
		case 'O':
		default:
		{
			double tem[3];

			s_pv2pv(pBasePm, pRobot->pBodyVel, this->pEE, nullptr, c);
			s_dgemmTN(3, 1, 3, 1, pBasePm, 4, c, 1, 0, tem, 1);
			s_dgemm(3, 1, 3, -1, *Jvi, 3, tem, 1, 0, c, 1);
			break;
		}
			
		}
	}
	void LEG_BASE::GetCad(double *c, const char *RelativeCoodinate) const
	{
		switch (*RelativeCoodinate)
		{
		case 'L':
			s_dgemm(3, 1, 3, 1, *vJvd, 3, vIn, 1, 0, c, 1);
			break;
		case 'M':
		case 'B':
		{
			double dJac[3][3];
			this->GetDifJvd(*dJac, "M");
			s_dgemm(3, 1, 3, 1, *dJac, 3, vIn, 1, 0, c, 1);
			break;
		}
		case 'G':
		case 'O':
		default:
		{
			/*推导如下*/
			//Vee_G = R_L2G * Vee_L + Vb + Wb x Pee_G = R_L2G * Jvd_L * Vin + Vb + Wb x Pee_G
			//      = Jvd_G * Vin + Vb + Wb x Pee_G
			//Aee_G = Jvd_G * Ain + dJvd_G * Vin + Ab + Xb x Pee_G + Vb x Vee_G
			//      = Jvi_G * Ain + dJvd_G * Vin + pa
			//c = dJvd_G * Vin + pa

			//Ain = Jvi_G * Aee_G + dJvi_G * Vee_G - Jvi_G * (Vb + Wb x Pee_G) - dJvi_G * (Ab + Xb x Pee_G + Wb x Vee_G)

			double pEE_G[3], vEE_G[3];
			this->GetPee(pEE_G, "G");
			this->GetVee(vEE_G, "G");

			s_pv(pEE_G, this->pRobot->pBodyAcc, c);
			s_cro3(1, this->pRobot->pBodyVel + 3, vEE_G, 1, c);

			double dJac[3][3];
			this->GetDifJvd(*dJac, "G");

			s_dgemm(3, 1, 3, 1, *dJac, 3, this->vIn, 1, 1, c, 1);
			break;
		}
		}
	}
	void LEG_BASE::GetCai(double *c, const char *RelativeCoodinate) const
	{
		switch (*RelativeCoodinate)
		{
		case 'L':
			s_dgemm(3, 1, 3, 1, *vJvi, 3, vEE, 1, 0, c, 1);
			break;
		case 'M':
		case 'B':
		{
			double dJac[3][3], vEE_M[3];
			this->GetDifJvi(*dJac, "M");
			this->GetVee(vEE_M, "M");

			s_dgemm(3, 1, 3, 1, *dJac, 3, vEE_M, 1, 0, c, 1);
			break;
		}
		case 'G':
		case 'O':
		default:
		{
			/*推导如下*/
			//Vee_G = R_L2G * Vee_L + Vb + Wb x Pee_G
			//Vee_L = R_G2L * (Vee_G - Vb - Wb x Pee_G)
			//Vin = Jvi_L * Vee_L = Jvi_L * R_G2L * (Vee_G - Vb - Wb x Pee_G)
			//    = Jvi_G * (Vee_G - Vb - Wb x Pee_G)

			//Ain = Jvi_G * Aee_G + dJvi_G * Vee_G - Jvi_G * (Vb + Wb x Pee_G) - dJvi_G * (Ab + Xb x Pee_G + Wb x Vee_G)

			double pEE_G[3], vEE_G[3];
			this->GetPee(pEE_G, "G");
			this->GetVee(vEE_G, "G");
			double pv[3], pa[3];


			/*！！！特别注意：！！！*/
			/*这里pv=Vb + Wb x Pee*/
			/*这里pa=Ab + Xb x Pee + Xb x Vee*/
			/*其中Vb Wb Ab Xb 分别为机器人身体的线速度，角速度，线加速度，角加速度，Pee和Vee相对于地面坐标系*/
			/*这里一定不能用s_pa2pa函数*/
			s_pv(pEE_G, this->pRobot->pBodyVel, pv);
			s_pv(pEE_G, this->pRobot->pBodyAcc, pa);
			s_cro3(1, this->pRobot->pBodyVel + 3, vEE_G, 1, pa);

			/*之后有：c = -J * pa - dJ * pv */
			double Jac[3][3], dJac[3][3];
			this->GetDifJvi(*dJac, "G");
			this->GetJvi(*Jac, "G");

			s_dgemm(3, 1, 3, 1, *dJac, 3, vEE_G, 1, 0, c, 1);
			s_dgemm(3, 1, 3, -1, *Jac, 3, pa, 1, 1, c, 1);
			s_dgemm(3, 1, 3, -1, *dJac, 3, pv, 1, 1, c, 1);
			break;
		}
			
		}
	}

	void LEG_BASE::TransformCoordinatePee(const double *bodyPe, const char *fromMak, const double *fromPee
		, const char *toMak, double *toPee) const
	{
		switch (*fromMak)
		{
		case 'L':
		{
			switch (*toMak)
			{
			case 'L':
			{
				std::copy_n(fromPee, 3, toPee);
				return;
			}
			case 'B':
			case 'M':
			{
				s_pm_dot_pnt(pBasePrtPm, fromPee, toPee);
				return;
			}
			case 'G':
			case 'O':
			default:
			{
				double bodyPm[16], pnt[3];
				s_pe2pm(bodyPe, bodyPm);
				s_pm_dot_pnt(pBasePrtPm, fromPee, pnt);
				s_pm_dot_pnt(bodyPm, pnt, toPee);
				return;
			}
			}
		}
		case 'B':
		case 'M':
		{
			switch (*toMak)
			{
			case 'L':
			{
				s_inv_pm_dot_pnt(pBasePrtPm, fromPee, toPee);
				return;
			}
			case 'B':
			case 'M':
			{
				std::copy_n(fromPee, 3, toPee);
				return;
			}
			case 'G':
			case 'O':
			default:
			{
				double bodyPm[16];
				s_pe2pm(bodyPe, bodyPm);
				s_pm_dot_pnt(bodyPm, fromPee, toPee);
				return;
			}
			}
		}
		case 'G':
		case 'O':
		default:
		{
			switch (*toMak)
			{
			case 'L':
			{
				double bodyPm[16];
				double pnt[3];
				s_pe2pm(bodyPe, bodyPm);
				s_inv_pm_dot_pnt(bodyPm, fromPee, pnt);
				s_inv_pm_dot_pnt(pBasePrtPm, pnt, toPee);
				return;
			}
			case 'B':
			case 'M':
			{
				double bodyPm[16];
				s_pe2pm(bodyPe, bodyPm);
				s_inv_pm_dot_pnt(bodyPm, fromPee, toPee);
				return;
			}
			case 'G':
			case 'O':
			default:
			{
				std::copy_n(fromPee, 3, toPee);
				return;
			}
			}
		}
		}
	}

	void ROBOT_BASE::GetBodyPm(double *bodypm) const
	{ 
		std::copy_n(pBodyPm, 16, bodypm);
	};
	void ROBOT_BASE::GetBodyPe(double *bodype, const char *eurType) const
	{ 
		s_pm2pe(pBodyPm, bodype, eurType); 
	};
	void ROBOT_BASE::GetBodyVel(double *bodyvel) const
	{ 
		std::copy_n(this->pBodyVel, 6, bodyvel);
	};
	void ROBOT_BASE::GetBodyAcc(double *bodyacc) const
	{ 
		std::copy_n(this->pBodyAcc, 6, bodyacc);
	};
	void ROBOT_BASE::GetPee(double *pEE, const char *RelativeCoodinate) const
	{
		for (int i = 0; i < 6; ++i)
		{
			pLegs[i]->GetPee(pEE + i * 3, RelativeCoodinate);
		}
	}
	void ROBOT_BASE::SetPee(const double *pEE, const double *bodyep, const char *RelativeCoodinate, const char *eurType)
	{
		if (bodyep)
		{
			s_pe2pm(bodyep, pBodyPm, eurType);
		}

		if (pEE)
		{
			for (int i = 0; i < 6; i++)
			{
				pLegs[i]->SetPee(pEE + i * 3, RelativeCoodinate);
			}
		}

		calculate_jac();
	}
	void ROBOT_BASE::GetVee(double *vEE, const char *RelativeCoodinate) const
	{
		for (int i = 0; i < 6; ++i)
		{
			pLegs[i]->GetVee(vEE + i * 3, RelativeCoodinate);
		}
	}
	void ROBOT_BASE::SetVee(const double *vEE, const double *bodyvel, const char *RelativeCoodinate)
	{
		if (bodyvel)
		{
			std::copy_n(bodyvel, 6, pBodyVel);
		}

		if (vEE)
		{
			for (int i = 0; i < 6; i++)
			{
				pLegs[i]->SetVee(vEE + i * 3, RelativeCoodinate);
			}
		}

		calculate_jac_c();
	}
	void ROBOT_BASE::GetAee(double *aEE, const char *RelativeCoodinate) const
	{
		for (int i = 0; i < 6; ++i)
		{
			pLegs[i]->GetAee(aEE + i * 3, RelativeCoodinate);
		}
	}
	void ROBOT_BASE::SetAee(const double *aEE, const double *bodyacc, const char *RelativeCoodinate)
	{
		if (bodyacc)
		{
			std::copy_n(bodyacc, 6, pBodyAcc);
		}
		if (aEE)
		{
			for (int i = 0; i < 6; i++)
			{
				pLegs[i]->SetAee(aEE + i * 3, RelativeCoodinate);
			}
		}
	}
	void ROBOT_BASE::GetFeeSta(double *fee_sta, const char *RelativeCoodinate) const
	{
		for (int i = 0; i < 6; i++)
		{
			pLegs[i]->GetFeeSta(fee_sta + i * 3, RelativeCoodinate);
		}
	}
	void ROBOT_BASE::SetFeeSta(const double *fee_sta, const char *RelativeCoodinate)
	{
		for (int i = 0; i < 6; i++)
		{
			pLegs[i]->SetFeeSta(fee_sta + i * 3, RelativeCoodinate);
		}
	}
	void ROBOT_BASE::GetPin(double *pIn) const
	{
		for (int i = 0; i < 6; ++i)
		{
			pLegs[i]->GetPin(pIn + i * 3);
		}
	}
	void ROBOT_BASE::SetPin(const double *pIn, const double *bodyep, const char *eurType)
	{
		if (bodyep)
		{
			s_pe2pm(bodyep, pBodyPm, eurType);
		}

		if (pIn)
		{
			for (int i = 0; i < 6; i++)
			{
				pLegs[i]->SetPin(pIn + i * 3);
			}
		}

		calculate_jac();
	}
	void ROBOT_BASE::GetVin(double *vIn) const
	{
		for (int i = 0; i < 6; ++i)
		{
			pLegs[i]->GetVin(vIn + i * 3);
		}
	}
	void ROBOT_BASE::SetVin(const double *vIn, const double *bodyvel)
	{
		if (bodyvel)
		{
			std::copy_n(bodyvel, 6, pBodyVel);
		}
		if (vIn)
		{
			for (int i = 0; i < 6; i++)
			{
				pLegs[i]->SetVin(vIn + i * 3);
			}
		}

		calculate_jac_c();
	}
	void ROBOT_BASE::GetAin(double *aIn) const
	{
		for (int i = 0; i < 6; ++i)
		{
			pLegs[i]->GetAin(aIn + i*3);
		}
	}
	void ROBOT_BASE::SetAin(const double *aIn, const double *bodyacc)
	{
		if (bodyacc)
		{
			std::copy_n(bodyacc, 6, pBodyAcc);
		}
		if (aIn)//= if(aIn!=nullptr)
		{
			for (int i = 0; i < 6; i++)
			{
				pLegs[i]->SetAin(aIn + i * 3);
			}
		}
	}
	void ROBOT_BASE::GetFinSta(double *fIn_sta) const
	{
		for (int i = 0; i < 6; ++i)
		{
			pLegs[i]->GetFinSta(fIn_sta + i * 3);
		}
	}
	void ROBOT_BASE::SetFinSta(const double *fIn_sta)
	{
		for (int i = 0; i < 6; ++i)
		{
			pLegs[i]->SetFinSta(fIn_sta + i * 3);
		}
	}

	void findSupportMotion(const char *fixFeet, const char *activeMotor, char* supportMotor, int* supportID, int&dim)
	{
		dim = 0;
		for (int i = 0; i < 6; ++i)
		{
			/*判断是否为支撑腿*/
			if ((i < static_cast<int>(strlen(fixFeet))) && (fixFeet[i] != '0'))
			{
				for (int j = i * 3; j < (i + 1) * 3; ++j)
				{
					supportMotor[j] = activeMotor[j];
					if (supportMotor[j] != '0')
					{
						supportID[dim] = j;
						dim++;
					}
				}
			}
		}
	}
	
	void ROBOT_BASE::SetPinFixFeet(const double *pIn_end, const char *fixFeet, const char *activeMotor, const double *initBodyPE)
	{
		/*找出支撑身体的所有电机及其维数等*/
		char supportMotor[19]{ "000000000000000000" };
		int supportID[18];
		int dim = 0;
		findSupportMotion(fixFeet, activeMotor, supportMotor, supportID, dim);

		/*给出身体的位姿初值*/
		double pm[16], pq[7], pe[6];
		double jac[18 * 6];
		double vb[6], vq[7];
		double pIn[18],vIn[18];
		double pEE_G[18];

		this->GetPee(pEE_G, "G");
		Aris::DynKer::s_pe2pq(initBodyPE, pq, "313");
		s_pq2pe(pq, pe, "313");
		this->SetPee(pEE_G, pe, "G");
		GetPin(pIn);

		for (int j = 0; j < dim; ++j)
		{
			vIn[j] = pIn_end[supportID[j]] - pIn[supportID[j]];
		}

		double error = s_dnrm2(dim,vIn,1);

		/*迭代计算身体的位姿*/
		while(error>1e-11)
		{
			GetJvi(jac, supportMotor);
			if (dim == 6)
			{
				int ipiv[6];
				s_dgesv(6, 1, jac, 6, ipiv, vIn, 1);
			}
			else
			{
				int rank;
				double s[6];
				s_dgelsd(dim, 6, 1, jac, 6, vIn, 1, s, 0.000000001, &rank);
			}

			std::copy_n(vIn, dim, vb);

			s_pq2pm(pq, pm);
			s_v2vq(pm, vb, vq);

			s_daxpy(7, 1, vq, 1, pq, 1);
			double norm = s_dnrm2(4, &pq[3], 1);
			for (int j = 3; j < 7; ++j)
			{
				pq[j] /= norm;
			}

			s_pq2pe(pq, pe, "313");
			SetPee(pEE_G, pe, "G");
			GetPin(pIn);

			for (int j = 0; j < dim; ++j)
			{
				vIn[j] = pIn_end[supportID[j]] - pIn[supportID[j]];
			}

			error = s_dnrm2(dim, vIn, 1);
		}

		/*根据结果首先设置支撑腿的末端及身体位姿，随后设置其他腿的输入*/
		s_pq2pe(pq, pe, "313");
		this->SetPee(pEE_G, pe, "313");
		for (int i = 0; i < 6; ++i)
		{
			/*判断是否为支撑腿*/
			if ((i >= static_cast<int>(strlen(fixFeet))) || (fixFeet[i] == '0'))
			{
				this->pLegs[i]->SetPin(pIn_end + i * 3);
			}
		}

		calculate_jac();
	}
	void ROBOT_BASE::SetVinFixFeet(const double *vIn, const char *fixFeet, const char *activeMotor)
	{
		/*找出支撑身体的所有电机及其维数等*/
		char supportMotor[19]{ "000000000000000000" };
		int supportID[18];
		int dim;
		findSupportMotion(fixFeet, activeMotor,supportMotor, supportID, dim);

		/*计算所有支撑电机的速度*/
		double vIn_loc[18];
		for (int j = 0; j < dim; ++j)
		{
			vIn_loc[j] = vIn[supportID[j]];
		}

		/*计算身体雅可比，只根据支撑的电机来计算，之后求解雅可比方程*/
		double jac[18*6];
		GetJvi(jac, supportMotor);
		if (dim == 6)
		{
			int ipiv[6];
			s_dgesv(6, 1, jac, 6, ipiv, vIn_loc, 1);
		}
		else
		{
			int rank;
			double s[6];
			s_dgelsd(dim, 6, 1, jac, 6, vIn_loc, 1, s, 0.000000001, &rank);
		}

		/*先设置支撑腿的末端速度为0，之后设置其他腿的输入*/
		double vEE[18]{ 0 };
		this->SetVee(vEE, vIn_loc);
		for (int i = 0; i < 6; ++i)
		{
			/*判断是否为支撑腿*/
			if ((i >= static_cast<int>(strlen(fixFeet))) || (fixFeet[i] == '0'))
			{
				this->pLegs[i]->SetVin(vIn + i * 3);
			}
		}
	}
	void ROBOT_BASE::SetAinFixFeet(const double *aIn, const char *fixFeet, const char *activeMotor)
	{
		/*找出支撑身体的所有电机及其维数等*/
		char supportMotor[19]{ "000000000000000000" };
		int supportID[18];
		int dim;
		findSupportMotion(fixFeet, activeMotor, supportMotor, supportID, dim);

		/*计算所有支撑电机的速度*/
		double aIn_loc[18];
		for (int j = 0; j < dim; ++j)
		{
			aIn_loc[j] = aIn[supportID[j]];
		}

		/*计算身体雅可比，只根据支撑的电机来计算，之后求解雅可比方程*/
		double jac[18 * 6], dJac[18 * 6];
		GetJvi(jac, supportMotor);
		GetDifJvi(dJac, supportMotor);
		s_dgemm(dim, 1, 6, -1, dJac, 6, this->pBodyVel, 1, 1, aIn_loc, 1);


		if (dim == 6)
		{
			int ipiv[6];
			s_dgesv(6, 1, jac, 6, ipiv, aIn_loc, 1);
		}
		else
		{
			int rank;
			double s[6];
			s_dgelsd(dim, 6, 1, jac, 6, aIn_loc, 1, s, 0.000000001, &rank);
		}

		/*先设置支撑腿的末端速度为0，之后设置其他腿的输入*/
		double aEE[18]{ 0 };
		this->SetAee(aEE, aIn_loc);
		for (int i = 0; i < 6; ++i)
		{
			/*判断是否为支撑腿*/
			if ((i >= static_cast<int>(strlen(fixFeet))) || (fixFeet[i] == '0'))
			{
				this->pLegs[i]->SetAin(aIn + i * 3);
			}
		}




	}

	void ROBOT_BASE::GetJvi(double *jac, const char *activeMotion) const
	{
		int min_num = std::min(strlen(activeMotion), std::size_t{ 18 });

		int dim = 0;
		for (int i = 0; i < min_num; i++)
		{
			if (activeMotion[i] != '0')
			{
				std::copy_n(&Jvi[i][0], 6, &jac[dim * 6]);
				++dim;
			}
		}
	}
	void ROBOT_BASE::GetJfd(double *jac, const char *activeMotion) const
	{
		int min_num = std::min(strlen(activeMotion), std::size_t{ 18 });

		int dim = 0;
		for (int i = 0; i < min_num; i++)
		{
			if (activeMotion[i] != '0')
			{
				++dim;
			}
		}
		
		
		double tem[18 * 6];
		GetJvi(tem, activeMotion);

		s_transpose(dim, 6, tem, 6, jac, dim);
	}
	void ROBOT_BASE::GetDifJvi(double *dJac, const char *activeMotion) const
	{
		int min_num = std::min(strlen(activeMotion), std::size_t{ 18 });

		int dim = 0;
		for (int i = 0; i < min_num; i++)
		{
			if (activeMotion[i] != '0')
			{
				std::copy_n(&vJvi[i][0], 6, &dJac[dim * 6]);
				++dim;
			}
		}
	}
	void ROBOT_BASE::GetDifJfd(double *dJac, const char *activeMotion) const
	{
		int min_num = std::min(strlen(activeMotion), std::size_t{ 18 });

		int dim = 0;
		for (int i = 0; i < min_num; i++)
		{
			if (activeMotion[i] != '0')
			{
				++dim;
			}
		}
		
		double tem[18 * 6];
		GetDifJvi(tem, activeMotion);

		s_transpose(dim, 6, tem, 6, dJac, dim);
	}
	
	void ROBOT_BASE::calculate_jac()
	{
		double jac[9], cm3[9], pEE[3];
		
		for (int i = 0; i < 6; ++i)
		{
			pLegs[i]->GetJvi(jac, "G");
			s_block_cpy(3, 3, -1, jac, 0, 0, 3, 0, *Jvi, i * 3, 0, 6);

			pLegs[i]->GetPee(pEE, "G");
			s_cm3(pEE, cm3);
			s_dgemm(3, 3, 3, 1, jac, 3, cm3, 3, 0, &Jvi[i * 3][3], 6);
		}
	}
	void ROBOT_BASE::calculate_jac_c()
	{
		double dJac[9], jac[9], pEE[3], vEE[3], cmP[9], cmV[9];
		
		for (int i = 0; i < 6; ++i)
		{
			pLegs[i]->GetJvi(jac, "G");
			pLegs[i]->GetDifJvi(dJac, "G");

			s_block_cpy(3, 3, -1, dJac, 0, 0, 3, 0, *vJvi, i * 3, 0, 6);

			/*理论上来说，vEE必须为0，但这里不做限制了，如果vEE不为零，不确定会有什么后果*/
			pLegs[i]->GetPee(pEE, "G");
			pLegs[i]->GetVee(vEE, "G");
			s_cm3(pEE, cmP);
			s_cm3(vEE, cmV);
			s_dgemm(3, 3, 3, 1, dJac, 3, cmP, 3, 0, &vJvi[i * 3][3], 6);
			s_dgemm(3, 3, 3, 1, jac, 3, cmV, 3, 1, &vJvi[i * 3][3], 6);
		}
	}
	
	

	void ROBOT_BASE::TransformCoordinatePee(const double *bodyPe, const char *fromMak, const double *fromPee
		, const char *toMak, double *toPee) const
	{
		for (int i = 0; i < 6; ++i)
		{
			this->pLegs[i]->TransformCoordinatePee(bodyPe, fromMak, fromPee + i * 3, toMak, toPee + i * 3);
		}
	}

}