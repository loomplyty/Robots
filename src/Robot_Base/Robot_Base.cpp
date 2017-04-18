#include <complex>
#include <cmath>
#include <ctime>
#include <iostream>

#include <aris.h>

#include "Robot_Base.h"

#define EIGEN_NO_MALLOC
#include <Eigen/Eigen>

using namespace aris::dynamic;
using namespace std;

namespace Robots
{
	LegBase::LegBase(RobotBase* pRobot, const char *name)
		: Object(static_cast<aris::dynamic::Model &>(*pRobot), name)
		, pRobot(pRobot)
	{
		base_mak_id_ = &pRobot->body().markerPool().add(std::string(name) + "_Base");
	}
	
	const aris::dynamic::Part& LegBase::ground() const { return pRobot->ground(); };
	aris::dynamic::Part& LegBase::ground() { return pRobot->ground(); };
	const aris::dynamic::Part& LegBase::body() const { return pRobot->body(); };
	aris::dynamic::Part& LegBase::body() { return pRobot->body(); };
	const aris::dynamic::Marker& LegBase::base() const { return *base_mak_id_; };
	aris::dynamic::Marker& LegBase::base() { return *base_mak_id_; };
	void LegBase::GetPee(double *pEE, const Coordinate &mak) const
	{
		double pEE_G[3];
		
		s_pp2pp(*base().pm(), this->pEE, pEE_G);
		s_inv_pp2pp(*mak.pm(), pEE_G, pEE);
	}
	void LegBase::SetPee(const double *pEE, const Coordinate &mak)
	{
		base().update();

		double pEE_G[3];
		
		s_pp2pp(*mak.pm(), pEE, pEE_G);
		s_inv_pp2pp(*base().pm(), pEE_G, this->pEE);

		calculate_from_pEE();
		calculate_jac();
	}
	void LegBase::GetVee(double *vEE, const Coordinate &mak) const
	{
		double pEE_G[3], vEE_G[3];
		
		s_vp2vp(*base().pm(), base().vel(), this->pEE, this->vEE, vEE_G, pEE_G);
		s_inv_vp2vp(*mak.pm(), mak.vel(), pEE_G, vEE_G, vEE);
	}
	void LegBase::SetVee(const double *vEE, const Coordinate &mak)
	{
		double pEE[3];
		GetPee(pEE, mak);

		double pEE_G[3], vEE_G[3];
		
		s_vp2vp(*mak.pm(), mak.vel(), pEE, vEE, vEE_G, pEE_G);
		s_inv_vp2vp(*base().pm(), base().vel(), pEE_G, vEE_G, this->vEE);

		calculate_from_vEE();
		calculate_diff_jac();
	}
	void LegBase::GetAee(double *aEE, const Coordinate &mak) const
	{
		double pEE_G[3], vEE_G[3], aEE_G[3];
		
		s_ap2ap(*base().pm(), base().vel(), base().acc(), this->pEE, this->vEE, this->aEE, aEE_G, vEE_G, pEE_G);
		s_inv_ap2ap(*mak.pm(), mak.vel(), mak.acc(), pEE_G, vEE_G, aEE_G, aEE);
	}
	void LegBase::SetAee(const double *aEE, const Coordinate &mak)
	{
		double pEE[3], vEE[3];
		GetPee(pEE, mak);
		GetVee(vEE, mak);

		double pEE_G[3], vEE_G[3], aEE_G[3];
		
		s_ap2ap(*mak.pm(), mak.vel(), mak.acc(), pEE, vEE, aEE, aEE_G, vEE_G, pEE_G);
		s_inv_ap2ap(*base().pm(), base().vel(),base().acc(), pEE_G, vEE_G,aEE_G, this->aEE);

		calculate_from_aEE();
	}
	void LegBase::GetFeeSta(double *fEE_sta, const Coordinate &mak) const
	{
		double f_G[3];
		
		s_pm_dot_v3(*base().pm(), this->fEE_sta, f_G);
		s_inv_pm_dot_v3(*mak.pm(), f_G, fEE_sta);
	}
	void LegBase::SetFeeSta(const double *fEE_sta, const Coordinate &mak)
	{

		double f_G[3];
		
		s_pm_dot_v3(*mak.pm(), fEE_sta, f_G);
		s_inv_pm_dot_v3(*base().pm(), f_G, this->fEE_sta);
	}
	void LegBase::GetPin(double *pIn) const
	{
		std::copy_n(this->pIn, 3, pIn);
	}
	void LegBase::SetPin(const double *pIn)
	{
		base().update();

		std::copy_n(pIn, 3, this->pIn);
		calculate_from_pIn();
		calculate_jac();
	}
	void LegBase::GetVin(double *vIn) const
	{
		std::copy_n(this->vIn, 3, vIn);
	}
	void LegBase::SetVin(const double *vIn)
	{
		std::copy_n(vIn, 3, this->vIn);

		calculate_from_vIn();
		calculate_diff_jac();
	}
	void LegBase::GetAin(double *aIn) const
	{
		std::copy_n(this->aIn, 3, aIn);
	}
	void LegBase::SetAin(const double *aIn)
	{
		std::copy_n(aIn, 3, this->aIn);
		calculate_from_aIn();
	}
	void LegBase::GetFinSta(double *fIn_sta) const
	{
		std::copy_n(this->fIn_sta, 3, fIn_sta);
	}
	void LegBase::SetFinSta(const double *fIn_sta)
	{
		std::copy_n(fIn_sta, 3, this->fIn_sta);
		s_dgemmTN(3, 1, 3, 1, *Jvi, 3, fIn_sta, 1, 0, fEE_sta, 1);
	}
	void LegBase::GetJfd(double *jac, const Coordinate &mak) const
	{
		std::fill_n(jac, 9, 0);
		
		/*力雅克比是速度雅克比转置的逆*/
		double locJac[3][3];

		GetJvi(*locJac, mak);
		s_transpose(3, 3, *locJac, 3, jac, 3);
	}
	void LegBase::GetJfi(double *jac, const Coordinate &mak) const
	{
		std::fill_n(jac, 9, 0);
		
		/*力雅克比是速度雅克比转置的逆*/
		double locJac[3][3];

		GetJvd(*locJac, mak);
		s_transpose(3, 3, *locJac, 3, jac, 3);
	}
	void LegBase::GetJvd(double *jac, const Coordinate &mak) const
	{
		std::fill_n(jac, 9, 0);
		
		double relativePm[16];
		
		s_inv_pm_dot_pm(*mak.pm(), *base().pm(), relativePm);
		s_dgemm(3, 3, 3, 1, relativePm, 4, *Jvd, 3, 0, jac, 3);
	}
	void LegBase::GetJvi(double *jac, const Coordinate &mak) const
	{
		std::fill_n(jac, 9, 0);
		
		double relativePm[16];
		
		s_inv_pm_dot_pm(*mak.pm(), *base().pm(), relativePm);
		s_dgemmNT(3, 3, 3, 1, *Jvi, 3, relativePm, 4, 0, jac, 3);
	}
	void LegBase::GetDifJfd(double *dJac, const Coordinate &mak) const
	{
		
		
		/*力雅克比是速度雅克比转置的逆*/
		double locJac[3][3];

		GetDifJvi(*locJac, mak);
		s_transpose(3, 3, *locJac, 3, dJac, 3);
	}
	void LegBase::GetDifJfi(double *dJac, const Coordinate &mak) const
	{
		
		
		/*力雅克比是速度雅克比转置的逆*/
		double locJac[3][3];

		GetJvd(*locJac, mak);
		s_transpose(3, 3, *locJac, 3, dJac, 3);
	}
	void LegBase::GetDifJvd(double *dJac, const Coordinate &mak) const
	{
		std::fill_n(dJac, 9, 0);
		
		double relativePm[16], relativeV[6];
		
		s_inv_pm_dot_pm(*mak.pm(), *base().pm(), relativePm);
		s_inv_v2v(*mak.pm(), mak.vel(), base().vel(), relativeV);
		
		double dR[4][4];
		s_v_cro_pm(relativeV, relativePm, *dR);

		s_dgemm(3, 3, 3, 1, *dR, 4, *Jvd, 3, 0, dJac, 3);
		s_dgemm(3, 3, 3, 1, relativePm, 4, *vJvd, 3, 1, dJac, 3);
	}
	void LegBase::GetDifJvi(double *dJac, const Coordinate &mak) const
	{
		std::fill_n(dJac, 9, 0);
		
		double relativePm[16], relativeV[6];

		s_inv_pm_dot_pm(*mak.pm(), *base().pm(), relativePm);
		s_inv_v2v(*mak.pm(), mak.vel(), base().vel(), relativeV);

		double dR[4][4];
		s_v_cro_pm(relativeV, relativePm, *dR);

		s_dgemmNT(3, 3, 3, 1, *vJvi, 3, relativePm, 4, 0, dJac, 3);
		s_dgemmNT(3, 3, 3, 1, *Jvi, 3, *dR, 4, 1, dJac, 3);
	}
	void LegBase::GetCvd(double *c, const Coordinate &mak) const
	{
		
		
		double relativePm[16], relativeV[6];
		
		s_inv_pm_dot_pm(*mak.pm(), *base().pm(), relativePm);
		s_inv_v2v(*mak.pm(), mak.vel(), base().vel(), relativeV);
		
		s_vp2vp(relativePm, relativeV, this->pEE, 0, c);
	}
	void LegBase::GetCvi(double *c, const Coordinate &mak) const
	{
		
		
		double relativePm[16], relativeV[6];
		
		s_inv_pm_dot_pm(*mak.pm(), *base().pm(), relativePm);
		s_inv_v2v(*mak.pm(), mak.vel(), base().vel(), relativeV);

		double tem[3];
		s_vp2vp(relativePm, relativeV, this->pEE, nullptr, c);
		s_dgemmTN(3, 1, 3, 1, *base().pm(), 4, c, 1, 0, tem, 1);
		s_dgemm(3, 1, 3, -1, *Jvi, 3, tem, 1, 0, c, 1);
	}
	void LegBase::GetCad(double *c, const Coordinate &mak) const
	{
		
		
		double relativeV[6], relativeA[6];
		
		s_inv_a2a(*mak.pm(), mak.vel(), mak.acc(), base().vel(), base().acc(), relativeA, relativeV);
		
		/*推导如下*/
		//Vee_G = R_L2G * Vee_L + Vb + Wb x Pee_G = R_L2G * Jvd_L * Vin + Vb + Wb x Pee_G
		//      = Jvd_G * Vin + Vb + Wb x Pee_G
		//Aee_G = Jvd_G * Ain + dJvd_G * Vin + Ab + Xb x Pee_G + Vb x Vee_G
		//      = Jvi_G * Ain + dJvd_G * Vin + pa
		//c = pa

		//Ain = Jvi_G * Aee_G + dJvi_G * Vee_G - Jvi_G * (Vb + Wb x Pee_G) - dJvi_G * (Ab + Xb x Pee_G + Wb x Vee_G)

		double pEE_G[3], vEE_G[3];
		this->GetPee(pEE_G, mak);
		this->GetVee(vEE_G, mak);

		s_vp(pEE_G, relativeA, c);
		s_cro3(1, relativeV + 3, vEE_G, 1, c);
	}
	void LegBase::GetCai(double *c, const Coordinate &mak) const
	{
		
		
		double relativeV[6], relativeA[6];
		
		s_inv_a2a(*mak.pm(), mak.vel(), mak.acc(), base().vel(), base().acc(), relativeA, relativeV);
		/*推导如下*/
		//Vee_G = R_L2G * Vee_L + Vb + Wb x Pee_G
		//Vee_L = R_G2L * (Vee_G - Vb - Wb x Pee_G)
		//Vin = Jvi_L * Vee_L = Jvi_L * R_G2L * (Vee_G - Vb - Wb x Pee_G)
		//    = Jvi_G * (Vee_G - Vb - Wb x Pee_G)

		//Ain = Jvi_G * Aee_G + dJvi_G * Vee_G - Jvi_G * (Vb + Wb x Pee_G) - dJvi_G * (Ab + Xb x Pee_G + Wb x Vee_G)

		

		double pEE_G[3], vEE_G[3];
		this->GetPee(pEE_G, mak);
		this->GetVee(vEE_G, mak);
		double pv[3], pa[3];


		/*！！！特别注意：！！！*/
		/*这里pv=Vb + Wb x Pee*/
		/*这里pa=Ab + Xb x Pee + Xb x Vee*/
		/*其中Vb Wb Ab Xb 分别为机器人身体的线速度，角速度，线加速度，角加速度，Pee和Vee相对于地面坐标系*/
		/*这里一定不能用s_pa2pa函数*/
		s_vp(pEE_G, relativeV, pv);
		s_vp(pEE_G, relativeA, pa);
		s_cro3(1, relativeV + 3, vEE_G, 1, pa);

		/*之后有：c = -J * pa - dJ * pv */
		double Jac[3][3], dJac[3][3];
		this->GetDifJvi(*dJac);
		this->GetJvi(*Jac);

		s_dgemm(3, 1, 3, -1, *Jac, 3, pa, 1, 0, c, 1);
		s_dgemm(3, 1, 3, -1, *dJac, 3, pv, 1, 1, c, 1);
	}
	

	void LegBase::TransformCoordinatePee(const double *bodyPe, const char *fromMak, const double *fromPee
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
				s_pm_dot_pnt(*base().prtPm(), fromPee, toPee);
				return;
			}
			case 'G':
			case 'O':
			default:
			{
				double bodyPm[16], pnt[3];
				s_pe2pm(bodyPe, bodyPm);
				s_pm_dot_pnt(*base().prtPm(), fromPee, pnt);
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
				s_inv_pm_dot_pnt(*base().prtPm(), fromPee, toPee);
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
				s_inv_pm_dot_pnt(*base().prtPm(), pnt, toPee);
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

	RobotBase::RobotBase()
	{
		body_ = &partPool().add<Part>("MainBody");
	}

	void RobotBase::GetPmb(double *pmb, const Coordinate &mak) const
	{
		s_inv_pm_dot_pm(*mak.pm(), *body().pm(), pmb);
	}
	void RobotBase::SetPmb(const double *pmb, const Coordinate &mak)
	{
		s_pm_dot_pm(*mak.pm(), pmb, *body().pm());
	}
	void RobotBase::GetPeb(double *peb, const Coordinate &mak, const char *eurType) const
	{
		double pm[16];
		GetPmb(pm, mak);
		s_pm2pe(pm, peb, eurType);
	}
	void RobotBase::SetPeb(const double *peb, const Coordinate &mak, const char *eurType)
	{
		double pm[16];
		s_pe2pm(peb, pm, eurType);
		SetPmb(pm, mak);
	}
	void RobotBase::GetPqb(double *pqb, const Coordinate &mak) const
	{
		double pm[16];
		GetPmb(pm, mak);
		s_pm2pq(pm, pqb);
	}
	void RobotBase::SetPqb(const double *pqb, const Coordinate &mak)
	{
		double pm[16];
		s_pq2pm(pqb, pm);
		SetPmb(pm, mak);
	}
	void RobotBase::GetVb(double *vb, const Coordinate &mak) const
	{
		s_inv_v2v(*mak.pm(), mak.vel(), body().vel(), vb);
	}
	void RobotBase::SetVb(const double *vb, const Coordinate &mak)
	{
		s_v2v(*mak.pm(), mak.vel(), vb, body().vel());
	}
	void RobotBase::GetAb(double *ab, const Coordinate &mak) const
	{
		s_inv_a2a(*mak.pm(), mak.vel(), mak.acc(), body().vel(), body().acc(), ab);
	}
	void RobotBase::SetAb(const double *ab, const Coordinate &mak)
	{
		double vb[6];
		s_inv_v2v(*mak.pm(), mak.vel(), body().vel(), vb);
		s_a2a(*mak.pm(), mak.vel(), mak.acc(), vb, ab, body().acc());
	}

	void RobotBase::GetPee(double *pEE, const Coordinate &mak) const
	{
		
		for (int i = 0; i < 6; ++i)
		{
			pLegs[i]->GetPee(pEE + i * 3, mak);
		}
	}
	void RobotBase::SetPee(const double *pEE, const Coordinate &mak)
	{
		for (int i = 0; i < 6; i++)
		{
			pLegs[i]->SetPee(pEE + i * 3, mak);
		}

		calculate_jac();
	}
	void RobotBase::GetVee(double *vEE, const Coordinate &mak) const
	{
		
		for (int i = 0; i < 6; ++i)
		{
			pLegs[i]->GetVee(vEE + i * 3, mak);
		}
	}
	void RobotBase::SetVee(const double *vEE, const Coordinate &mak)
	{
		
		for (int i = 0; i < 6; i++)
		{
			pLegs[i]->SetVee(vEE + i * 3, mak);
		}

		calculate_jac_c();
	}
	void RobotBase::GetAee(double *aEE, const Coordinate &mak) const
	{
		
		for (int i = 0; i < 6; ++i)
		{
			pLegs[i]->GetAee(aEE + i * 3, mak);
		}
	}
	void RobotBase::SetAee(const double *aEE, const Coordinate &mak)
	{
		for (int i = 0; i < 6; i++)
		{
			pLegs[i]->SetAee(aEE + i * 3, mak);
		}
	}
	void RobotBase::GetFeeSta(double *fee_sta, const Coordinate &mak) const
	{
		
		for (int i = 0; i < 6; i++)
		{
			pLegs[i]->GetFeeSta(fee_sta + i * 3, mak);
		}
	}
	void RobotBase::SetFeeSta(const double *fee_sta, const Coordinate &mak)
	{
		
		for (int i = 0; i < 6; i++)
		{
			pLegs[i]->SetFeeSta(fee_sta + i * 3, mak);
		}
	}
	void RobotBase::GetPee(double *pEE, const Coordinate* const pMaks[]) const
	{
		for (int i = 0; i < 6; ++i)
		{
			pLegs[i]->GetPee(pEE + i * 3, *pMaks[i]);
		}
	}
	void RobotBase::SetPee(const double *pEE, const Coordinate* const pMaks[])
	{
		for (int i = 0; i < 6; i++)
		{
			pLegs[i]->SetPee(pEE + i * 3, *pMaks[i]);
		}

		calculate_jac();
	}
	void RobotBase::GetVee(double *vEE, const Coordinate* const pMaks[]) const
	{
		for (int i = 0; i < 6; ++i)
		{
			pLegs[i]->GetVee(vEE + i * 3, *pMaks[i]);
		}
	}
	void RobotBase::SetVee(const double *vEE, const Coordinate* const pMaks[])
	{
		if (vEE)
		{
			for (int i = 0; i < 6; i++)
			{
				pLegs[i]->SetVee(vEE + i * 3, *pMaks[i]);
			}
		}

		calculate_jac_c();
	}
	void RobotBase::GetAee(double *aEE, const Coordinate* const pMaks[]) const
	{
		for (int i = 0; i < 6; ++i)
		{
			pLegs[i]->GetAee(aEE + i * 3, *pMaks[i]);
		}
	}
	void RobotBase::SetAee(const double *aEE, const Coordinate* const pMaks[])
	{
		if (aEE)
		{
			for (int i = 0; i < 6; i++)
			{
				pLegs[i]->SetAee(aEE + i * 3, *pMaks[i]);
			}
		}
	}
	void RobotBase::GetFeeSta(double *fee_sta, const Coordinate* const pMaks[]) const
	{
		for (int i = 0; i < 6; i++)
		{
			pLegs[i]->GetFeeSta(fee_sta + i * 3, *pMaks[i]);
		}
	}
	void RobotBase::SetFeeSta(const double *fee_sta, const Coordinate* const pMaks[])
	{
		for (int i = 0; i < 6; i++)
		{
			pLegs[i]->SetFeeSta(fee_sta + i * 3, *pMaks[i]);
		}
	}
	void RobotBase::GetPin(double *pIn) const
	{
		for (int i = 0; i < 6; ++i)
		{
			pLegs[i]->GetPin(pIn + i * 3);
		}
	}
	void RobotBase::SetPin(const double *pIn)
	{
		for (int i = 0; i < 6; i++)
		{
			pLegs[i]->SetPin(pIn + i * 3);
		}

		calculate_jac();
	}
	void RobotBase::GetVin(double *vIn) const
	{
		for (int i = 0; i < 6; ++i)
		{
			pLegs[i]->GetVin(vIn + i * 3);
		}
	}
	void RobotBase::SetVin(const double *vIn)
	{
		for (int i = 0; i < 6; i++)
		{
			pLegs[i]->SetVin(vIn + i * 3);
		}

		calculate_jac_c();
	}
	void RobotBase::GetAin(double *aIn) const
	{
		for (int i = 0; i < 6; ++i)
		{
			pLegs[i]->GetAin(aIn + i * 3);
		}
	}
	void RobotBase::SetAin(const double *aIn)
	{
		for (int i = 0; i < 6; i++)
		{
			pLegs[i]->SetAin(aIn + i * 3);
		}
	}
	void RobotBase::GetFinSta(double *fIn_sta) const
	{
		for (int i = 0; i < 6; ++i)
		{
			pLegs[i]->GetFinSta(fIn_sta + i * 3);
		}
	}
	void RobotBase::SetFinSta(const double *fIn_sta)
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
	
	void RobotBase::SetPinFixFeet(const double *pIn_end, const char *fixFeet, const char *activeMotor, const double *initBodyPE)
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

		this->GetPee(pEE_G);
		aris::dynamic::s_pe2pq(initBodyPE, pq, "313");
		s_pq2pe(pq, pe, "313");
		this->SetPeb(pe);
		this->SetPee(pEE_G);
		GetPin(pIn);

		for (int j = 0; j < dim; ++j)
		{
			vIn[j] = pIn_end[supportID[j]] - pIn[supportID[j]];
		}

		double error = s_dnrm2(dim,vIn,1);
		int compute_count = 0;
		/*迭代计算身体的位姿*/
		while((error>1e-10)&&(compute_count<=100))
		{
			++compute_count;

			GetJvi(jac, supportMotor);
			if (dim == 6)
			{
				//int ipiv[6];
				//s_dgesv(6, 1, jac, 6, ipiv, vIn, 1);

				Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor> > jac_m(jac);
				Eigen::Map<Eigen::Matrix<double, 6, 1, Eigen::ColMajor> > vIn_m(vIn);
				Eigen::Map<Eigen::Matrix<double, 6, 1, Eigen::ColMajor> > vb_m(vb);
				vb_m = jac_m.partialPivLu().solve(vIn_m);
			}
			else
			{
				//int rank;
				//double s[6];
				//s_dgelsd(dim, 6, 1, jac, 6, vIn, 1, s, 0.000000001, &rank);

				Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 6, Eigen::RowMajor, 18, 6> > jac_m(jac, dim, 6);
				Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor, 18, 1> > vIn_m(vIn, dim);
				Eigen::Map<Eigen::Matrix<double, 6, 1, Eigen::ColMajor> > vb_m(vb);
				vb_m = jac_m.jacobiSvd(Eigen::ComputeFullU|Eigen::ComputeFullV).solve(vIn_m);
			}

			//std::copy_n(vIn, dim, vb);

			s_pq2pm(pq, pm);
			s_v2vq(pm, vb, vq);

			s_daxpy(7, 1, vq, 1, pq, 1);
			double norm = s_dnrm2(4, &pq[3], 1);
			for (int j = 3; j < 7; ++j)
			{
				pq[j] /= norm;
			}

			s_pq2pe(pq, pe, "313");
			SetPeb(pe);
			SetPee(pEE_G);
			GetPin(pIn);

			for (int j = 0; j < dim; ++j)
			{
				vIn[j] = pIn_end[supportID[j]] - pIn[supportID[j]];
			}

			error = s_dnrm2(dim, vIn, 1);
		}

		if (compute_count > 100)std::cout << "iterator in SetPinFixFeet failed, with error:" << error << std::endl;
		/*根据结果首先设置支撑腿的末端及身体位姿，随后设置其他腿的输入*/
		s_pq2pe(pq, pe, "313");
		this->SetPeb(pe);
		this->SetPee(pEE_G);
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
	void RobotBase::SetVinFixFeet(const double *vIn, const char *fixFeet, const char *activeMotor)
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
		double vb[6];
		GetJvi(jac, supportMotor);
		if (dim == 6)
		{
			//int ipiv[6];
			//s_dgesv(6, 1, jac, 6, ipiv, vIn_loc, 1);

			Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor> > jac_m(jac);
			Eigen::Map<Eigen::Matrix<double, 6, 1, Eigen::ColMajor> > vIn_m(vIn_loc);
			Eigen::Map<Eigen::Matrix<double, 6, 1, Eigen::ColMajor> > vb_m(vb);
			vb_m = jac_m.partialPivLu().solve(vIn_m);
		}
		else
		{
			//int rank;
			//double s[6];
			//s_dgelsd(dim, 6, 1, jac, 6, vIn_loc, 1, s, 0.000000001, &rank);

			Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 6, Eigen::RowMajor, 18, 6> > jac_m(jac, dim, 6);
			Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor, 18, 1> > vIn_m(vIn_loc, dim);
			Eigen::Map<Eigen::Matrix<double, 6, 1, Eigen::ColMajor> > vb_m(vb);
			vb_m = jac_m.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(vIn_m);


		}

		/*先设置支撑腿的末端速度为0，之后设置其他腿的输入*/
		double vEE[18]{ 0 };
		this->SetVb(vb);
		this->SetVee(vEE);
		for (int i = 0; i < 6; ++i)
		{
			/*判断是否为支撑腿*/
			if ((i >= static_cast<int>(strlen(fixFeet))) || (fixFeet[i] == '0'))
			{
				this->pLegs[i]->SetVin(vIn + i * 3);
			}
		}
	}
	void RobotBase::SetAinFixFeet(const double *aIn, const char *fixFeet, const char *activeMotor)
	{
		/*找出支撑身体的所有电机及其维数等*/
		char supportMotor[19]{ "000000000000000000" };
		int supportID[18];
		int dim;
		findSupportMotion(fixFeet, activeMotor, supportMotor, supportID, dim);

		/*计算所有支撑电机的速度*/
		double aIn_loc[18];
		double ab[6];

		for (int j = 0; j < dim; ++j)
		{
			aIn_loc[j] = aIn[supportID[j]];
		}

		/*计算身体雅可比，只根据支撑的电机来计算，之后求解雅可比方程*/
		double jac[18 * 6], dJac[18 * 6];
		GetJvi(jac, supportMotor);
		GetDifJvi(dJac, supportMotor);
		s_dgemm(dim, 1, 6, -1, dJac, 6, this->body().vel(), 1, 1, aIn_loc, 1);

		

		if (dim == 6)
		{
			Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor> > jac_m(jac);
			Eigen::Map<Eigen::Matrix<double, 6, 1, Eigen::ColMajor> > aIn_m(aIn_loc);
			Eigen::Map<Eigen::Matrix<double, 6, 1, Eigen::ColMajor> > ab_m(ab);
			ab_m = jac_m.partialPivLu().solve(aIn_m);
		}
		else
		{
			Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 6, Eigen::RowMajor, 18, 6> > jac_m(jac, dim, 6);
			Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor, 18, 1> > aIn_m(aIn_loc, dim);
			Eigen::Map<Eigen::Matrix<double, 6, 1, Eigen::ColMajor> > ab_m(ab);
			ab_m = jac_m.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(aIn_m);
		}

		/*先设置支撑腿的末端速度为0，之后设置其他腿的输入*/
		double aEE[18]{ 0 };
		this->SetAb(ab);
		this->SetAee(aEE);
		for (int i = 0; i < 6; ++i)
		{
			/*判断是否为支撑腿*/
			if ((i >= static_cast<int>(strlen(fixFeet))) || (fixFeet[i] == '0'))
			{
				this->pLegs[i]->SetAin(aIn + i * 3);
			}
		}




	}

	void RobotBase::GetJvi(double *jac, const char *activeMotion) const
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
	void RobotBase::GetJfd(double *jac, const char *activeMotion) const
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
	void RobotBase::GetDifJvi(double *dJac, const char *activeMotion) const
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
	void RobotBase::GetDifJfd(double *dJac, const char *activeMotion) const
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
	
	void RobotBase::calculate_jac()
	{
		double jac[9]{ 0 }, cm3[9]{ 0 }, pEE[3]{0};
		
		for (int i = 0; i < 6; ++i)
		{
			pLegs[i]->GetJvi(jac);
			s_block_cpy(3, 3, -1, jac, 0, 0, 3, 0, &this->Jvi[0][0], i * 3, 0, 6);

			pLegs[i]->GetPee(pEE);
			s_cm3(pEE, cm3);
			s_dgemm(3, 3, 3, 1, jac, 3, cm3, 3, 0, &this->Jvi[i * 3][3], 6);
		}
	}
	void RobotBase::calculate_jac_c()
	{
		double dJac[9], jac[9], pEE[3], vEE[3], cmP[9], cmV[9];
		
		for (int i = 0; i < 6; ++i)
		{
			pLegs[i]->GetJvi(jac);
			pLegs[i]->GetDifJvi(dJac);

			s_block_cpy(3, 3, -1, dJac, 0, 0, 3, 0, *vJvi, i * 3, 0, 6);

			/*理论上来说，vEE必须为0，但这里不做限制了，如果vEE不为零，不确定会有什么后果*/
			pLegs[i]->GetPee(pEE);
			pLegs[i]->GetVee(vEE);
			s_cm3(pEE, cmP);
			s_cm3(vEE, cmV);
			s_dgemm(3, 3, 3, 1, dJac, 3, cmP, 3, 0, &vJvi[i * 3][3], 6);
			s_dgemm(3, 3, 3, 1, jac, 3, cmV, 3, 1, &vJvi[i * 3][3], 6);
		}
	}
	
	void RobotBase::TransformCoordinatePee(const double *bodyPe, const char *fromMak, const double *fromPee, const char *toMak, double *toPee) const
	{
		for (int i = 0; i < 6; ++i)
		{
			this->pLegs[i]->TransformCoordinatePee(bodyPe, fromMak, fromPee + i * 3, toMak, toPee + i * 3);
		}
	}

}
