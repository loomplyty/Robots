#include <complex>
#include <cmath>
#include <ctime>
#include <iostream>
#include <fstream>

#include <aris.h>

#include "Robot_Gait.h"
#include "Robot_Type_I.h"

//#define EIGEN_NO_MALLOC
#include "Eigen/Eigen"	

using namespace aris::dynamic;
using namespace std;

namespace Robots
{	
	LegI::LegI(const char *Name, RobotTypeI* pRobot)
		: LegBase(static_cast<RobotBase *>(pRobot), Name)
		, pRobot(pRobot)
	{
	}

	void LegI::GetdJacOverPee(double *dJi_x, double *dJi_y, double *dJi_z, const char *relativeCoordinate) const
	{
		double dk21_a1 = -U2x*(S2x + l1)*ca1*cb1 + U2x*S2y*ca1*sb1 + U2x*S2z*(-sa1) - U2z*(S2x + l1)*(-sa1)*cb1 + U2z*S2y*(-sa1)*sb1 - U2z*S2z*ca1;
		double dk22_a1 = -U2x*(S2x + l1)*(-sa1)*sb1 - U2x*S2y*(-sa1)*cb1 + U2z*(S2x + l1)*ca1*sb1 + U2z*S2y*ca1*cb1;
		double dk23_a1 = U2x*(-sa1)*cb1 - U2z*ca1*cb1;

		double dk31_a1 = -U3x*(S3x + l1)*ca1*cb1 + U3x*S3y*ca1*sb1 + U3x*S3z*(-sa1) - U3z*(S3x + l1)*(-sa1)*cb1 + U3z*S3y*(-sa1)*sb1 - U3z*S3z*ca1;
		double dk32_a1 = -U3x*(S3x + l1)*(-sa1)*sb1 - U3x*S3y*(-sa1)*cb1 + U3z*(S3x + l1)*ca1*sb1 + U3z*S3y*ca1*cb1;
		double dk33_a1 = U3x*(-sa1)*cb1 - U3z*ca1*cb1;

		double dk21_b1 = -U2x*(S2x + l1)*sa1*(-sb1) + U2x*S2y*sa1*cb1 - U2z*(S2x + l1)*ca1*(-sb1) + U2z*S2y*ca1*cb1;
		double dk22_b1 = -U2x*(S2x + l1)*ca1*cb1 - U2x*S2y*ca1*(-sb1) + U2y*(S2x + l1)*(-sb1) - U2y*S2y*cb1 + U2z*(S2x + l1)*sa1*cb1 + U2z*S2y*sa1*(-sb1);
		double dk23_b1 = U2x*ca1*(-sb1) + U2y*cb1 - U2z*sa1*(-sb1);

		double dk31_b1 = -U3x*(S3x + l1)*sa1*(-sb1) + U3x*S3y*sa1*cb1 - U3z*(S3x + l1)*ca1*(-sb1) + U3z*S3y*ca1*cb1;
		double dk32_b1 = -U3x*(S3x + l1)*ca1*cb1 - U3x*S3y*ca1*(-sb1) + U3y*(S3x + l1)*(-sb1) - U3y*S3y*cb1 + U3z*(S3x + l1)*sa1*cb1 + U3z*S3y*sa1*(-sb1);
		double dk33_b1 = U3x*ca1*(-sb1) + U3y*cb1 - U3z*sa1*(-sb1);

		double dk21_l1 = -U2x*sa1*cb1 - U2z*ca1*cb1;
		double dk22_l1 = -U2x*ca1*sb1 + U2y*cb1 + U2z*sa1*sb1;
		double dk23_l1 = -1;

		double dk31_l1 = -U3x*sa1*cb1 - U3z*ca1*cb1;
		double dk32_l1 = -U3x*ca1*sb1 + U3y*cb1 + U3z*sa1*sb1;
		double dk33_l1 = -1;

		double dJ2_a1[3][3]{ 0 }, dJ2_b1[3][3]{ 0 }, dJ2_l1[3][3]{ 0 };

		dJ2_a1[1][0] = -dk21_a1 / l2 + k21 / l2 / l2*J2[1][0];
		dJ2_a1[1][1] = -dk22_a1 / l2 + k22 / l2 / l2*J2[1][0];
		dJ2_a1[1][2] = -dk23_a1 / l2 + k23 / l2 / l2*J2[1][0];
		dJ2_a1[2][0] = -dk31_a1 / l3 + k31 / l3 / l3*J2[2][0];
		dJ2_a1[2][1] = -dk32_a1 / l3 + k32 / l3 / l3*J2[2][0];
		dJ2_a1[2][2] = -dk33_a1 / l3 + k33 / l3 / l3*J2[2][0];

		dJ2_b1[1][0] = -dk21_b1 / l2 + k21 / l2 / l2*J2[1][1];
		dJ2_b1[1][1] = -dk22_b1 / l2 + k22 / l2 / l2*J2[1][1];
		dJ2_b1[1][2] = -dk23_b1 / l2 + k23 / l2 / l2*J2[1][1];
		dJ2_b1[2][0] = -dk31_b1 / l3 + k31 / l3 / l3*J2[2][1];
		dJ2_b1[2][1] = -dk32_b1 / l3 + k32 / l3 / l3*J2[2][1];
		dJ2_b1[2][2] = -dk33_b1 / l3 + k33 / l3 / l3*J2[2][1];

		dJ2_l1[1][0] = -dk21_l1 / l2 + k21 / l2 / l2*J2[1][2];
		dJ2_l1[1][1] = -dk22_l1 / l2 + k22 / l2 / l2*J2[1][2];
		dJ2_l1[1][2] = -dk23_l1 / l2 + k23 / l2 / l2*J2[1][2];
		dJ2_l1[2][0] = -dk31_l1 / l3 + k31 / l3 / l3*J2[2][2];
		dJ2_l1[2][1] = -dk32_l1 / l3 + k32 / l3 / l3*J2[2][2];
		dJ2_l1[2][2] = -dk33_l1 / l3 + k33 / l3 / l3*J2[2][2];

		double dJ2_x[3][3], dJ2_y[3][3], dJ2_z[3][3];

		for (int i = 0; i < 3; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				dJ2_x[i][j] = dJ2_a1[i][j] * inv_J1[0][0] + dJ2_b1[i][j] * inv_J1[1][0] + dJ2_l1[i][j] * inv_J1[2][0];
				dJ2_y[i][j] = dJ2_a1[i][j] * inv_J1[0][1] + dJ2_b1[i][j] * inv_J1[1][1] + dJ2_l1[i][j] * inv_J1[2][1];
				dJ2_z[i][j] = dJ2_a1[i][j] * inv_J1[0][2] + dJ2_b1[i][j] * inv_J1[1][2] + dJ2_l1[i][j] * inv_J1[2][2];
			}
		}

		double dJ1_x[3][3]{ 0 }, dJ1_y[3][3]{ 0 }, dJ1_z[3][3]{0};

		dJ1_x[0][0] = 0;
		dJ1_x[0][1] = y*sa1*inv_J1[0][0];
		dJ1_x[0][2] = -sa1*cb1*inv_J1[0][0] - ca1*sb1*inv_J1[1][0];
		dJ1_x[1][0] = 0;
		dJ1_x[1][1] = ca1 + (-x*sa1 - z*ca1)*inv_J1[0][0];
		dJ1_x[1][2] = cb1*inv_J1[1][0];
		dJ1_x[2][0] = -1;
		dJ1_x[2][1] = y*ca1*inv_J1[0][0];
		dJ1_x[2][2] = -ca1*cb1*inv_J1[0][0]+sa1*sb1*inv_J1[1][0];

		dJ1_y[0][0] = 0;
		dJ1_y[0][1] = -ca1 + y*sa1*inv_J1[0][1];
		dJ1_y[0][2] = -sa1*cb1*inv_J1[0][1] - ca1*sb1*inv_J1[1][1];
		dJ1_y[1][0] = 0;
		dJ1_y[1][1] = (-x*sa1 - z*ca1)*inv_J1[0][1];
		dJ1_y[1][2] = cb1*inv_J1[1][1];
		dJ1_y[2][0] = 0;
		dJ1_y[2][1] = y*ca1*inv_J1[0][1];
		dJ1_y[2][2] = -ca1*cb1*inv_J1[0][1] + sa1*sb1*inv_J1[1][1];

		dJ1_z[0][0] = 1;
		dJ1_z[0][1] = y*sa1*inv_J1[0][2];
		dJ1_z[0][2] = -sa1*cb1*inv_J1[0][2] - ca1*sb1*inv_J1[1][2];
		dJ1_z[1][0] = 0;
		dJ1_z[1][1] = -sa1 + (-x*sa1 - z*ca1)*inv_J1[0][2];
		dJ1_z[1][2] = cb1*inv_J1[1][2];
		dJ1_z[2][0] = 0;
		dJ1_z[2][1] = y*ca1*inv_J1[0][2];
		dJ1_z[2][2] = -ca1*cb1*inv_J1[0][2] + sa1*sb1*inv_J1[1][2];

		/*以下计算*/
		double dJi_x_L[9], dJi_y_L[9], dJi_z_L[9];
		double tem1[3][3], tem2[3][3];


		s_dgemm(3, 3, 3, 1, *dJ2_x, 3, *inv_J1, 3, 0, dJi_x_L, 3);
		s_dgemm(3, 3, 3, 1, *J2, 3, *inv_J1, 3, 0, *tem1, 3);
		s_dgemm(3, 3, 3, 1, *tem1, 3, *dJ1_x, 3, 0, *tem2, 3);
		s_dgemm(3, 3, 3, -1, *tem2, 3, *inv_J1, 3, 1, dJi_x_L, 3);

		s_dgemm(3, 3, 3, 1, *dJ2_y, 3, *inv_J1, 3, 0, dJi_y_L, 3);
		s_dgemm(3, 3, 3, 1, *J2, 3, *inv_J1, 3, 0, *tem1, 3);
		s_dgemm(3, 3, 3, 1, *tem1, 3, *dJ1_y, 3, 0, *tem2, 3);
		s_dgemm(3, 3, 3, -1, *tem2, 3, *inv_J1, 3, 1, dJi_y_L, 3);

		s_dgemm(3, 3, 3, 1, *dJ2_z, 3, *inv_J1, 3, 0, dJi_z_L, 3);
		s_dgemm(3, 3, 3, 1, *J2, 3, *inv_J1, 3, 0, *tem1, 3);
		s_dgemm(3, 3, 3, 1, *tem1, 3, *dJ1_z, 3, 0, *tem2, 3);
		s_dgemm(3, 3, 3, -1, *tem2, 3, *inv_J1, 3, 1, dJi_z_L, 3);


		switch (*relativeCoordinate)
		{
		case 'L':
			std::copy_n(dJi_x_L, 9, dJi_x);
			std::copy_n(dJi_y_L, 9, dJi_y);
			std::copy_n(dJi_z_L, 9, dJi_z);
			break;
		case 'M':
		case 'B':
			std::fill_n(dJi_x, 9, 0);
			std::fill_n(dJi_y, 9, 0);
			std::fill_n(dJi_z, 9, 0);

			s_daxpy(9, base().prtPm()[0][0], dJi_x_L, 1, dJi_x, 1);
			s_daxpy(9, base().prtPm()[0][1], dJi_y_L, 1, dJi_x, 1);
			s_daxpy(9, base().prtPm()[0][2], dJi_z_L, 1, dJi_x, 1);

			s_daxpy(9, base().prtPm()[1][0], dJi_x_L, 1, dJi_y, 1);
			s_daxpy(9, base().prtPm()[1][1], dJi_y_L, 1, dJi_y, 1);
			s_daxpy(9, base().prtPm()[1][2], dJi_z_L, 1, dJi_y, 1);

			s_daxpy(9, base().prtPm()[2][0], dJi_x_L, 1, dJi_z, 1);
			s_daxpy(9, base().prtPm()[2][1], dJi_y_L, 1, dJi_z, 1);
			s_daxpy(9, base().prtPm()[2][2], dJi_z_L, 1, dJi_z, 1);

			break;
		case 'G':
		case 'O':
		default:
			std::fill_n(dJi_x, 9, 0);
			std::fill_n(dJi_y, 9, 0);
			std::fill_n(dJi_z, 9, 0);

			dsp(*base().pm(), 4, 4);

			const double *a = &(base().pm()[0][0]);

			s_daxpy(9, base().pm()[0][0], dJi_x_L, 1, dJi_x, 1);
			s_daxpy(9, base().pm()[0][1], dJi_y_L, 1, dJi_x, 1);
			s_daxpy(9, base().pm()[0][2], dJi_z_L, 1, dJi_x, 1);

			s_daxpy(9, base().pm()[1][0], dJi_x_L, 1, dJi_y, 1);
			s_daxpy(9, base().pm()[1][1], dJi_y_L, 1, dJi_y, 1);
			s_daxpy(9, base().pm()[1][2], dJi_z_L, 1, dJi_y, 1);

			s_daxpy(9, base().pm()[2][0], dJi_x_L, 1, dJi_z, 1);
			s_daxpy(9, base().pm()[2][1], dJi_y_L, 1, dJi_z, 1);
			s_daxpy(9, base().pm()[2][2], dJi_z_L, 1, dJi_z, 1);

			//s_daxpy(9, *base().pm()[0], dJi_x_L, 1, dJi_x, 1);
			//s_daxpy(9, *base().pm()[4], dJi_y_L, 1, dJi_x, 1);
			//s_daxpy(9, *base().pm()[8], dJi_z_L, 1, dJi_x, 1);

			//s_daxpy(9, *base().pm()[1], dJi_x_L, 1, dJi_y, 1);
			//s_daxpy(9, *base().pm()[5], dJi_y_L, 1, dJi_y, 1);
			//s_daxpy(9, *base().pm()[9], dJi_z_L, 1, dJi_y, 1);

			//s_daxpy(9, *base().pm()[2], dJi_x_L, 1, dJi_z, 1);
			//s_daxpy(9, *base().pm()[6], dJi_y_L, 1, dJi_z, 1);
			//s_daxpy(9, *base().pm()[10], dJi_z_L, 1, dJi_z, 1);

			break;
		}

		double dJi[9]{0};
		double vEE[3],vEE_L[3];


		/*please resume this after new coordinate system*/
		//this->GetVee(vEE_L, "L");
		/*end*/
		
		
		//s_pm_dot_v3(*base().pm(), vEE_L, vEE);

		s_daxpy(9, vEE[0], dJi_x, 1, dJi, 1);
		s_daxpy(9, vEE[1], dJi_y, 1, dJi, 1);
		s_daxpy(9, vEE[2], dJi_z, 1, dJi, 1);

		double jac_c[3]{0};
		s_dgemm(3, 1, 3, 1, dJi, 3, vEE, 1, 0, jac_c, 1);

		//dsp(jac_c, 3, 1);
		//dsp(_c_acc_inv, 3, 1);

		//double c[3];
		//this->GetAccJacInv(nullptr,c,"G");
		//dsp(c,3,1);
	}

	void LegI::GetFin(double *fIn) const
	{
		fIn[0] = f1_dyn + f1_frc;
		fIn[1] = f2_dyn + f2_frc;
		fIn[2] = f3_dyn + f3_frc;
	}
	void LegI::GetFinDyn(double *fIn) const
	{
		fIn[0] = f1_dyn;
		fIn[1] = f2_dyn;
		fIn[2] = f3_dyn;
	}
	void LegI::GetFinFrc(double *fIn) const
	{
		fIn[0] = f1_frc;
		fIn[1] = f2_frc;
		fIn[2] = f3_frc;
	}

	void LegI::calculate_from_pEE()
	{
		_CalCdByPos();
		_CalVarByCd();
		_CalPartByVar();
	}
	void LegI::calculate_from_pIn()
	{
		_CalCdByPlen();
		_CalVarByCd();
		_CalPartByVar();
	}
	void LegI::calculate_from_vEE()
	{
		_CalVcdByVpos();
		_CalVvarByVcd();
		_CalVpartByVvar();
	}
	void LegI::calculate_from_vIn()
	{
		_CalVcdByVplen();
		_CalVvarByVcd();
		_CalVpartByVvar();
	}
	void LegI::calculate_from_aEE()
	{
		_CalAcdByApos();
		_CalAvarByAcd();
		_CalApartByAvar();
	}
	void LegI::calculate_from_aIn()
	{
		_CalAcdByAplen();
		_CalAvarByAcd();
		_CalApartByAvar();
	}
	void LegI::calculate_jac()
	{
		Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > J1_m(*J1);
		Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > J2_m(*J2);
		Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > inv_J1_m(*inv_J1);
		Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > inv_J2_m(*inv_J2);
		
		inv_J1_m = J1_m.inverse();
		inv_J2_m = J2_m.inverse();

		Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > Jvd_m(*Jvd);
		Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > Jvi_m(*Jvi);

		Jvd_m = J1_m*inv_J2_m;
		Jvi_m = J2_m*inv_J1_m;
	}
	void LegI::calculate_diff_jac()
	{
		Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > J1_m(*J1), J2_m(*J2), vJ1_m(*vJ1), vJ2_m(*vJ2);
		Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > inv_J1_m(*inv_J1), inv_J2_m(*inv_J2);
		Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > Jvd_m(*Jvd), Jvi_m(*Jvi);
		Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > vJvd_m(*vJvd), vJvi_m(*vJvi);

		vJvd_m = vJ1_m * inv_J2_m - Jvd_m * vJ2_m * inv_J2_m;
		vJvi_m = vJ2_m * inv_J1_m - Jvi_m * vJ1_m * inv_J1_m;
		
		Eigen::Map<Eigen::Matrix<double, 3, 1> > cd_m(_c_acc_dir), ci_m(_c_acc_inv);
		Eigen::Map<Eigen::Matrix<double, 3, 1> > vEE_m(this->vEE), vIn_m(this->vIn);

		cd_m = vJvd_m*vIn_m;
		ci_m = vJvi_m*vEE_m;

		/*
		if (is_debug)
			dsp(*vJvi, 3, 3);*/
	}

	void LegI::_CalCdByPos()
	{
		l1 = sqrt(x*x + y*y + z*z - Sfy*Sfy - Sfz*Sfz) - Sfx;
		b1 = asin(y / sqrt((l1 + Sfx)*(l1 + Sfx) + Sfy*Sfy)) - asin(Sfy / sqrt((l1 + Sfx)*(l1 + Sfx) + Sfy*Sfy));
		a1 = atan2(Sfz*x - ((l1 + Sfx)*cos(b1) - Sfy*sin(b1))*z, ((l1 + Sfx)*cos(b1) - Sfy*sin(b1))*x + Sfz*z);
	}
	void LegI::_CalCdByPlen()
	{
		std::complex<double>  M, N;
		std::complex<double>  K1, K2, K3;
		std::complex<double>  p1, p2, p3, p4, p5, p6, p7, p8, p9, p10;
		std::complex<double>  A, B, C;

		double  X;

		M = (2 * (l1*l1 + H1*H1 + D1*D1 + H2*H2 + D2*D2) - l2*l2 - l3*l3) / 4;
		N = (l2*l2 - l3*l3) / 4;

		K1 = -2.0*M / (H1*sqrt(l1*l1 + H2*H2));
		K2 = (M*M - D1*D1*D2*D2) / (H1*H1*(l1*l1 + H2*H2)) - 1.0;
		K3 = -(M*M - D1*D1*D2*D2) / (H1*H1*(l1*l1 + H2*H2)) - D2*D2*N*N / (H1*H1*(l1*l1 + H2*H2)*(l1*l1 + H2*H2));

		//方程为：
		//x^4+K1*x^3+K2*x^2-K1*x+K3=0
		//solve 4th equation
		p1 = K1 + (K1*K2) / 2.0 - K1*K1*K1 / 8.0;
		p2 = p1*p1;
		p3 = K2 - 3.0*K1*K1 / 8.0;
		p4 = K3 + K1*K1*K2 / 16.0 + K1*K1 / 4.0 - (3.0*K1*K1*K1*K1) / 256.0;
		p5 = sqrt(4.0*p2*p3*p3*p3 - 16.0*p3*p3*p3*p3*p4 + 27.0*p2*p2 + 128.0*p3*p3*p4*p4 - 256.0*p4*p4*p4 - 144.0*p2*p3*p4);
		p6 = p3*p3*p3 / 27.0 + sqrt(3.0)*p5 / 18.0 + p2 / 2.0 - (4.0*p3*p4) / 3.0;
		p7 = pow(p6, 1.0 / 3.0);
		p8 = sqrt(9.0*p7*p7 + p3*p3 - 6.0*p3*p7 + 12.0*p4);
		p9 = 54.0*p1*sqrt(p6);
		p10 = -p3*p3*p8 - 12.0*p4*p8 - 12.0*p3*p7*p8 - 9.0*p7*p7*p8;

		A = p8 / (6.0*sqrt(p7));
		B = sqrt(p9 + p10) / (6.0*sqrt(p7*p8));
		C = sqrt(-p9 + p10) / (6.0*sqrt(p7*p8));

		//方程的四个根为：
		//root1 = -K1/4.0 + A + B;
		//root2 = -K1/4.0 + A - B;
		//root3 = -K1/4.0 - A + C;
		//root4 = -K1/4.0 - A - C;

		X = (-K1 / 4.0 - A + C).real();
		//solve finished

		b1 = asin(X) - atan(H2 / l1);
		a1 = asin(N.real() / (D1*(l1*cos(b1) - H2*sin(b1))));
	}
	void LegI::_CalCdByPlen2()
	{
		double T1, T2, F1, F2, dq1, dq2, Ja, Jb, Jc, Jd;
		int i;

		a1 = 0; b1 = 0;

		T1 = (U2x*U2x + U2y*U2y + U2z*U2z + (l1 + S2x)*(l1 + S2x) + S2y*S2y + S2z*S2z - l2*l2) / 2;
		T2 = (U3x*U3x + U3y*U3y + U3z*U3z + (l1 + S3x)*(l1 + S3x) + S3y*S3y + S3z*S3z - l3*l3) / 2;

		for (i = 0; i < 10; ++i)
		{
			sa1 = sin(a1);
			ca1 = cos(a1);
			sb1 = sin(b1);
			cb1 = cos(b1);

			Ja = -U2x*(S2x + l1)*sa1*cb1 + U2x*S2y*sa1*sb1 + U2x*S2z*ca1 - U2z*(S2x + l1)*ca1*cb1 + U2z*S2y*ca1*sb1 - U2z*S2z*sa1;
			Jb = -U2x*(S2x + l1)*ca1*sb1 - U2x*S2y*ca1*cb1 + U2y*(S2x + l1)*cb1 - U2y*S2y*sb1 + U2z*(S2x + l1)*sa1*sb1 + U2z*S2y*sa1*cb1;
			Jc = -U3x*(S3x + l1)*sa1*cb1 + U3x*S3y*sa1*sb1 + U3x*S3z*ca1 - U3z*(S3x + l1)*ca1*cb1 + U3z*S3y*ca1*sb1 - U3z*S3z*sa1;
			Jd = -U3x*(S3x + l1)*ca1*sb1 - U3x*S3y*ca1*cb1 + U3y*(S3x + l1)*cb1 - U3y*S3y*sb1 + U3z*(S3x + l1)*sa1*sb1 + U3z*S3y*sa1*cb1;

			F1 = U2x*(S2x + l1)*ca1*cb1 - U2x*S2y*ca1*sb1 + U2x*S2z*sa1
				+ U2y*(S2x + l1)*sb1 + U2y*S2y*cb1
				- U2z*(S2x + l1)*sa1*cb1 + U2z*S2y*sa1*sb1 + U2z*S2z*ca1;
			F2 = U3x*(S3x + l1)*ca1*cb1 - U3x*S3y*ca1*sb1 + U3x*S3z*sa1
				+ U3y*(S3x + l1)*sb1 + U3y*S3y*cb1
				- U3z*(S3x + l1)*sa1*cb1 + U3z*S3y*sa1*sb1 + U3z*S3z*ca1;

			dq1 = (Jd*(T1 - F1) - Jb*(T2 - F2)) / (Ja*Jd - Jb*Jc);
			dq2 = (-Jc*(T1 - F1) + Ja*(T2 - F2)) / (Ja*Jd - Jb*Jc);

			a1 = a1 + dq1;
			b1 = b1 + dq2;
		}

		this->l1 = l1;

	}
	void LegI::_CalVarByCd()
	{
		sa1 = sin(a1);
		ca1 = cos(a1);
		sb1 = sin(b1);
		cb1 = cos(b1);

		x = (l1 + Sfx)*ca1*cb1 - Sfy*ca1*sb1 + Sfz*sa1;
		y = (l1 + Sfx)*sb1 + Sfy*cb1 + 0;
		z = -(l1 + Sfx)*sa1*cb1 + Sfy*sa1*sb1 + Sfz*ca1;

		x2 = (l1 + S2x)*ca1*cb1 - S2y*ca1*sb1 + S2z*sa1 - U2x;
		y2 = (l1 + S2x)*sb1 + S2y*cb1 + 0 - U2y;
		z2 = -(l1 + S2x)*sa1*cb1 + S2y*sa1*sb1 + S2z*ca1 - U2z;

		x3 = (l1 + S3x)*ca1*cb1 - S3y*ca1*sb1 + S3z*sa1 - U3x;
		y3 = (l1 + S3x)*sb1 + S3y*cb1 + 0 - U3y;
		z3 = -(l1 + S3x)*sa1*cb1 + S3y*sa1*sb1 + S3z*ca1 - U3z;

		l2 = sqrt(x2*x2 + y2*y2 + z2*z2);
		b2 = asin(y2 / l2);
		a2 = -atan(z2 / x2);

		l3 = sqrt(x3*x3 + y3*y3 + z3*z3);
		b3 = asin(y3 / l3);
		a3 = -atan(z3 / x3);

		sa2 = sin(a2);
		ca2 = cos(a2);
		sb2 = sin(b2);
		cb2 = cos(b2);
		sa3 = sin(a3);
		ca3 = cos(a3);
		sb3 = sin(b3);
		cb3 = cos(b3);

		J1[0][0] = z;
		J1[0][1] = -y*ca1;
		J1[0][2] = ca1*cb1;
		J1[1][0] = 0;
		J1[1][1] = x*ca1 - z*sa1;
		J1[1][2] = sb1;
		J1[2][0] = -x;
		J1[2][1] = y*sa1;
		J1[2][2] = -sa1*cb1;

		k21 = -U2x*(S2x + l1)*sa1*cb1 + U2x*S2y*sa1*sb1 + U2x*S2z*ca1 - U2z*(S2x + l1)*ca1*cb1 + U2z*S2y*ca1*sb1 - U2z*S2z*sa1;
		k22 = -U2x*(S2x + l1)*ca1*sb1 - U2x*S2y*ca1*cb1 + U2y*(S2x + l1)*cb1 - U2y*S2y*sb1 + U2z*(S2x + l1)*sa1*sb1 + U2z*S2y*sa1*cb1;
		k23 = U2x*ca1*cb1 + U2y*sb1 - U2z*sa1*cb1 - (l1 + S2x);

		k31 = -U3x*(S3x + l1)*sa1*cb1 + U3x*S3y*sa1*sb1 + U3x*S3z*ca1 - U3z*(S3x + l1)*ca1*cb1 + U3z*S3y*ca1*sb1 - U3z*S3z*sa1;
		k32 = -U3x*(S3x + l1)*ca1*sb1 - U3x*S3y*ca1*cb1 + U3y*(S3x + l1)*cb1 - U3y*S3y*sb1 + U3z*(S3x + l1)*sa1*sb1 + U3z*S3y*sa1*cb1;
		k33 = U3x*ca1*cb1 + U3y*sb1 - U3z*sa1*cb1 - (l1 + S3x);

		J2[0][0] = 0;
		J2[0][1] = 0;
		J2[0][2] = 1;
		J2[1][0] = -k21 / l2;
		J2[1][1] = -k22 / l2;
		J2[1][2] = -k23 / l2;
		J2[2][0] = -k31 / l3;
		J2[2][1] = -k32 / l3;
		J2[2][2] = -k33 / l3;
	}
	void LegI::_CalPartByVar()
	{
		double pm[4][4], pm1[4][4];

		base().update();

		double pe[6] = { 0, 0, 0, PI / 2, a1, -PI / 2 + b1 };
		s_pe2pm(pe, *pm);
		s_pm_dot_pm(*base().pm(), *pm, *pm1);
		p1a().setPm(*pm1);

		double pe1[6] = { l1, 0, 0, 0, 0, 0 };
		s_pe2pm(pe1, *pm);
		s_pm_dot_pm(*p1a().pm(), *pm, *pm1);
		thigh().setPm(*pm1);

		double pe2[6] = { U2x, U2y, U2z, PI / 2, a2, -PI / 2 + b2 };
		s_pe2pm(pe2, *pm);
		s_pm_dot_pm(*base().pm(), *pm, *pm1);
		p2a().setPm(*pm1);

		double pe3[6] = { l2, 0, 0, 0, 0, 0 };
		s_pe2pm(pe3, *pm);
		s_pm_dot_pm(*p2a().pm(), *pm, *pm1);
		p2b().setPm(*pm1);

		double pe4[6] = { U3x, U3y, U3z, PI / 2, a3, -PI / 2 + b3 };
		s_pe2pm(pe4, *pm);
		s_pm_dot_pm(*base().pm(), *pm, *pm1);
		p3a().setPm(*pm1);

		double pe5[6] = { l3, 0, 0, 0, 0, 0 };
		s_pe2pm(pe5, *pm);
		s_pm_dot_pm(*p3a().pm(), *pm, *pm1);
		p3b().setPm(*pm1);

		/*驱动位置*/
		m1().update();
		m2().update();
		m3().update();

		/*地面球铰的位置*/
		sfi().update();
		std::copy_n((const double*)*sfi().pm(), 16, const_cast<double *>(*sfj().prtPm()));
	}
	void LegI::_CalVcdByVpos()
	{
		vl1 = (x*vx + y*vy + z*vz) / (l1 + Sfx);
		vb1 = (vy - vl1*sb1) / (x*ca1 - z*sa1);
		va1 = (vx*sa1 + vz*ca1) / (-x*ca1 + z*sa1);
	}
	void LegI::_CalVcdByVplen()
	{
		double K1, K2;

		K1 = -l2*vl2 - k23*vl1;
		K2 = -l3*vl3 - k33*vl1;

		va1 = (k32*K1 - k22*K2) / (k21*k32 - k22*k31);
		vb1 = (-k31*K1 + k21*K2) / (k21*k32 - k22*k31);
		this->vl1 = vl1;
	}
	void LegI::_CalVvarByVcd()
	{
		vx = z*va1 - y*ca1*vb1 + ca1*cb1*vl1;
		vy = 0 + (x*ca1 - z*sa1)*vb1 + sb1*vl1;
		vz = -x*va1 + y*sa1*vb1 - sa1*cb1*vl1;

		vx2 = (z2 + U2z)*va1 - (y2 + U2y)*ca1*vb1 + ca1*cb1*vl1;
		vy2 = 0 + ((x2 + U2x)*ca1 - (z2 + U2z)*sa1)*vb1 + sb1*vl1;
		vz2 = -(x2 + U2x)*va1 + (y2 + U2y)*sa1*vb1 - sa1*cb1*vl1;

		vl2 = (x2*vx2 + y2*vy2 + z2*vz2) / l2;
		vb2 = (vy2 - vl2*sb2) / (l2*cb2);
		va2 = (vx2*sa2 + vz2*ca2) / (-x2*ca2 + z2*sa2);

		vx3 = (z3 + U3z)*va1 - (y3 + U3y)*ca1*vb1 + ca1*cb1*vl1;
		vy3 = 0 + ((x3 + U3x)*ca1 - (z3 + U3z)*sa1)*vb1 + sb1*vl1;
		vz3 = -(x3 + U3x)*va1 + (y3 + U3y)*sa1*vb1 - sa1*cb1*vl1;

		vl3 = (x3*vx3 + y3*vy3 + z3*vz3) / l3;
		vb3 = (vy3 - vl3*sb3) / (l3*cb3);
		va3 = (vx3*sa3 + vz3*ca3) / (-x3*ca3 + z3*sa3);

		pa1 = ca1*va1;
		qa1 = -sa1*va1;
		pb1 = cb1*vb1;
		qb1 = -sb1*vb1;
		pa2 = ca2*va2;
		qa2 = -sa2*va2;
		pb2 = cb2*vb2;
		qb2 = -sb2*vb2;
		pa3 = ca3*va3;
		qa3 = -sa3*va3;
		pb3 = cb3*vb3;
		qb3 = -sb3*vb3;

		H11 = sa1*pb1 + pa1*sb1;
		H12 = sa1*qb1 + pa1*cb1;
		H21 = ca1*pb1 + qa1*sb1;
		H22 = ca1*qb1 + qa1*cb1;

		vk21 = -U2x*(S2x + l1)*H12 + U2x*S2y*H11 + U2x*S2z*qa1
			- U2z*(S2x + l1)*H22 + U2z*S2y*H21 - U2z*S2z*pa1
			- U2x*sa1*cb1*vl1 - U2z*ca1*cb1*vl1;
		vk22 = -U2x*(S2x + l1)*H21 - U2x*S2y*H22
			+ U2y*(S2x + l1)*qb1 - U2y*S2y*pb1
			+ U2z*(S2x + l1)*H11 + U2z*S2y*H12
			- U2x*ca1*sb1*vl1 + U2y*cb1*vl1 + U2z*sa1*sb1*vl1;
		vk23 = U2x*H22 + U2y*pb1 - U2z*H12 - vl1;
		vk31 = -U3x*(S3x + l1)*H12 + U3x*S3y*H11 + U3x*S3z*qa1
			- U3z*(S3x + l1)*H22 + U3z*S3y*H21 - U3z*S3z*pa1
			- U3x*sa1*cb1*vl1 - U3z*ca1*cb1*vl1;
		vk32 = -U3x*(S3x + l1)*H21 - U3x*S3y*H22
			+ U3y*(S3x + l1)*qb1 - U3y*S3y*pb1
			+ U3z*(S3x + l1)*H11 + U3z*S3y*H12
			- U3x*ca1*sb1*vl1 + U3y*cb1*vl1 + U3z*sa1*sb1*vl1;
		vk33 = U3x*H22 + U3y*pb1 - U3z*H12 - vl1;

		vJ1[0][0] = vz;
		vJ1[0][1] = -vy*ca1 - y*qa1;
		vJ1[0][2] = H22;
		vJ1[1][0] = 0;
		vJ1[1][1] = vx*ca1 + x*qa1 - vz*sa1 - z*pa1;
		vJ1[1][2] = pb1;
		vJ1[2][0] = -vx;
		vJ1[2][1] = vy*sa1 + y*pa1;
		vJ1[2][2] = -H12;

		vJ2[0][0] = 0;
		vJ2[0][1] = 0;
		vJ2[0][2] = 0;
		vJ2[1][0] = -(vk21*l2 - k21*vl2) / (l2*l2);
		vJ2[1][1] = -(vk22*l2 - k22*vl2) / (l2*l2);
		vJ2[1][2] = -(vk23*l2 - k23*vl2) / (l2*l2);
		vJ2[2][0] = -(vk31*l3 - k31*vl3) / (l3*l3);
		vJ2[2][1] = -(vk32*l3 - k32*vl3) / (l3*l3);
		vJ2[2][2] = -(vk33*l3 - k33*vl3) / (l3*l3);

	}
	void LegI::_CalVpartByVvar()
	{
		double v_G[6], v_L[6];

		/*p1a*/
		v_L[0] = 0;
		v_L[1] = 0;
		v_L[2] = 0;
		v_L[3] = sa1*vb1;
		v_L[4] = va1;
		v_L[5] = ca1*vb1;

		s_v2v(*base().pm(), base().vel(), v_L, v_G);
		p1a().setVel(v_G);

		/*thigh*/
		v_L[0] += ca1*cb1*vl1;
		v_L[1] += sb1*vl1;
		v_L[2] += -sa1*cb1*vl1;

		s_v2v(*base().pm(), base().vel(), v_L, v_G);
		thigh().setVel(v_G);

		/*p2a*/
		v_L[0] = -U2z*va2 + U2y*ca2*vb2;
		v_L[1] = +U2z*sa2*vb2 - U2x*ca2*vb2;
		v_L[2] = -U2y*sa2*vb2 + U2x*va2;
		v_L[3] = sa2*vb2;
		v_L[4] = va2;
		v_L[5] = ca2*vb2;

		s_v2v(*base().pm(), base().vel(), v_L, v_G);
		p2a().setVel(v_G);

		/*p2b*/
		v_L[0] += ca2*cb2*vl2;
		v_L[1] += sb2*vl2;
		v_L[2] += -sa2*cb2*vl2;

		s_v2v(*base().pm(), base().vel(), v_L, v_G);
		p2b().setVel(v_G);

		/*p3a*/
		v_L[0] = -U3z*va3 + U3y*ca3*vb3;
		v_L[1] = +U3z*sa3*vb3 - U3x*ca3*vb3;
		v_L[2] = -U3y*sa3*vb3 + U3x*va3;
		v_L[3] = sa3*vb3;
		v_L[4] = va3;
		v_L[5] = ca3*vb3;

		s_v2v(*base().pm(), base().vel(), v_L, v_G);
		p3a().setVel(v_G);

		/*p3b*/
		v_L[0] += ca3*cb3*vl3;
		v_L[1] += sb3*vl3;
		v_L[2] += -sa3*cb3*vl3;

		s_v2v(*base().pm(), base().vel(), v_L, v_G);
		p3b().setVel(v_G);
	}
	void LegI::_CalAcdByApos()
	{
		double xd, yd, zd;

		xd = ax - vJ1[0][0] * va1 - vJ1[0][1] * vb1 - vJ1[0][2] * vl1;
		yd = ay - vJ1[1][0] * va1 - vJ1[1][1] * vb1 - vJ1[1][2] * vl1;
		zd = az - vJ1[2][0] * va1 - vJ1[2][1] * vb1 - vJ1[2][2] * vl1;

		al1 = (x*xd + y*yd + z*zd) / (l1 + Sfx);
		ab1 = (yd - al1*sb1) / (x*ca1 - z*sa1);
		aa1 = (xd*sa1 + zd*ca1) / (-x*ca1 + z*sa1);
	}
	void LegI::_CalAcdByAplen()
	{
		double vK1, vK2, M1, M2;

		vK1 = -vl2*vl2 - l2*al2 - vk23*vl1 - k23*al1;
		vK2 = -vl3*vl3 - l3*al3 - vk33*vl1 - k33*al1;

		M1 = vK1 - vk21*va1 - vk22*vb1;
		M2 = vK2 - vk31*va1 - vk32*vb1;

		aa1 = (k32*M1 - k22*M2) / (k21*k32 - k22*k31);
		ab1 = (-k31*M1 + k21*M2) / (k21*k32 - k22*k31);
	}
	void LegI::_CalAvarByAcd()
	{
		ax = z*aa1 - y*ca1*ab1 + ca1*cb1*al1
			+ vz*va1 + (-vy*ca1 - y*qa1)*vb1 + H22*vl1;
		ay = (x*ca1 - z*sa1)*ab1 + sb1*al1
			+ (vx*ca1 + x*qa1 - vz*sa1 - z*pa1)*vb1 + pb1*vl1;
		az = -x*aa1 + y*sa1*ab1 - sa1*cb1*al1
			- vx*va1 + vy*sa1*vb1 + y*pa1*vb1 - H12*vl1;

		ax2 = (z2 + U2z)*aa1 - (y2 + U2y)*ca1*ab1 + ca1*cb1*al1
			+ vz2*va1 + (-vy2*ca1 - (y2 + U2y)*qa1)*vb1 + H22*vl1;
		ay2 = ((x2 + U2x)*ca1 - (z2 + U2z)*sa1)*ab1 + sb1*al1
			+ (vx2*ca1 + (x2 + U2x)*qa1 - vz2*sa1 - (z2 + U2z)*pa1)*vb1 + pb1*vl1;
		az2 = (-x2 - U2x)*aa1 + (y2 + U2y)*sa1*ab1 - sa1*cb1*al1
			- vx2*va1 + vy2*sa1*vb1 + (y2 + U2y)*pa1*vb1 - H12*vl1;

		al2 = (x2*ax2 + vx2*vx2 + y2*ay2 + vy2*vy2 + z2*az2 + vz2*vz2 - vl2*vl2)
			/ l2;
		ab2 = (ay2 - pb2*vl2 - sb2*al2 - vl2*pb2 - l2*qb2*vb2)
			/ (l2*cb2);
		aa2 = (pa2*vx2 + sa2*ax2 + ca2*az2 + qa2*vz2 - (-vx2*ca2 - x2*qa2 + vz2*sa2 + z2*pa2)*va2)
			/ (-x2*ca2 + z2*sa2);

		ax3 = (z3 + U3z)*aa1 - (y3 + U3y)*ca1*ab1 + ca1*cb1*al1
			+ vz3*va1 + (-vy3*ca1 - (y3 + U3y)*qa1)*vb1 + H22*vl1;
		ay3 = ((x3 + U3x)*ca1 - (z3 + U3z)*sa1)*ab1 + sb1*al1
			+ (vx3*ca1 + (x3 + U3x)*qa1 - vz3*sa1 - (z3 + U3z)*pa1)*vb1 + pb1*vl1;
		az3 = (-x3 - U3x)*aa1 + (y3 + U3y)*sa1*ab1 - sa1*cb1*al1
			- vx3*va1 + vy3*sa1*vb1 + (y3 + U3y)*pa1*vb1 - H12*vl1;

		al3 = (x3*ax3 + vx3*vx3 + y3*ay3 + vy3*vy3 + z3*az3 + vz3*vz3 - vl3*vl3)
			/ l3;
		ab3 = (ay3 - pb3*vl3 - sb3*al3 - vl3*pb3 - l3*qb3*vb3)
			/ (l3*cb3);
		aa3 = (pa3*vx3 + sa3*ax3 + ca3*az3 + qa3*vz3 - (-vx3*ca3 - x3*qa3 + vz3*sa3 + z3*pa3)*va3)
			/ (-x3*ca3 + z3*sa3);
	}
	void LegI::_CalApartByAvar()
	{
		double a_L[6], a_G[6], v_G[6], v_L[6];

		/* p1a */
		v_L[0] = 0;
		v_L[1] = 0;
		v_L[2] = 0;
		v_L[3] = sa1*vb1;
		v_L[4] = va1;
		v_L[5] = ca1*vb1;

		a_L[0] = 0;
		a_L[1] = 0;
		a_L[2] = 0;
		a_L[3] = pa1*vb1 + sa1*ab1;
		a_L[4] = aa1;
		a_L[5] = qa1*vb1 + ca1*ab1;

		s_a2a(*base().pm(), base().vel(), base().acc(), v_L, a_L, a_G, v_G);
		p1a().setAcc(a_G);

		/* Thigh */
		v_L[0] += ca1*cb1*vl1;
		v_L[1] += sb1*vl1;
		v_L[2] += -sa1*cb1*vl1;

		a_L[0] = H22*vl1 + ca1*cb1*al1;
		a_L[1] = cb1*vb1*vl1 + sb1*al1;
		a_L[2] = -H12*vl1 - sa1*cb1*al1;

		s_a2a(*base().pm(), base().vel(), base().acc(), v_L, a_L, a_G, v_G);

		thigh().setAcc(a_G);

		/* P2a */
		v_L[0] = -U2z*va2 + U2y*ca2*vb2;
		v_L[1] = +U2z*sa2*vb2 - U2x*ca2*vb2;
		v_L[2] = -U2y*sa2*vb2 + U2x*va2;
		v_L[3] = sa2*vb2;
		v_L[4] = va2;
		v_L[5] = ca2*vb2;

		a_L[0] = -U2z*aa2 + U2y*qa2*vb2 + U2y*ca2*ab2;
		a_L[1] = U2z*pa2*vb2 + U2z*sa2*ab2 - U2x*qa2*vb2 - U2x*ca2*ab2;
		a_L[2] = -U2y*pa2*vb2 - U2y*sa2*ab2 + U2x*aa2;
		a_L[3] = pa2*vb2 + sa2*ab2;
		a_L[4] = aa2;
		a_L[5] = qa2*vb2 + ca2*ab2;

		s_a2a(*base().pm(), base().vel(), base().acc(), v_L, a_L, a_G, v_G);
		p2a().setAcc(a_G);
		/* P2b */
		v_L[0] += ca2*cb2*vl2;
		v_L[1] += sb2*vl2;
		v_L[2] += -sa2*cb2*vl2;

		a_L[0] += qa2*cb2*vl2 + ca2*qb2*vl2 + ca2*cb2*al2;
		a_L[1] += pb2*vl2 + sb2*al2;
		a_L[2] += -pa2*cb2*vl2 - sa2*qb2*vl2 - sa2*cb2*al2;

		s_a2a(*base().pm(), base().vel(), base().acc(), v_L, a_L, a_G, v_G);
		p2b().setAcc(a_G);
		/* P3a */
		v_L[0] = -U3z*va3 + U3y*ca3*vb3;
		v_L[1] = +U3z*sa3*vb3 - U3x*ca3*vb3;
		v_L[2] = -U3y*sa3*vb3 + U3x*va3;
		v_L[3] = sa3*vb3;
		v_L[4] = va3;
		v_L[5] = ca3*vb3;

		a_L[0] = -U3z*aa3 + U3y*qa3*vb3 + U3y*ca3*ab3;
		a_L[1] = U3z*pa3*vb3 + U3z*sa3*ab3 - U3x*qa3*vb3 - U3x*ca3*ab3;
		a_L[2] = -U3y*pa3*vb3 - U3y*sa3*ab3 + U3x*aa3;
		a_L[3] = pa3*vb3 + sa3*ab3;
		a_L[4] = aa3;
		a_L[5] = qa3*vb3 + ca3*ab3;

		s_a2a(*base().pm(), base().vel(), base().acc(), v_L, a_L, a_G, v_G);
		p3a().setAcc(a_G);
		/* P3b */
		v_L[0] += ca3*cb3*vl3;
		v_L[1] += sb3*vl3;
		v_L[2] += -sa3*cb3*vl3;

		a_L[0] += qa3*cb3*vl3 + ca3*qb3*vl3 + ca3*cb3*al3;
		a_L[1] += pb3*vl3 + sb3*al3;
		a_L[2] += -pa3*cb3*vl3 - sa3*qb3*vl3 - sa3*cb3*al3;

		s_a2a(*base().pm(), base().vel(), base().acc(), v_L, a_L, a_G, v_G);
		p3b().setAcc(a_G);

		m1().setMotAcc(al1);
		m2().setMotAcc(al2);
		m3().setMotAcc(al3);
	}

	void LegI::FastDyn()
	{
		double rcond = 0.0000001;

		/*初始化*/
		std::fill_n(&_C[0][0], 36 * 36, 0);
		std::fill_n(&_c_M[0][0], 36 * 4, 0);

		/*计算C*/
		/*
		/////////U1    U2    U3     P1       P2      P3      S2      S3      M1      M2      M3
		P1a      0*0                -0*12                                  -0*33
		P2a            6*4                  -6*17                                  -6*34
		P3a                  12*8                  -12*22                                  -12*35
		Thigh                      18*12                    18*27   18*30  18*33
		P2b                                24*17           -24*27                   24*34
		P3b                                         30*22          -30*30                   30*35
		*/

		/*更新每个元素*/
		pRobot->body().update();

		for (auto i : prt_array_)
		{
			i->update();
		}
		for (auto i : jnt_array_)
		{
			i->update();
		}
		for (auto i : mot_array_)
		{
			i->update();
		}

		/*复制约束矩阵*/
		s_block_cpy(6, 4, u1().cstMtxJ(), 0, 0, 4, *_C, 0, 0, 36);
		s_block_cpy(6, 4, u2().cstMtxJ(), 0, 0, 4, *_C, 6, 4, 36);
		s_block_cpy(6, 4, u3().cstMtxJ(), 0, 0, 4, *_C, 12, 8, 36);
		s_block_cpy(6, 5, p1().cstMtxI(), 0, 0, 5, *_C, 18, 12, 36);
		s_block_cpy(6, 5, p1().cstMtxJ(), 0, 0, 5, *_C, 0, 12, 36);
		s_block_cpy(6, 5, p2().cstMtxI(), 0, 0, 5, *_C, 24, 17, 36);
		s_block_cpy(6, 5, p2().cstMtxJ(), 0, 0, 5, *_C, 6, 17, 36);
		s_block_cpy(6, 5, p3().cstMtxI(), 0, 0, 5, *_C, 30, 22, 36);
		s_block_cpy(6, 5, p3().cstMtxJ(), 0, 0, 5, *_C, 12, 22, 36);
		s_block_cpy(6, 3, s2().cstMtxI(), 0, 0, 3, *_C, 18, 27, 36);
		s_block_cpy(6, 3, s2().cstMtxJ(), 0, 0, 3, *_C, 24, 27, 36);
		s_block_cpy(6, 3, s3().cstMtxI(), 0, 0, 3, *_C, 18, 30, 36);
		s_block_cpy(6, 3, s3().cstMtxJ(), 0, 0, 3, *_C, 30, 30, 36);

		if (sf().active())
		{
			/*若该腿支撑，则使用Sf副约束*/
			s_block_cpy(6, 3, sf().cstMtxI(), 0, 0, 3, *_C, 18, 33, 36);

			/*更新驱动矩阵M*/
			s_block_cpy(6, 1, m1().cstMtxI(), 0, 0, 1, *_c_M, 18, 1, 4);
			s_block_cpy(6, 1, m1().cstMtxJ(), 0, 0, 1, *_c_M, 0, 1, 4);
			s_block_cpy(6, 1, m2().cstMtxI(), 0, 0, 1, *_c_M, 24, 2, 4);
			s_block_cpy(6, 1, m2().cstMtxJ(), 0, 0, 1, *_c_M, 6, 2, 4);
			s_block_cpy(6, 1, m3().cstMtxI(), 0, 0, 1, *_c_M, 30, 3, 4);
			s_block_cpy(6, 1, m3().cstMtxJ(), 0, 0, 1, *_c_M, 12, 3, 4);
		}
		else
		{
			/*否则，更新驱动的约束矩阵*/
			s_block_cpy(6, 1, m1().cstMtxI(), 0, 0, 1, *_C, 18, 33, 36);
			s_block_cpy(6, 1, m1().cstMtxJ(), 0, 0, 1, *_C, 0, 33, 36);
			s_block_cpy(6, 1, m2().cstMtxI(), 0, 0, 1, *_C, 24, 34, 36);
			s_block_cpy(6, 1, m2().cstMtxJ(), 0, 0, 1, *_C, 6, 34, 36);
			s_block_cpy(6, 1, m3().cstMtxI(), 0, 0, 1, *_C, 30, 35, 36);
			s_block_cpy(6, 1, m3().cstMtxJ(), 0, 0, 1, *_C, 12, 35, 36);
		}

		/*更新右侧的c_M矩阵*/
		s_daxpy(6, -1, p1a().prtFg(), 1, &_c_M[0][0], 4);
		s_daxpy(6, 1, p1a().prtFv(), 1, &_c_M[0][0], 4);
		s_dgemm(6, 1, 6, 1, *p1a().prtIm(), 6, p1a().prtAcc(), 1, 1, &_c_M[0][0], 4);

		s_daxpy(6, -1, p2a().prtFg(), 1, &_c_M[6][0], 4);
		s_daxpy(6, 1, p2a().prtFv(), 1, &_c_M[6][0], 4);
		s_dgemm(6, 1, 6, 1, *p2a().prtIm(), 6, p2a().prtAcc(), 1, 1, &_c_M[6][0], 4);

		s_daxpy(6, -1, p3a().prtFg(), 1, &_c_M[12][0], 4);
		s_daxpy(6, 1, p3a().prtFv(), 1, &_c_M[12][0], 4);
		s_dgemm(6, 1, 6, 1, *p3a().prtIm(), 6, p3a().prtAcc(), 1, 1, &_c_M[12][0], 4);

		s_daxpy(6, -1, thigh().prtFg(), 1, &_c_M[18][0], 4);
		s_daxpy(6, 1, thigh().prtFv(), 1, &_c_M[18][0], 4);
		s_dgemm(6, 1, 6, 1, *thigh().prtIm(), 6, thigh().prtAcc(), 1, 1, &_c_M[18][0], 4);

		s_daxpy(6, -1, p2b().prtFg(), 1, &_c_M[24][0], 4);
		s_daxpy(6, 1, p2b().prtFv(), 1, &_c_M[24][0], 4);
		s_dgemm(6, 1, 6, 1, *p2b().prtIm(), 6, p2b().prtAcc(), 1, 1, &_c_M[24][0], 4);

		s_daxpy(6, -1, p3b().prtFg(), 1, &_c_M[30][0], 4);
		s_daxpy(6, 1, p3b().prtFv(), 1, &_c_M[30][0], 4);
		s_dgemm(6, 1, 6, 1, *p3b().prtIm(), 6, p3b().prtAcc(), 1, 1, &_c_M[30][0], 4);
	}

	RobotTypeI::RobotTypeI(): pLF{ &LF_Leg }, pLM{ &LM_Leg }, pLR{ &LR_Leg }, pRF{ &RF_Leg }, pRM{ &RM_Leg }, pRR{ &RR_Leg }
	{
		for (int i = 0; i < 6; ++i)
		{
			Robots::RobotBase::pLegs[i] = static_cast<Robots::LegBase *>(pLegs[i]);
		}

		this->scriptPool().add<aris::dynamic::Script>("default_script");
	}
	void RobotTypeI::GetFin(double *fIn) const
	{
		for (int i = 0; i < 6; ++i)
		{
			pLegs[i]->GetFin(&fIn[i*3]);
		}
	}
	void RobotTypeI::GetFinDyn(double *fIn) const
	{
		for (int i = 0; i < 6; ++i)
		{
			pLegs[i]->GetFinDyn(&fIn[i * 3]);
		}
	}
	void RobotTypeI::GetFinFrc(double *fIn) const
	{
		for (int i = 0; i < 6; ++i)
		{
			pLegs[i]->GetFinFrc(&fIn[i * 3]);
		}
	}

	void RobotTypeI::SetFixFeet(const char* fixFeet)
	{
		for (int i = 0; i < 6; ++i)
		{
			if (fixFeet[i] == '0')
			{
				pLegs[i]->sf().activate(false);
			}
			else
			{
				pLegs[i]->sf().activate();
			}
		}
	}
	const char* RobotTypeI::FixFeet() const
	{
		thread_local static char fixFeet[7]{ "000000" };

		for (int i = 0; i < 6; ++i)
		{
			if (pLegs[i]->sf().active())
			{
				fixFeet[i] = '1';
			}
			else
			{
				fixFeet[i] = '0';
			}
		}

		return fixFeet;
	}
	void RobotTypeI::SetActiveMotion(const char* activeMotion)
	{
		for (int i = 0; i < 6; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				if (activeMotion[i * 3 + j] == '0')
				{
					pLegs[i]->motionAt(j).activate(false);
					pLegs[i]->forceAt(j).activate();
				}
				else
				{
					pLegs[i]->motionAt(j).activate();
					pLegs[i]->forceAt(j).activate(false);
				}
			}
		}

	}
	const char* RobotTypeI::ActiveMotion() const
	{
		thread_local static char activeMotion[19]{ "000000000000000000" };

		for (int i = 0; i < 6; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				if (pLegs[i]->motionAt(j).active())
				{
					activeMotion[i * 3 + j] = '1';
				}
				else
				{
					activeMotion[i * 3 + j] = '0';
				}
			}
		}

		return activeMotion;
	}
	void RobotTypeI::FastDyn()
	{
		double Cb[6][12]{ 0 };
		double H[6][18]{ 0 };
		double h[18]{ 0 };

		int supported_Leg_Num{ 0 }, supported_id[6]{ 0 };

		body().update();
		
		/*更新h，机身只有重力和惯性力*/
		s_daxpy(6, -1, body().prtFg(), 1, h, 1);
		s_daxpy(6, 1, body().prtFv(), 1, h, 1);
		s_dgemm(6, 1, 6, 1, *body().prtIm(), 6, body().prtAcc(), 1, 1, h, 1);

		/*对每条腿操作*/
		for (int i = 0; i < 6; ++i)
		{
			pLegs[i]->FastDyn();

			/*更新Cb*/
			s_block_cpy(6, 4, pLegs[i]->u1().cstMtxI(), 0, 0, 4, *Cb, 0, 0, 12);
			s_block_cpy(6, 4, pLegs[i]->u2().cstMtxI(), 0, 0, 4, *Cb, 0, 4, 12);
			s_block_cpy(6, 4, pLegs[i]->u3().cstMtxI(), 0, 0, 4, *Cb, 0, 8, 12);

			/*复制C与c_M*/
			if (pLegs[i]->sf().active())
			{
				/*更新支撑腿数量与id*/
				supported_id[i] = supported_Leg_Num;
				supported_Leg_Num++;

				/*计算k_L*/
				Eigen::Map<Eigen::Matrix<double, 36, 36, Eigen::RowMajor>  > A(*pLegs[i]->_C);
				Eigen::Map<Eigen::Matrix<double, 36, 4, Eigen::RowMajor>  > b(*(pLegs[i]->_c_M));
				auto x = A.partialPivLu().solve(b);
				b = x;

				/*更新H，对于机身，只有U副跟腿连接，所以4*3=12列相乘即可*/
				s_dgemm(6, 3, 12, -1, *Cb, 12, &pLegs[i]->_c_M[0][1], 4, 1, &H[0][supported_Leg_Num * 3 - 3], 18);
			}
			else
			{
				/*计算k，计算所有支撑腿的驱动输入力*/
				Eigen::Map<Eigen::Matrix<double, 36, 36, Eigen::RowMajor>   > A(*pLegs[i]->_C);
				Eigen::Map<Eigen::Matrix<double, 36, 1, Eigen::ColMajor>, Eigen::AutoAlign, Eigen::Stride<4*36, 4>  > b(*(pLegs[i]->_c_M));
				auto x = A.partialPivLu().solve(b);
				b = x;
			}

			/*更新h，对于机身，只有U副跟腿连接，所以4*3=12列相乘即可*/
			s_dgemm(6, 1, 12, -1, *Cb, 12, *(pLegs[i]->_c_M), 4, 1, h, 1);
		}

		/*求解支撑腿的驱动力*/
		if (supported_Leg_Num > 0)
		{
			double loc_h[18];
			std::copy_n(h, 18, loc_h);

			Eigen::Map<Eigen::Matrix<double, 6, Eigen::Dynamic, Eigen::RowMajor, 6, 18>, Eigen::AutoAlign, Eigen::Stride<18, 1>> A(*H, 6, supported_Leg_Num * 3);
			Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor, 18, 1>  > b(loc_h, 6);
			Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor, 18, 1>  > x(h, supported_Leg_Num * 3);

			auto svd = A.jacobiSvd(Eigen::ComputeThinU|Eigen::ComputeThinV);

			//以下计算参考网址
		    //http://eigen.tuxfamily.org/index.php?title=FAQ
			//中关于moore_penrose_pseudo_inverse的介绍

			double pinvtoler = 0.0000000001;
			auto singularValues_inv = svd.singularValues();
			for (long i = 0; i < A.diagonalSize(); ++i)
			{
				if (singularValues_inv(i) > pinvtoler)
					singularValues_inv(i) = 1.0 / singularValues_inv(i);
				else
					singularValues_inv(i) = 0;
			}

			x = svd.matrixV()*(singularValues_inv.asDiagonal()*(svd.matrixU().transpose() * b));
			
			//auto moore_penrose_pseudo_inverse = (svd.matrixV()*singularValues_inv.asDiagonal()*svd.matrixU().transpose());
			//x = moore_penrose_pseudo_inverse * b;
		}
			
		/*保存结果*/
		for (int i = 0; i < 6; ++i)
		{
			if (pLegs[i]->sf().active())
			{
				/*以下输入动力学计算，并补偿摩擦力*/
				for (int j = 0; j < 3; ++j)
				{
					pLegs[i]->fIn_dyn[j] = h[supported_id[i] * 3 + j];
					pLegs[i]->fIn_frc[j] = pLegs[i]->motionAt(j).motFceFrc();
					pLegs[i]->forceAt(j).setFce(h[supported_id[i] * 3 + j]);
				}
			}
			else
			{
				/*以下输入动力学计算，并补偿摩擦力*/
				for (int j = 0; j < 3; ++j)
				{
					pLegs[i]->fIn_dyn[j] = pLegs[i]->_c_M[33 + j][0];
					pLegs[i]->fIn_frc[j] = pLegs[i]->motionAt(j).motFceFrc();
					pLegs[i]->forceAt(j).setFce(pLegs[i]->_c_M[33 + j][0]);
				}
			}
		}
	}
	void RobotTypeI::dyn()
	{
		this->dynSetSolveMethod([](int n, const double *D, const double *b, double *x)
		{
			Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> >Dm(D, n, n);
			Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, 1> >bm(b, n);
			Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 1> >xm(x, n);

			xm = Dm.lu().solve(bm);
		});
		
		this->aris::dynamic::Model::dyn();

		// 更新数据
		for (int i = 0; i < 6; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				if (this->pLegs[i]->motionAt(j).active())
				{
					this->pLegs[i]->fIn_dyn[j] = this->pLegs[i]->motionAt(j).motFceDyn();
				}
				else
				{
					this->pLegs[i]->fIn_dyn[j] = this->pLegs[i]->forceAt(j).fce();
				}
			}
		}
	}

	auto RobotTypeI::loadXml(const aris::core::XmlElement &ele)->void
	{
		Model::loadXml(ele);

		/*Update Parts*/
		body_ = partPool().find("MainBody");
		force_sensor_mak_ = body_->markerPool().find("ForceSensorMak");

		for (int j = 0; j < 6; ++j)
		{
			pLegs[j]->p1a_ = partPool().find(pLegs[j]->name() + "_P1a");
			pLegs[j]->p2a_ = partPool().find(pLegs[j]->name() + "_P2a");
			pLegs[j]->p3a_ = partPool().find(pLegs[j]->name() + "_P3a");
			pLegs[j]->thigh_ = partPool().find(pLegs[j]->name() + "_Thigh");
			pLegs[j]->p2b_ = partPool().find(pLegs[j]->name() + "_P2b");
			pLegs[j]->p3b_ = partPool().find(pLegs[j]->name() + "_P3b");
		}

		// Update Markers //
		for (int j = 0; j < 6; ++j)
		{
			pLegs[j]->base_mak_id_ = body().markerPool().find(pLegs[j]->name() + "_Base");

			pLegs[j]->u1i_ = body().markerPool().find(pLegs[j]->name() + "_U1i");
			pLegs[j]->u2i_ = body().markerPool().find(pLegs[j]->name() + "_U2i");
			pLegs[j]->u3i_ = body().markerPool().find(pLegs[j]->name() + "_U3i");

			pLegs[j]->u1j_ = pLegs[j]->p1a().markerPool().find("U1j");
			pLegs[j]->u2j_ = pLegs[j]->p2a().markerPool().find("U2j");
			pLegs[j]->u3j_ = pLegs[j]->p3a().markerPool().find("U3j");

			pLegs[j]->p1i_ = pLegs[j]->thigh().markerPool().find("P1i");
			pLegs[j]->p2i_ = pLegs[j]->p2b().markerPool().find("P2i");
			pLegs[j]->p3i_ = pLegs[j]->p3b().markerPool().find("P3i");

			pLegs[j]->p1j_ = pLegs[j]->p1a().markerPool().find("P1j");
			pLegs[j]->p2j_ = pLegs[j]->p2a().markerPool().find("P2j");
			pLegs[j]->p3j_ = pLegs[j]->p3a().markerPool().find("P3j");

			pLegs[j]->sfi_ = pLegs[j]->thigh().markerPool().find("Sfi");
			pLegs[j]->s2i_ = pLegs[j]->thigh().markerPool().find("S2i");
			pLegs[j]->s3i_ = pLegs[j]->thigh().markerPool().find("S3i");

			pLegs[j]->s2j_ = pLegs[j]->p2b().markerPool().find("S2j");
			pLegs[j]->s3j_ = pLegs[j]->p3b().markerPool().find("S3j");
			pLegs[j]->sfj_ = ground().markerPool().find(pLegs[j]->name() + "_Sfj");
		}

		// Update Joints //
		for (int j = 0; j < 6; ++j)
		{
			pLegs[j]->u1_ = jointPool().find(pLegs[j]->name() + "_U1");
			pLegs[j]->u2_ = jointPool().find(pLegs[j]->name() + "_U2");
			pLegs[j]->u3_ = jointPool().find(pLegs[j]->name() + "_U3");
			pLegs[j]->p1_ = jointPool().find(pLegs[j]->name() + "_P1");
			pLegs[j]->p2_ = jointPool().find(pLegs[j]->name() + "_P2");
			pLegs[j]->p3_ = jointPool().find(pLegs[j]->name() + "_P3");
			pLegs[j]->sf_ = jointPool().find(pLegs[j]->name() + "_Sf");
			pLegs[j]->s2_ = jointPool().find(pLegs[j]->name() + "_S2");
			pLegs[j]->s3_ = jointPool().find(pLegs[j]->name() + "_S3");
		}

		// Update Motions //
		for (int j = 0; j < 6; ++j)
		{
			pLegs[j]->m1_ = motionPool().find(pLegs[j]->name() + "_M1");
			pLegs[j]->m2_ = motionPool().find(pLegs[j]->name() + "_M2");
			pLegs[j]->m3_ = motionPool().find(pLegs[j]->name() + "_M3");
		}

		// Update Forces //
		for (int j = 0; j < 6; ++j)
		{
			pLegs[j]->f1_ = forcePool().find(pLegs[j]->name() + "_F1");
			pLegs[j]->f2_ = forcePool().find(pLegs[j]->name() + "_F2");
			pLegs[j]->f3_ = forcePool().find(pLegs[j]->name() + "_F3");
		}

		// Update Dimension Variables //
		for (int i = 0; i < 6; ++i)
		{
			double pm[4][4];
			s_inv_pm_dot_pm(*pLegs[i]->base().prtPm(), *pLegs[i]->u2i().prtPm(), *pm);
			*const_cast<double *>(&pLegs[i]->U2x) = pm[0][3];
			*const_cast<double *>(&pLegs[i]->U2y) = pm[1][3];
			*const_cast<double *>(&pLegs[i]->U2z) = pm[2][3];
			s_inv_pm_dot_pm(*pLegs[i]->base().prtPm(), *pLegs[i]->u3i().prtPm(), *pm);
			*const_cast<double *>(&pLegs[i]->U3x) = pm[0][3];
			*const_cast<double *>(&pLegs[i]->U3y) = pm[1][3];
			*const_cast<double *>(&pLegs[i]->U3z) = pm[2][3];
			*const_cast<double *>(&pLegs[i]->S2x) = pLegs[i]->s2i().prtPm()[0][3];
			*const_cast<double *>(&pLegs[i]->S2y) = pLegs[i]->s2i().prtPm()[1][3];
			*const_cast<double *>(&pLegs[i]->S2z) = pLegs[i]->s2i().prtPm()[2][3];
			*const_cast<double *>(&pLegs[i]->S3x) = pLegs[i]->s3i().prtPm()[0][3];
			*const_cast<double *>(&pLegs[i]->S3y) = pLegs[i]->s3i().prtPm()[1][3];
			*const_cast<double *>(&pLegs[i]->S3z) = pLegs[i]->s3i().prtPm()[2][3];
			*const_cast<double *>(&pLegs[i]->Sfx) = pLegs[i]->sfi().prtPm()[0][3];
			*const_cast<double *>(&pLegs[i]->Sfy) = pLegs[i]->sfi().prtPm()[1][3];
			*const_cast<double *>(&pLegs[i]->Sfz) = pLegs[i]->sfi().prtPm()[2][3];

			*const_cast<double *>(&pLegs[i]->D1) = pLegs[i]->U2z;
			*const_cast<double *>(&pLegs[i]->H1) = pLegs[i]->U2y;
			*const_cast<double *>(&pLegs[i]->D2) = pLegs[i]->s2i().prtPm()[2][3];
			*const_cast<double *>(&pLegs[i]->H2) = pLegs[i]->s2i().prtPm()[1][3];
		}

		// Update robot to current Pee //
		for (int i = 0; i < 6; ++i)
		{
			pLegs[i]->sfi().update();
			double pe[6];
			pLegs[i]->sfi().getPe(pe);
			pLegs[i]->SetPee(pe);
		}



	}
	auto RobotTypeI::saveXml(aris::core::XmlElement &xml_ele)const->void
	{
		Model::saveXml(xml_ele);

		auto prt_xml_ele = xml_ele.FirstChildElement("Part");

		std::vector<std::string> leg_name_vec{"LF", "LM", "LR", "RF", "RM", "RR"};

		for (auto &name : leg_name_vec)
		{
			prt_xml_ele->FirstChildElement((name+"_P1a").c_str())->SetAttribute("inertia", "P1aGamma");
			prt_xml_ele->FirstChildElement((name+"_P1a").c_str())->SetAttribute("graphic_file_path", "${P1a_graphic}");
			prt_xml_ele->FirstChildElement((name+"_P1a").c_str())->FirstChildElement("ChildMarker")->FirstChildElement("P1j")->SetAttribute("pe", "P1jpe");
			prt_xml_ele->FirstChildElement((name+"_P1a").c_str())->FirstChildElement("ChildMarker")->FirstChildElement("U1j")->SetAttribute("pe", "U1jpe");
			prt_xml_ele->FirstChildElement((name+"_P2a").c_str())->SetAttribute("inertia", "P2aGamma");
			prt_xml_ele->FirstChildElement((name+"_P2a").c_str())->SetAttribute("graphic_file_path", "${P23a_graphic}");
			prt_xml_ele->FirstChildElement((name+"_P2a").c_str())->FirstChildElement("ChildMarker")->FirstChildElement("P2j")->SetAttribute("pe", "P2jpe");
			prt_xml_ele->FirstChildElement((name+"_P2a").c_str())->FirstChildElement("ChildMarker")->FirstChildElement("U2j")->SetAttribute("pe", "U2jpe");
			prt_xml_ele->FirstChildElement((name+"_P2b").c_str())->SetAttribute("inertia", "P2bGamma");
			prt_xml_ele->FirstChildElement((name+"_P2b").c_str())->SetAttribute("graphic_file_path", "${P23b_graphic}");
			prt_xml_ele->FirstChildElement((name+"_P2b").c_str())->FirstChildElement("ChildMarker")->FirstChildElement("P2i")->SetAttribute("pe", "P2ipe");
			prt_xml_ele->FirstChildElement((name+"_P2b").c_str())->FirstChildElement("ChildMarker")->FirstChildElement("S2j")->SetAttribute("pe", "S2jpe");
			prt_xml_ele->FirstChildElement((name+"_P3a").c_str())->SetAttribute("inertia", "P3aGamma");
			prt_xml_ele->FirstChildElement((name+"_P3a").c_str())->SetAttribute("graphic_file_path", "${P23a_graphic}");
			prt_xml_ele->FirstChildElement((name+"_P3a").c_str())->FirstChildElement("ChildMarker")->FirstChildElement("P3j")->SetAttribute("pe", "P3jpe");
			prt_xml_ele->FirstChildElement((name+"_P3a").c_str())->FirstChildElement("ChildMarker")->FirstChildElement("U3j")->SetAttribute("pe", "U3jpe");
			prt_xml_ele->FirstChildElement((name+"_P3b").c_str())->SetAttribute("inertia", "P3bGamma");
			prt_xml_ele->FirstChildElement((name+"_P3b").c_str())->SetAttribute("graphic_file_path", "${P23b_graphic}");
			prt_xml_ele->FirstChildElement((name+"_P3b").c_str())->FirstChildElement("ChildMarker")->FirstChildElement("P3i")->SetAttribute("pe", "P3ipe");
			prt_xml_ele->FirstChildElement((name+"_P3b").c_str())->FirstChildElement("ChildMarker")->FirstChildElement("S3j")->SetAttribute("pe", "S3jpe");
			prt_xml_ele->FirstChildElement((name+"_Thigh").c_str())->SetAttribute("inertia", "ThighGamma");
			prt_xml_ele->FirstChildElement((name+"_Thigh").c_str())->SetAttribute("graphic_file_path", "${P1b_graphic}");
			prt_xml_ele->FirstChildElement((name+"_Thigh").c_str())->FirstChildElement("ChildMarker")->FirstChildElement("P1i")->SetAttribute("pe", "P1ipe");
			prt_xml_ele->FirstChildElement((name+"_Thigh").c_str())->FirstChildElement("ChildMarker")->FirstChildElement("S2i")->SetAttribute("pe", "S2ipe");
			prt_xml_ele->FirstChildElement((name+"_Thigh").c_str())->FirstChildElement("ChildMarker")->FirstChildElement("S3i")->SetAttribute("pe", "S3ipe");
			prt_xml_ele->FirstChildElement((name+"_Thigh").c_str())->FirstChildElement("ChildMarker")->FirstChildElement("Sfi")->SetAttribute("pe", "Sfipe");

			prt_xml_ele->FirstChildElement("MainBody")->FirstChildElement("ChildMarker")->FirstChildElement((name + "_Base").c_str())->SetAttribute("pe", (name + "pe").c_str());
			prt_xml_ele->FirstChildElement("MainBody")->FirstChildElement("ChildMarker")->FirstChildElement((name + "_U1i").c_str())->SetAttribute("pe", "U1ipe");
			prt_xml_ele->FirstChildElement("MainBody")->FirstChildElement("ChildMarker")->FirstChildElement((name + "_U2i").c_str())->SetAttribute("pe", "U2ipe");
			prt_xml_ele->FirstChildElement("MainBody")->FirstChildElement("ChildMarker")->FirstChildElement((name + "_U3i").c_str())->SetAttribute("pe", "U3ipe");
			prt_xml_ele->FirstChildElement("MainBody")->FirstChildElement("ChildMarker")->FirstChildElement((name + "_U1i").c_str())->SetAttribute("relative_to", (name + "_Base").c_str());
			prt_xml_ele->FirstChildElement("MainBody")->FirstChildElement("ChildMarker")->FirstChildElement((name + "_U2i").c_str())->SetAttribute("relative_to", (name + "_Base").c_str());
			prt_xml_ele->FirstChildElement("MainBody")->FirstChildElement("ChildMarker")->FirstChildElement((name + "_U3i").c_str())->SetAttribute("relative_to", (name + "_Base").c_str());
		}
		
		prt_xml_ele->FirstChildElement("MainBody")->SetAttribute("inertia", "BodyGamma");
		prt_xml_ele->FirstChildElement("MainBody")->SetAttribute("graphic_file_path", "${Body_graphic}");

		auto mot_xml_ele = xml_ele.FirstChildElement("Motion");
		for (auto i = mot_xml_ele->FirstChildElement(); i != nullptr; i = i->NextSiblingElement())
		{
			i->SetAttribute("frc_coe", "Mot_friction");
		}


	};

	auto RobotTypeI::simToAdams(const std::string &adams_file, const aris::dynamic::PlanFunc &func, const aris::dynamic::PlanParamBase &param, int ms_dt)->aris::dynamic::SimResult
	{
		double begin_pee[18], begin_peb[6];
		this->GetPee(begin_pee);
		this->GetPeb(begin_peb);
		this->saveDynEle("before_robotTypeI_simToAdams");
		


		enum STATE { STAND, SUSPEND, MOVE,} last_state[6], this_state[6];
		std::int32_t sim_time{ 0 };
		double last_Pee[6][3];
		
		scriptPool().at(0).clear();
		
		auto getState = [&]()->void
		{
			int stand_num = 0;
			for (auto i = 0; i < 6; ++i)
			{
				double Pee_loc[3];
				pLegs[i]->GetPee(Pee_loc);

				bool is_equal = aris::dynamic::s_is_equal(3, last_Pee[i], Pee_loc, 1e-10);

				if ((is_equal) && (stand_num<3))
				{
					this_state[i] = STAND;
					stand_num++;
				}
				else if (is_equal)
				{
					this_state[i] = SUSPEND;
					stand_num++;
				}
				else
				{
					this_state[i] = MOVE;
				}

				pLegs[i]->GetPee(last_Pee[i]);
			}

			if (stand_num < 3)throw std::runtime_error("can't sim gait because at some time the stand leg num is less than 3");
		};
		
		//起始位置
		this->GetPee(*last_Pee);

		param.count = 0;
		func(*this, param);

		getState();
		
		for (auto i = 0; i < 6; ++i)
		{
			switch (this_state[i])
			{
			case STAND:
			{
				aris::dynamic::DynEle *ele_group[]{ &pLegs[i]->m1(),&pLegs[i]->f2(),&pLegs[i]->f3() };
				for (auto &ele : ele_group)scriptPool().at(0).act(*ele, false);
				break;
			}
			case SUSPEND:
			{
				aris::dynamic::DynEle *ele_group[]{ &pLegs[i]->m1(),&pLegs[i]->m2(),&pLegs[i]->m3() };
				for (auto &ele : ele_group)scriptPool().at(0).act(*ele, false);
				break;
			}
			case MOVE:
			{
				aris::dynamic::DynEle *ele_group[]{ &pLegs[i]->sf(),&pLegs[i]->f1(),&pLegs[i]->f2(),&pLegs[i]->f3() };
				for (auto &ele : ele_group)scriptPool().at(0).act(*ele, false);
				break;
			}
			}
		}
		std::copy_n(this_state, 6, last_state);
		sim_time++;

		//其他位置
		
		for (param.count = 1; true; ++param.count)
		{
			auto is_sim = func(*this, param);

			getState();

			bool is_change_topology = false;
			for (auto i = 0; i < 6; ++i) if (last_state[i] != this_state[i]) is_change_topology = true;

			if (is_change_topology)
			{
				scriptPool().at(0).sim(sim_time, ms_dt);
				sim_time = 0;
			}
			
			for (auto i = 0; i < 6; ++i)
			{
				switch (last_state[i])
				{
				case STAND:
				{
					switch (this_state[i])
					{
					case STAND: break;
					case SUSPEND:
					{
						scriptPool().at(0).act(pLegs[i]->m2(), false);
						scriptPool().at(0).act(pLegs[i]->m3(), false);
						scriptPool().at(0).act(pLegs[i]->f2(), true);
						scriptPool().at(0).act(pLegs[i]->f3(), true);
						break;
					}
					case MOVE:
					{
						scriptPool().at(0).act(pLegs[i]->sf(), false);
						scriptPool().at(0).act(pLegs[i]->m1(), true);
						scriptPool().at(0).act(pLegs[i]->f1(), false);
						break;
					}
					}
					break;
				}
				case SUSPEND:
				{
					switch (this_state[i])
					{
					case STAND: 
					{
						scriptPool().at(0).act(pLegs[i]->m2(), true);
						scriptPool().at(0).act(pLegs[i]->m3(), true);
						scriptPool().at(0).act(pLegs[i]->f2(), false);
						scriptPool().at(0).act(pLegs[i]->f3(), false);
						break;
					}
					case SUSPEND:break;
					case MOVE:
					{
						scriptPool().at(0).act(pLegs[i]->sf(), false);
						scriptPool().at(0).act(pLegs[i]->m1(), true);
						scriptPool().at(0).act(pLegs[i]->f1(), false);
						scriptPool().at(0).act(pLegs[i]->m2(), true);
						scriptPool().at(0).act(pLegs[i]->f2(), false);
						scriptPool().at(0).act(pLegs[i]->m3(), true);
						scriptPool().at(0).act(pLegs[i]->f3(), false);
						break;
					}
					}
					break;
				}
				case MOVE:
				{
					switch (this_state[i])
					{
					case STAND: 
					{
						scriptPool().at(0).act(pLegs[i]->sf(), true);
						scriptPool().at(0).act(pLegs[i]->m1(), false);
						scriptPool().at(0).act(pLegs[i]->f1(), true);
						scriptPool().at(0).aln(pLegs[i]->sfj(), pLegs[i]->sfi());
						break;
					}
					case SUSPEND:
					{
						scriptPool().at(0).act(pLegs[i]->sf(), true);
						scriptPool().at(0).act(pLegs[i]->m1(), false);
						scriptPool().at(0).act(pLegs[i]->f1(), true);
						scriptPool().at(0).act(pLegs[i]->m2(), false);
						scriptPool().at(0).act(pLegs[i]->f2(), true);
						scriptPool().at(0).act(pLegs[i]->m3(), false);
						scriptPool().at(0).act(pLegs[i]->f3(), true);
						scriptPool().at(0).aln(pLegs[i]->sfj(), pLegs[i]->sfi());
						break;
					}
					case MOVE:break;
					}
					break;
				}
					
				}
			}

			std::copy_n(this_state, 6, last_state);

			sim_time++;

			if (!is_sim) 
			{
				scriptPool().at(0).sim(sim_time, ms_dt);
				break;
			}
		}

		this->loadDynEle("before_robotTypeI_simToAdams");
		this->SetPee(begin_pee);
		this->SetPeb(begin_peb);

		return this->aris::dynamic::Model::simToAdams(adams_file, func, param, ms_dt, &scriptPool().at(0));
	}
}