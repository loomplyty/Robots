#ifndef ROBOT_BASE_H
#define ROBOT_BASE_H

#include <cstdint>
#include <vector>

#include <aris.h>

namespace Robots
{
	class RobotBase;

	class LegBase:public aris::dynamic::Object
	{
	public:
		using Coordinate = aris::dynamic::Coordinate;
		using Marker = aris::dynamic::Marker;
		using Part = aris::dynamic::Part;

		const Part& ground() const;
		Part& ground();
		const Part& body() const;
		Part& body();
		const Marker& base() const;
		Marker& base();
		


		void GetPee(double *pEE) const { GetPee(pEE, ground()); };
		void GetPee(double *pEE, const Coordinate &Mak) const;
		void SetPee(const double *pEE) { SetPee(pEE, ground()); };
		void SetPee(const double *pEE, const Coordinate &Mak);
		void GetVee(double *vEE) const { GetVee(vEE, ground()); };
		void GetVee(double *vEE, const Coordinate &Mak) const;
		void SetVee(const double *vEE) { SetVee(vEE, ground()); };
		void SetVee(const double *vEE, const Coordinate &Mak);
		void GetAee(double *aEE) const { GetAee(aEE, ground()); };
		void GetAee(double *aEE, const Coordinate &Mak) const;
		void SetAee(const double *aEE) { SetAee(aEE, ground()); };
		void SetAee(const double *aEE, const Coordinate &Mak);
		void GetFeeSta(double *fEE_sta) const { GetFeeSta(fEE_sta, ground()); };
		void GetFeeSta(double *fEE_sta, const Coordinate &Mak) const;
		void SetFeeSta(const double *fEE_sta) { SetFeeSta(fEE_sta, ground()); };
		void SetFeeSta(const double *fEE_sta, const Coordinate &Mak);
		/*!
		* \brief Get the position of inputs
		* \param pIn position of inputs, double array with 3 elements
		*/
		void GetPin(double *pIn) const;
		/*!
		* \brief Set the position of inputs
		* \param pIn position of inputs, double array with 3 elements
		*/
		void SetPin(const double *pIn);
		/*!
		* \brief Get the velocity of inputs
		* \param vIn velocity of inputs, double array with 3 elements
		*/
		void GetVin(double *vIn) const;
		/*!
		* \brief Set the velocity of inputs
		* \param vIn velocity of inputs, double array with 3 elements
		*/
		void SetVin(const double *vIn);
		/*!
		* \brief Get the acceleration of inputs
		* \param aIn acceleration of inputs, double array with 3 elements
		*/
		void GetAin(double *aIn) const;
		/*!
		* \brief Set the acceleration of inputs
		* \param aIn acceleration of inputs, double array with 3 elements
		*/
		void SetAin(const double *aIn);
		/*!
		* \brief Get the static actuation force of inputs, which is equal to Jfi * Fee_sta
		* \param fIn actuation force of inputs, double array with 3 elements
		*/
		void GetFinSta(double *fIn_sta) const;
		/*!
		* \brief Set the static actuation force of inputs, meanwhile the Fee_sta will be Jfd * Fin_sta
		* \param aIn actuation force of inputs, double array with 3 elements
		*/
		void SetFinSta(const double *fIn_sta);

		void GetJfd(double *jac) const { GetJfd(jac, ground()); };
		void GetJfd(double *jac, const Coordinate &Mak) const;
		void GetJfi(double *jac) const { GetJfi(jac, ground()); };
		void GetJfi(double *jac, const Coordinate &Mak) const;
		void GetJvd(double *jac) const { GetJvd(jac, ground()); };
		void GetJvd(double *jac, const Coordinate &Mak) const;
		void GetJvi(double *jac) const { GetJvi(jac, ground()); };
		void GetJvi(double *jac, const Coordinate &Mak) const;
		void GetDifJfd(double *dJac) const { GetDifJfd(dJac, ground()); };
		void GetDifJfd(double *dJac, const Coordinate &Mak) const;
		void GetDifJfi(double *dJac) const { GetDifJfi(dJac, ground()); };
		void GetDifJfi(double *dJac, const Coordinate &Mak) const;
		void GetDifJvd(double *dJac) const { GetDifJvd(dJac, ground()); };
		void GetDifJvd(double *dJac, const Coordinate &Mak) const;
		void GetDifJvi(double *dJac) const { GetDifJvi(dJac, ground()); };
		void GetDifJvi(double *dJac, const Coordinate &Mak) const;
		void GetCvd(double *c) const { GetCvd(c, ground()); };
		void GetCvd(double *c, const Coordinate &Mak) const;
		void GetCvi(double *c) const { GetCvi(c, ground()); };
		void GetCvi(double *c, const Coordinate &Mak) const;
		void GetCad(double *c) const { GetCad(c, ground()); };
		/*!
		* \brief follow equation: Aee = Jvd * Ain + dJvd * Vin + Cad
		* \param c Cad
		*/
		void GetCad(double *c, const Coordinate &Mak) const;
		void GetCai(double *c) const { GetCai(c, ground()); };
		/*!
		* \brief follow equation: Ain = Jvi * Aee + dJvi * Vee + Cai
		* \param c Cai
		*/
		void GetCai(double *c, const Coordinate &Mak) const;

		void TransformCoordinatePee(const double *bodyPe, const char *fromMak, const double *fromPee
			, const char *toMak, double *toPee) const;

	protected:
		LegBase(RobotBase* pRobot, const char *Name);
		virtual ~LegBase() = default;
		
		virtual void calculate_from_pEE(){};
		virtual void calculate_from_pIn(){};
		virtual void calculate_from_vEE(){};
		virtual void calculate_from_vIn(){};
		virtual void calculate_from_aEE(){};
		virtual void calculate_from_aIn(){};
		virtual void calculate_jac(){};
		virtual void calculate_diff_jac(){};

	protected:		
		RobotBase *pRobot;
		aris::dynamic::Marker *base_mak_id_;

		union 
		{
			double pEE[3]{ 0 };
			struct
			{
				double x;
				double y;
				double z;
			};
		};
		union 
		{
			double vEE[3]{ 0 };
			struct
			{
				double vx;
				double vy;
				double vz;
			};
		};
		union 
		{
			double aEE[3]{ 0 };
			struct
			{
				double ax;
				double ay;
				double az;
			};
		};
		union 
		{
			double fEE_sta[3]{ 0 };
			struct
			{
				double fx_sta;
				double fy_sta;
				double fz_sta;
			};
		};
		union 
		{
			double pIn[3]{ 0 };
			struct
			{
				double l1;
				double l2;
				double l3;
			};
		};
		union 
		{
			double vIn[3]{ 0 };
			struct
			{
				double vl1;
				double vl2;
				double vl3;
			};
		};
		union 
		{
			double aIn[3]{ 0 };
			struct
			{
				double al1;
				double al2;
				double al3;
			};
		};
		union 
		{
			double fIn_sta[3]{ 0 };
			struct
			{
				double f1_sta;
				double f2_sta;
				double f3_sta;
			};
		};

		double _c_acc_dir[3]{ 0 };
		double _c_acc_inv[3]{ 0 };

		double Jvd[3][3]{ { 0 } };
		double Jvi[3][3]{ { 0 } };

		double vJvd[3][3]{ { 0 } };
		double vJvi[3][3]{ { 0 } };

		friend class RobotBase;
	};

	class RobotBase:public aris::dynamic::Model
	{
	public:
		using Coordinate = aris::dynamic::Coordinate;
		using Part = aris::dynamic::Part;
		using Marker = aris::dynamic::Marker;

		RobotBase();
		virtual ~RobotBase() = default;

		const Part& body() const { return *body_; };
		Part& body() { return *body_;};
		const Marker& forceSensorMak() const { return *force_sensor_mak_; };
		Marker& forceSensorMak() { return *force_sensor_mak_; };
		const Coordinate * const * const LegBases() const
		{ 
			static Coordinate *pBases[6]
			{
				&pLegs[0]->base(),
				&pLegs[1]->base(),
				&pLegs[2]->base(),
				&pLegs[3]->base(),
				&pLegs[4]->base(),
				&pLegs[5]->base(),
			};
			return pBases;
		};

		void GetPmb(double *pmb) const { GetPmb(pmb, ground()); };
		void GetPmb(double *pmb, const Coordinate &mak) const;
		void SetPmb(const double *pmb) { SetPmb(pmb, ground()); };
		void SetPmb(const double *pmb, const Coordinate &mak);
		void GetPeb(double *peb, const char *eurType = "313") const { GetPeb(peb, ground(), eurType); };
		void GetPeb(double *peb, const Coordinate &mak, const char *eurType = "313") const;
		void SetPeb(const double *peb, const char *eurType = "313") { SetPeb(peb, ground(), eurType); };
		void SetPeb(const double *peb, const Coordinate &mak, const char *eurType = "313");
		void GetPqb(double *pqb) const { GetPqb(pqb, ground()); };
		void GetPqb(double *pqb, const Coordinate &mak) const;
		void SetPqb(const double *pqb) { SetPqb(pqb, ground()); };
		void SetPqb(const double *pqb, const Coordinate &mak);
		void GetVb(double *vb) const { GetVb(vb, ground()); };
		void GetVb(double *vb, const Coordinate &mak) const;
		void SetVb(const double *vb) { SetVb(vb, ground()); };
		void SetVb(const double *vb, const Coordinate &mak);
		void GetAb(double *ab) const { GetAb(ab, ground()); };
		void GetAb(double *ab, const Coordinate &mak) const;
		void SetAb(const double *ab) { SetAb(ab, ground()); };
		void SetAb(const double *ab, const Coordinate &mak);

		void GetPee(double *pEE) const { GetPee(pEE, ground()); };
		void GetPee(double *pEE, const Coordinate &Mak) const;
		void SetPee(const double *pEE) { SetPee(pEE, ground()); };
		void SetPee(const double *pEE, const Coordinate &Mak);
		void GetVee(double *vEE) const { GetVee(vEE, ground()); };
		void GetVee(double *vEE, const Coordinate &Mak) const;
		void SetVee(const double *vEE) { SetVee(vEE, ground()); };
		void SetVee(const double *vEE, const Coordinate &Mak);
		void GetAee(double *aEE) const { GetAee(aEE, ground()); };
		void GetAee(double *aEE, const Coordinate &Mak) const;
		void SetAee(const double *aEE) { SetAee(aEE, ground()); };
		void SetAee(const double *aEE, const Coordinate &Mak);
		void GetFeeSta(double *fEE_sta) const { GetFeeSta(fEE_sta, ground()); };
		void GetFeeSta(double *fEE_sta, const Coordinate &Mak) const;
		void SetFeeSta(const double *fEE_sta) { SetFeeSta(fEE_sta, ground()); };
		void SetFeeSta(const double *fEE_sta, const Coordinate &Mak);
		
		void GetPee(double *pEE, const Coordinate* const pMaks[]) const;
		void SetPee(const double *pEE, const Coordinate* const pMaks[]);
		void GetVee(double *vEE, const Coordinate* const pMaks[]) const;
		void SetVee(const double *vEE, const Coordinate* const pMaks[]);
		void GetAee(double *aEE, const Coordinate* const pMaks[]) const;
		void SetAee(const double *aEE, const Coordinate* const pMaks[]);
		void GetFeeSta(double *fee_sta, const Coordinate* const pMaks[]) const;
		void SetFeeSta(const double *fee_sta, const Coordinate* const pMaks[]);
		
		void GetPin(double *pIn) const;
		void SetPin(const double *pIn);
		void GetVin(double *vIn) const;
		void SetVin(const double *pIn);
		void GetAin(double *aIn) const;
		void SetAin(const double *pIn);
		void GetFinSta(double *fIn_sta) const;
		void SetFinSta(const double *fIn_sta);

		/*!
		* \brief 根据雅可比矩阵和初值迭代求解，主要用来在已知足端位置和输入位置时，机器人身体的位置。
		* 对于固定于地面的腿，对其SetPee，然后迭代求解身体位姿，之后对不在地面上的腿SetPin
		*/
		void SetPinFixFeet(const double *pIn, const char *fixFeet, const char *activeMotor, const double *initBodyPE);
		void SetVinFixFeet(const double *vIn, const char *fixFeet, const char *activeMotor);
		void SetAinFixFeet(const double *aIn, const char *fixFeet, const char *activeMotor);

		void GetJfd(double *jac_out, const char *activeMotion = "111111111111111111") const;
		void GetJvi(double *jac_out, const char *activeMotion = "111111111111111111") const;
		void GetDifJfd(double *jac_out, const char *activeMotion = "111111111111111111") const;
		void GetDifJvi(double *jac_out, const char *activeMotion = "111111111111111111") const;

		void TransformCoordinatePee(const double *bodyPe, const char *fromMak, const double *fromPee
			, const char *toMak, double *toPee) const;

		LegBase *pLegs[6];

	protected:
		aris::dynamic::Part *body_;
		aris::dynamic::Marker *force_sensor_mak_;

	private:
		double _BodyPm[4][4]{ {0} }, _BodyVel[6]{ 0 }, _BodyAcc[6]{ 0 };
		double Jvi[18][6]{ {0} };
		double vJvi[18][6]{ {0} };
		void calculate_jac();
		void calculate_jac_c();

		friend class LegBase;
	};
}

#endif
