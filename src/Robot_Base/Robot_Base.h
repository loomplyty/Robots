#ifndef ROBOT_BASE_H
#define ROBOT_BASE_H

#include <cstdint>
#include <vector>

namespace Robots
{
	class ROBOT_BASE;
	//struct GAIT_PARAM_BASE;
	struct GAIT_PARAM_BASE;

	typedef std::function<int(ROBOT_BASE *, const GAIT_PARAM_BASE * )> GAIT_FUNC;



	class LEG_BASE
	{
	public:
		/*!
		* \brief Set the position of end-effector
		* \param pEE position of end-effector, double array with 3 elements
		* \param RelativeCoodinate coordinate, which can be "O" or "G" both means origin ground, "M" or "B" mainbody, "L" leg
		*/
		void SetPee(const double *pEE, const char *RelativeCoodinate = "G");
		/*!
		* \brief Set the velocity of end-effector, must be called after position is set
		* \param vEE velocity of end-effector, double array with 3 elements
		* \param RelativeCoodinate coordinate, which can be "O" or "G" both means origin ground, "M" or "B" mainbody, "L" leg
		*/
		void SetVee(const double *vEE, const char *RelativeCoodinate = "G");
		/*!
		* \brief Set the velocity of end-effector, must be called after velocity is set
		* \param vEE acceleration of end-effector, double array with 3 elements
		* \param RelativeCoodinate coordinate, which can be "O" or "G" both means origin ground, "M" or "B" mainbody, "L" leg
		*/
		void SetAee(const double *aEE, const char *RelativeCoodinate = "G");
		/*!
		* \brief Set the position of inputs
		* \param pIn position of inputs, double array with 3 elements
		*/
		void SetPin(const double *pIn);
		/*!
		* \brief Set the velocity of inputs
		* \param vIn velocity of inputs, double array with 3 elements
		*/
		void SetVin(const double *vIn);
		/*!
		* \brief Set the acceleration of inputs
		* \param aIn acceleration of inputs, double array with 3 elements
		*/
		void SetAin(const double *aIn);
		/*!
		* \brief Get the position of end-effector
		* \param pEE position of end-effector, double array with 3 elements
		* \param RelativeCoodinate coordinate, which can be "O" or "G" both means origin ground, "M" or "B" mainbody, "L" leg
		*/
		void GetPee(double *pEE, const char *RelativeCoodinate = "G") const;
		/*!
		* \brief Get the velocity of end-effector
		* \param vEE velocity of end-effector, double array with 3 elements
		* \param RelativeCoodinate coordinate, which can be "O" or "G" both means origin ground, "M" or "B" mainbody, "L" leg
		*/
		void GetVee(double *vEE, const char *RelativeCoodinate = "G") const;
		/*!
		* \brief Get the acceleration of end-effector
		* \param pEE acceleration of end-effector, double array with 3 elements
		* \param RelativeCoodinate coordinate, which can be "O" or "G" both means origin ground, "M" or "B" mainbody, "L" leg
		*/
		void GetAee(double *aEE, const char *RelativeCoodinate = "G") const;
		/*!
		* \brief Get the position of inputs
		* \param pIn position of inputs, double array with 3 elements
		*/
		void GetPin(double *pIn) const;
		/*!
		* \brief Get the velocity of inputs
		* \param vIn velocity of inputs, double array with 3 elements
		*/
		void GetVin(double *vIn) const;
		/*!
		* \brief Get the acceleration of inputs
		* \param aIn acceleration of inputs, double array with 3 elements
		*/
		void GetAin(double *aIn) const;

		void GetFceJacDir(double *jac, const char *RelativeCoodinate = "G") const;
		void GetFceJacInv(double *jac, const char *RelativeCoodinate = "G") const;
		void GetVelJacDir(double *jac, double *c = 0, const char *RelativeCoodinate = "G") const;
		void GetVelJacInv(double *jac, double *c = 0, const char *RelativeCoodinate = "G") const;
		void GetAccJacDir(double *jac, double *c = 0, const char *RelativeCoodinate = "G") const;
		void GetAccJacInv(double *jac, double *c = 0, const char *RelativeCoodinate = "G") const;

		void TransformCoordinatePee(const double *bodyPe, const char *fromMak, const double *fromPee
			, const char *toMak, double *toPee) const;

	protected:
		LEG_BASE(ROBOT_BASE* pRobot);
		virtual ~LEG_BASE() = default;
		
		virtual void calculate_from_pEE(){};
		virtual void calculate_from_pIn(){};
		virtual void calculate_from_vEE(){};
		virtual void calculate_from_vIn(){};
		virtual void calculate_from_aEE(){};
		virtual void calculate_from_aIn(){};
		virtual void calculate_jac(){};
		virtual void calculate_jac_c(){};

	protected:		
		ROBOT_BASE *pRobot;

		const double * const pBasePrtPm{ *_BasePrtPm };
		double * const pBasePm{ *_BasePm };

		union
		{
			double pEE[3];
			struct
			{
				double x;
				double y;
				double z;
			};
		};
		union
		{
			double vEE[3];
			struct
			{
				double vx;
				double vy;
				double vz;
			};
		};
		union
		{
			double aEE[3];
			struct
			{
				double ax;
				double ay;
				double az;
			};
		};
		union
		{
			double pIn[3];
			struct
			{
				double l1;
				double l2;
				double l3;
			};
		};
		union
		{
			double vIn[3];
			struct
			{
				double vl1;
				double vl2;
				double vl3;
			};
		};
		union
		{
			double aIn[3];
			struct
			{
				double al1;
				double al2;
				double al3;
			};
		};

		double _c_acc_dir[3];
		double _c_acc_inv[3];

		double _jac_vel_dir[3][3];
		double _jac_vel_inv[3][3];

	private:
		double _BasePrtPm[4][4];
		double _BasePm[4][4];

		friend class ROBOT_BASE;
};
	class ROBOT_BASE
	{
	public:
		ROBOT_BASE() = default;
		virtual ~ROBOT_BASE() = default;

		/*!
		* \brief Get the body pose matrix
		* \param body pose matrix, double array with 16 elements
		*/
		void GetBodyPm(double *bodyPm) const;
		/*!
		* \brief Get the body PE
		* \param body pose matrix, double array with 16 elements
		*/
		void GetBodyPe(double *bodyPe, const char *eurType = "313") const;
		/*!
		* \brief Get the body velocity
		* \param body velocity, double array with 6 elements
		*/
		void GetBodyVel(double *bodyVel) const;
		/*!
		* \brief Get the body acceleration
		* \param body acceleration, double array with 6 elements
		*/
		void GetBodyAcc(double *bodyAcc) const;

		void SetPee(const double *pEE = nullptr, const double *bodyPe = nullptr, const char *coodinate = "G", const char *eurType = "313");
		void SetVee(const double *aEE = nullptr, const double *bodyPe = nullptr, const char *coodinate = "G");
		void SetAee(const double *vEE = nullptr, const double *bodyPe = nullptr, const char *coodinate = "G");
		void SetPin(const double *pIn = nullptr, const double *bodyPe = nullptr, const char *eurType = "313");
		void SetVin(const double *vIn = nullptr, const double *bodyPe = nullptr);
		void SetAin(const double *aIn = nullptr, const double *bodyPe = nullptr);
		void GetPee(double *pEE, const char *RelativeCoodinate = "G") const;
		void GetVee(double *aEE, const char *RelativeCoodinate = "G") const;
		void GetAee(double *vEE, const char *RelativeCoodinate = "G") const;
		void GetPin(double *pIn) const;
		void GetVin(double *vIn) const;
		void GetAin(double *aIn) const;

		void TransformCoordinatePee(const double *bodyPe, const char *fromMak, const double *fromPee
			, const char *toMak, double *toPee) const;

		virtual void LoadXml(const char *) {};

		LEG_BASE *pLegs[6];

	protected:
		double *const pBodyPm{ *_BodyPm };
		double *const pBodyVel{ _BodyVel };
		double *const pBodyAcc{ _BodyAcc };

	private:
		double _BodyPm[4][4], _BodyVel[6], _BodyAcc[6];

		std::vector<GAIT_FUNC> gaitList;

		friend class LEG_BASE;
	};
}

#endif
