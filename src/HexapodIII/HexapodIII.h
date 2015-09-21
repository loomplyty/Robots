#ifndef ROBOT_III_H
#define ROBOT_III_H

#include <Aris_DynKer.h>
#include <Aris_DynModel.h>
#include <fstream>

#include "Robot_Base.h"
#include "Robot_Gait.h"

/** \brief 命名空间：六足机器人
*
* 用来计算机器人的运动学和动力学等。
*/
namespace Robots
{
	class ROBOT_III;
	class LEG_III :public Aris::DynKer::OBJECT, public Robots::LEG_BASE
	{
	public:
		union
		{
			struct
			{
				Aris::DynKer::PART* pP1a;/*!< \brief 指向部件P1a的指针 */
				Aris::DynKer::PART* pP2a;/*!< \brief 指向部件P2a的指针 */
				Aris::DynKer::PART* pP3a;/*!< \brief 指向部件P3a的指针 */
				Aris::DynKer::PART* pThigh;/*!< \brief 指向部件Thigh的指针 */
				Aris::DynKer::PART* pP2b;/*!< \brief 指向部件P2b的指针 */
				Aris::DynKer::PART* pP3b;/*!< \brief 指向部件P3b的指针 */
			};
			Aris::DynKer::PART* pPrts[6];
		};
		union
		{
			struct
			{
				Aris::DynKer::JOINT_BASE *pU1;/*!< \brief 指向关节U1的指针 */
				Aris::DynKer::JOINT_BASE *pU2;/*!< \brief 指向关节U2的指针 */
				Aris::DynKer::JOINT_BASE *pU3;/*!< \brief 指向关节U3的指针 */
				Aris::DynKer::JOINT_BASE *pP1;/*!< \brief 指向关节P1的指针 */
				Aris::DynKer::JOINT_BASE *pP2;/*!< \brief 指向关节P2的指针 */
				Aris::DynKer::JOINT_BASE *pP3;/*!< \brief 指向关节P3的指针 */
				Aris::DynKer::JOINT_BASE *pS2;/*!< \brief 指向关节S2的指针 */
				Aris::DynKer::JOINT_BASE *pS3;/*!< \brief 指向关节S3的指针 */
				Aris::DynKer::JOINT_BASE *pSf;/*!< \brief 指向关节Sf的指针 */
			};
			Aris::DynKer::JOINT_BASE *pJnts[9];
		};
		union
		{
			struct
			{
				Aris::DynKer::MARKER *pBase;/*!< \brief 指向坐标系Base的指针，位于部件MainBody上 */
				Aris::DynKer::MARKER *pU1i;/*!< \brief 指向坐标系U1i的指针，位于部件MainBody上 */
				Aris::DynKer::MARKER *pU1j;/*!< \brief 指向坐标系U1j的指针，位于部件P1a上 */
				Aris::DynKer::MARKER *pU2i;/*!< \brief 指向坐标系U2i的指针，位于部件MainBody上 */
				Aris::DynKer::MARKER *pU2j;/*!< \brief 指向坐标系U2j的指针，位于部件P2a上 */
				Aris::DynKer::MARKER *pU3i;/*!< \brief 指向坐标系U3i的指针，位于部件MainBody上 */
				Aris::DynKer::MARKER *pU3j;/*!< \brief 指向坐标系U3j的指针，位于部件P3a上 */
				Aris::DynKer::MARKER *pP1i;/*!< \brief 指向坐标系P1i的指针，位于部件Thigh上 */
				Aris::DynKer::MARKER *pP1j;/*!< \brief 指向坐标系P1j的指针，位于部件P1a上 */
				Aris::DynKer::MARKER *pP2i;/*!< \brief 指向坐标系P2i的指针，位于部件P2b上 */
				Aris::DynKer::MARKER *pP2j;/*!< \brief 指向坐标系P2j的指针，位于部件P2a上 */
				Aris::DynKer::MARKER *pP3i;/*!< \brief 指向坐标系P3i的指针，位于部件P3b上 */
				Aris::DynKer::MARKER *pP3j;/*!< \brief 指向坐标系P3i的指针，位于部件P3a上 */
				Aris::DynKer::MARKER *pS2i;/*!< \brief 指向坐标系S2i的指针，位于部件Thigh上 */
				Aris::DynKer::MARKER *pS2j;/*!< \brief 指向坐标系S2j的指针，位于部件P2b上 */
				Aris::DynKer::MARKER *pS3i;/*!< \brief 指向坐标系S3i的指针，位于部件Thigh上 */
				Aris::DynKer::MARKER *pS3j;/*!< \brief 指向坐标系S3j的指针，位于部件P3b上 */
				Aris::DynKer::MARKER *pSfi;/*!< \brief 指向坐标系Sfi的指针，位于部件Thigh上 */
				Aris::DynKer::MARKER *pSfj;/*!< \brief 指向坐标系Sfj的指针，位于部件Ground上 */
			};
			Aris::DynKer::MARKER *pMaks[19];
		};
		union
		{
			struct
			{
				Aris::DynKer::MOTION_BASE *pM1;/*!< \brief 指向驱动M1的指针 */
				Aris::DynKer::MOTION_BASE *pM2;/*!< \brief 指向驱动M2的指针 */
				Aris::DynKer::MOTION_BASE *pM3;/*!< \brief 指向驱动M3的指针 */
			};

			Aris::DynKer::MOTION_BASE *pMots[3];
		};
		union
		{
			struct
			{
				Aris::DynKer::SINGLE_COMPONENT_FORCE *pF1;/*!< \brief 指向驱动M1的指针 */
				Aris::DynKer::SINGLE_COMPONENT_FORCE *pF2;/*!< \brief 指向驱动M2的指针 */
				Aris::DynKer::SINGLE_COMPONENT_FORCE *pF3;/*!< \brief 指向驱动M3的指针 */
			};

			Aris::DynKer::SINGLE_COMPONENT_FORCE *pFces[3];
		};
		
	public:
		void GetdJacOverPee(double *dJi_x, double *dJi_y, double *dJi_z, const char *relativeCoordinate="G")const;

	private:
		LEG_III(const char *Name, ROBOT_III* pRobot);
		virtual ~LEG_III() = default;

		void FastDyn();
		
		void GetFin(double *fIn) const;
		void GetFinDyn(double *fIn) const;
		void GetFinFrc(double *fIn) const;

		virtual void calculate_from_pEE();
		virtual void calculate_from_pIn();
		virtual void calculate_from_vEE();
		virtual void calculate_from_vIn();
		virtual void calculate_from_aEE();
		virtual void calculate_from_aIn();

		virtual void calculate_jac();
		virtual void calculate_diff_jac();

		void _CalCdByPos();
		void _CalCdByPlen();
		void _CalCdByPlen2();
		void _CalVarByCd();
		void _CalPartByVar();

		void _CalVcdByVpos();
		void _CalVcdByVplen();
		void _CalVvarByVcd();
		void _CalVpartByVvar();

		void _CalAcdByApos();
		void _CalAcdByAplen();
		void _CalAvarByAcd();
		void _CalApartByAvar();

	private:
		union
		{
			double fIn_dyn[3];
			struct
			{
				double f1_dyn;
				double f2_dyn;
				double f3_dyn;
			};
		};
		union
		{
			double fIn_frc[3];
			struct
			{
				double f1_frc;
				double f2_frc;
				double f3_frc;
			};
		};

	private:
		ROBOT_III *pRobot;

		const double U2x{ 0 }, U2y{ 0 }, U2z{ 0 }, U3x{ 0 }, U3y{ 0 }, U3z{ 0 };
		const double S2x{ 0 }, S2y{ 0 }, S2z{ 0 }, S3x{ 0 }, S3y{ 0 }, S3z{ 0 };
		const double Sfx{ 0 }, Sfy{ 0 }, Sfz{ 0 };

		const double D1{ 0 }, H1{ 0 }, D2{ 0 }, H2{ 0 };

		double a1, b1, va1, vb1, aa1, ab1;
		double x2, y2, z2, x3, y3, z3;
		double a2, b2, a3, b3;
		double sa1, ca1, sb1, cb1, sa2, ca2, sb2, cb2, sa3, ca3, sb3, cb3;
		double pa1, qa1, pb1, qb1, pa2, qa2, pb2, qb2, pa3, qa3, pb3, qb3;

		double vx2, vy2, vz2, vx3, vy3, vz3;
		double va2, vb2, va3, vb3;

		double ax2, ay2, az2, ax3, ay3, az3;
		double aa2, ab2, aa3, ab3;

		double H11, H12, H21, H22;
		double k21, k22, k23, k31, k32, k33;
		double vk21, vk22, vk23, vk31, vk32, vk33;
		double J1[3][3], J2[3][3], vJ1[3][3], vJ2[3][3];
		double inv_J1[3][3], inv_J2[3][3];

		double _C[36][36];
		double _c_M[36][4];

		friend class ROBOT_III;
	};
	class ROBOT_III :public Aris::DynKer::MODEL, public Robots::ROBOT_BASE
	{
	public:
		struct STATE
		{
			double pEE[18], vEE[18], aEE[18], fIn[18];
			double bodyPE[6], bodyVel[6], bodyAcc[6];
			bool isMotorActive[18];
			bool isSfActive[6];
		};

		ROBOT_III();
		~ROBOT_III() = default;
		virtual void LoadXml(const char *filename);


		void GetFin(double *fIn) const;
		void GetFinDyn(double *fIn) const;
		void GetFinFrc(double *fIn) const;

		void GetState(ROBOT_III::STATE &state) const;
		void SetState(const ROBOT_III::STATE &state);

		void SetFixFeet(const char* fixFeet);
		const char* GetFixFeet() const;
		void SetActiveMotion(const char* activeMotion);
		const char* GetActiveMotion() const;
		void FastDyn();
		
		void SimByAdamsResultAt(int momentTime);
		void SimByAdams(const char *adamsFile, const GAIT_FUNC &fun, GAIT_PARAM_BASE *param, const Aris::DynKer::SIMULATE_SCRIPT *script = nullptr);
		void SimByAdams(const char *adamsFile, const GAIT_FUNC &fun, GAIT_PARAM_BASE *param, int dt);
	public:
		union
		{
			struct
			{
				LEG_III *const pLF;
				LEG_III *const pLM;
				LEG_III *const pLR;
				LEG_III *const pRF;
				LEG_III *const pRM;
				LEG_III *const pRR;
			};
			LEG_III *const pLegs[6];
		};

		Aris::DynKer::PART* pBody;
		Aris::DynKer::MARKER* pBodyCenter;

	public:
		

	private:
		LEG_III LF_Leg{ "LF", this };
		LEG_III LM_Leg{ "LM", this };
		LEG_III LR_Leg{ "LR", this };
		LEG_III RF_Leg{ "RF", this };
		LEG_III RM_Leg{ "RM", this };
		LEG_III RR_Leg{ "RR", this };
	};

	inline void Activate024(int time, ROBOT_III *pRobot, Aris::DynKer::SIMULATE_SCRIPT *script)
	{
		script->ScriptDeactivate(time, pRobot->pLF->pSf);
		script->ScriptActivate(time, pRobot->pLM->pSf);
		script->ScriptDeactivate(time, pRobot->pLR->pSf);
		script->ScriptActivate(time, pRobot->pRF->pSf);
		script->ScriptDeactivate(time, pRobot->pRM->pSf);
		script->ScriptActivate(time, pRobot->pRR->pSf);

		script->ScriptActivate(time, pRobot->pLF->pM1);
		script->ScriptActivate(time, pRobot->pLF->pM2);
		script->ScriptActivate(time, pRobot->pLF->pM3);
		script->ScriptDeactivate(time, pRobot->pLF->pF1);
		script->ScriptDeactivate(time, pRobot->pLF->pF2);
		script->ScriptDeactivate(time, pRobot->pLF->pF3);

		script->ScriptActivate(time, pRobot->pLR->pM1);
		script->ScriptActivate(time, pRobot->pLR->pM2);
		script->ScriptActivate(time, pRobot->pLR->pM3);
		script->ScriptDeactivate(time, pRobot->pLR->pF1);
		script->ScriptDeactivate(time, pRobot->pLR->pF2);
		script->ScriptDeactivate(time, pRobot->pLR->pF3);

		script->ScriptActivate(time, pRobot->pRM->pM1);
		script->ScriptActivate(time, pRobot->pRM->pM2);
		script->ScriptActivate(time, pRobot->pRM->pM3);
		script->ScriptDeactivate(time, pRobot->pRM->pF1);
		script->ScriptDeactivate(time, pRobot->pRM->pF2);
		script->ScriptDeactivate(time, pRobot->pRM->pF3);

		script->ScriptDeactivate(time, pRobot->pLM->pM1);
		script->ScriptActivate(time, pRobot->pLM->pM2);
		script->ScriptActivate(time, pRobot->pLM->pM3);
		script->ScriptActivate(time, pRobot->pLM->pF1);
		script->ScriptDeactivate(time, pRobot->pLM->pF2);
		script->ScriptDeactivate(time, pRobot->pLM->pF3);

		script->ScriptDeactivate(time, pRobot->pRF->pM1);
		script->ScriptActivate(time, pRobot->pRF->pM2);
		script->ScriptActivate(time, pRobot->pRF->pM3);
		script->ScriptActivate(time, pRobot->pRF->pF1);
		script->ScriptDeactivate(time, pRobot->pRF->pF2);
		script->ScriptDeactivate(time, pRobot->pRF->pF3);

		script->ScriptDeactivate(time, pRobot->pRR->pM1);
		script->ScriptActivate(time, pRobot->pRR->pM2);
		script->ScriptActivate(time, pRobot->pRR->pM3);
		script->ScriptActivate(time, pRobot->pRR->pF1);
		script->ScriptDeactivate(time, pRobot->pRR->pF2);
		script->ScriptDeactivate(time, pRobot->pRR->pF3);

		
	}
	inline void Activate135(int time, ROBOT_III *pRobot, Aris::DynKer::SIMULATE_SCRIPT *script)
	{
		script->ScriptActivate(time, pRobot->pLF->pSf);
		script->ScriptDeactivate(time, pRobot->pLM->pSf);
		script->ScriptActivate(time, pRobot->pLR->pSf);
		script->ScriptDeactivate(time, pRobot->pRF->pSf);
		script->ScriptActivate(time, pRobot->pRM->pSf);
		script->ScriptDeactivate(time, pRobot->pRR->pSf);

		script->ScriptDeactivate(time, pRobot->pLF->pM1);
		script->ScriptActivate(time, pRobot->pLF->pM2);
		script->ScriptActivate(time, pRobot->pLF->pM3);
		script->ScriptActivate(time, pRobot->pLF->pF1);
		script->ScriptDeactivate(time, pRobot->pLF->pF2);
		script->ScriptDeactivate(time, pRobot->pLF->pF3);

		script->ScriptDeactivate(time, pRobot->pLR->pM1);
		script->ScriptActivate(time, pRobot->pLR->pM2);
		script->ScriptActivate(time, pRobot->pLR->pM3);
		script->ScriptActivate(time, pRobot->pLR->pF1);
		script->ScriptDeactivate(time, pRobot->pLR->pF2);
		script->ScriptDeactivate(time, pRobot->pLR->pF3);

		script->ScriptDeactivate(time, pRobot->pRM->pM1);
		script->ScriptActivate(time, pRobot->pRM->pM2);
		script->ScriptActivate(time, pRobot->pRM->pM3);
		script->ScriptActivate(time, pRobot->pRM->pF1);
		script->ScriptDeactivate(time, pRobot->pRM->pF2);
		script->ScriptDeactivate(time, pRobot->pRM->pF3);

		script->ScriptActivate(time, pRobot->pLM->pM1);
		script->ScriptActivate(time, pRobot->pLM->pM2);
		script->ScriptActivate(time, pRobot->pLM->pM3);
		script->ScriptDeactivate(time, pRobot->pLM->pF1);
		script->ScriptDeactivate(time, pRobot->pLM->pF2);
		script->ScriptDeactivate(time, pRobot->pLM->pF3);

		script->ScriptActivate(time, pRobot->pRF->pM1);
		script->ScriptActivate(time, pRobot->pRF->pM2);
		script->ScriptActivate(time, pRobot->pRF->pM3);
		script->ScriptDeactivate(time, pRobot->pRF->pF1);
		script->ScriptDeactivate(time, pRobot->pRF->pF2);
		script->ScriptDeactivate(time, pRobot->pRF->pF3);

		script->ScriptActivate(time, pRobot->pRR->pM1);
		script->ScriptActivate(time, pRobot->pRR->pM2);
		script->ScriptActivate(time, pRobot->pRR->pM3);
		script->ScriptDeactivate(time, pRobot->pRR->pF1);
		script->ScriptDeactivate(time, pRobot->pRR->pF2);
		script->ScriptDeactivate(time, pRobot->pRR->pF3);
	}
	inline void Activate024(ROBOT_III *pRobot)
	{
		pRobot->pLF->pSf->Deactivate();
		pRobot->pLF->pM1->Activate();
		pRobot->pLF->pM2->Activate();
		pRobot->pLF->pM3->Activate();
		pRobot->pLF->pF1->Deactivate();
		pRobot->pLF->pF2->Deactivate();
		pRobot->pLF->pF3->Deactivate();

		pRobot->pLR->pSf->Deactivate();
		pRobot->pLR->pM1->Activate();
		pRobot->pLR->pM2->Activate();
		pRobot->pLR->pM3->Activate();
		pRobot->pLR->pF1->Deactivate();
		pRobot->pLR->pF2->Deactivate();
		pRobot->pLR->pF3->Deactivate();

		pRobot->pRM->pSf->Deactivate();
		pRobot->pRM->pM1->Activate();
		pRobot->pRM->pM2->Activate();
		pRobot->pRM->pM3->Activate();
		pRobot->pRM->pF1->Deactivate();
		pRobot->pRM->pF2->Deactivate();
		pRobot->pRM->pF3->Deactivate();

		pRobot->pLM->pSf->Activate();
		pRobot->pLM->pM1->Deactivate();
		pRobot->pLM->pM2->Activate();
		pRobot->pLM->pM3->Activate();
		pRobot->pLM->pF1->Activate();
		pRobot->pLM->pF2->Deactivate();
		pRobot->pLM->pF3->Deactivate();

		pRobot->pRF->pSf->Activate();
		pRobot->pRF->pM1->Deactivate();
		pRobot->pRF->pM2->Activate();
		pRobot->pRF->pM3->Activate();
		pRobot->pRF->pF1->Activate();
		pRobot->pRF->pF2->Deactivate();
		pRobot->pRF->pF3->Deactivate();

		pRobot->pRR->pSf->Activate();
		pRobot->pRR->pM1->Deactivate();
		pRobot->pRR->pM2->Activate();
		pRobot->pRR->pM3->Activate();
		pRobot->pRR->pF1->Activate();
		pRobot->pRR->pF2->Deactivate();
		pRobot->pRR->pF3->Deactivate();
	}
	inline void Activate135(ROBOT_III *pRobot)
	{
		pRobot->pLF->pSf->Activate();
		pRobot->pLF->pM1->Deactivate();
		pRobot->pLF->pM2->Activate();
		pRobot->pLF->pM3->Activate();
		pRobot->pLF->pF1->Activate();
		pRobot->pLF->pF2->Deactivate();
		pRobot->pLF->pF3->Deactivate();

		pRobot->pLR->pSf->Activate();
		pRobot->pLR->pM1->Deactivate();
		pRobot->pLR->pM2->Activate();
		pRobot->pLR->pM3->Activate();
		pRobot->pLR->pF1->Activate();
		pRobot->pLR->pF2->Deactivate();
		pRobot->pLR->pF3->Deactivate();

		pRobot->pRM->pSf->Activate();
		pRobot->pRM->pM1->Deactivate();
		pRobot->pRM->pM2->Activate();
		pRobot->pRM->pM3->Activate();
		pRobot->pRM->pF1->Activate();
		pRobot->pRM->pF2->Deactivate();
		pRobot->pRM->pF3->Deactivate();

		pRobot->pLM->pSf->Deactivate();
		pRobot->pLM->pM1->Activate();
		pRobot->pLM->pM2->Activate();
		pRobot->pLM->pM3->Activate();
		pRobot->pLM->pF1->Deactivate();
		pRobot->pLM->pF2->Deactivate();
		pRobot->pLM->pF3->Deactivate();

		pRobot->pRF->pSf->Deactivate();
		pRobot->pRF->pM1->Activate();
		pRobot->pRF->pM2->Activate();
		pRobot->pRF->pM3->Activate();
		pRobot->pRF->pF1->Deactivate();
		pRobot->pRF->pF2->Deactivate();
		pRobot->pRF->pF3->Deactivate();

		pRobot->pRR->pSf->Deactivate();
		pRobot->pRR->pM1->Activate();
		pRobot->pRR->pM2->Activate();
		pRobot->pRR->pM3->Activate();
		pRobot->pRR->pF1->Deactivate();
		pRobot->pRR->pF2->Deactivate();
		pRobot->pRR->pF3->Deactivate();
	}

}

#endif