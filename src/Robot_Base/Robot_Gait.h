#ifndef ROBOT_GAIT_H
#define ROBOT_GAIT_H

#include <functional>
#include <cstdint>
#include <map>

#include <Aris_ControlData.h>
#include <Aris_Core.h>
#include <Robot_Base.h>

namespace Robots
{
	class ROBOT_BASE;

	struct GAIT_PARAM_BASE //generated every cycle
	{
		std::int32_t cmdType{0};
		std::int32_t cmdID{0};
		std::int32_t count{0};
		std::int16_t motorNum{18};
		std::int16_t legNum{6};
		std::int32_t motorID[18]{0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17};
		std::int32_t legID[6]{0,1,2,3,4,5};
		double beginPee[18]{0};
		double beginVee[18]{0};
		double beginBodyPE[6]{0};
		double beginBodyVel[6]{0};
        Aris::RT_CONTROL::EOperationMode actuationMode{Aris::RT_CONTROL::OM_CYCLICVEL};

        double modelForcein[18]{0};
        double modelFrictionin[18]{0};
        double actualForcein[18]{0};
        double modelAccee[18]{0};
        double modelVelee[18]{0};


		const Aris::RT_CONTROL::CMachineData *pActuationData{nullptr};

	};


    enum LEGSTATE
    {
        LEG_INIT=0,
        SWING=1,
        TOUCHDOWN=2,
        SUPPORT=3,
        LIFTOFF=4,
    };

    enum ROBOTSTATE
    {
        INIT=0,
        PHASE1=1,
        PHASE2=2,
        PHASE3=3,
        PHASE4=4,
        PHASE5=5,
        PHASE6=6,
        FINISHED=7,
     };

    enum ROBOTEVENT
    {
        NONE=-1,
        BEGIN=0,
        TOUCHDOWN_LEG1=1,
        SUPPORT_LEG1=2,
        TOUCHDOWN_LEG2=3,
        SUPPORT_LEG2=4,
        TOUCHDOWN_LEG3=5,
        SUPPORT_LEG3=6,
        END=7,
    };

    struct FORCE_PARAM_BASE
    {
        double Fin_modeled[18]{0};
        double Fin_read[18]{0};
        double Fin_read_filtered[18]{0};
        double Fin_write[18]{0};
        double Fee_ext[18]{0};
        double Vee_desired[18]{0};
        double Pee_desired[18]{0};
        double Vee_filtered[18]{0};
        double Pee_filtered[18]{0};
        LEGSTATE LegState[6]{Robots::LEGSTATE::LEG_INIT};
        ROBOTSTATE RobotState{Robots::ROBOTSTATE::INIT};
        ROBOTEVENT RobotEvent{Robots::ROBOTEVENT::NONE};
        int eventLegID{0};
        double Force_threshold{0};
        double Kp[18];
        double Kd[18];
    };


	struct WALK_PARAM :public GAIT_PARAM_BASE
	{
		std::int32_t totalCount{3000};
		std::int32_t n{1};
		std::int32_t walkDirection{-3};// 1 means positive x axis; while -3 means negative z axis
		std::int32_t upDirection{2};
		double d{0.5};
		double h{0.05};
		double alpha{0};
		double beta{0};
	};

    int walk(ROBOT_BASE * pRobot, const GAIT_PARAM_BASE * pParam);
	int walk2(ROBOT_BASE * pRobot, const GAIT_PARAM_BASE * pParam);
	Aris::Core::MSG parseWalk(const std::string &cmd, const std::map<std::string, std::string> &params);
	


	struct ADJUST_PARAM :public GAIT_PARAM_BASE
	{
		enum { MAX_PERIOD_NUM = 10};
		
		double targetPee[MAX_PERIOD_NUM][18];
		double targetBodyPE[MAX_PERIOD_NUM][6];
		std::int32_t periodCount[MAX_PERIOD_NUM]{1000};
		std::int32_t periodNum{1};
		char relativeCoordinate[8]{'G',0};
		char relativeBodyCoordinate[8]{ 'G',0 };
	};
	int adjust(ROBOT_BASE * pRobot, const GAIT_PARAM_BASE * pParam);
	Aris::Core::MSG parseAdjust(const std::string &cmd, const std::map<std::string, std::string> &params);

	struct MOVE_PARAM :public GAIT_PARAM_BASE
	{
		double targetPee[18];
		double targetVee[18];
		double targetBodyVel[6];
		double targetBodyPE[6];
		std::int32_t totalCount;
	};
}

#endif
