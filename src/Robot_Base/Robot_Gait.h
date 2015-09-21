#ifndef ROBOT_GAIT_H
#define ROBOT_GAIT_H

#include <functional>
#include <cstdint>

#include <Aris_ControlData.h>
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
        //double modelForcein[18]{0};
        //double actualForcein[18]{0};

		const Aris::RT_CONTROL::CMachineData *pActuationData{nullptr};

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

	struct ADJUST_PARAM :public GAIT_PARAM_BASE
	{
		enum { MAX_PERIOD_NUM = 10};
		
		double targetPee[MAX_PERIOD_NUM][18];
		double targetBodyPE[MAX_PERIOD_NUM][6];
		std::int32_t periodCount[MAX_PERIOD_NUM];
		std::int32_t periodNum;
		char relativeCoordinate[8]{'G',0};
		char relativeBodyCoordinate[8]{ 'G',0 };
	};
	int adjust(ROBOT_BASE * pRobot, const GAIT_PARAM_BASE * pParam);

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
