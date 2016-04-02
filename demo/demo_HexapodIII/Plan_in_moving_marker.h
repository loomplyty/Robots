#include <Aris_Plan.h>
#include <Aris_Thread.h>
#include <Aris_DynKer.h>

#include "Hexapod_Robot.h"

extern Hexapod_Robot::ROBOT robot;
extern Aris::Plan::TRAJECTORY_GENERATOR<3, 3> generator;

namespace PlanInOtherMarker
{
	int Plan_Output_Const();
	int Plan_Output_Acc();
	int Plan_Output_Dec();
}