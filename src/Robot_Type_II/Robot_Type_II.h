#ifndef ROBOT_IV_H
#define ROBOT_IV_H

#include <aris.h>
#include <fstream>

#include "Robot_Base.h"	



//需要修改默认的堆栈大小
//在windows里，位于程序属性->链接器->系统->堆栈保留大小


namespace Robots
{
	class LEG_IV:public Robots::LegBase
	{
	public:
		LEG_IV(RobotBase* pRobot, const char *Name);
		~LEG_IV() = default;

		virtual void calculate_from_pEE();

	private:
		/*你需要修改这里，这里是单腿你要用到的尺寸变量*/
		const double U2x{ 0 }, U2y{ 0.234 }, U2z{ 0.135 }, U3x{ 0 }, U3y{ 0.234 }, U3z{ -0.135 };
		const double S2x{ 0 }, S2y{ 0.059 }, S2z{ 0.034 }, S3x{ 0 }, S3y{ 0.059 }, S3z{ -0.034 };
		const double Sfx{ 0.1 }, Sfy{ 0 }, Sfz{ 0 };
	
		friend class ROBOT_IV;
	};

	class ROBOT_IV :public Robots::RobotBase
	{
		LEG_IV leg0{ this, "LF" };
		LEG_IV leg1{ this, "LM" };
		LEG_IV leg2{ this, "LR" };
		LEG_IV leg3{ this, "RF" };
		LEG_IV leg4{ this, "RM" };
		LEG_IV leg5{ this, "RR" };

	public:
		ROBOT_IV();
		~ROBOT_IV() = default;
	};
}

#endif