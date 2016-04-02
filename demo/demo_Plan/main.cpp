
#include "plan.h"

int main()
{

	plan_prepare();
	plan_acc("C:\\Users\\yang\\Desktop\\pIn_acc.txt");
	plan_const("C:\\Users\\yang\\Desktop\\pIn_const.txt");
	plan_dec("C:\\Users\\yang\\Desktop\\pIn_dec.txt");
	

	//double pEE_all[NUM][3];
	//double pIn_all[NUM][3];

	//for (int i = 0; i < NUM; ++i)
	//{
	//	b_const(tg.Result().at(i), pEE_all[i]);

	//	double pe[6]{ 0, 0, v * (i+1)*0.001, 0, 0, 0 };
	//	double pm[16];
	//	s_pe2pm(pe, pm);
	//	robot.pBody->setPm(pm);
	//	robot.pLegs[leg_index]->SetPee(pEE_all[i]);
	//	robot.pLegs[leg_index]->GetPin(pIn_all[i]);
	//}
	//aris::dynamic::dlmwrite("C:\\Users\\yang\\Desktop\\pIn.txt", *pIn_all, NUM, 3);
	//aris::dynamic::dlmwrite("C:\\Users\\yang\\Desktop\\pEE.txt", *pEE_all, NUM, 3);

//#define NUM 1900
//	double s[NUM];
//	double ds[NUM];
//	double dds[NUM];
//
//
//
//	for (auto p = tg.list.begin(); p != tg.list.end(); ++p)
//	{
//		static auto i = 0;
//
//
//		s[i] = p->s;
//		ds[i] = p->ds;
//		dds[i] = p->dds;
//		++i;
//	}
//	aris::dynamic::dlmwrite("C:\\Users\\yang\\Desktop\\s.txt", s, tg.list.size(), 1);
//	aris::dynamic::dlmwrite("C:\\Users\\yang\\Desktop\\ds.txt", ds, tg.list.size(), 1);
//	aris::dynamic::dlmwrite("C:\\Users\\yang\\Desktop\\dds.txt", dds, tg.list.size(), 1);

	//std::cout << "size:" << tg.list.size();

	char aaa;
	std::cin >> aaa;
	return 0;
}

