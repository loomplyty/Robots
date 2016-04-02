#include <Platform.h>

#ifdef PLATFORM_IS_LINUX
#include <Aris_Control.h>
#endif

#include <Aris_ControlData.h>
#include <Aris_Socket.h>
#include <Robot_Gait.h>
#include <HexapodIII.h>
#include <string>
#include <sstream>
#include <map>

#include <memory>


namespace Robots {

namespace Filter{

template<int LEN>
class CFilterFIR
{
public:
    CFilterFIR();
    ~CFilterFIR();
    void Filter(double inData,double& outData);
    void ResetReg();
    void FeedData(double inData);
    void GetData(double &outData);
    void SetCoef(double coef[LEN]);

private:
    double m_InData;
    double m_OutData;
    int m_CurrentIndex=0;
    int m_FilterLength=LEN;
    double m_Coef[LEN];
    double m_Reg[LEN];

};


template<int LEN>
CFilterFIR<LEN>::CFilterFIR()
{
    memset(m_Reg,0,m_FilterLength*sizeof(double));
    //memset(m_Coef,0,m_FilterLength*sizeof(double));

    m_CurrentIndex=0;
    m_InData=0;
    m_OutData=0;

}

template<int LEN>
CFilterFIR<LEN>::~CFilterFIR()
{
    return;
}

template<int LEN>
void CFilterFIR<LEN>::ResetReg()
{
    memset(this->m_Reg,0,this->m_FilterLength*sizeof(double));
    this->m_CurrentIndex=0;
    this->m_InData=0;
    this->m_OutData=0;

}

template<int LEN>
void CFilterFIR<LEN>::Filter(double inData, double &outData)
{
    m_InData=inData;
    //shift data
//    memcpy(&this->m_reg[1],&this->m_reg[0],(this->m_FilterLength-1)*sizeof(double));
    m_Reg[m_CurrentIndex]=inData;
    m_CurrentIndex++;
    if(m_CurrentIndex==LEN)
        this->m_CurrentIndex=0;
    outData=0;
    for(int i=0;i<m_FilterLength;i++)
    {
        outData+=m_Reg[
                (m_CurrentIndex-i)<0?
                    m_FilterLength+m_CurrentIndex-i:
                    m_CurrentIndex-i]*m_Coef[i];
    }
    m_OutData=outData;

}

template<int LEN>
void CFilterFIR<LEN>::GetData(double &outData)
{
    outData=m_OutData;
}

template<int LEN>
void CFilterFIR<LEN>::FeedData(double inData)
{
    m_InData=inData;

    //shift data
//    memcpy(&this->m_reg[1],&this->m_reg[0],(this->m_FilterLength-1)*sizeof(double));
    m_Reg[m_CurrentIndex]=inData;
    m_CurrentIndex++;
    if(m_CurrentIndex==LEN)
        m_CurrentIndex=0;
    m_OutData=0;
    for(int i=0;i<m_FilterLength;i++)
    {
        m_OutData+=m_Reg[
                (m_CurrentIndex-i)<0?
                    m_FilterLength+m_CurrentIndex-i:
                    m_CurrentIndex-i]*m_Coef[i];
    }

}

template<int LEN>
void CFilterFIR<LEN>::SetCoef(double coef[LEN])
{
    for(int i=0;i<LEN;i++)
    {
        m_Coef[i]=coef[i];
    }

}


// specialize a Filter for Rofo

class CFilterFIR_I: public CFilterFIR<41>
{
public:
    CFilterFIR_I()
    {

        double coef[41]={
                        0.01991188823604,  0.02053491965099,  0.02113804146569,  0.02171951567691,
                        0.02227765575763,  0.02281083311437,  0.02331748334669,  0.02379611228109,
                        0.02424530175286,  0.02466371510982,  0.02505010241382,  0.02540330531656,
                        0.02572226158817,  0.0260060092781,   0.02625369048986,  0.02646455475257,
                        0.02663796197434,  0.02677338496401,  0.02687041151014,  0.02692874600769,
                        0.02694821062531,  0.02692874600769,  0.02687041151014,  0.02677338496401,
                        0.02663796197434,  0.02646455475257,  0.02625369048986,  0.0260060092781,
                        0.02572226158817,  0.02540330531656,  0.02505010241382,  0.02466371510982,
                        0.02424530175286,  0.02379611228109,  0.02331748334669,  0.02281083311437,
                        0.02227765575763,  0.02171951567691,  0.02113804146569,  0.02053491965099,
                        0.01991188823604
                    };
        SetCoef(coef);
    }

};



}


	typedef std::function<Aris::Core::MSG(const std::string &cmd, const std::map<std::string, std::string> &params)> PARSE_FUNC;
	typedef std::function<int(ROBOT_BASE *, const GAIT_PARAM_BASE *,Aris::RT_CONTROL::CMachineData &)> GAIT_ONLINE_FUNC;
    const double meter2count = 1 / 0.01*3.5 * 65536;
    const double current2force=84.9*9.38*(2*PI*3.5/0.01)/1000000;


	enum ROBOT_CMD_ID
	{
		ENABLE,
		DISABLE,
		HOME,
		RESET_ORIGIN,
		RUN_GAIT,
		ROBOT_CMD_COUNT
	};
	
	class ROBOT_SERVER
	{
	public:
		static ROBOT_SERVER * GetInstance()
		{
			static ROBOT_SERVER instance;
			return &instance;
		}

		template<typename T>
		void CreateRobot() 
		{
			if (pRobot.get() == nullptr)
			{
				pRobot = std::unique_ptr<Robots::ROBOT_BASE>{ new T };
			}
			else
			{
				throw std::logic_error("already has a robot instance");
			}
		};
		void LoadXml(const char *fileName);
		void AddGait(std::string cmdName, GAIT_FUNC gaitFunc, PARSE_FUNC parseFunc);
		void AddOnlineGait(std::string cmdName, GAIT_ONLINE_FUNC gaitFunc, PARSE_FUNC parseFunc);

		void Start();

	private:
		ROBOT_SERVER() = default;
		ROBOT_SERVER(const ROBOT_SERVER&) = delete;

		void DecodeMsg(const Aris::Core::MSG &msg, std::string &cmd, std::map<std::string, std::string> &params);
		void GenerateCmdMsg(const std::string &cmd, const std::map<std::string, std::string> &params, Aris::Core::MSG &msg);
		void ExecuteMsg(const Aris::Core::MSG &m, Aris::Core::MSG &retError);
		
		void inline p2a(const int *phy, int *abs, int num = 18)
		{
			for (int i = 0; i<num; ++i)
			{
				abs[i] = mapPhy2Abs[phy[i]];
			}
		}
		void inline a2p(const int *abs, int *phy, int num = 18)
		{
			for (int i = 0; i<num; ++i)
			{
				phy[i] = mapAbs2Phy[abs[i]];
			}
		}

		int home(Robots::ROBOT_BASE *pRobot, const Robots::GAIT_PARAM_BASE *param, Aris::RT_CONTROL::CMachineData &data);
		int enable(Robots::ROBOT_BASE *pRobot, const Robots::GAIT_PARAM_BASE *param, Aris::RT_CONTROL::CMachineData &data);
		int disable(Robots::ROBOT_BASE *pRobot, const Robots::GAIT_PARAM_BASE *param, Aris::RT_CONTROL::CMachineData &data);
		int resetOrigin(Robots::ROBOT_BASE *pRobot, const Robots::GAIT_PARAM_BASE *param, Aris::RT_CONTROL::CMachineData &data);
        int runGait(Robots::ROBOT_BASE *pRobot,  Robots::GAIT_PARAM_BASE *pParam, Aris::RT_CONTROL::CMachineData &data);

        int LegImpedance(ROBOT_BASE *pRobot,const int LegID,const int count,FORCE_PARAM_BASE *pForce, const Aris::RT_CONTROL::CMachineData & pData);
        int ImpedanceAlgorithm(ROBOT_BASE *pRobot,const int count,FORCE_PARAM_BASE *pForce, const Aris::RT_CONTROL::CMachineData & pData);

        int LegStateAndTransition(FORCE_PARAM_BASE *pForce,const Aris::RT_CONTROL::CMachineData & pData);

        int RobotStateMachine(FORCE_PARAM_BASE *pForce);

        int execute_cmd(int count, char *cmd, Aris::RT_CONTROL::CMachineData &data,Aris::Core::RT_MSG& msgSend);
		static int tg(Aris::RT_CONTROL::CMachineData &data, Aris::Core::RT_MSG &recvMsg, Aris::Core::RT_MSG &sendMsg);

	private:
        std::unique_ptr<Robots::ROBOT_BASE> pRobot;
        Robots::ROBOT_III Robot_for_cal;
        Robots::Filter::CFilterFIR_I vin_filter[18];
        Robots::Filter::CFilterFIR_I pin_filter[18];
        Robots::Filter::CFilterFIR_I fin_filter[18];

		std::map<std::string, int> mapName2ID;
		std::vector<GAIT_FUNC> allGaits;
		std::vector<GAIT_ONLINE_FUNC> allOnlineGaits;

		std::vector<PARSE_FUNC> allParsers;
		std::vector<PARSE_FUNC> allOnlineParsers;

		Aris::Core::CONN server;
		std::string ip,port;

		double homeEE[18], homeIn[18];
		int homeCount[18];
		int homeCur{ 0 };
		double meter2count{ 0 };

		int mapPhy2Abs[18];
		int mapAbs2Phy[18];
#ifdef PLATFORM_IS_LINUX
		Aris::RT_CONTROL::ACTUATION cs;
#endif

	};

}


