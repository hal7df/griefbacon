#include "HotSubsystem.h"

/*
 * Hot Subsystem
 */

/********** Internal Function: Print Float Value **********/
void HotSubsystem::Print(std::string id, float value)
{
	SmartDashboard::PutNumber(m_id + ":" + id, value);
}
/********** Internal Function: Printa Boolean Value **********/
void HotSubsystem::Print(std::string id, bool value)
{
	SmartDashboard::PutBoolean(m_id + ":" + id, value);
}


/*
 * Hot Subsystem Handler
 */

/********** Constructor **********/
HotSubsystemHandler::HotSubsystemHandler()
	: HotThread("HotSubsystemHandler") 
{
	/***** Set Default Flag *****/
	f_runUpdate = true;
	f_runPrintData = true;
}

/********** Deconstructor **********/
HotSubsystemHandler::~HotSubsystemHandler()
{
	/***** Stop Thread *****/
	Stop();
	
	/***** Delete List of Subsystems *****/
	delete &m_subsystems;
}

/********** Add New Subsystem **********/
void HotSubsystemHandler::Add(HotSubsystem* subsystem)
{
	m_subsystems.push_back(subsystem);
}

/********** Main Function **********/
void HotSubsystemHandler::Run()
{
	/***** Each Subsystems *****/
	for (unsigned int i = 0; i < m_subsystems.size(); ++i)
	{
		if (f_runUpdate)
		{
			m_subsystems[i]->Update();
		}
		if (f_runUpdate)
		{
			m_subsystems[i]->PrintData();
		}
	}
}
