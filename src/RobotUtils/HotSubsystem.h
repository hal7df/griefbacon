#ifndef HOTSUBSYSTEM_H
#define HOTSUBSYSTEM_H

#include "WPILib.h"
#include "HotThread.h"
#include <vector>
//#include <string>


class HotSubsystem
{
public:
	/********** Constructor **********/
	HotSubsystem(std::string id) { m_id = id; }
	virtual ~HotSubsystem();
	
	/********** Constents **********/
	virtual void Update() {}
	virtual void PrintData() {}
	
	/********** Internal Functions **********/
	void Print (std::string id, float value);
	void Print (std::string id, bool value);
	
private:
	std::string m_id;
};


class HotSubsystemHandler : public HotThread
{
public:
	/********** Constructor **********/
	HotSubsystemHandler();
	~HotSubsystemHandler();
	
	/********** Add New Subsystem to Handler **********/
	void Add(HotSubsystem* subsystem);
	
	/********** Flag Control **********/
	void SetUpdate(bool on) { f_runUpdate = on; }
	void SetPrintData(bool on) { f_runPrintData = on; }
	bool GetUpdate() { return f_runUpdate; }
	bool GetPrintData() { return f_runPrintData; }
	
private:
	std::vector<HotSubsystem*> m_subsystems;
	
	/********** Main Function **********/
	void Run();
	
	/********** Flags **********/
	bool f_runUpdate;
	bool f_runPrintData;
};


#endif
