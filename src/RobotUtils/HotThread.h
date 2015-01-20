#ifndef THREAD_H
#define THREAD_H

#include "WPILib.h"

class HotThread
{
public:
	/********** Constructor **********/
	HotThread(const char* id, UINT32 priority = Task::kDefaultPriority);
	virtual ~HotThread();
	
	/********** Control Thread **********/
	void Start();
	void Stop();
	void Kill();
	
	/********** Access Status **********/
	bool IsDead() { return f_isDead; }
	bool IsRunning() { return f_running; }
	
private:
	/********** Starter Function **********/
	static void TaskStarter(HotThread* thread);
	
	/********** Main Function **********/
	/*
	 * This function will run in loop
	 */
	virtual void Run() = 0;
	
	/********** Flags **********/
	bool f_kill;
	bool f_running;
	bool f_isDead;
	
	/********** Task **********/
	Task* m_task;
};

#endif
