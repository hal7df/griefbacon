#include "HotThread.h"

HotThread::HotThread(const char* id, UINT32 priority)
{
	/***** Set Flags *****/
	f_kill = false;
	f_running = false;
	f_isDead = false;
	
	/***** Initialize Task Object *****/
	m_task = new Task(id, (FUNCPTR)HotThread::TaskStarter, priority);
	m_task->Start((UINT32)this);
}

HotThread::~HotThread()
{
	/***** Stop Thread *****/
	m_task->Stop();
	
	/***** Delete Object *****/
	delete m_task;
}

void HotThread::TaskStarter(HotThread* thread)
{
	while (!thread->f_kill)
	{
		if (thread->f_running)
		{
			thread->Run();
			Wait(0.005);
		}
		else
		{
			Wait(0.02);
		}
	}
	
	thread->f_isDead = true;
}

void HotThread::Start()
{
	f_running = true;
}

void HotThread::Stop()
{
	f_running = false;
}

void HotThread::Kill()
{
	f_kill = true;
}
