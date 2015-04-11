/*
 * Elevator.h
 *
 *  Created on: Feb 3, 2015
 *      Author: paul
 */

#ifndef SRC_ELEVATOR_H_
#define SRC_ELEVATOR_H_

#include "RobotUtils/HotSubsystem.h"
#include "RobotUtils/RobotUtils.h"
#include "WPILib.h"
#include <semaphore.h>

//#define PRACTICE_BOT
#define COMPETITION_BOT

#ifdef PRACTICE_BOT
#define ELEVATOR_CARRY 0.180
#endif

#ifdef COMPETITION_BOT
#define ELEVATOR_CARRY 0.12 //0.114 //0.107
#endif

#define ELEVATOR_P 15.0
#define ELEVATOR_I 0.01
#define ELEVATOR_D 0.0

#define ELEVATOR_BOTTOM 0.0000
#define ELEVATOR_TOP 1.0000
#define ELEVATOR_LMID 0.4700
#define ELEVATOR_UMID 0.7600

enum pos_t {
	kBottom,
	kTop,
	kLMid,
	kUMid,
	kCarry
};

class Elevator: public HotSubsystem, public PIDOutput {
public:
	friend class HotSubsystemHandler;

	Elevator(Victor* lElevator, Victor* rElevator, Relay* binExt, Encoder* encode);
	Elevator(int lElevator, int rElevator, int binExt, int encode);

	virtual ~Elevator();

	void Set (double speed);
	void Set (Relay::Value direction);
	void Set (pos_t position);

	void Stack (pos_t finish);
	void AbortStack () { m_stackCase = 3; }
	bool Stacking () { return m_stackCase < 3; }

	void Enable ();
	void Disable ();
	bool IsEnabled () { return m_pid->IsEnabled(); }
	void Reset () { m_elevEncode->Reset(); m_pid->Reset(); }
	bool AtSetpoint ();
	bool AtSetpoint (float tolerance);
	pos_t GetSetpoint () { return GetSetpoint(m_pid->GetSetpoint()); }

	void PIDWrite (float input) { Set((double)input); }

	void SetPID (bool set) { f_setPID = set; }

	double GetRate () {return m_elevEncode -> GetRate();}
	double GetDistance () { return m_elevEncode -> GetDistance(); }

	Encoder* GetEncoder() { return m_elevEncode; }
protected:
	void Update();
	void PrintData();
private:
	double GetPosition (pos_t position);
	pos_t GetSetpoint (double position);
	void Stack_internal();
	void ElevatorEStop();

	Victor* m_lElevator;
	Victor* m_rElevator;
	Relay* m_binExt;
	Encoder* m_elevEncode;
	PIDController* m_pid;

	Timer* m_stackTime;

	unsigned m_stackCase;
	pos_t m_stackFin;

	bool f_stacking;

	bool f_setPID;
};

#endif /* SRC_ELEVATOR_H_ */
