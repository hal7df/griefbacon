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

#define ELEVATOR_P 8.0
#define ELEVATOR_I 0.01
#define ELEVATOR_D 0.0

#define ELEVATOR_BOTTOM 0.0
#define ELEVATOR_TOP 1.0
#define ELEVATOR_LMID 0.43
#define ELEVATOR_UMID 0.76
#define ELEVATOR_CARRY 0.114

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

	void Enable ();
	void Disable ();
	bool IsEnabled () { return m_pid->IsEnabled(); }
	void Reset () { m_elevEncode->Reset(); }
	bool AtSetpoint ();

	void PIDWrite (float input) { Set((double)input*0.8); }

	void SetPID (bool set) { f_setPID = set; }

	double GetRate () {return m_elevEncode -> GetRate();}
	double GetDistance () { return m_elevEncode -> GetDistance(); }

	bool GetEStop () { return f_elevEStop; }
	void ResetEStop ();
protected:
	void Update();
	void PrintData();
private:
	double GetPosition (pos_t position);
	void ElevatorEStop();

	Victor* m_lElevator;
	Victor* m_rElevator;
	Relay* m_binExt;
	Encoder* m_elevEncode;
	PIDController* m_pid;

	Timer* m_stopTime;
	sem_t m_semaphore;

	unsigned m_stackCase;

	bool f_setPID;
	bool f_elevEStop;
	bool f_eStopRunning;
	bool f_setpointChanged;
};

#endif /* SRC_ELEVATOR_H_ */
