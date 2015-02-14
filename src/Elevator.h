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
#include <cmath>

//PID gains
#define ELEVATOR_P 8.0
#define ELEVATOR_I 0.01
#define ELEVATOR_D 0.0

//PID setpoints
#define ELEVATOR_BOTTOM 0.0
#define ELEVATOR_TOP 1.0
#define ELEVATOR_LMID 0.43
#define ELEVATOR_UMID 0.76
#define ELEVATOR_CARRY 0.177

//Profile times
#define ELEVATOR_TIME_FULL 0.0

enum pos_t {
	kBottom,
	kTop,
	kLMid,
	kUMid,
	kCarry
};

struct co_t {
	double a3;
	double a4;
	double a5;
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

	void Enable ();
	void Disable ();
	bool IsEnabled();
	bool AtPosition () { return m_profileCase > 0; }
	void Reset () { m_elevEncode->Reset(); }

	void PIDWrite (float input) { Set((double)input); }

	void SetPID (bool set) { f_setPID = set; }
	void SetOldPID (bool old) { f_oldPID = old; Disable(); }
protected:
	void Update();
	void PrintData();
private:
	void SetSetpoint_old (pos_t position);
	void SetSetpoint (pos_t position);

	void FindCoefficients ();
	float GetProfilePosition (double time);

	double GetProfileTime (double start, double end);
	double GetDelta (double start, double end);
	double GetReverseDelta (double start, double end);
	double GetPosition (pos_t position);

	Victor* m_lElevator;
	Victor* m_rElevator;
	Relay* m_binExt;
	Encoder* m_elevEncode;
	PIDController* m_pid;
	Timer* m_timer;

	bool f_setPID;
	bool f_oldPID;
	bool f_pidRunning;

	unsigned m_profileCase;

	double m_setpoint;
	double m_lastSetpoint;
	co_t m_coeff;
};

#endif /* SRC_ELEVATOR_H_ */
