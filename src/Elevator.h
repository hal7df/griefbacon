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

#define ELEVATOR_P 0.1
#define ELEVATOR_I 0.0
#define ELEVATOR_D 0.0

#define ELEVATOR_BOTTOM 0
#define ELEVATOR_TOP 100
#define ELEVATOR_LMID 33
#define ELEVATOR_UMID 66

enum pos_t {
	kBottom,
	kTop,
	kLMid,
	kUMid
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

	void Enable () { m_pid->Enable(); }
	void Disable (){ m_pid ->Disable();}
	bool IsEnabled(){return m_pid -> IsEnabled();}

	void PIDWrite (float input) { Set((double)input); }
protected:
	void Update();
	void PrintData();
private:
	Victor* m_lElevator;
	Victor* m_rElevator;
	Relay* m_binExt;
	Encoder* m_elevEncode;
	PIDController* m_pid;
};

#endif /* SRC_ELEVATOR_H_ */
