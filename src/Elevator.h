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

#define ELEVATOR_P 0.0
#define ELEVATOR_I 0.0
#define ELEVATOR_D 0.0

enum pos_t {
	kBottom,
	kTop,
	klmid,
	kumid
};

class Elevator: public HotSubsystem, public PIDOutput {
public:
	friend class HotSubsystemHandler;

	Elevator(Victor* lElevator, Victor* rElevator, Relay* binExt, Encoder* encode);
	Elevator(int lElevator, int rElevator, int binExt, int encode);

	virtual ~Elevator();

	void Set (double speed);
	void Set (Relay::Value direction);
	void Update();
	void PrintData();

	void PIDWrite (float input) { Set((double)input); }
private:
	Victor* m_lElevator;
	Victor* m_rElevator;
	Relay* m_binExt;
	Encoder* m_elevEncode;
	PIDController* m_Elepid;
};

#endif /* SRC_ELEVATOR_H_ */
