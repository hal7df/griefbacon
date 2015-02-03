/*
 * Elevator.h
 *
 *  Created on: Feb 3, 2015
 *      Author: paul
 */

#ifndef SRC_ELEVATOR_H_
#define SRC_ELEVATOR_H_

#include <RobotUtils/HotSubsystem.h>
#include "WPILib.h"

class Elevator: public HotSubsystem {
public:
	friend class HotSubsystemHandler;

	Elevator(Victor* lElevator, Victor* rElevator);
	Elevator(int lElevator, int rElevator);
	virtual ~Elevator();

	void Set (double speed);
protected:
	void Update();
	void PrintData();
private:
	Victor* m_lElevator;
	Victor* m_rElevator;
};

#endif /* SRC_ELEVATOR_H_ */
