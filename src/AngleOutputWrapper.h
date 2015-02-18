/*
 * AngleOutputWrapper.h
 *
 *  Created on: Feb 17, 2015
 *      Author: ROBO6
 */

#ifndef ANGLEOUTPUTWRAPPER_H_
#define ANGLEOUTPUTWRAPPER_H_

#include "PIDOutput.h"
#include "WPILib.h"

class AngleOutputWrapper: public PIDOutput {
public:
	AngleOutputWrapper(RobotDrive* drive, PIDController* dStraight);
	virtual ~AngleOutputWrapper();

	void PIDWrite (float output);
private:
	RobotDrive* m_drive;
	PIDController* m_dStraight;
};

#endif /* ANGLEOUTPUTWRAPPER_H_ */
