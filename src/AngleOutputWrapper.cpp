/*
 * AngleOutputWrapper.cpp
 *
 *  Created on: Feb 17, 2015
 *      Author: ROBO6
 */

#include <AngleOutputWrapper.h>

AngleOutputWrapper::AngleOutputWrapper(RobotDrive* drive, PIDController* dStraight) {
	m_drive = drive;
	m_dStraight = dStraight;
}

AngleOutputWrapper::~AngleOutputWrapper() {
	// TODO Auto-generated destructor stub
}

void AngleOutputWrapper::PIDWrite(float output) {
	if (!m_dStraight->IsEnabled())
		m_drive->TankDrive(output,-output);
}
