/*
 * AngleOutputWrapper.cpp
 *
 *  Created on: Feb 17, 2015
 *      Author: ROBO6
 */

#include <AngleOutputWrapper.h>

AngleOutputWrapper::AngleOutputWrapper(RobotDrive* drive) {
	m_drive = drive;
}

AngleOutputWrapper::~AngleOutputWrapper() {
	// TODO Auto-generated destructor stub
}

void AngleOutputWrapper::PIDWrite(float output) {
		m_drive->TankDrive(output,-output);
}
