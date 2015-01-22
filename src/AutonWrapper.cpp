/*
 * AutonWrapper.cpp
 *
 *  Created on: Jan 20, 2015
 *      Author: ROBO6
 */

#include "AutonWrapper.h"

AutonWrapper::AutonWrapper(RobotDrive* drive, Encoder* encodeL, Encoder* encodeR) {
	// TODO Auto-generated constructor stub

	m_PID = new PIDController (AUTO_P, AUTO_I, AUTO_D, this, this);
	m_drive = drive;
	m_encodeL = encodeL;
	m_encodeR = encodeR;

}

AutonWrapper::~AutonWrapper() {
	// TODO Auto-generated destructor stub
}

double AutonWrapper::PIDGet () {
	return((m_encodeL->GetDistance() + m_encodeR->GetDistance()) / 2);
}

void AutonWrapper::PIDWrite (float output) {
	if (output > 0.8)
		output = 0.8;
	else if (output < -0.8)
		output = -0.8;

	if (m_encodeL->GetDistance() + 5 > m_encodeR->GetDistance())
		m_drive->TankDrive(output - 0.1, output + 0.1);
	else if (m_encodeR->GetDistance() + 5 > m_encodeL->GetDistance())
		m_drive->TankDrive(output + 0.1, output - 0.1);
	else
		m_drive->TankDrive(output, output);

}

