/*
 * GyroWrapper.cpp
 *
 *  Created on: Feb 12, 2015
 *      Author: ROBO6
 */

#include "GyroWrapper.h"

GyroWrapper::GyroWrapper(Gyro* gyro, Timer* gyroTime): HotSubsystem("Gyro") {
	// TODO Auto-generated constructor stub
	m_gyro = gyro;

	m_driftTime = new Timer;

	m_gyroTime = new Timer;

	m_driftRatio = 0.0;

	m_driftRatioCase = 0;

}

GyroWrapper::~GyroWrapper() {
	// TODO Auto-generated destructor stub
}

void GyroWrapper::GyroRatio() {

		switch(m_driftRatioCase){
		case 0:
			m_gyro->Reset();
			m_driftTime->Start();
			m_driftTime->Reset();
			m_driftRatioCase++;
			break;
		case 1:
			if(m_driftTime->HasPeriodPassed(5)) {
				m_driftTime->Stop();
				m_driftRatio = (((double)m_gyro->GetAngle()) / 5);
				m_driftRatioCase++;
			}
			break;
		}
	}

double GyroWrapper::PIDGet() {
	return ((double)m_gyro->GetAngle() + (m_driftRatio * m_gyroTime->Get()));
}
void GyroWrapper::Update ()
{

}

void GyroWrapper::PrintData(){
	SmartDashboard::PutNumber("Gyro_wDriftRatio",(double)m_gyro->GetAngle() + (m_driftRatio * m_gyroTime->Get()));
	SmartDashboard::PutNumber("DriftRatio",m_driftRatio);
}