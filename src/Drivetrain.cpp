/*
 * Drivetrain.cpp
 *
 *  Created on: Feb 3, 2015
 *      Author: ROBO6
 */

#include "Drivetrain.h"

Drivetrain::Drivetrain(int lDrive1, int lDrive2, int rDrive1, int rDrive2, int lEncode, int rEncode) :
	HotSubsystem("drivetrain")
{
	// TODO Auto-generated constructor stub
	m_lDrive1 = new Talon (lDrive1);
	m_lDrive2 = new Talon (lDrive2);
	m_rDrive1 = new Talon (rDrive1);
	m_rDrive2 = new Talon (rDrive2);

	m_lEncode = new Encoder (lEncode,lEncode++,false);
	m_rEncode = new Encoder (rEncode, rEncode++,true);

	m_drive = new RobotDrive (m_lDrive1, m_lDrive2, m_rDrive1, m_rDrive2);
	m_drive->SetSafetyEnabled(false);

}

Drivetrain::~Drivetrain() {
	// TODO Auto-generated destructor stub
}

void Drivetrain::Update() {

}

void Drivetrain::PrintData() {
	SmartDashboard::PutNumber("Left Drive 1",m_lDrive1->Get());
	SmartDashboard::PutNumber("Left Drive 2",m_lDrive2->Get());
	SmartDashboard::PutNumber("Right Drive 1",m_rDrive1->Get());
	SmartDashboard::PutNumber("Right Drive 2",m_rDrive2->Get());

	SmartDashboard::PutNumber("m_lEncode Distance", m_lEncode->GetDistance());
	SmartDashboard::PutNumber("m_rEncode Distance", m_rEncode->GetDistance());
}

void Drivetrain::ETA(double time, double distance, double angle)
	{
		double speedSC;
		double speed;
		speed = distance / time;
		speedSC = pow(speed/RATIO_1, 1/RATIO_2);
		switch(m_etaFlag){
			case 0:
				m_lEncode->Reset();
				m_rEncode->Reset();
				m_timer->Reset();
				m_timer->Start();
				m_timer->Reset();
				m_turnPID->SetSetpoint(angle);
				m_FeedbackPID->SetSetpoint(speed);
				m_distancePID->SetSetpoint(distance);
				m_etaFlag++;
				break;
			case 1:
				if(!m_timer->HasPeriodPassed(time * 0.9)){
					m_drive->ArcadeDrive((speedSC + (m_FeedbackPID->Get()/5)), m_turnPID->Get());
					m_FeedbackPID->Enable();
					m_turnPID->Enable();
				}
				else
					m_etaFlag++;
				break;
			case 2:
				m_FeedbackPID->Disable();
				m_timer->Stop();
				m_drive->ArcadeDrive(m_distancePID->Get(), m_turnPID->Get());
				m_distancePID->Enable();

				if (m_lEncode->GetRate() == 0.0 && fabs(m_distancePID->Get()) <= 0.5) {
					m_turnPID->Disable();
					m_distancePID->Disable();
					m_drive->ArcadeDrive(0.0,0.0);
					m_etaFlag++;
				}
				break;
			case 3:

				break;
		}
	}
