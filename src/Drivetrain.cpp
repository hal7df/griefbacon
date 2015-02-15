/*
 * Drivetrain.cpp
 *
 *  Created on: Feb 3, 2015
 *      Author: ROBO6
 */

#include "Drivetrain.h"

Drivetrain::Drivetrain(int lDrive1, int lDrive2, int rDrive1, int rDrive2, int lEncode, int rEncode, int gyro) :
	HotSubsystem("Drivetrain")
{
	// TODO Auto-generated constructor stub
	m_lDrive1 = new Talon (lDrive1);
	m_lDrive2 = new Talon (lDrive2);
	m_rDrive1 = new Talon (rDrive1);
	m_rDrive2 = new Talon (rDrive2);
	m_dummy = new Talon (9001);

	m_lEncode = new Encoder (lEncode,lEncode++,false);
	m_rEncode = new Encoder (rEncode, rEncode++,true);
	m_lEncode->SetDistancePerPulse(1./1200.);
	m_rEncode->SetDistancePerPulse(1./1200.);

	m_FeedbackWrapper = new FeedbackWrapper(m_lEncode, m_rEncode);
	m_distancePIDWrapper = new DistancePIDWrapper(m_lEncode, m_rEncode);

	m_etaFlag = 0;

	m_timer = new Timer;

	m_drive = new RobotDrive (m_lDrive1, m_lDrive2, m_rDrive1, m_rDrive2);
	m_drive->SetSafetyEnabled(false);

	m_GyroWrapper = new GyroWrapper(gyro);

	m_turnPID = new PIDController(GYRO_P, GYRO_I, GYRO_D, m_GyroWrapper, m_dummy);
	m_distancePID = new PIDController(DISTANCE_P,DISTANCE_I,DISTANCE_D,m_distancePIDWrapper, this);
	m_FeedbackPID = new PIDController(FEEDBACK_P,FEEDBACK_I,FEEDBACK_D, m_FeedbackWrapper, m_dummy);

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

	SmartDashboard::PutNumber("m_timer", m_timer->Get());
	SmartDashboard::PutNumber("ETA:",  4.0 - m_timer->Get());
	SmartDashboard::PutNumber("m_etaFlag", m_etaFlag);
	SmartDashboard::PutNumber("Set Point", m_turnPID->GetSetpoint());

	SmartDashboard::PutNumber("m_turnPID",m_turnPID->Get());
	SmartDashboard::PutBoolean("m_turnPID IsEnabled", m_turnPID->IsEnabled());
	SmartDashboard::PutNumber("m_FeedbackPID",m_FeedbackPID->Get());
	SmartDashboard::PutNumber("Auto Drive P", m_distancePID->GetP());
	SmartDashboard::PutNumber("Auto Drive I", m_distancePID->GetI());
	SmartDashboard::PutNumber("Auto Drive D", m_distancePID->GetD());
	SmartDashboard::PutNumber("Distance PIDGet", m_distancePID->Get());
	SmartDashboard::PutNumber("Auto Drive Setpoint", m_distancePID->GetSetpoint());
	SmartDashboard::PutBoolean("Auto Drive Enabled", m_distancePID->IsEnabled());
	SmartDashboard::PutNumber("Encoder Rate Left", m_lEncode->GetRate() / 1200);
	SmartDashboard::PutNumber("Encoder Rate Right", m_rEncode->GetRate() / 1200);
	SmartDashboard::PutNumber("Encoder Rate Average", ((m_lEncode->GetRate() / 1200) - (m_rEncode->GetRate() / 1200)) / 2);

	SmartDashboard::PutNumber("Feedback PID",m_FeedbackPID->Get()/5);
	SmartDashboard::PutNumber("Angle", m_GyroWrapper->GetAngle());
	SmartDashboard::PutNumber("Rate", m_GyroWrapper->GetRate());
	SmartDashboard::PutNumber("Left Encoder",m_lEncode->GetDistance());
	SmartDashboard::PutNumber("Right Encoder",m_rEncode->GetDistance());

}

void Drivetrain::PIDWrite(float output)
{
	if (output > 0.8)
		output = 0.8;
	else if (output < -0.8)
		output = -0.8;

	if (m_lEncode->GetDistance() + 5 > m_rEncode->GetDistance())
		m_drive->TankDrive(output - 0.1, output + 0.1);
	else if (m_rEncode->GetDistance() + 5 > m_lEncode->GetDistance())
			m_drive->TankDrive(output + 0.1, output - 0.1);
	else
		m_drive->TankDrive(output, output);
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
