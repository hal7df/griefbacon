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

	m_dStraight = new PIDController ()

}

Drivetrain::~Drivetrain() {
	// TODO Auto-generated destructor stub
}

void Drivetrain::Update() {

}

void Drivetrain::PIDWrite(float input){
	if (input > 0.8)
		input = 0.8;
	else if (input < -0.8)
		input = -0.8;

	if (m_lEncode->GetDistance() + 5 > m_rEncode->GetDistance())
		m_drive->TankDrive(input -0.1, input + 0.1);
	else if (m_rEncode->GetDistance() + 5 > m_lEncode->GetDistance())
		m_drive->TankDrive(input + 0.1, input - 0.1);
	else
		m_drive->TankDrive(input, input);
}
double Drivetrain::PIDGet() {
	return (m_lEncode->GetDistance() + m_rEncode->GetDistance()) /2;
}

void Drivetrain::PrintData() {
	SmartDashboard::PutNumber("Left Drive 1",m_lDrive1->Get());
	SmartDashboard::PutNumber("Left Drive 2",m_lDrive2->Get());
	SmartDashboard::PutNumber("Right Drive 1",m_rDrive1->Get());
	SmartDashboard::PutNumber("Right Drive 2",m_rDrive2->Get());
	SmartDashboard::PutNumber("m_lEncode Distance", m_lEncode->GetDistance());
	SmartDashboard::PutNumber("m_rEncode Distance", m_rEncode->GetDistance());
}
