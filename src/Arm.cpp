/*
 * Arm.cpp
 *
 *  Created on: Feb 7, 2015
 *      Author: Robo10
 */

#include "Arm.h"

Arm::Arm(int pickSL, int pickSR, int pickW, int pickRL, int pickRR, int intakeL, int intakeR) :
	HotSubsystem("Arm")
{
	// TODO Auto-generated constructor stub
	m_pickSL = new CANTalon (pickSL);
	m_pickSR = new CANTalon (pickSR);
	m_pickW = new CANTalon (pickW);
	m_pickRL = new CANTalon (pickRL);
	m_pickRR = new CANTalon (pickRR);
	m_intakeL = new CANTalon (intakeL);
	m_intakeR = new CANTalon (intakeR);

	m_shoulderEncode = new Encoder (4,5,false);
	m_shoulderEncode->SetDistancePerPulse(1.0);
	m_wristEncode = new Encoder (6,7,false);
	m_wristEncode->SetDistancePerPulse(1.0);
}

Arm::~Arm() {
	// TODO Auto-generated destructor stub
}

void Arm::shoulderSet(double speed){
	m_pickSL->Set(speed);
	m_pickSR->Set(-speed);
}

void Arm::wristSet(double speed){
	m_pickW->Set(speed);
}

void Arm::rollerSet(double speed){
	m_pickRL->Set(speed);
	m_pickRR->Set(-speed);
}

void Arm::intakeSet(double speed){
	m_intakeL->Set(speed);
	m_intakeR->Set(speed);
}

void Arm::PIDWrite(float input){
	m_pickSL->Set(input);
	m_pickSR->Set(-input);
}

void Arm::PrintData()
{
	SmartDashboard::PutNumber("Arm Shoulder Left",m_pickSL->Get());
	SmartDashboard::PutNumber("Arm Shoulder Right",m_pickSR->Get());
	SmartDashboard::PutNumber("Arm Wrist",m_pickW->Get());
	SmartDashboard::PutNumber("Arm Roller Left",m_pickRL->Get());
	SmartDashboard::PutNumber("Arm Roller Right",m_pickRR->Get());
	SmartDashboard::PutNumber("Arm Intake Left",m_intakeL->Get());
	SmartDashboard::PutNumber("Arm Intake Right",m_intakeR->Get());

	SmartDashboard::PutNumber("Shoulder Encoder",m_shoulderEncode->GetDistance());
	SmartDashboard::PutNumber("Wrist Encoder",m_wristEncode->GetDistance());
}

void Arm::Update()
{

}
