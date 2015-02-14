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

	m_wristPid = new PIDController(WRIST_P,WRIST_I,WRIST_D,m_wristEncode,m_pickW);
	m_shoulderPid = new PIDController(SHOULDER_P, SHOULDER_I, SHOULDER_D, m_shoulderEncode, this);
	f_getPID = false;

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

void Arm::shoulderSetPos (sPos_t position)
{
	switch (position){
	case ksGround:
		m_shoulderPid->SetSetpoint(SHOULDER_GROUND);
		break;
	case ksTwoTote:
		m_shoulderPid->SetSetpoint(SHOULDER_TWOTOTE);
		break;
	case ksMid:
		m_shoulderPid->SetSetpoint(SHOULDER_MID);
		break;
	case ksCan:
		m_shoulderPid->SetSetpoint(SHOULDER_CAN);
		break;
	case ksPackage:
		m_shoulderPid ->SetSetpoint(SHOULDER_PACKAGE);
		break;
	}

	if (!m_shoulderPid->IsEnabled())
		m_shoulderPid->Enable();
}


void Arm::wristSetPos (wPos_t position)
{
	switch (position){
	case kwGround:
		m_wristPid->SetSetpoint(WRIST_GROUND);
		break;
	case kwTwoTote:
		m_wristPid->SetSetpoint(WRIST_TWOTOTE);
		break;
	case kwMid:
		m_wristPid->SetSetpoint(WRIST_MID);
		break;
	case kwCan:
		m_wristPid->SetSetpoint(WRIST_CAN);
		break;
	case kwPackage:
		m_wristPid ->SetSetpoint(WRIST_PACKAGE);
		break;
}

	if (!m_wristPid->IsEnabled())
		m_wristPid->Enable();
}


void Arm::shoulderSetSetpoint(int point){
	m_shoulderPid->SetSetpoint(point);
}

void Arm::wristSetSetpoint(int point){
	m_wristPid->SetSetpoint(point);
}

void Arm::Enable(int pid){
	switch(pid){
	case 0:
		m_wristPid->Enable();
		m_shoulderPid->Enable();
		break;

	case 1:
		m_shoulderPid->Enable();
		break;

	case 2:
		m_wristPid->Enable();
		break;
	}
}
void Arm::Disable(int pid){
	switch(pid){
	case 0:
		m_wristPid->Disable();
		m_shoulderPid->Disable();
		break;

	case 1:
		m_shoulderPid->Disable();
		break;

	case 2:
		m_wristPid->Disable();
		break;
	}
}


void Arm::PIDWrite(float input){
	shoulderSet((double)input);
}

void Arm::PrintData()
{
	if (f_getPID)
	{
		m_shoulderPid->SetPID(SmartDashboard::GetNumber("Shoulder P"),SmartDashboard::GetNumber("Shoulder I"),SmartDashboard::GetNumber("Shoulder D"));
		m_wristPid->SetPID(SmartDashboard::GetNumber("Wrist P"),SmartDashboard::GetNumber("Wrist I"),SmartDashboard::GetNumber("Wrist D"));
	}
	else
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

		SmartDashboard::PutNumber("Shoulder PID Output",m_shoulderPid->Get());
		SmartDashboard::PutNumber("Wrist PID Output",m_wristPid->Get());

	}
}

void Arm::Update()
{

}
