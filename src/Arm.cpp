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

void Arm::PrintData()
{

}

void Arm::Update()
{

}
