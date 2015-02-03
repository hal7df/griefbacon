/*
 * Elevator.cpp
 *
 *  Created on: Feb 3, 2015
 *      Author: paul
 */

#include <Elevator.h>

Elevator::Elevator(Victor* lElevator, Victor* rElevator)
	: HotSubsystem("Elevator")
{
	m_lElevator = lElevator;
	m_rElevator = rElevator;
}

Elevator::Elevator(int lElevator, int rElevator)
	: HotSubsystem("Elevator")
{
	m_lElevator = new Victor (lElevator);
	m_rElevator = new Victor (rElevator);
}

Elevator::~Elevator() {
	// TODO Auto-generated destructor stub
}

void Elevator::Set (double speed)
{
	m_lElevator->Set(speed);
	m_rElevator->Set(-speed);
}

void Elevator::Update ()
{

}

void Elevator::PrintData()
{
	SmartDashboard::PutNumber("Elevator Throttle",m_lElevator->Get());
}
