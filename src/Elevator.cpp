/*
 * Elevator.cpp
 *
 *  Created on: Feb 3, 2015
 *      Author: paul
 */

#include <Elevator.h>

Elevator::Elevator(Victor* lElevator, Victor* rElevator, Relay* binExt)
	: HotSubsystem("Elevator")
{
	m_lElevator = lElevator;
	m_rElevator = rElevator;
	m_binExt = binExt;
}

Elevator::Elevator(int lElevator, int rElevator, int binExt)
	: HotSubsystem("Elevator")
{
	m_lElevator = new Victor (lElevator);
	m_rElevator = new Victor (rElevator);
	m_binExt = new Relay (binExt);
}

Elevator::~Elevator() {
	// TODO Auto-generated destructor stub
}

void Elevator::Set (double speed)
{
	m_lElevator->Set(speed);
	m_rElevator->Set(-speed);
}

void Elevator::Set (Relay::Value direction)
{
	m_binExt->Set(direction);
}

void Elevator::Update ()
{

}

void Elevator::PrintData()
{
	SmartDashboard::PutNumber("Elevator Throttle",m_lElevator->Get());
}
