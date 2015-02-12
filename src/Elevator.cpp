/*
 * Elevator.cpp
 *
 *  Created on: Feb 3, 2015
 *      Author: paul
 */

#include <Elevator.h>

Elevator::Elevator(Victor* lElevator, Victor* rElevator, Relay* binExt, Encoder* encode)
	: HotSubsystem("Elevator")
{
	m_lElevator = lElevator;
	m_rElevator = rElevator;
	m_binExt = binExt;
	m_elevEncode = encode;

	m_pid = new PIDController(ELEVATOR_P, ELEVATOR_I, ELEVATOR_D, m_elevEncode, this);

	m_elevEncode->SetDistancePerPulse(250);
}

Elevator::Elevator(int lElevator, int rElevator, int binExt, int encode)
	: HotSubsystem("Elevator")
{
	m_lElevator = new Victor (lElevator);
	m_rElevator = new Victor (rElevator);
	m_binExt = new Relay (binExt);
	m_elevEncode = new Encoder (encode,encode++,false);

	m_pid = new PIDController (ELEVATOR_P, ELEVATOR_I, ELEVATOR_D, m_elevEncode, this);

	m_elevEncode->SetDistancePerPulse(250);
}

Elevator::~Elevator() {
	// TODO Auto-generated destructor stub
}

void Elevator::Set (double speed)
{
	m_lElevator->Set(-speed);
	m_rElevator->Set(speed);
}

void Elevator::Set (Relay::Value direction)
{
	m_binExt->Set(direction);
}

void Elevator::Set (pos_t position)
{
	switch (position){
	case kBottom:
		m_pid->SetSetpoint(ELEVATOR_BOTTOM);
		break;
	case kTop:
		m_pid->SetSetpoint(ELEVATOR_TOP);
		break;
	case kLMid:
		m_pid->SetSetpoint(ELEVATOR_LMID);
		break;
	case kUMid:
		m_pid->SetSetpoint(ELEVATOR_UMID);
		break;
	}

	if (!m_pid->IsEnabled())
		m_pid->Enable();
}

void Elevator::Update ()
{

}

void Elevator::PrintData()
{
	SmartDashboard::PutNumber("Left Elevator",m_lElevator->Get());
	SmartDashboard::PutNumber("Right Elevator",m_rElevator->Get());
	SmartDashboard::PutNumber("Elevator Distance",m_elevEncode->GetDistance());
}
