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

	m_elevEncode->SetDistancePerPulse(1./1270.);

	m_stopTime = new Timer;
	sem_init(&m_semaphore,0,1);

	f_setPID = false;
	f_elevEStop = false;
	f_eStopRunning = false;
	f_setpointChanged = false;
}

Elevator::Elevator(int lElevator, int rElevator, int binExt, int encode)
	: HotSubsystem("Elevator")
{
	m_lElevator = new Victor (lElevator);
	m_rElevator = new Victor (rElevator);
	m_binExt = new Relay (binExt);
	m_elevEncode = new Encoder (encode,encode++,true);

	m_pid = new PIDController (ELEVATOR_P, ELEVATOR_I, ELEVATOR_D, m_elevEncode, this);

	m_elevEncode->SetDistancePerPulse(1./1270.);

	m_stopTime = new Timer;
	sem_init(&m_semaphore,0,1);

	f_setPID = false;
	f_elevEStop = false;
	f_eStopRunning = false;
	f_setpointChanged = false;
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
	case kCarry:
		m_pid ->SetSetpoint(ELEVATOR_CARRY);
		break;
	}

	f_setpointChanged = true;

	m_pid->Enable();
}

void Elevator::Enable()
{
	//BEGIN SEMAPHORE REGION
	sem_wait(&m_semaphore);

	if (!IsEnabled() && !f_elevEStop)
		m_pid->Enable();

	sem_post(&m_semaphore);
	//END SEMAPHORE REGION
}

void Elevator::Disable()
{
	if (IsEnabled())
		m_pid->Disable();
}

bool Elevator::AtSetpoint()
{
	return fabs(m_elevEncode->GetDistance() - m_pid->GetSetpoint()) < 0.03;
}

void Elevator::Update ()
{
	if (DriverStation::GetInstance()->IsAutonomous())
	{
		ElevatorEStop();
		f_eStopRunning = true;
	}
	else if (f_eStopRunning)
	{
		f_elevEStop = false;
		f_eStopRunning = false;
		m_stopTime->Stop();
		m_stopTime->Reset();
	}
}

void Elevator::PrintData()
{
	if (f_setPID)
		m_pid->SetPID(SmartDashboard::GetNumber("Elevator P"),SmartDashboard::GetNumber("Elevator I"),SmartDashboard::GetNumber("Elevator D"));
	else
	{
		SmartDashboard::PutNumber("Left Elevator",m_lElevator->Get());
		SmartDashboard::PutNumber("Right Elevator",m_rElevator->Get());
		SmartDashboard::PutNumber("Elevator Distance",m_elevEncode->GetDistance());
		SmartDashboard::PutNumber("Elevator P",m_pid->GetP());
		SmartDashboard::PutNumber("Elevator I",m_pid->GetI());
		SmartDashboard::PutNumber("Elevator D",m_pid->GetD());
		SmartDashboard::PutNumber("Elevator PID Output",m_pid->Get());

		SmartDashboard::PutBoolean("Elevator At Setpoint",AtSetpoint());
		SmartDashboard::PutNumber("Elevator Differance",fabs(m_elevEncode->GetDistance() - m_pid->GetSetpoint()));
	}
}

double Elevator::GetPosition (pos_t position)
{
	double pos;
	switch (position)
	{
	case kBottom:
		pos = ELEVATOR_BOTTOM;
		break;
	case kCarry:
		pos = ELEVATOR_CARRY;
		break;
	case kLMid:
		pos = ELEVATOR_LMID;
		break;
	case kUMid:
		pos = ELEVATOR_UMID;
		break;
	case kTop:
		pos = ELEVATOR_TOP;
		break;
	default:
		pos = -1.0;
	}

	return pos;
}

void Elevator::ElevatorEStop()
{
	if (m_stopTime->HasPeriodPassed(1.5) && !AtSetpoint() && IsEnabled())
	{
		//BEGIN SEMAPHORE REGION
		sem_wait(&m_semaphore);

		f_elevEStop = true;
		m_pid->Disable();
		m_stopTime->Stop();

		sem_post(&m_semaphore);
	}

	if (f_setpointChanged)
	{
		m_stopTime->Reset();
		f_setpointChanged = false;
	}
}
