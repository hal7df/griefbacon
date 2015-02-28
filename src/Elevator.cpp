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
	m_stackTime = new Timer;
	sem_init(&m_semaphore,0,1);

	m_stackCase = 3;
	m_stackFin = kCarry;

	f_stacking = false;
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
	m_stackTime = new Timer;
	sem_init(&m_semaphore,0,1);

	m_stackCase = 3;
	m_stackFin = kCarry;

	f_stacking = false;
	f_setPID = false;
	f_elevEStop = false;
	f_eStopRunning = false;
	f_setpointChanged = false;
}

Elevator::~Elevator() {
	sem_destroy(&m_semaphore);
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

void Elevator::Stack (pos_t finish)
{
	m_stackFin = finish;
	m_stackCase = 0;
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
	{
		m_pid->Disable();
		m_pid->Reset();
	}
}

bool Elevator::AtSetpoint()
{
	return fabs(m_elevEncode->GetDistance() - m_pid->GetSetpoint()) < 0.01;
}

void Elevator::ResetEStop()
{
	f_elevEStop = false;
	f_eStopRunning = false;
	f_setpointChanged = false;
	m_stopTime->Stop();
	m_stopTime->Reset();
}

void Elevator::Update ()
{
	if (DriverStation::GetInstance()->IsAutonomous())
	{
		if (!f_eStopRunning)
			m_stopTime->Start();
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

	if (m_stackCase < 3)
		Stack_internal();
	else if (m_stackTime->Get() != 0.0)
	{
		m_stackTime->Stop();
		m_stackTime->Reset();
	}
	if (m_pid->IsEnabled() && AtSetpoint())
		m_pid->Reset();
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
		SmartDashboard::PutNumber("Elevator Difference",fabs(m_elevEncode->GetDistance() - m_pid->GetSetpoint()));

		SmartDashboard::PutBoolean("Elevator E-Stop",f_elevEStop);
		SmartDashboard::PutNumber("Elevator E-Stop Timer",m_stopTime->Get());

		SmartDashboard::PutNumber("Elevator Stack Case",m_stackCase);
		SmartDashboard::PutNumber("Elevator Stack Finish",GetPosition(m_stackFin));
		SmartDashboard::PutBoolean("Stacking",Stacking());
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

pos_t Elevator::GetSetpoint (double position)
{
	pos_t buf;
	if (position == ELEVATOR_BOTTOM)
		buf = kBottom;
	else if (position == ELEVATOR_TOP)
		buf = kTop;
	else if (position == ELEVATOR_LMID)
		buf = kLMid;
	else if (position == ELEVATOR_UMID)
		buf = kUMid;
	else if (position == ELEVATOR_CARRY)
		buf = kCarry;

	return buf;
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

void Elevator::Stack_internal()
{
	switch (m_stackCase)
	{
	case 0:
		Set(kBottom);

		if (AtSetpoint())
		{
			m_stackCase++;
			m_stackTime->Stop();
			m_stackTime->Start();
			m_stackTime->Reset();
		}
		break;
	case 1:
		if (m_stackTime->HasPeriodPassed(.1))
		{
			m_stackCase++;
			m_stackTime->Stop();
			m_stackTime->Reset();
		}
		break;
	case 2:
		Set(m_stackFin);

		if (AtSetpoint())
		{
			m_stackCase++;
		}
		break;
	}
}

