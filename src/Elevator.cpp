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
	m_elevEncode->SetDistancePerPulse(1./1270.);

	m_pid = new PIDController(ELEVATOR_P, ELEVATOR_I, ELEVATOR_D, m_elevEncode, this);

	m_timer = new Timer;
	m_profileCase = 0;

	f_setPID = false;
	f_oldPID = false;
	f_pidRunning = false;

	m_setpoint = 0.0;
}

Elevator::Elevator(int lElevator, int rElevator, int binExt, int encode)
	: HotSubsystem("Elevator")
{
	m_lElevator = new Victor (lElevator);
	m_rElevator = new Victor (rElevator);
	m_binExt = new Relay (binExt);
	m_elevEncode = new Encoder (encode,encode++,true);
	m_elevEncode->SetDistancePerPulse(1./1270.);

	m_pid = new PIDController (ELEVATOR_P, ELEVATOR_I, ELEVATOR_D, m_elevEncode, this);

	m_timer = new Timer;
	m_profileCase = 0;

	f_setPID = false;
	f_oldPID = false;
	f_pidRunning = false;

	m_setpoint = 0.0;
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
	if (f_oldPID)
		SetSetpoint_old(position);
	else
		SetSetpoint(position);
}

void Elevator::Enable ()
{
	if (f_oldPID)
		m_pid->Enable();
	else
	{
		FindCoefficients();
		f_pidRunning = true;
		m_profileCase = 0;
		m_timer->Stop();
		m_timer->Start();
		m_timer->Reset();
	}
}

void Elevator::Disable()
{
	if (f_oldPID && m_pid->IsEnabled())
		m_pid->Disable();
	else if (!f_oldPID && m_pid->IsEnabled())
	{
		f_pidRunning = false;
		m_pid->Disable();
	}
}

bool Elevator::IsEnabled()
{
	return (!f_oldPID && f_pidRunning) || m_pid->IsEnabled();
}

void Elevator::Update ()
{
	if (!f_oldPID && f_pidRunning)
	{
		switch (m_profileCase)
		{
		case 0:
			m_pid->SetSetpoint(GetProfilePosition(m_timer->Get()));

			if (!m_pid->IsEnabled())
				m_pid->Enable();

			if (m_timer->HasPeriodPassed(GetProfileTime(m_lastSetpoint,m_setpoint)))
				m_profileCase++;

			break;
		case 1:
			m_timer->Stop();
			m_timer->Reset();
			m_pid->SetSetpoint(m_setpoint);
			m_profileCase++;
		}
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
	}
}

/** PRIVATE **/

void Elevator::SetSetpoint(pos_t position)
{
	m_lastSetpoint = m_elevEncode->GetDistance();
	m_setpoint = GetPosition(position);
	Enable();
}

void Elevator::SetSetpoint_old(pos_t position)
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

	if (!m_pid->IsEnabled())
		m_pid->Enable();
}

void Elevator::FindCoefficients ()
{
	//3rd-order coefficient
	m_coeff.a3 = ((1/(2*pow(GetProfileTime(m_lastSetpoint,m_setpoint),3)))*20*GetDelta(m_lastSetpoint,m_setpoint));

	//4th-order coefficient
	m_coeff.a4 = ((1/(2*pow(GetProfileTime(m_lastSetpoint,m_setpoint),4)))*30*GetReverseDelta(m_lastSetpoint,m_setpoint));

	//5th-order coefficient
	m_coeff.a5 = ((1/(2*pow(GetProfileTime(m_lastSetpoint,m_setpoint),5)))*12*GetDelta(m_lastSetpoint,m_setpoint));
}

float Elevator::GetProfilePosition (double time)
{
	double term3, term4, term5;

	term3 = m_coeff.a3*(pow(time,3));
	term4 = m_coeff.a4*(pow(time,4));
	term5 = m_coeff.a5*(pow(time,5));

	return (float)(term5 + term4 + term3 + m_lastSetpoint);
}

double Elevator::GetProfileTime(double start, double end)
{
	return (ELEVATOR_TIME_FULL*GetDelta(start,end));
}

double Elevator::GetDelta(double start, double end)
{
	return end - start;
}

double Elevator::GetReverseDelta(double start, double end)
{
	return start - end;
}

double Elevator::GetPosition (pos_t position)
{
	double pos;

	switch (position)
	{
	case kBottom:
		pos = ELEVATOR_BOTTOM;
		break;
	case kTop:
		pos = ELEVATOR_TOP;
		break;
	case kLMid:
		pos = ELEVATOR_LMID;
		break;
	case kUMid:
		pos = ELEVATOR_UMID;
		break;
	case kCarry:
		pos = ELEVATOR_CARRY;
		break;
	}

	return pos;
}
