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

	m_lCanBurgle = new Victor (8);
	m_rCanBurgle = new Victor (7);

	m_pickSL->SetVoltageRampRate(0.1);
	m_pickSR->SetVoltageRampRate(0.1);
	m_pickW->SetVoltageRampRate(0.1);

	m_shoulderEncode = new Encoder (4,5,true);
	m_shoulderEncode->SetDistancePerPulse(1./732.25);
	m_wristEncode = new Encoder (6,7,ARM_ENCODER_REVERSE);
	m_wristEncode->SetDistancePerPulse(1./2613.);

	m_wristPid = new PIDController(WRIST_P, 0, WRIST_D,m_wristEncode,m_pickW);
	m_shoulderPid = new PIDController(SHOULDER_P, SHOULDER_I, SHOULDER_D, m_shoulderEncode, this);

	f_getPID = false;

	sem_init(&m_semaphore,0,1);

	m_wStopTime = new Timer;
	m_sStopTime = new Timer;
	f_sEStop = false;
	f_wEStop = false;
	f_eStopRunning = false;
	f_wSetpointChanged = false;
	f_sSetpointChanged = false;
	f_shoulderStop = false;
}

Arm::~Arm() {
	sem_destroy(&m_semaphore);

}
bool Arm::ShoulderAtSetpoint()
{
	return fabs(m_shoulderEncode->GetDistance() - m_shoulderPid->GetSetpoint()) < 0.01;
}
bool Arm::WristAtSetpoint()
{
	return fabs(m_wristEncode->GetDistance() - m_wristPid->GetSetpoint()) < 0.01;
}

void Arm::shoulderSet(double speed){
	m_pickSL->Set(-speed);
	m_pickSR->Set(speed);
}

void Arm::wristSet(double speed){
	m_pickW->Set(-speed);
}

void Arm::rollerSet(double speed){
	m_pickRL->Set(speed);
	m_pickRR->Set(-speed);
}

void Arm::canRotate(bool speed){
	if (speed){
		m_pickRL->Set(0.5);
		m_pickRR->Set(0.5);
	}
	if (!speed){
		m_pickRL->Set(-0.5);
		m_pickRR->Set(-0.5);
	}
}

void Arm::intakeSet(double speed){
	m_intakeL->Set(-speed);
	m_intakeR->Set(speed);
}

void Arm::shoulderSetPos (sPos_t position)
{
	if (position == ksAutoPlace)
		m_shoulderPid->SetOutputRange(-0.18,0.18);
	else
		m_shoulderPid->SetOutputRange(-1.0,1.0);

	switch (position){
	case ksGround:
		m_shoulderPid->SetSetpoint(SHOULDER_GROUND);
		break;
	case ksTwoTote:
		m_shoulderPid->SetSetpoint(SHOULDER_TWOTOTE);
		break;
	case ksDriving:
		m_shoulderPid->SetSetpoint(SHOULDER_DRIVING);
		break;
	case ksCanStack:
		m_shoulderPid->SetSetpoint(SHOULDER_CANSTACK);
		break;
	case ksPackage:
		m_shoulderPid ->SetSetpoint(SHOULDER_PACKAGE);
		break;
	case ksFiveCan:
		m_shoulderPid->SetSetpoint(SHOULDER_FIVECAN);
		break;
	case ksCanKnock:
		m_shoulderPid->SetSetpoint(SHOULDER_CANKNOCK);
		break;
	case ksAutoPlace:
		m_shoulderPid->SetSetpoint(SHOULDER_AUTOPLACE);
		break;
	case ksGroundM:
		m_shoulderPid->SetSetpoint(SHOULDER_GROUND_M);
		break;
	case ksFourCanPlace:
		m_shoulderPid->SetSetpoint(SHOULDER_FOURCAN);
		break;
	}

	f_sSetpointChanged = true;

	if (!m_shoulderPid->IsEnabled())
		m_shoulderPid->Enable();
}

void Arm::sEnable ()
{
	int lock;
	//BEGIN SEMAPHORE REGION
	if (f_eStopRunning)
		sem_wait(&m_semaphore);

	if (!sIsEnabled() && !GetEStop() && !f_shoulderStop)
		m_shoulderPid->Enable();

	lock = sem_getvalue(&m_semaphore, &lock);

	if (lock == 0)
		sem_post(&m_semaphore);
	//END SEMAPHORE REGION
}

void Arm::sDisable ()
{
	if (sIsEnabled())
	{
		m_shoulderPid->Disable();
		m_shoulderPid->Reset();
	}
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
	case kwDriving:
		m_wristPid->SetSetpoint(WRIST_DRIVING);
		break;
	case kwCanStack:
		m_wristPid->SetSetpoint(WRIST_CANSTACK);
		break;
	case kwPackage:
		m_wristPid ->SetSetpoint(WRIST_PACKAGE);
		break;
	case kwFiveCan:
		m_wristPid ->SetSetpoint(WRIST_FIVECAN);
		break;
	case kwCanKnock:
		m_wristPid->SetSetpoint(WRIST_CANKNOCK);
		break;
	case kwAutoPlace:
		m_wristPid->SetSetpoint(WRIST_AUTOPLACE);
		break;
	case kwGroundM:
		m_wristPid->SetSetpoint(WRIST_GROUND_M);
		break;
	case kwAuton:
		m_wristPid->SetSetpoint(WRIST_AUTON);
		break;
	case kwFourCanPlace:
		m_wristPid->SetSetpoint(WRIST_FOURCAN);
		break;
	}

	f_wSetpointChanged = true;

	if (!m_wristPid->IsEnabled())
		m_wristPid->Enable();
}

void Arm::wEnable()
{
	int lock;
	//BEGIN SEMAPHORE REGION
	if (f_eStopRunning)
		sem_wait(&m_semaphore);

	if (!wIsEnabled() && !GetEStop())
		m_wristPid->Enable();

	sem_getvalue(&m_semaphore, &lock);

	if (lock == 0)
		sem_post(&m_semaphore);
	//END SEMAPHORE REGION
}

void Arm::wDisable()
{
	if (wIsEnabled())
	{
		m_wristPid->Disable();
		m_wristPid->Reset();
	}
}

void Arm::shoulderSetSetpoint(int point){
	m_shoulderPid->SetSetpoint(point);
}

void Arm::wristSetSetpoint(float point){
	m_wristPid->SetSetpoint(point);
}

void Arm::clearCans(bool on) {
	if (on)
	{
		m_intakeL->Set(-1.0);
		m_intakeR->Set(1.0);
	}
	else
	{
		m_intakeL->Set(0.0);
		m_intakeR->Set(0.0);
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

		SmartDashboard::PutBoolean("Arm Intake Running",(m_intakeR->Get() != 0.0) && (m_intakeL->Get() != 0.0));
		SmartDashboard::PutBoolean("Arm Intake Running",(m_pickRR->Get() != 0.0) && (m_pickRL->Get() != 0.0));

		SmartDashboard::PutNumber("Shoulder Encoder",m_shoulderEncode->GetDistance());
		SmartDashboard::PutNumber("Wrist Encoder",m_wristEncode->GetDistance());

		SmartDashboard::PutNumber("Shoulder PID Output",m_shoulderPid->Get());
		SmartDashboard::PutNumber("Wrist PID Output",m_wristPid->Get());
		SmartDashboard::PutNumber("Shoulder PID SetPoint",m_shoulderPid->GetSetpoint());
		SmartDashboard::PutNumber("Wrist PID SetPoint",m_wristPid->GetSetpoint());
		SmartDashboard::PutNumber("Shoulder PID Input",m_shoulderEncode->PIDGet());
		SmartDashboard::PutNumber("Wrist PID Input",m_wristEncode->PIDGet());

		SmartDashboard::PutNumber("Shoulder P",m_shoulderPid->GetP());
		SmartDashboard::PutNumber("Wrist P",m_wristPid->GetP());
		SmartDashboard::PutNumber("Shoulder I",m_shoulderPid->GetI());
		SmartDashboard::PutNumber("Wrist I",m_wristPid->GetI());
		SmartDashboard::PutNumber("Shoulder D",m_shoulderPid->GetD());
		SmartDashboard::PutNumber("Wrist D",m_wristPid->GetD());

		SmartDashboard::PutBoolean("Wrist At Setpoint",WristAtSetpoint());
		SmartDashboard::PutBoolean("Shoulder At Setpoint",ShoulderAtSetpoint());
		SmartDashboard::PutNumber("Wrist Difference",fabs(m_wristEncode->GetDistance() - m_wristPid->GetSetpoint()));
		SmartDashboard::PutNumber("Shoulder Difference",fabs(m_shoulderEncode->GetDistance() - m_shoulderPid->GetSetpoint()));

		SmartDashboard::PutBoolean("Wrist E-Stop",f_wEStop);
		SmartDashboard::PutBoolean("Shoulder E-Stop",f_sEStop);

		SmartDashboard::PutNumber("Wrist E-Stop Timer",m_wStopTime->Get());
		SmartDashboard::PutNumber("Shoulder E-Stop Timer",m_sStopTime->Get());
	}
}

void Arm::Update()
{
	/*if (DriverStation::GetInstance()->IsAutonomous())
	{

		if (!f_eStopRunning)
		{
			m_wStopTime->Start();
			m_sStopTime->Start();
		}
		EStopCheck();
		f_eStopRunning = true;
	}
	else if (f_eStopRunning)
	{
		f_wEStop = false;
		f_sEStop = false;
		f_eStopRunning = false;
		m_wStopTime->Stop();
		m_wStopTime->Reset();
		m_sStopTime->Stop();
		m_sStopTime->Reset();
	}*/

	/*if (sIsEnabled() && m_shoulderEncode->GetDistance() < -0.745 && m_wristEncode->GetDistance() < -0.411)
	{
		sDisable();
		f_shoulderStop = true;
	}
	else
		f_shoulderStop = false;*/


	if (WristAtSetpoint() && (m_wristPid->GetI() == 0))
		m_wristPid-> SetPID(WRIST_P, WRIST_I, WRIST_D);
	else if ((m_wristPid->GetI() != 0) && !WristAtSetpoint())
		m_wristPid-> SetPID(WRIST_P, 0, WRIST_D);
}
void Arm::EStopCheck()
{
	if (m_wStopTime->HasPeriodPassed(3) && !WristAtSetpoint() && wIsEnabled())
	{
		//BEGIN SEMAPHORE REGION
		sem_wait (&m_semaphore);

		f_wEStop = true;
		m_wristPid->Disable();
		m_shoulderPid->Disable();
		m_wStopTime->Stop();
		m_sStopTime->Stop();

		sem_post(&m_semaphore);
		//END SEMAPHORE REGION
	}
	if (m_sStopTime->HasPeriodPassed(1.5) && !ShoulderAtSetpoint() && sIsEnabled())
	{
		//BEGIN SEMAPHORE REGION
		sem_wait(&m_semaphore);

		f_sEStop = true;
		m_shoulderPid->Disable();
		m_wristPid->Disable();
		m_sStopTime->Stop();
		m_wStopTime->Stop();

		sem_post(&m_semaphore);
		//END SEMAPHORE REGION
	}

	if (f_wSetpointChanged)
	{
		m_wStopTime->Reset();
		f_wSetpointChanged = false;
	}
	if (f_sSetpointChanged)
	{
		m_sStopTime->Reset();
		f_sSetpointChanged = false;
	}
}

void Arm::ResetEStop()
{
	f_wEStop = false;
	f_sEStop = false;
	f_eStopRunning = false;
	f_wSetpointChanged = false;
	f_sSetpointChanged = false;
	m_wStopTime->Stop();
	m_wStopTime->Reset();
	m_sStopTime->Stop();
	m_sStopTime->Reset();
}
