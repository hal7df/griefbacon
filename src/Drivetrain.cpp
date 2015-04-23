/*
 * Drivetrain.cpp
 *
 *  Created on: Feb 3, 2015
 *      Author: ROBO6
 */

#include "Drivetrain.h"

Drivetrain::Drivetrain(int lDrive1, int lDrive2, int rDrive1, int rDrive2, int lEncode, int rEncode) :
	HotSubsystem("Drivetrain")
{
	m_lDrive1 = new Talon (lDrive1);
	m_lDrive2 = new Talon (lDrive2);
	m_rDrive1 = new Talon (rDrive1);
	m_rDrive2 = new Talon (rDrive2);

	m_lEncode = new Encoder (lEncode,lEncode++,false);
	m_rEncode = new Encoder (rEncode, rEncode++,true);
	m_lEncode->SetDistancePerPulse(1./281.5);
	m_rEncode->SetDistancePerPulse(1./281.5);

	m_distancePIDWrapper = new DistancePIDWrapper(m_lEncode, m_rEncode);

	m_timer = new Timer;

	f_setPID = false;
	f_DisabledDistance = false;

	m_drive = new RobotDrive (m_lDrive1, m_lDrive2, m_rDrive1, m_rDrive2);
	m_drive->SetSafetyEnabled(false);

#ifdef NAVX_ENABLED
	SerialPort* serial = new SerialPort (57600,SerialPort::kMXP);
	m_gyro = new AHRS (serial,(uint8_t)50);
	m_firstGyroIt = true;
#else //ADXRS453Z
	m_gyro = new ADXRS453Z;
#endif
	m_angleOut = new AngleOutputWrapper (m_drive);

	m_turnPID = new PIDController(TURN_P,TURN_I,TURN_D, m_gyro, m_angleOut);
	m_distancePID = new PIDController(DISTANCE_P,DISTANCE_I,DISTANCE_D,m_distancePIDWrapper, this);

	m_angleHeading = 0.0;
	m_speedLimit = 0.65;
	m_correctLimit = 0.2;
	m_gyroOffset = 0.0;

	f_tipping = false;
}

Drivetrain::~Drivetrain() {
	// TODO Auto-generated destructor stub
}

void Drivetrain::SetTurnPIDHeading(float angle) {

	if (fabs(angle) < 30)
		m_turnPID->SetPID(-0.5, TURN_I, TURN_D);
	else
		m_turnPID->SetPID(TURN_P, TURN_I, TURN_D);

	m_turnPID->SetSetpoint(angle - m_gyroOffset);
}

void Drivetrain::Update() {
#ifdef NAVX_ENABLED
	GyroCal();
	if (DriverStation::GetInstance()->IsAutonomous())
		TipCheck();
#else
	m_gyro->Update();
#endif
}

void Drivetrain::PrintData() {
	if (f_setPID)
	{
		m_distancePID->SetPID(SmartDashboard::GetNumber("Distance P"), SmartDashboard::GetNumber("Distance I"), SmartDashboard::GetNumber("Distance D"));
		//m_turnPID->SetPID(SmartDashboard::GetNumber("Turn P"), SmartDashboard::GetNumber("Turn I"), SmartDashboard::GetNumber("Turn D"));

		m_speedLimit = SmartDashboard::GetNumber("Speed Limit");
		m_angleHeading = SmartDashboard::GetNumber("Set Heading");
		m_distancePID->SetSetpoint(SmartDashboard::GetNumber("Distance PID Setpoint"));
		m_correctLimit = SmartDashboard::GetNumber("Angle Compensation Limit");
		//m_turnPID->SetSetpoint(SmartDashboard::GetNumber("TurnPID SetPoint"));
	}
	else
	{
		SmartDashboard::PutNumber("Left Drive 1",m_lDrive1->Get());
		SmartDashboard::PutNumber("Left Drive 2",m_lDrive2->Get());
		SmartDashboard::PutNumber("Right Drive 1",m_rDrive1->Get());
		SmartDashboard::PutNumber("Right Drive 2",m_rDrive2->Get());

		SmartDashboard::PutNumber("m_lEncode Distance", m_lEncode->GetDistance());
		SmartDashboard::PutNumber("m_rEncode Distance", m_rEncode->GetDistance());

		SmartDashboard::PutNumber("m_timer", m_timer->Get());
		SmartDashboard::PutNumber("ETA:",  4.0 - m_timer->Get());

		SmartDashboard::PutNumber("Turn PID Set Point", m_turnPID->GetSetpoint());
		SmartDashboard::PutNumber("Turn PID output",m_turnPID->Get());
		SmartDashboard::PutBoolean("Turn PID Enabled", m_turnPID->IsEnabled());
		SmartDashboard::PutNumber("Turn P",m_turnPID->GetP());
		SmartDashboard::PutNumber("Turn I",m_turnPID->GetI());
		SmartDashboard::PutNumber("Turn D",m_turnPID->GetD());
		SmartDashboard::PutBoolean("Turn PID At Setpoint",TurnPIDatSetpoint());

		SmartDashboard::PutNumber("Distance P", m_distancePID->GetP());
		SmartDashboard::PutNumber("Distance I", m_distancePID->GetI());
		SmartDashboard::PutNumber("Distance D", m_distancePID->GetD());
		SmartDashboard::PutNumber("Distance PID Output", m_distancePID->Get());
		SmartDashboard::PutNumber("Distance PID Setpoint", m_distancePID->GetSetpoint());
		SmartDashboard::PutBoolean("Distance PID Enabled", m_distancePID->IsEnabled());
		SmartDashboard::PutBoolean("Distance At Setpoint",DistanceAtSetpoint());
		SmartDashboard::PutNumber("Distance Difference",fabs(m_distancePIDWrapper->PIDGet() - m_distancePID->GetSetpoint()));

		SmartDashboard::PutNumber("Encoder Rate Left", m_lEncode->GetRate());
		SmartDashboard::PutNumber("Encoder Rate Right", m_rEncode->GetRate());
		SmartDashboard::PutNumber("Encoder Rate Average", ((m_lEncode->GetRate()) + (m_rEncode->GetRate())) / 2);

		SmartDashboard::PutNumber("Set Heading",m_angleHeading);
		SmartDashboard::PutNumber("Speed Limit",m_speedLimit);
		SmartDashboard::PutNumber("Angle Compensation Limit",m_correctLimit);
		SmartDashboard::PutNumber("Gyro Offset",m_gyroOffset);

		SmartDashboard::PutNumber("Raw Angle", GetGyroAngle());
		SmartDashboard::PutNumber("Adjusted Angle", (GetGyroAngle() - m_gyroOffset));
#ifdef NAVX_ENABLED
		SmartDashboard::PutNumber("Roll", m_gyro->GetPitch());
#else //!NAVX_ENABLED
		SmartDashboard::PutNumber("Rate", m_gyro->GetRate());
#endif
		SmartDashboard::PutNumber("Left Encoder",m_lEncode->GetDistance());
		SmartDashboard::PutNumber("Right Encoder",m_rEncode->GetDistance());
		SmartDashboard::PutNumber("Average Drive Encoder",m_distancePIDWrapper->PIDGet());
		SmartDashboard::PutBoolean("f_setPID", f_setPID);
		SmartDashboard::PutBoolean("f_DisabledDistance", f_DisabledDistance);
	}
}

void Drivetrain::PIDWrite(float output)
{
	//float driveComp;
	if (output > m_speedLimit)
		output = m_speedLimit;
	else if (output < -m_speedLimit)
		output = -m_speedLimit;

	/*
	if (fabs(m_gyro->GetAngle() - m_angleHeading) > 90)
		driveComp = m_correctLimit;
	else
		driveComp = m_correctLimit * sqrt(fabs(m_gyro->GetAngle() - m_angleHeading) / 90);
	 */

	if ((GetGyroAngle() - m_angleHeading) > 0.5)
		m_drive->TankDrive(output+m_correctLimit,output-m_correctLimit);
	else if ((GetGyroAngle() - m_angleHeading) < -0.5)
		m_drive->TankDrive(output-m_correctLimit,output+m_correctLimit);
	else
		m_drive->TankDrive(output,output);
}

void Drivetrain::GyroCal()
{
	if (!m_gyro->IsCalibrating() && m_firstGyroIt)
	{
		Wait(0.3);
		m_gyro->ZeroYaw();
		m_firstGyroIt = false;
	}
}

void Drivetrain::TipCheck()
{
	if (fabs(m_gyro->GetPitch()) > 10)
		f_tipping = true;
}
