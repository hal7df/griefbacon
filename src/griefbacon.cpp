#include "WPILib.h"
#include "RobotUtils/AdvancedJoystick.h"
#include "AutonWrapper.h"

class griefbacon: public IterativeRobot
{
private:
	AdvancedJoystick* m_driver;

	Talon* m_lDrive1;
	Talon* m_lDrive2;
	Talon* m_rDrive1;
	Talon* m_rDrive2;

	Talon* m_STUPID;

	Gyro* m_euro;

	Encoder* m_encodeL;
	Encoder* m_encodeR;

	PIDController* m_euroTurnPID;

	RobotDrive* m_drive;

	AutonWrapper* m_AutonWrapper;

	public:
	griefbacon()
	{
		m_driver = new AdvancedJoystick (0);

		m_driver->SetDeadband(0.2);
		m_driver->SetDeadbandType(AdvancedJoystick::kQuad);

		m_rDrive1 = new Talon (0);
		m_rDrive2 = new Talon (1);
		m_lDrive1 = new Talon (2);
		m_lDrive2 = new Talon (3);

		m_drive = new RobotDrive (m_lDrive1, m_lDrive2, m_rDrive1, m_rDrive2);
		m_drive->SetSafetyEnabled(false);

		m_euro = new Gyro(0);

		m_encodeR = new Encoder(0,1);
		m_encodeL = new Encoder(2,3);

		double euro_P = 0.015;
		double euro_I = 0.0;
		double euro_D = 0.01;

		m_STUPID = new Talon(5);

		m_euroTurnPID = new PIDController(euro_P, euro_I, euro_D, m_euro,m_STUPID);
		m_AutonWrapper = new AutonWrapper (m_drive, m_encodeL, m_encodeR);
		m_AutonWrapper->Disable();

	}

	void RobotInit()
	{

	}

	void AutonomousInit()
	{

	}

	void AutonomousPeriodic()
	{

	}

	void TeleopInit()
	{

	}

	void TeleopPeriodic()
	{
		PrintData();

		if (!m_driver->GetRawButton(AdvancedJoystick::kButtonX) && !m_driver->GetRawButton(AdvancedJoystick::kButtonA) && !m_driver->GetRawButton(AdvancedJoystick::kButtonB)){
			m_drive->ArcadeDrive(-m_driver->GetRawAxis(AdvancedJoystick::kLeftY), -m_driver->GetRawAxis(AdvancedJoystick::kRightX));
			m_euroTurnPID->Disable();
			m_AutonWrapper->Disable();
		}

		else if (m_driver->GetRawButton(AdvancedJoystick::kButtonX)) {
			m_drive->ArcadeDrive(0.0,0.0);
			if(m_encodeL->GetRate() == 0 && m_encodeR->GetRate() == 0){
				m_euro->Reset();
			}
		}

		else if (m_driver->GetRawButton(AdvancedJoystick::kButtonA)){
			m_euroTurnPID->Enable();
			if (m_euroTurnPID->Get() > 0)
				m_drive->ArcadeDrive(-m_driver->GetRawAxis(AdvancedJoystick::kLeftY), m_euroTurnPID->Get()*.33+.3);
			else if(m_euroTurnPID->Get() < 0)
				m_drive->ArcadeDrive(-m_driver->GetRawAxis(AdvancedJoystick::kLeftY), m_euroTurnPID->Get()*.33-.3);

		}
		else if (m_driver->GetRawButton(AdvancedJoystick::kButtonB)){
			m_AutonWrapper->Enable();
		}
		else if (m_driver->GetRawButton(AdvancedJoystick::kButtonLB)){
			m_encodeL->Reset();
			m_encodeR->Reset();
		}





	}

	void TestPeriodic()
	{

	}

	/** MISCELLANEOUS FUNCTIONS **/
	void PrintData ()
	{
		SmartDashboard::PutNumber("Angle", m_euro->GetAngle());
		SmartDashboard::PutNumber("Current Angle", m_euro->GetAngle());
		//*1.02857142857142857142857142857143
		SmartDashboard::PutNumber("Rate", m_euro->GetRate());
		SmartDashboard::PutBoolean("PIDGET", m_euroTurnPID->IsEnabled());
		SmartDashboard::PutNumber("PID",m_euroTurnPID->Get());
		SmartDashboard::PutNumber("Left Encoder",m_encodeL->GetDistance());
		SmartDashboard::PutNumber("Right Encoder",m_encodeR->GetDistance());

		if (m_driver->GetRawButton(AdvancedJoystick::kButtonY)){
			m_euroTurnPID->SetPID(SmartDashboard::GetNumber("Gyro P"), SmartDashboard::GetNumber("Gyro I"), SmartDashboard::GetNumber("Gyro D"));
			m_euroTurnPID->SetSetpoint(SmartDashboard::GetNumber("Set Point"));
			m_AutonWrapper->SetPID(SmartDashboard::GetNumber("Auto Drive P"), SmartDashboard::GetNumber("Auto Drive I"), SmartDashboard::GetNumber("Auto Drive D"));
			m_AutonWrapper->SetSetpoint(SmartDashboard::GetNumber("Auto Drive Setpoint"));
		}

		SmartDashboard::PutNumber("Gyro P",m_euroTurnPID->GetP());
		SmartDashboard::PutNumber("Gyro I",m_euroTurnPID->GetI());
		SmartDashboard::PutNumber("Gyro D",m_euroTurnPID->GetD());
		SmartDashboard::PutNumber("Set Point", m_euroTurnPID->GetSetpoint());
		SmartDashboard::PutNumber("Joystick Y", -m_driver->GetRawAxis(AdvancedJoystick::kRightX));

		SmartDashboard::PutNumber("Auton Drive Output", m_AutonWrapper->GetPIDOutput());
		SmartDashboard::PutNumber("Auto Drive P", m_AutonWrapper->GetP());
		SmartDashboard::PutNumber("Auto Drive I", m_AutonWrapper->GetI());
		SmartDashboard::PutNumber("Auto Drive D", m_AutonWrapper->GetD());
		SmartDashboard::PutNumber("Auto Drive Setpoint", m_AutonWrapper->GetSetpoint());
		SmartDashboard::PutBoolean("Auto Drive Enabled", m_AutonWrapper->IsEnabled());
	}
};

START_ROBOT_CLASS(griefbacon);
