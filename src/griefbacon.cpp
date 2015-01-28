#include "WPILib.h"
#include "RobotUtils/RobotUtils.h"

class griefbacon: public IterativeRobot
{
private:
	AdvancedJoystick* m_driver;

	Talon* m_lDrive1;
	Talon* m_lDrive2;
	Talon* m_rDrive1;
	Talon* m_rDrive2;

	RobotDrive* m_drive;

	BackgroundDebugger* m_debug;
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

		m_debug = new BackgroundDebugger;
	}

	void RobotInit()
	{

	}

	void DisabledInit ()
	{
		m_debug->CloseFile();
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
		m_drive->ArcadeDrive(-m_driver->GetRawAxis(AdvancedJoystick::kLeftY), -m_driver->GetRawAxis(AdvancedJoystick::kRightX));
		SmartDashboard::PutBoolean("Test",m_driver->GetButtonPress(AdvancedJoystick::kButtonA));


		m_drive->ArcadeDrive(-m_driver->GetRawAxis(AdvancedJoystick::kLeftY), -m_driver->GetRawAxis(AdvancedJoystick::kRightX));
		SmartDashboard::PutBoolean("Test",m_driver->GetButtonPress(AdvancedJoystick::kButtonA));

		if(m_driver->GetButtonPress(AdvancedJoystick::kButtonA))
		{
				m_debug->LogData("Test", m_driver->GetRawAxis(AdvancedJoystick::kLeftY));
		}
				///m_debug->Update();

	}

	void TestPeriodic()
	{

	}
};

START_ROBOT_CLASS(griefbacon);
