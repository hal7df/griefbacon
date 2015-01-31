#include "WPILib.h"
#include "RobotUtils/RobotUtils.h"
#include <fstream>
#include <ctime>
using namespace std;

class griefbacon: public IterativeRobot
{
private:
	AdvancedJoystick* m_driver;

	Talon* m_lDrive1;
	Talon* m_lDrive2;
	Talon* m_rDrive1;
	Talon* m_rDrive2;

	Encoder* m_lEncode;

	RobotDrive* m_drive;

	BackgroundDebugger* m_debug;
	HotSubsystemHandler* m_subsys;

	double m_loopcounter;
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

		m_lEncode = new Encoder (2,3,false);

		m_drive = new RobotDrive (m_lDrive1, m_lDrive2, m_rDrive1, m_rDrive2);
		m_drive->SetSafetyEnabled(false);

		m_debug = new BackgroundDebugger;
		m_debug->AddValue("Left Encoder",m_lEncode);
		m_debug->AddValue("Loop Counter",&m_loopcounter);

		m_subsys = new HotSubsystemHandler;
		m_subsys->Add(m_debug);
		m_subsys->Start();

		m_loopcounter = 0;
	}

	void RobotInit()
	{

	}

	void DisabledInit ()
	{
		m_debug->StopRun();
	}

	void AutonomousInit()
	{

	}

	void AutonomousPeriodic()
	{

	}

	void TeleopInit()
	{
		m_debug->StartRun();
		m_debug->SetTempMessage("** TELEOP **");
	}

	void TeleopPeriodic()
	{
		if (m_driver->GetRawButton(AdvancedJoystick::kButtonX))
		{
			m_debug->LogData("Joystick Left Axis X",m_driver->GetRawAxis(AdvancedJoystick::kLeftX));
			m_debug->LogData("Joystick Left Axis Y",m_driver->GetRawAxis(AdvancedJoystick::kLeftY));
			SmartDashboard::PutBoolean("Debugging",true);
		}
		else
			SmartDashboard::PutBoolean("Debugging",false);

		m_drive->ArcadeDrive(-m_driver->GetRawAxis(AdvancedJoystick::kLeftY),-m_driver->GetRawAxis(AdvancedJoystick::kRightX));

		m_loopcounter++;
	}

	void TestPeriodic()
	{

	}
};

START_ROBOT_CLASS(griefbacon);
