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

	RobotDrive* m_drive;

	BackgroundDebugger* m_debug;

	ofstream m_fout;
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
		m_fout.close();
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
		if (m_driver->GetRawButton(AdvancedJoystick::kButtonA))
		{
			if (!m_fout.is_open())
				m_fout.open("/home/lvuser/DisabledLogs/manualLog.txt", ios::app);

			SmartDashboard::PutBoolean("Debug Log Stream Status",m_fout.good());

			if (m_driver->GetRawButton(AdvancedJoystick::kButtonB) && m_fout.is_open())
			{
				m_fout<<"Test 1"<<"Testing MAA STRINGS"<<endl;
				m_fout<<m_driver->GetRawAxis(AdvancedJoystick::kLeftX)<<endl;
				m_fout<<"Joystick Left Axis Y"<<m_driver->GetRawAxis(AdvancedJoystick::kLeftY)<<endl;


				cout<<"Done debugging"<<endl;

				SmartDashboard::PutBoolean("Debugging",true);

			}
		}
		else if (m_driver->GetRawButton(AdvancedJoystick::kButtonX))
		{
			m_debug->LogData("Joystick Left Axis X",m_driver->GetRawAxis(AdvancedJoystick::kLeftX));
			m_debug->LogData("Joystick Left Axis Y",m_driver->GetRawAxis(AdvancedJoystick::kLeftY));
			SmartDashboard::PutBoolean("Debugging",true);

		}
		else
			SmartDashboard::PutBoolean("Debugging",false);

		SmartDashboard::PutBoolean("Debug Log Open",m_fout.is_open());

		cout<<"End of periodic loop"<<endl;
	}

	void TestPeriodic()
	{

	}
};

START_ROBOT_CLASS(griefbacon);
