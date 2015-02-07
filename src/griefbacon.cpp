#include "WPILib.h"
#include "RobotUtils/RobotUtils.h"
#include "Elevator.h"
#include <fstream>
#include <ctime>

using namespace std;

class griefbacon: public IterativeRobot
{
private:
	AdvancedJoystick* m_driver;
	AdvancedJoystick* m_operator;

	Talon* m_lDrive1;
	Talon* m_lDrive2;
	Talon* m_rDrive1;
	Talon* m_rDrive2;

	Encoder* m_lEncode;

	RobotDrive* m_drive;
	Elevator* m_elev;

	HotSubsystemHandler* m_subsys;
public:
	griefbacon()
	{
		m_driver = new AdvancedJoystick (0);
		m_operator = new AdvancedJoystick (1);

		m_driver->SetDeadband(0.2);
		m_driver->SetDeadbandType(AdvancedJoystick::kQuad);
		m_operator->SetDeadband(0.2);
		m_operator->SetDeadbandType(AdvancedJoystick::kQuad);

		m_rDrive1 = new Talon (0);
		m_rDrive2 = new Talon (1);
		m_lDrive1 = new Talon (2);
		m_lDrive2 = new Talon (3);

		m_lEncode = new Encoder (2,3,false);

		m_drive = new RobotDrive (m_lDrive1, m_lDrive2, m_rDrive1, m_rDrive2);
		m_drive->SetSafetyEnabled(false);

		m_elev = new Elevator (4,5,0);

		m_subsys = new HotSubsystemHandler;
		m_subsys->Add(m_elev);
		m_subsys->Start();
	}

	~griefbacon()
	{
		m_subsys->Stop();
	}

	void RobotInit()
	{

	}

	void DisabledInit ()
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
		m_elev->Set(m_operator->GetRawAxis(AdvancedJoystick::kLeftY));
	}

	void TestPeriodic()
	{

		if (m_operator->GetRawButton(AdvancedJoystick::kButtonA))
			m_elev->Set(Relay::kForward);
		else if (m_operator->GetRawButton(AdvancedJoystick::kButtonB))
			m_elev->Set(Relay::kReverse);
		else
			m_elev->Set(Relay::kOff);
	}
};

START_ROBOT_CLASS(griefbacon);
