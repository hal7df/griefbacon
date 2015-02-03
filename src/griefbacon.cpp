#include "WPILib.h"
#include "RobotUtils/RobotUtils.h"
#include "Drivetrain.h"
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

	HotSubsystemHandler* m_subsys;

	Drivetrain* m_drivetrain;

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

		m_drivetrain = new Drivetrain (0,1,2,3);

		m_lEncode = new Encoder (2,3,false);

		m_subsys = new HotSubsystemHandler;
		m_subsys-> Add (m_drivetrain);
		m_subsys->Start();

		m_loopcounter = 0;
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
		m_drivetrain->ArcadeDrive(m_driver->GetRawAxis(AdvancedJoystick::kLeftY), m_driver->GetRawAxis(AdvancedJoystick::kRightX));
	}

	void TestPeriodic()
	{

	}
};

START_ROBOT_CLASS(griefbacon);
