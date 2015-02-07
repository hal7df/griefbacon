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

	Encoder* m_lEncode;

	HotSubsystemHandler* m_subsys;

	Drivetrain* m_drivetrain;

	double m_loopcounter;
public:
	griefbacon()
	{
		m_driver = new AdvancedJoystick (0);

		m_driver->SetDeadband(0.2);
		m_driver->SetDeadbandType(AdvancedJoystick::kQuad);

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

	void TestInit ()
	{

	}

	void TestPeriodic()
	{
		m_drivetrain->ArcadeDrive(m_driver->GetRawAxis(AdvancedJoystick::kLeftY), m_driver->GetRawAxis(AdvancedJoystick::kRightX));
	}
};

START_ROBOT_CLASS(griefbacon);
