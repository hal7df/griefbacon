#include "WPILib.h"
#include "RobotUtils/RobotUtils.h"
#include "Elevator.h"
#include "Arm.h"
#include "Drivetrain.h"

using namespace std;

class griefbacon: public IterativeRobot
{
private:
	AdvancedJoystick* m_driver;
	AdvancedJoystick* m_operator;

	Encoder* m_lEncode;

	Elevator* m_elev;
	HotSubsystemHandler* m_subsys;
	Drivetrain* m_drivetrain;
	Arm* m_arm;
public:
	griefbacon()
	{
		m_driver = new AdvancedJoystick(0);
		m_operator = new AdvancedJoystick(1);

		m_driver->SetDeadband(0.2);
		m_driver->SetDeadbandType(AdvancedJoystick::kQuad);
		m_operator->SetDeadband(0.2);
		m_operator->SetDeadbandType(AdvancedJoystick::kQuad);

		m_drivetrain = new Drivetrain (0,1,2,3);
		m_operator->SetDeadbandType(AdvancedJoystick::kQuad);



		m_lEncode = new Encoder (2,3,false);

		m_arm = new Arm(11,16,14,10,15,12,13);
		m_subsys = new HotSubsystemHandler;
		m_subsys->Add(m_elev);
		m_subsys->Add(m_drivetrain);
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
		m_drivetrain->ArcadeDrive(m_driver->GetRawAxis(AdvancedJoystick::kLeftY), m_driver->GetRawAxis(AdvancedJoystick::kRightX));
	}

	void TestPeriodic()
	{
		if (m_operator->GetRawButton(AdvancedJoystick::kButtonA))
			m_elev->Set(Relay::kForward);
		else if (m_operator->GetRawButton(AdvancedJoystick::kButtonB))
			m_elev->Set(Relay::kReverse);
		else
			m_elev->Set(Relay::kOff);

		if (m_operator->GetPOV() == 0)
			m_elev->Set(0.5);
		else if (m_operator->GetPOV() == 180)
			m_elev->Set(-0.5);

		m_drivetrain->ArcadeDrive(m_driver->GetRawAxis(AdvancedJoystick::kLeftY), m_driver->GetRawAxis(AdvancedJoystick::kRightX));

		m_arm->shoulderSet(-m_operator->GetRawAxis(AdvancedJoystick::kLeftY));
		m_arm->wristSet(-m_operator->GetRawAxis(AdvancedJoystick::kRightY));

		if (m_operator->GetRawButton(AdvancedJoystick::kButtonRB)){
			m_arm->rollerSet(1);
		}
		else if (m_operator->GetRawButton(AdvancedJoystick::kButtonLB)){
			m_arm->rollerSet(-1);
		}
		else{
			m_arm->rollerSet(0);
		}


		if (m_operator->GetRawButton(AdvancedJoystick::kTriggerL)){
			m_arm->intakeSet(1);
		}
		else if (m_operator->GetRawButton(AdvancedJoystick::kTriggerR)){
			m_arm->intakeSet(-1);
		}
		else{
			m_arm->intakeSet(0);
		}
	}
};

START_ROBOT_CLASS(griefbacon);
