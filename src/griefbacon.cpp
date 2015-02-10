#include "WPILib.h"
#include "RobotUtils/RobotUtils.h"
#include "Arm.h"
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

	HotSubsystemHandler* m_subsys;
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


		m_rDrive1 = new Talon (0);
		m_rDrive2 = new Talon (1);
		m_lDrive1 = new Talon (2);
		m_lDrive2 = new Talon (3);

		m_lEncode = new Encoder (2,3,false);

		m_drive = new RobotDrive (m_lDrive1, m_lDrive2, m_rDrive1, m_rDrive2);
		m_drive->SetSafetyEnabled(false);

		m_arm = new Arm(11,16,14,10,15,12,13);

		m_subsys = new HotSubsystemHandler;
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
		teleopArm();


	}

	void TestPeriodic()
	{
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

	void teleopArm(){

		m_arm->shoulderSet(-m_operator->GetRawAxis(AdvancedJoystick::kRightY));

		if (m_operator->GetRawButton(AdvancedJoystick::kButtonRB)){
			m_arm->rollerSet(1);
		}
		else if (m_operator->GetRawButton(AdvancedJoystick::kButtonLB)){
			m_arm->rollerSet(-1);
		}
		else{
			m_arm->rollerSet(0);
		}


		if (m_operator->GetRawAxis(AdvancedJoystick::kRightTrigger)){
			m_arm->wristSet(1);
		}
		else if (m_operator->GetRawAxis(AdvancedJoystick::kLeftTrigger)){
			m_arm->wristSet(-1);
		}
		else{
			m_arm->wristSet(0);
		}
	}
};

START_ROBOT_CLASS(griefbacon);
