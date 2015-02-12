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

	CANTalon* m_wrist;

	Encoder* m_lEncode;
	Encoder* m_shoulderEncode;
	Encoder* m_wristEncode;

	Elevator* m_elev;
	PIDController* m_shoulderPID;
	PIDController* m_wristPID;

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

		m_wrist = new CANTalon (14);

		m_drivetrain = new Drivetrain (0,1,2,3);
		m_arm = new Arm(11,16,14,10,15,12,13);
		m_elev = new Elevator (4,5,0);

		m_shoulderPID = new PIDController(0.1,0.0,0.0, m_shoulderEncode, m_arm);
		m_wristPID = new PIDController(0.1,0.0,0.0, m_wristEncode, m_wrist);

		m_subsys = new HotSubsystemHandler;
		m_subsys->Add(m_elev);
		m_subsys->Add(m_drivetrain);
		m_subsys->Add(m_arm);
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
		testDrive();
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
		else
			m_elev->Set(0);

		m_arm->shoulderSet(-m_operator->GetRawAxis(AdvancedJoystick::kLeftY));
		m_arm->wristSet(-m_operator->GetRawAxis(AdvancedJoystick::kRightY));

	}

	void teleopArm(){

		m_arm->shoulderSet(-m_operator->GetRawAxis(AdvancedJoystick::kRightY));

		if (m_operator->GetRawButton(AdvancedJoystick::kButtonRB)){
			m_arm->rollerSet(1);
		}
		else if (m_operator->GetRawButton(AdvancedJoystick::kButtonLB)){
			m_arm->rollerSet(-0.5);
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

	void testDrive(){

		m_drivetrain->ArcadeDrive(m_driver->GetRawAxis(AdvancedJoystick::kLeftY), m_driver->GetRawAxis(AdvancedJoystick::kRightX));

	}
};

START_ROBOT_CLASS(griefbacon);
