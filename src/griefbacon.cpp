#include "WPILib.h"
#include "RobotUtils/RobotUtils.h"
#include "Elevator.h"
#include "Arm.h"
#include "Drivetrain.h"
#include "GyroWrapper.h"

using namespace std;
#include <cmath>


//Add back in variables

class griefbacon: public IterativeRobot
{
private:
	AdvancedJoystick* m_driver;
	AdvancedJoystick* m_operator;

	HotSubsystemHandler* m_subsys;
	Drivetrain* m_drivetrain;
	Arm* m_arm;
	Elevator* m_elev;
public:
	griefbacon()
	{
		m_driver = new AdvancedJoystick(0);
		m_operator = new AdvancedJoystick(1);

		m_driver->SetDeadband(0.2);
		m_driver->SetDeadbandType(AdvancedJoystick::kQuad);
		m_operator->SetDeadband(0.2);
		m_operator->SetDeadbandType(AdvancedJoystick::kQuad);

		m_drivetrain = new Drivetrain (0,1,2,3,0,2,0);
		m_arm = new Arm(11,16,14,10,15,12,13);
		m_elev = new Elevator (4,5,0,8);

		m_subsys = new HotSubsystemHandler;
		m_subsys->Add(m_elev);
		m_subsys->Add(m_drivetrain);
		m_subsys->Add(m_arm);
		m_subsys->Add(m_drivetrain->GetGyroWrapper());

	}

	void RobotInit()
	{
		m_subsys->Start();
	}

	void DisabledPeriodic()
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

		TeleopElevator();
		TeleopDrive();
		TeleopArm();
		PrintData();
	}

	void TestInit ()
	{

	}

	void TestPeriodic()
	{
		TeleopDrive();
		if (m_operator->GetRawButton(AdvancedJoystick::kButtonA))
			m_elev->Set(Relay::kForward);
		else if (m_operator->GetRawButton(AdvancedJoystick::kButtonB))
			m_elev->Set(Relay::kReverse);

		else if(m_driver->GetRawButton(AdvancedJoystick::kButtonA))
			m_elev->Set(Relay::kOff);

		if (m_operator->GetPOV() == 0)
			m_elev->Set(0.5);
		else if (m_operator->GetPOV() == 180)
			m_elev->Set(-0.5);
		else
			m_elev->Set(0);

		m_arm->shoulderSet(-m_operator->GetRawAxis(AdvancedJoystick::kLeftY));
		m_arm->wristSet(-m_operator->GetRawAxis(AdvancedJoystick::kRightY));
		if (m_operator->GetRawButton(AdvancedJoystick::kButtonRB)){
			m_arm->rollerSet(1);
		}

		else if (m_operator->GetRawButton(AdvancedJoystick::kButtonLB)){
			m_arm->rollerSet(-0.5);
		}
		else{
			m_arm->rollerSet(0);
		}

		PrintData();

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

	/** SPECIALIZED FUNCTIONS **/
	void TeleopElevator ()
	{
		//Manual Control
		if ((m_operator->GetRawAxis(AdvancedJoystick::kLeftTrigger) > 0.1) && !m_elev->IsEnabled())
			m_elev->Set(-m_operator->GetRawAxis(AdvancedJoystick::kLeftTrigger));
		else if (m_operator->GetRawAxis(AdvancedJoystick::kRightTrigger) > 0.1 && !m_elev->IsEnabled())
			m_elev->Set(m_operator->GetRawAxis(AdvancedJoystick::kRightTrigger));
		else if (!m_elev->IsEnabled())
			m_elev->Set(0);

		if (m_operator->GetRawButton(AdvancedJoystick::kButtonStart))
		{
			m_elev->Reset();
			m_arm->wReset();
			m_arm->sReset();
		}

		m_elev->GetPID(m_operator->GetRawButton(AdvancedJoystick::kButtonBack));
		m_arm->GetPID(m_operator->GetRawButton(AdvancedJoystick::kButtonBack));

		if (m_operator->GetRawButton(AdvancedJoystick::kButtonA) && m_operator->GetRawButton(AdvancedJoystick::kButtonBack))
			m_elev->Set(kCarry);
		else if (m_operator->GetRawButton(AdvancedJoystick::kButtonB))
			m_elev->Set(kUMid);
		else if (m_operator->GetRawButton(AdvancedJoystick::kButtonX))
			m_elev->Set(kLMid);
		else if (m_operator->GetRawButton(AdvancedJoystick::kButtonY))
			m_elev->Set(kTop);
		else if (m_operator->GetRawButton(AdvancedJoystick::kButtonA))
			m_elev->Set(kBottom);
		else
			m_elev->Disable();
	}

	void TeleopDrive() {
		if (m_driver->GetRawButton(AdvancedJoystick::kButtonY)){
			m_drivetrain->SetDistance(2.0);
			m_drivetrain->EnableDistance();
		}
		else if (m_drivetrain->IsEnabledDistance())
			m_drivetrain->DisableDistance();

		if (m_driver->GetRawButton(AdvancedJoystick::kButtonBack) && m_driver->GetRawButton(AdvancedJoystick::kButtonStart)){
			m_drivetrain->ResetRatio();
		}
		else
			m_drivetrain->ArcadeDrive(m_driver->GetRawAxis(AdvancedJoystick::kLeftY), m_driver->GetRawAxis(AdvancedJoystick::kRightX));

	}

	void TeleopArm(){
		if (m_operator->GetPOV() == 180){
				m_arm->shoulderSetPos(ksGround);
				m_arm->wristSetPos(kwGround);
				m_arm->sEnable();
				m_arm->wEnable();
			}
			else if(m_operator ->GetPOV() == 225){
				m_arm->shoulderSetPos(ksTwoTote);
				m_arm->wristSetPos(kwTwoTote);
				m_arm->sEnable();
				m_arm->wEnable();
			}
			else if(m_operator ->GetPOV() == 270){
				m_arm->shoulderSetPos(ksDriving);
				m_arm->wristSetPos(kwDriving);
				m_arm->sEnable();
				m_arm->wEnable();
			}
			else if(m_operator ->GetPOV() == 315){
				m_arm->shoulderSetPos(ksCanStack);
				m_arm->wristSetPos(kwCanStack);
				m_arm->sEnable();
				m_arm->wEnable();
			}
			else if(m_operator ->GetPOV() == 0){
				m_arm->shoulderSetPos(ksPackage);
				m_arm->wristSetPos(kwPackage);
				m_arm->sEnable();
				m_arm->wEnable();
			}
			else
			{
				m_arm->sDisable();
				m_arm->wDisable();
				m_arm->shoulderSet(m_operator->GetRawAxis(AdvancedJoystick::kRightY));
				m_arm->wristSet(-m_operator->GetRawAxis(AdvancedJoystick::kLeftY));
			}
			if (m_operator->GetRawButton(AdvancedJoystick::kButtonRB)){
				m_arm->rollerSet(1);
			}
			else if (m_operator->GetRawButton(AdvancedJoystick::kButtonLB)){
				m_arm->rollerSet(-0.5);
			}
			else{
				m_arm->rollerSet(0);
			}

			if (m_driver->GetRawAxis(AdvancedJoystick::kRightTrigger) > 0.2){
				m_arm->intakeSet(-m_driver->GetRawAxis(AdvancedJoystick::kRightTrigger));
			}
			else if (m_driver->GetRawAxis(AdvancedJoystick::kLeftTrigger) > 0.2){
				m_arm->intakeSet(m_driver->GetRawAxis(AdvancedJoystick::kLeftTrigger));
			}
			else
			{
				m_arm->intakeSet(0);
			}

		}

	/** MISCELLANEOUS FUNCTIONS **/
	void PrintData ()
	{
		SmartDashboard::PutNumber("Driver Left Y",m_driver->GetRawAxis(AdvancedJoystick::kLeftY));
		SmartDashboard::PutNumber("Driver Right X",m_driver->GetRawAxis(AdvancedJoystick::kRightX));
		SmartDashboard::PutNumber("Operator Left Y",m_operator->GetRawAxis(AdvancedJoystick::kLeftY));
		SmartDashboard::PutNumber("Operator Right Y",m_operator->GetRawAxis(AdvancedJoystick::kRightY));
		SmartDashboard::PutNumber("Operator Left Trigger",m_operator->GetRawAxis(AdvancedJoystick::kLeftTrigger));
		SmartDashboard::PutNumber("Operator Right Trigger",m_operator->GetRawAxis(AdvancedJoystick::kRightTrigger));

		SmartDashboard::PutBoolean("Operator Left Bumper",m_operator->GetRawButton(AdvancedJoystick::kButtonLB));
		SmartDashboard::PutBoolean("Operator Right Bumper",m_operator->GetRawButton(AdvancedJoystick::kButtonRB));

		SmartDashboard::PutNumber("Driver Raw Left Y",m_driver->GetJoystick()->GetRawAxis(1));
		SmartDashboard::PutNumber("Driver Calc",(m_driver->GetJoystick()->GetRawAxis(1)/fabs(m_driver->GetJoystick()->GetRawAxis(1)))*(pow(((fabs(m_driver->GetJoystick()->GetRawAxis(1))-0.2)*(1/1-0.2)),2)));

		SmartDashboard::PutNumber("Joystick Y", -m_driver->GetRawAxis(AdvancedJoystick::kRightX));
	}
};

START_ROBOT_CLASS(griefbacon);
