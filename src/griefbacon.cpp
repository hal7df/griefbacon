#include "WPILib.h"
#include "RobotUtils/RobotUtils.h"
#include "Elevator.h"
#include "Arm.h"
#include "Drivetrain.h"

using namespace std;
#include <cmath>


enum auton_t {
	kThreeTote
};

class griefbacon: public IterativeRobot
{
private:
	AdvancedJoystick* m_driver;
	AdvancedJoystick* m_operator;

	HotSubsystemHandler* m_subsys;
	Drivetrain* m_drivetrain;
	Arm* m_arm;
	Elevator* m_elev;

	Timer* m_resetTime;

	bool f_elevReset, f_shoulderReset, f_wristReset;
	auton_t m_autonChoice;
	unsigned m_autonCase;
	unsigned m_autonLoop;
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

		m_resetTime = new Timer;

		f_elevReset = false;
		f_shoulderReset = false;
		f_wristReset = false;

		m_autonChoice = kThreeTote;
		m_autonCase= 0;
		m_autonLoop = 0;
	}

	void RobotInit()
	{
		m_subsys->Start();
	}

	void DisabledPeriodic()
	{
		PrintData();

		switch(m_autonChoice)
		{
		case kThreeTote:
			SmartDashboard::PutString("Auton Mode","Three Tote");
			break;
		}
	}

	void AutonomousInit()
	{
		m_autonCase = 0;
		m_autonLoop = 0;
		m_drivetrain->SetLimit(0.6);
		m_drivetrain->ResetEncoders();
		m_drivetrain->ResetGyroAngle();

		f_elevReset = false;
		f_shoulderReset = false;
		f_wristReset = false;

		m_resetTime->Stop();
		m_resetTime->Start();
		m_resetTime->Reset();
	}

	void AutonomousPeriodic()
	{
		ZeroAll();
		PrintData();

		switch (m_autonChoice)
		{
		case kThreeTote:
			JayAutonThreeTote();
			break;
		}
	}

	/** AUTONOMOUS FUNCTIONS **/

	void AutonThreeTote ()
	{
		switch (m_autonCase)
		{
		case 0:
			if (f_elevReset && f_shoulderReset && f_wristReset)
				m_autonCase++;
			break;
		case 1:
			m_elev->Set(kCarry);
			m_arm->shoulderSetPos(ksDriving);
			m_arm->wristSetPos(kwDriving);

			if (!m_arm->sIsEnabled())
				m_arm->sEnable();
			if (!m_arm->wIsEnabled())
				m_arm->wEnable();

			if (m_elev->AtSetpoint())
			{
				m_elev->Disable();
				m_autonCase++;
			}
			break;
		case 2:
			m_drivetrain->SetDistance(-0.25);

			if (!m_drivetrain->IsEnabledDistance())
				m_drivetrain->EnableDistance();

			if(m_drivetrain->DistanceAtSetpoint())
			{
				m_drivetrain-> DisableDistance();
				m_autonCase++;
			}
				break;
		case 3:
				if (m_autonLoop < 2)
				{
					m_elev->Set(kTop);
					if (m_elev->GetDistance() > ELEVATOR_UMID)
					{
						m_drivetrain->ResetEncoders();
						m_autonCase++;
					}
				}
				else
				{
					m_elev->Set(kCarry);
					m_drivetrain->ResetEncoders();
					m_autonCase = 6;
				}
			break;
		case 4:
			m_drivetrain->SetDistance(6.5);
			m_arm->clearCans(true);
			m_drivetrain->EnableDistance();
			if (m_drivetrain->GetDistancePID() > 4)
			{
				m_arm->clearCans(false);
				m_arm->intakeSet(-1);
			}
			if(m_drivetrain->DistanceAtSetpoint())
			{
				m_drivetrain-> DisableDistance();
				m_arm->intakeSet(0);
				m_autonCase++;
			}
			break;
		case 5:
			m_elev->Set(kBottom);
			if(m_elev->AtSetpoint())
			{
				if(m_autonLoop < 2)
				{
					m_drivetrain->ResetEncoders();
					m_autonCase = 1;
					m_autonLoop++;
				}
			}
			break;
		case 6:
			break;
		}
	}

	void JayAutonThreeTote ()
	{
		switch (m_autonCase)
		{
		case 0:
			if (f_elevReset && f_shoulderReset && f_wristReset)
				m_autonCase++;
			break;
		case 1:
			m_elev->Set(kCarry);
			m_arm->shoulderSetPos(ksDriving);
			m_arm->wristSetPos(kwDriving);

			if (!m_arm->sIsEnabled())
				m_arm->sEnable();
			if (!m_arm->wIsEnabled())
				m_arm->wEnable();

			if (m_elev->AtSetpoint())
			{
				m_elev->Disable();
				m_autonCase++;
			}
			break;
		case 2:
			m_drivetrain->SetDistance(-0.25);

			if (!m_drivetrain->IsEnabledDistance())
				m_drivetrain->EnableDistance();

			if(m_drivetrain->DistanceAtSetpoint())
			{
				m_drivetrain-> DisableDistance();
				m_autonCase++;
			}
				break;
		case 3:
				if (m_autonLoop < 2)
				{
					m_elev->Set(kTop);
					if (m_elev->GetDistance() > ELEVATOR_UMID)
					{
						m_drivetrain->ResetEncoders();
						m_autonCase++;
					}
				}
				else
				{
					m_elev->Set(kCarry);
					m_drivetrain->ResetEncoders();
					m_autonCase = 6;
					m_drivetrain->SetAngleHeading(-90.0);
					m_drivetrain->SetLimit(0.4);
					m_drivetrain->SetCorrLimit(0.5);
					m_drivetrain->SetDistance(-7.);
					m_drivetrain->EnableDistance();
				}
			break;
		case 4:
			m_drivetrain->SetDistance(6.5);
			m_arm->clearCans(true);
			m_drivetrain->SetLimit(0.4);
			m_drivetrain->EnableDistance();
			if(m_drivetrain->GetDistancePID() > 0.5)
				m_drivetrain->SetLimit(0.65);
			if (m_drivetrain->GetDistancePID() > 4)
			{
				m_arm->clearCans(false);
				m_arm->intakeSet(-0.7);
			}
			if(m_drivetrain->DistanceAtSetpoint())
			{
				m_drivetrain-> DisableDistance();
				m_arm->intakeSet(0);
				m_autonCase++;
			}
			break;
		case 5:
			m_elev->Set(kBottom);
			if(m_elev->AtSetpoint())
			{
				if(m_autonLoop < 2)
				{
					m_drivetrain->ResetEncoders();
					m_autonCase = 1;
					m_autonLoop++;
				}
			}
			break;
		case 6:
			if(m_drivetrain->DistanceAtSetpoint())
			{
				m_drivetrain->ResetEncoders();
				m_drivetrain->SetDistance(-4.0);
				m_drivetrain->SetAngleHeading(0.0);
				m_elev->Set(kBottom);
				m_arm->intakeSet(1);
				m_autonCase++;
			}
			break;
		case 7:
			if (m_drivetrain->DistanceAtSetpoint())
			{
				m_drivetrain->DisableDistance();
				m_arm->intakeSet(0);
				SmartDashboard::PutNumber("Auton Time",DriverStation::GetInstance()->GetMatchTime());
				m_autonCase++;
			}
			break;
		}
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
			m_elev->Set(kBottom);
		else if (m_operator->GetRawButton(AdvancedJoystick::kButtonB))
			m_elev->Set(kUMid);
		else if (m_operator->GetRawButton(AdvancedJoystick::kButtonX))
			m_elev->Set(kLMid);
		else if (m_operator->GetRawButton(AdvancedJoystick::kButtonY))
			m_elev->Set(kTop);
		else if (m_operator->GetRawButton(AdvancedJoystick::kButtonA))
			m_elev->Set(kCarry);
		else
			m_elev->Disable();
	}

	void TeleopDrive() {
		if (m_driver->GetRawButton(AdvancedJoystick::kButtonX)){
			m_drivetrain->PIDWrite(-.5);
			//This function is DriveStraightForward
		}
		else if (m_driver->GetRawButton(AdvancedJoystick::kButtonB)){
			m_drivetrain->PIDWrite(.5);
			//This function is DriveStraightBackward
		}
		else if (m_driver->GetRawButton(AdvancedJoystick::kButtonY))
		{
			m_drivetrain->EnableDistance();
		}

		else if (m_driver->GetRawButton(AdvancedJoystick::kButtonLB)){
			m_drivetrain->ResetFlags();
			m_drivetrain->ResetGyroAngle();
		}

		else if (m_drivetrain->IsEnabledDistance())
			m_drivetrain->DisableDistance();
		else if (m_driver->GetRawButton(AdvancedJoystick::kButtonStart))
			m_drivetrain->ResetEncoders();

		else
			m_drivetrain->ArcadeDrive(m_driver->GetRawAxis(AdvancedJoystick::kLeftY), m_driver->GetRawAxis(AdvancedJoystick::kRightX));

		m_drivetrain->SetPID(m_driver->GetRawButton(AdvancedJoystick::kButtonBack));
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

		SmartDashboard::PutNumber("Auton Case",m_autonCase);
		SmartDashboard::PutNumber("Auton Loop",m_autonLoop);
	}
	void ZeroAll ()
	{
		if (!f_elevReset)
			m_elev->Set(-.3);
		if (!f_shoulderReset)
			m_arm->shoulderSet(-.2);
		if (!f_wristReset)
			m_arm->wristSet(.2);

		if (m_resetTime->Get() > 0.1)
		{
			m_resetTime->Stop();
			if (fabs(m_elev->GetRate()) < 0.15 && !f_elevReset)
			{
				m_elev->Reset();
				f_elevReset = true;
				m_elev->Set(0.0);
			}
			if (fabs(m_arm->GetWristRate()) < 0.04 && !f_wristReset)
			{
				m_arm->wReset();
				f_wristReset = true;
				m_arm->wristSet(0.0);
			}
			if (fabs(m_arm->GetShoulderRate()) < 0.04 && !f_shoulderReset)
			{
				m_arm->sReset();
				f_shoulderReset = true;
				m_arm->shoulderSet(0.0);
			}
		}
	}
};

START_ROBOT_CLASS(griefbacon);
