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

	CANTalon* m_wrist;

	Encoder* m_lEncode;
	Gyro* m_gyro;
	Encoder* m_shoulderEncode;
	Encoder* m_wristEncode;

	Encoder* m_encodeL;
	Encoder* m_encodeR;

	RobotDrive* m_drive;

	PIDController* m_shoulderPID;
	PIDController* m_wristPID;

	HotSubsystemHandler* m_subsys;
	GyroWrapper* m_GyroWrapper;
	BackgroundDebugger* m_debug;
	Drivetrain* m_drivetrain;
	Arm* m_arm;
	Elevator* m_elev;
public:

	Timer* m_timer;

	Timer* m_gyroTime;

	double m_speed;

	double m_ratio1;

	double m_ratio2;

	double m_distance;

	double m_angle;

	int m_etaFlag;

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
		m_elev = new Elevator (4,5,0,8);

		m_gyro = new Gyro(0);

		m_shoulderPID = new PIDController(0.1,0.0,0.0, m_shoulderEncode, m_arm);
		m_wristPID = new PIDController(0.1,0.0,0.0, m_wristEncode, m_wrist);

		m_subsys = new HotSubsystemHandler;
		m_subsys->Add(m_elev);
		m_subsys->Add(m_drivetrain);
		m_subsys->Add(m_arm);
		m_encodeR = new Encoder(0,1);
		m_encodeL = new Encoder(2,3);

		m_speed = .4;

		m_ratio1 = 26.11001704;

		m_ratio2 = 2.986676441;


		//Ratio: y=(.1955321253)(259.3608864)^x

		m_distance = 12.0;

		m_etaFlag = 0;

		m_angle = 0;

		m_timer = new Timer;

		m_gyroTime = new Timer;

		m_GyroWrapper = new GyroWrapper(m_gyro, m_gyroTime);

		m_debug = new BackgroundDebugger;
	}

	void RobotInit()
	{
		m_subsys->Start();
		m_gyroTime->Start();
	}

	void DisabledPeriodic()
	{
		m_GyroWrapper->GyroRatio();
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

	void TestPeriodic()
	{
		TestDrive();
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
		m_drive->ArcadeDrive(-m_driver->GetRawAxis(AdvancedJoystick::kLeftY), -m_driver->GetRawAxis(AdvancedJoystick::kRightX));
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

	void TestDrive() {
		m_drivetrain->ArcadeDrive(m_driver->GetRawAxis(AdvancedJoystick::kLeftY), m_driver->GetRawAxis(AdvancedJoystick::kRightX));
	}

	void armSetPoints(){
			}

	void PrintData() {
		SmartDashboard::PutNumber("Operator POV",m_operator ->GetPOV());
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

		SmartDashboard::PutNumber("m_driftRatio", m_GyroWrapper->GetRatio());
		SmartDashboard::PutNumber("Angle", m_gyro->GetAngle());
		SmartDashboard::PutNumber("Current Angle", m_gyro->GetAngle());
		//*1.02857142857142857142857142857143
		SmartDashboard::PutNumber("Rate", m_gyro->GetRate());
		SmartDashboard::PutNumber("Left Encoder",m_encodeL->GetDistance());
		SmartDashboard::PutNumber("Right Encoder",m_encodeR->GetDistance());



		SmartDashboard::PutNumber("Joystick Y", -m_driver->GetRawAxis(AdvancedJoystick::kRightX));


		SmartDashboard::PutNumber("Encoder Rate Left", m_encodeL->GetRate() / 1200);
		SmartDashboard::PutNumber("Encoder Rate Right", m_encodeR->GetRate() / 1200);

		SmartDashboard::PutNumber("Encoder Rate Average", ((m_encodeL->GetRate() / 1200) - (m_encodeR->GetRate() / 1200)) / 2);
		SmartDashboard::PutNumber("m_distance", m_distance);
		SmartDashboard::PutNumber("m_speed", m_speed);
		SmartDashboard::PutNumber("ETA:",  4.0 - m_timer->Get());
		SmartDashboard::PutNumber("m_etaFlag", m_etaFlag);
		SmartDashboard::PutNumber("m_timer", m_timer->Get());

	}
/*
	void ETA(double time, double distance, double angle)
	{
		double speedSC;
		double speed;
		speed = distance / time;
		speedSC = pow(speed/m_ratio1, 1/m_ratio2);
		switch(m_etaFlag){
			case 0:
				m_encodeL->Reset();
				m_encodeR->Reset();
				m_timer->Reset();
				m_timer->Start();
				m_timer->Reset();
				m_turnPID->SetSetpoint(angle);
				m_FeedbackPID->SetSetpoint(speed);
				m_distancePID->SetSetpoint(distance);
				m_etaFlag++;
				break;
			case 1:
				if(!m_timer->HasPeriodPassed(time * 0.9)){
					m_drive->ArcadeDrive((speedSC + (m_FeedbackPID->Get()/5)), m_turnPID->Get());
					m_FeedbackPID->Enable();
					m_turnPID->Enable();
				}
				else
					m_etaFlag++;
				break;
			case 2:
				m_FeedbackPID->Disable();
				m_timer->Stop();
				m_drive->ArcadeDrive(m_distancePID->Get(), m_turnPID->Get());
				m_distancePID->Enable();

				if (m_encodeL->GetRate() == 0.0 && fabs(m_distancePID->Get()) <= 0.5) {
					m_turnPID->Disable();
					m_distancePID->Disable();
					m_drive->ArcadeDrive(0.0,0.0);
					m_etaFlag++;
				}
				break;
			case 3:

				break;
		}
	}
	*/

};

START_ROBOT_CLASS(griefbacon);
