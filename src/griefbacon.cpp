#include "WPILib.h"
#include "RobotUtils/RobotUtils.h"
#include "Elevator.h"
#include "Arm.h"
#include "Drivetrain.h"
#include "DistancePIDWrapper.h"
#include "FeedbackWrapper.h"
#include "GyroWrapper.h"

using namespace std;
#include <cmath>


//Add back in variables

class griefbacon: public IterativeRobot
{
private:
	AdvancedJoystick* m_driver;
	AdvancedJoystick* m_operator;

	Talon* m_dummy;
	Talon* m_dummy2;
	Talon* m_dummy3;
	Gyro* m_gyro;

	Encoder* m_encodeL;
	Encoder* m_encodeR;

	PIDController* m_turnPID;
	PIDController* m_distancePID;
	PIDController* m_FeedbackPID;

	RobotDrive* m_drive;

	DistancePIDWrapper* m_distancePIDWrapper;
	GyroWrapper* m_GyroWrapper;
	FeedbackWrapper* m_FeedbackWrapper;
	BackgroundDebugger* m_debug;
	Drivetrain* m_drivetrain;
	Arm* m_arm;
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

		m_dummy = new Talon(5);
		m_dummy2 = new Talon(6);
		m_dummy3 = new Talon(7);

		m_drivetrain = new Drivetrain (0,1,2,3);
		m_arm = new Arm(11,16,14,10,15,12,13);
		m_elev = new Elevator (4,5,0,8);

		m_gyro = new Gyro(0);

		m_subsys = new HotSubsystemHandler;
		m_subsys->Add(m_elev);
		m_subsys->Add(m_drivetrain);
		m_subsys->Add(m_arm);
	}
		m_encodeR = new Encoder(0,1);
		m_encodeL = new Encoder(2,3);

		double gyro_P = 0.015;
		double gyro_I = 0.0;
		double gyro_D = 0.01;
		m_turnPID = new PIDController(gyro_P, gyro_I, gyro_D, m_gyro, m_dummy);
		m_distancePIDWrapper = new DistancePIDWrapper (m_encodeL, m_encodeR);
		m_distancePID = new PIDController(0.16,0.0,0.0,m_distancePIDWrapper,m_dummy2);
		m_FeedbackWrapper = new FeedbackWrapper(m_encodeL, m_encodeR);
		m_FeedbackPID = new PIDController(0.1,0.0,0.0, m_FeedbackWrapper, m_dummy3);
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
		m_turnPID->SetSetpoint(0);
		m_distancePID->SetSetpoint(2000);
	}

	void TeleopPeriodic()
	{



		if (m_driver->GetRawButton(AdvancedJoystick::kButtonB))
		{
			m_encodeL->Reset();
			m_encodeR->Reset();
		}
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

		if (m_operator->GetRawButton(AdvancedJoystick::kButtonRB)){
			m_arm->rollerSet(1);
		{
			m_debug->LogData("Encoder Rate", m_FeedbackWrapper->PIDGet());
			m_drive->ArcadeDrive(-m_driver->GetRawAxis(AdvancedJoystick::kLeftY), -m_driver->GetRawAxis(AdvancedJoystick::kRightX));
		}
		else if (m_operator->GetRawButton(AdvancedJoystick::kButtonLB)){
			m_arm->rollerSet(-0.5);
		}
		else{
			m_arm->rollerSet(0);
		}

		else if (m_driver->GetRawButton(AdvancedJoystick::kButtonX))
		{
			m_gyro->Reset();
			m_gyroTime->Reset();
		}

		else if (m_driver->GetRawButton(AdvancedJoystick::kButtonLB)){
			ETA(2.4, 12.0, 0.0);
		}
		else if (m_driver->GetRawButton(AdvancedJoystick::kButtonRB)){
			m_etaFlag = 0;
		}

		else if (m_driver->GetRawButton(AdvancedJoystick::kButtonY)){
			m_FeedbackPID->SetSetpoint(3);
			m_FeedbackPID->Enable();
			m_turnPID->Enable();
			m_drive->ArcadeDrive(m_FeedbackPID->Get(), m_turnPID->Get());
		}

		else{
			m_drive->ArcadeDrive(-m_driver->GetRawAxis(AdvancedJoystick::kLeftY), -m_driver->GetRawAxis(AdvancedJoystick::kRightX));
			m_turnPID->Disable();
			m_FeedbackPID->Disable();
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
			m_elev->Reset();

		m_elev->GetPID(m_operator->GetRawButton(AdvancedJoystick::kButtonBack));

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


			m_arm->wristSet(m_operator->GetRawAxis(AdvancedJoystick::kLeftY));
		}

	void TestDrive() {
		m_drivetrain->ArcadeDrive(m_driver->GetRawAxis(AdvancedJoystick::kLeftY), m_driver->GetRawAxis(AdvancedJoystick::kRightX));
	}

	void PrintData() {
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
	}
};

	/** MISCELLANEOUS FUNCTIONS **/
	void PrintData ()
	{
		SmartDashboard::PutNumber("m_driftRatio", m_GyroWrapper->GetRatio());
		SmartDashboard::PutNumber("Feedback PID",m_FeedbackPID->Get()/5);
		SmartDashboard::PutNumber("Angle", m_gyro->GetAngle());
		SmartDashboard::PutNumber("Current Angle", m_gyro->GetAngle());
		//*1.02857142857142857142857142857143
		SmartDashboard::PutNumber("Rate", m_gyro->GetRate());
		SmartDashboard::PutBoolean("PIDGET", m_turnPID->IsEnabled());
		SmartDashboard::PutNumber("PID",m_turnPID->Get());
		SmartDashboard::PutNumber("Left Encoder",m_encodeL->GetDistance());
		SmartDashboard::PutNumber("Right Encoder",m_encodeR->GetDistance());
		SmartDashboard::PutNumber("Turn PIDGet", m_turnPID->Get());


		SmartDashboard::PutNumber("Gyro P",m_turnPID->GetP());
		SmartDashboard::PutNumber("Gyro I",m_turnPID->GetI());
		SmartDashboard::PutNumber("Gyro D",m_turnPID->GetD());
		SmartDashboard::PutNumber("Set Point", m_turnPID->GetSetpoint());
		SmartDashboard::PutNumber("Joystick Y", -m_driver->GetRawAxis(AdvancedJoystick::kRightX));

		SmartDashboard::PutNumber("Auto Drive P", m_distancePID->GetP());
		SmartDashboard::PutNumber("Auto Drive I", m_distancePID->GetI());
		SmartDashboard::PutNumber("Auto Drive D", m_distancePID->GetD());
		SmartDashboard::PutNumber("Distance PIDGet", m_distancePID->Get());
		SmartDashboard::PutNumber("Auto Drive Setpoint", m_distancePID->GetSetpoint());
		SmartDashboard::PutBoolean("Auto Drive Enabled", m_distancePID->IsEnabled());
		SmartDashboard::PutNumber("Encoder Rate Left", m_encodeL->GetRate() / 1200);
		SmartDashboard::PutNumber("Encoder Rate Right", m_encodeR->GetRate() / 1200);

		SmartDashboard::PutNumber("Encoder Rate Average", ((m_encodeL->GetRate() / 1200) - (m_encodeR->GetRate() / 1200)) / 2);
		SmartDashboard::PutNumber("m_distance", m_distance);
		SmartDashboard::PutNumber("m_speed", m_speed);
		SmartDashboard::PutNumber("ETA:",  4.0 - m_timer->Get());
		SmartDashboard::PutNumber("m_etaFlag", m_etaFlag);
		SmartDashboard::PutNumber("m_FeedbackPID",m_FeedbackPID->Get());
		SmartDashboard::PutNumber("m_timer", m_timer->Get());

	}

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

};

START_ROBOT_CLASS(griefbacon);
