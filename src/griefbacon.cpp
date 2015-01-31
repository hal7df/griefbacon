#include "WPILib.h"
#include "RobotUtils/AdvancedJoystick.h"
#include "DistancePIDWrapper.h"
#include "FeedbackWrapper.h"
#include <cmath>

class griefbacon: public IterativeRobot
{
private:
	AdvancedJoystick* m_driver;

	Talon* m_lDrive1;
	Talon* m_lDrive2;
	Talon* m_rDrive1;
	Talon* m_rDrive2;
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

	FeedbackWrapper* m_FeedbackWrapper;

	Timer* m_timer;

	double m_speed;

	double m_ratio1;

	double m_ratio2;

	double m_distance;

	double m_angle;

	int m_etaFlag;

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

		m_dummy = new Talon(5);
		m_dummy2 = new Talon(6);
		m_dummy3 = new Talon(7);

		m_drive = new RobotDrive (m_lDrive1, m_lDrive2, m_rDrive1, m_rDrive2);
		m_drive->SetSafetyEnabled(false);

		m_gyro = new Gyro(0);

		m_encodeR = new Encoder(0,1);
		m_encodeL = new Encoder(2,3);

		double gyro_P = 0.015;
		double gyro_I = 0.0;
		double gyro_D = 0.01;

		m_turnPID = new PIDController(gyro_P, gyro_I, gyro_D, m_gyro, m_dummy);
		m_distancePIDWrapper = new DistancePIDWrapper (m_encodeL, m_encodeR);
		m_distancePID = new PIDController(0.01,0.,0.,m_distancePIDWrapper,m_dummy2);
		m_FeedbackWrapper = new FeedbackWrapper(m_encodeL, m_encodeR);
		m_FeedbackPID = new PIDController(FEED_P, FEED_I, FEED_D, m_FeedbackWrapper, m_dummy3);
		m_speed = .4;

		m_ratio1 = 26.11001704;

		m_ratio2 = 2.986676441;


		//Ratio: y=(.1955321253)(259.3608864)^x

		m_distance = 5.0;

		m_etaFlag = 0;

		m_angle = 0;

		m_timer = new Timer;
	}

	void RobotInit()
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

		else if(m_driver->GetRawButton(AdvancedJoystick::kButtonA))
		{
			m_turnPID->Enable();
			m_distancePID->Enable();
			m_drive->ArcadeDrive(m_distancePID->Get(),m_turnPID->Get());
		}

		else if (m_driver->GetRawButton(AdvancedJoystick::kButtonX))
		{
			m_gyro->Reset();
		}

		if (m_driver->GetRawButton(AdvancedJoystick::kButtonLB)){
			ETA(3.0, m_distance, m_angle);
		}
		else if (m_driver->GetRawButton(AdvancedJoystick::kButtonRB)){
			m_etaFlag = 0;
		}

		else{
			m_drive->ArcadeDrive(-m_driver->GetRawAxis(AdvancedJoystick::kLeftY), -m_driver->GetRawAxis(AdvancedJoystick::kRightX));
		//	m_turnPID->Disable();
		}

		PrintData();




	}

	void TestPeriodic()
	{

	}

	/** MISCELLANEOUS FUNCTIONS **/
	void PrintData ()
	{
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
		SmartDashboard::PutNumber("ETA:",  ((m_distance / (m_ratio1 * (pow(m_speed, (m_ratio2))))) - m_timer->Get()));
		SmartDashboard::PutNumber("m_etaFlag", m_etaFlag);

	}

	void ETA(double time, double distance, double angle)
	{
		double speedSC;
		double speed;
		speed = distance / time;
		m_FeedbackPID->SetSetpoint(speed);
		speedSC = pow(speed/m_ratio1, 1/m_ratio2);
		switch(m_etaFlag){
		case 0:
			m_encodeL->Reset();
			m_encodeR->Reset();
			m_timer->Reset();
			m_timer->Start();
			m_turnPID->SetSetpoint(angle);
			m_etaFlag++;
			break;
		case 1:
			if(!m_timer->HasPeriodPassed(time)){
				m_drive->ArcadeDrive((speedSC + (m_FeedbackPID->Get()/5)), m_turnPID->Get());
				SmartDashboard::PutNumber("Encoder Count Average", (m_encodeL->GetDistance() - m_encodeR->GetDistance()) / 2400);
				m_turnPID->Enable();
				m_FeedbackPID->Enable();
				SmartDashboard::PutNumber("HOT STUFF",m_FeedbackPID->Get());
			}
			else
				m_etaFlag++;
			break;
		case 2:
			m_timer->Stop();
			m_drive->ArcadeDrive(0.0, m_turnPID->Get());
			if (m_encodeL->GetRate() == 0.0){
				m_turnPID->Disable();
				m_etaFlag++;
			}
			break;
		case 3:
			m_timer->Reset();
			break;
		}
	}
};

START_ROBOT_CLASS(griefbacon);
