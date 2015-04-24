#include "WPILib.h"
#include "RobotUtils/RobotUtils.h"
#include "Elevator.h"
#include "Arm.h"
#include "Drivetrain.h"

using namespace std;
#include <cmath>


enum auton_t {
	kThreeTote,
	kThreeTotew90Turn,
	kThreeToteBack,
	kThreeToteSmack,
	kTwoCan,
	kDriveForward,
	kKnockCanGoAutoZone,
	kNothing,
	kCanBurglar,
	kCanBurglarStay,
	kCanBurglarDelayDrive
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
	BackgroundDebugger* m_debug;

	Timer* m_resetTime;
	Timer* m_autonTime;
	Timer* m_burgleTime;

	PowerDistributionPanel* m_pdp;

	bool f_elevReset, f_shoulderReset, f_wristReset;
	bool f_autonTipping;
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

		m_drivetrain = new Drivetrain (0,1,2,3,0,2);
		m_arm = new Arm(11,16,14,10,15,12,13);
		m_elev = new Elevator (4,5,0,8);
		m_debug = new BackgroundDebugger (1000,true);

		m_subsys = new HotSubsystemHandler;
		m_subsys->Add(m_elev);
		m_subsys->Add(m_drivetrain);
		m_subsys->Add(m_arm);
		m_subsys->Add(m_debug);

		m_pdp = new PowerDistributionPanel;

		m_resetTime = new Timer;
		m_autonTime = new Timer;
		m_burgleTime = new Timer;

		f_elevReset = false;
		f_shoulderReset = false;
		f_wristReset = false;
		f_autonTipping = false;

		m_autonChoice = kNothing;
		m_autonCase= 0;
		m_autonLoop = 0;
	}

	void RobotInit()
	{
		m_debug->AddValue("Elevator Encoder",m_elev->GetEncoder());
		m_debug->AddValue("Shoulder Encoder",m_arm->GetShoulder());
		m_debug->AddValue("Wrist Encoder",m_arm->GetWrist());
		m_debug->AddValue("Gyro",m_drivetrain->GetGyro());
		m_debug->AddValue("Drive Encoder Average",m_drivetrain->GetEncoders());
		m_debug->AddValue("Left Drive Encoder",m_drivetrain->GetLEncode());
		m_debug->AddValue("Right Drive Encoder",m_drivetrain->GetREncode());

		m_debug->SetAutonCase(&m_autonCase);
		m_debug->SetCaseDuration(4.0);
		m_subsys->Start();
		//bam_subsys->SetPrintData(false);
	}

	void DisabledInit()
	{
		if (m_autonCase > 0)
		{
			m_debug->SetManualLoggingName("EndOfAuton");
			m_debug->LogData("Autonomous Case",(double)m_autonCase);
			m_debug->LogData("Gyro Tipping",(double)f_autonTipping);
			m_debug->LogData("Gyro Angle",m_drivetrain->GetGyroAngle());
			m_debug->LogData("Elevator position",m_elev->GetDistance());
			m_debug->LogData("Wrist position",m_arm->GetWrist()->GetDistance());
			m_debug->LogData("Shoulder position",m_arm->GetShoulder()->GetDistance());
			m_debug->LogData("Encoder average",m_drivetrain->GetDistancePID());
			m_debug->LogData("Left drive encoder",m_drivetrain->GetLEncode()->GetDistance());
			m_debug->LogData("Right drive encoder",m_drivetrain->GetREncode()->GetDistance());
			m_debug->LogData("Drive at setpoint",m_drivetrain->DistanceAtSetpoint());
			m_debug->LogData("Elevator at setpoint",m_elev->AtSetpoint());
			m_debug->LogData("Wrist at setpoint",m_arm->WristAtSetpoint());
			m_debug->LogData("Shoulder at setpoint",m_arm->ShoulderAtSetpoint());
			m_debug->SetManualLoggingName("Teleop");
		}

		m_debug->StopRun();

		m_drivetrain->DisableDistance();
		m_elev->Disable();
		m_arm->wDisable();
		m_arm->sDisable();

		SmartDashboard::PutString("DB/String0","Auton Mode");
	}

	void DisabledPeriodic()
	{
		//PrintData();


		if (m_operator->GetRawButton(AdvancedJoystick::kButtonBack) && m_operator->GetRawButton(AdvancedJoystick::kButtonB))
			m_autonChoice = kCanBurglarDelayDrive;
		else if (m_operator->GetRawButton(AdvancedJoystick::kButtonB))
			m_autonChoice = kCanBurglar;
		else if (m_operator->GetRawButton(AdvancedJoystick::kButtonX))
			m_autonChoice = kKnockCanGoAutoZone;
		else if (m_operator->GetRawButton(AdvancedJoystick::kButtonY))
			m_autonChoice = kDriveForward;
		else if (m_operator->GetRawButton(AdvancedJoystick::kButtonA) && m_operator->GetRawButton(AdvancedJoystick::kButtonBack))
			m_autonChoice = kThreeTote;
		else if (m_operator->GetRawButton(AdvancedJoystick::kButtonA))
			m_autonChoice = kThreeTotew90Turn;
		else if (m_operator->GetRawButton(AdvancedJoystick::kButtonBack) && m_operator->GetRawButton(AdvancedJoystick::kButtonStart))
			m_autonChoice = kNothing;
		else if (m_operator->GetRawButton(AdvancedJoystick::kButtonBack) && m_operator->GetRawButton(AdvancedJoystick::kButtonRB))
			m_autonChoice = kThreeToteSmack;
		else if (m_operator->GetRawButton(AdvancedJoystick::kButtonRB))
			m_autonChoice = kThreeToteBack;
		else if (m_operator->GetRawButton(AdvancedJoystick::kButtonLB))
			m_autonChoice = kCanBurglarStay;

		switch(m_autonChoice)
		{
		case kThreeTote:
			SmartDashboard::PutString("Auton Mode","Three Tote");
			break;
		case kThreeTotew90Turn:
			SmartDashboard::PutString("Auton Mode","Three Tote w 90 Turn");
			break;
		case kCanBurglar:
			SmartDashboard::PutString("Auton Mode","Can Burglar");
			break;
		case kDriveForward:
			SmartDashboard::PutString("Auton Mode", "Drive Forward");
			break;
		case kKnockCanGoAutoZone:
			SmartDashboard::PutString("Auton Mode", "Knock Can, Go Auto Zone");
			break;
		case kNothing:
			SmartDashboard::PutString("Auton Mode", "Do Nothing");
			break;
		case kThreeToteBack:
			SmartDashboard::PutString("Auton Mode", "Three Tote, Drive Back");
			break;
		case kThreeToteSmack:
			SmartDashboard::PutString("Auton Mode", "Three Tote SMACK");
			break;
		case kCanBurglarStay:
			SmartDashboard::PutString("Auton Mode", "Burglar Arm Down, Stays");
			break;
		case kCanBurglarDelayDrive:
			SmartDashboard::PutString("Auton Mode", "Can Burglar Delay and Drive");
			break;
		}
	}

	void AutonomousInit()
	{
		m_autonCase = 0;
		m_autonLoop = 0;
		m_drivetrain->SetLimit(0.55);
		m_drivetrain->ResetEncoders();
#ifdef NAVX_ENABLED
		m_drivetrain->ResetGyroAngle();
#endif
		m_drivetrain->SetGyroOffset();

		f_elevReset = false;
		f_shoulderReset = false;
		f_wristReset = false;

		m_resetTime->Stop();
		m_resetTime->Start();
		m_resetTime->Reset();

		f_autonTipping = false;

		switch (m_autonChoice)
		{
		case kThreeTote:
			m_debug->SetMaxAutonCase(8);
			break;
		case kThreeTotew90Turn:
			m_debug->SetMaxAutonCase(10);
			m_autonTime->Stop();
			m_autonTime->Start();
			m_autonTime->Reset();
			break;
		case kThreeToteBack:
			m_debug->SetMaxAutonCase(11);
			break;
		case kTwoCan:
			m_debug->SetMaxAutonCase(7);
			break;
		case kDriveForward:
			m_debug->SetMaxAutonCase(3);
			break;
		case kKnockCanGoAutoZone:
			m_debug->SetMaxAutonCase(7);
			break;
		case kNothing:
			m_debug->SetMaxAutonCase(0);
			break;
		case kCanBurglar:
			m_debug->SetMaxAutonCase(3);
			break;
		case kCanBurglarStay:
			m_debug->SetMaxAutonCase(1);
			break;
		case kThreeToteSmack:
			m_debug->SetMaxAutonCase(11);
			break;
		case kCanBurglarDelayDrive:
			m_debug->SetMaxAutonCase(3);
			break;
		}
/*
		if (!m_debug->Running())
			m_debug->StartRun();
		m_debug->EnableWatch(true);
*/
	}

	void AutonomousPeriodic()
	{
		//PrintData();
		if (!(((m_autonChoice != kCanBurglar) && (m_autonChoice == kCanBurglarStay)) || ((m_autonChoice == kCanBurglar) && (m_autonChoice != kCanBurglarStay))))
			ZeroAll();

		switch (m_autonChoice)
		{
		case kThreeTote:
			AutonThreeTote();
			break;
		case kThreeTotew90Turn:
			AutonThreeTotew90Turn();
			break;
		case kTwoCan:
			AutonTwoCan();
			break;
		case kDriveForward:
			AutonDriveForward();
			break;
		case kKnockCanGoAutoZone:
			AutonKnockCanGoAutoZone();
			break;
		case kThreeToteBack:
			AutonThreeToteBackFaster();
			break;
		case kThreeToteSmack:
			AutonThreeToteBackwTurns();
			break;
		case kCanBurglar:
			AutonCanBurglar();
			break;
		case kCanBurglarStay:
			AutonCanBurglarStay();
			break;
		case kCanBurglarDelayDrive:
			AutonCanBurglarDelayDrive();
			break;
		}

		m_debug->LogData("Robot Roll", m_drivetrain->GetGyroRoll());

	}

	/** AUTONOMOUS FUNCTIONS **/

	void AutonThreeTote ()
	{
		switch (m_autonCase)
			{
		//Waits until all pids are 0
			case 0:
				if (f_elevReset && f_shoulderReset && f_wristReset)
					m_autonCase++;
				break;
		//Sets all pids to driving position. If elevator is at setpoint, it moves on to the next case.
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
					m_autonCase++;
				}
				break;
			//Robot backs up. Goes to the next case when the distance is at setpoint.
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
		//if auton loop is less than two, then elevator is raised. Will go on to next case if elevator is at midpoint and resets encoders
		//else carries tote, resets encoders, changes turning heading, and allows it to turn quickly, also starts driving and skips to case six
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
						m_drivetrain->SetAngleHeading(-90.0);
						m_drivetrain->SetLimit(0.85);
						m_drivetrain->SetCorrLimit(0.5);
						m_drivetrain->SetDistance(-7.);
						m_drivetrain->EnableDistance();
						m_autonCase = 6;
					}
				break;
		//clears can while driving to next tote and speeds up after clearing can. Then begins to take in next tote.
		//once robot is around next tote, disables pid, and goes to next case
			case 4:
				m_drivetrain->SetDistance(5.85);
				m_arm->clearCans(true);
				m_drivetrain->SetLimit(0.40);
				m_drivetrain->SetAngleHeading(0.);
				m_drivetrain->EnableDistance();
				if(m_drivetrain->GetDistancePID() > 1.5)
					m_drivetrain->SetLimit(0.7);
				if (m_drivetrain->GetDistancePID() > 3)
				{
					m_drivetrain->SetAngleHeading(0.);
					m_arm->clearCans(false);
					m_arm->intakeSet(-1.0);
				}
				if(m_drivetrain->DistanceAtSetpoint())
				{
					m_drivetrain-> DisableDistance();
					m_autonCase++;
				}
				break;
		//elevator picks up next tote
		//if autonLoop is less than two, then it will reset encoders then go to case 1
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
		//if robot reaches setpoint, encoders will reset, distance and heading will change, setting down totes and goes to next case
			case 6:
				if(m_drivetrain->DistanceAtSetpoint())
				{
					m_drivetrain->DisableDistance();
					m_drivetrain->ResetEncoders();
					m_drivetrain->SetDistance(-7.5);
					m_drivetrain->SetAngleHeading(0.0);
					m_drivetrain->SetCorrLimit(0.3);

					m_autonCase++;
				}
				break;
			case 7:
				if (!m_drivetrain->IsEnabledDistance())
					m_drivetrain->EnableDistance();
				if (m_drivetrain->GetDistancePID() < -3.5)
				{
					m_elev->Set(kBottom);
					m_arm->intakeSet(1);
				}
				if (m_drivetrain->DistanceAtSetpoint())
				{
					m_drivetrain->DisableDistance();
					m_drivetrain->ArcadeDrive(0.,0.);
					//m_arm->intakeSet(0);
					SmartDashboard::PutNumber("Auton Time",DriverStation::GetInstance()->GetMatchTime());
					m_autonCase++;
				}
				break;
			}
	}

	void AutonThreeTotew90Turn ()
		{
		if (m_autonTime->Get()>14 && m_autonCase < 9)
		{
			m_drivetrain->DisableAngle();
			m_drivetrain->ResetEncoders();
			m_drivetrain->SetDistance(-6.0);
			m_drivetrain->SetAngleHeading(0.0);
			m_drivetrain->SetCorrLimit(0.3);
			m_drivetrain->EnableDistance();
			m_autonCase = 9;
		}
		switch (m_autonCase)
				{
			//Waits until all pids are 0
				case 0:
					if (f_elevReset && f_shoulderReset && f_wristReset)
						m_autonCase++;
					break;
			//Sets all pids to driving position. If elevator is at setpoint, it moves on to the next case.
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
						m_autonCase=3;
					}
					break;
				//Robot backs up. Goes to the next case when the distance is at setpoint.
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
			//if auton loop is less than two, then elevator is raised. Will go on to next case if elevator is at midpoint and resets encoders
			//else carries tote, resets encoders, changes turning heading, and allows it to turn quickly, also starts driving and skips to case six
				case 3:
						if (m_autonLoop < 2)
						{
							m_elev->Set(kUMid);
							m_drivetrain->ResetEncoders();
							m_autonCase++;
						}
						else
						{
							m_elev->Set(kCarry);
							m_drivetrain->ResetEncoders();
							//m_drivetrain->SetAngleHeading(-90.0);
							//m_drivetrain->SetLimit(0.85);
							//m_drivetrain->SetCorrLimit(0.5);
							//m_drivetrain->SetDistance(-7.);
							//m_drivetrain->EnableDistance();
							m_drivetrain->SetTurnPIDHeading(-90.);
							m_drivetrain->EnableAngle();
							m_autonCase = 6;
						}
					break;
			//clears can while driving to next tote and speeds up after clearing can. Then begins to take in next tote.
			//once robot is around next tote, disables pid, and goes to next case
				case 4:
					m_drivetrain->SetDistance(5.7);
					m_arm->clearCans(true);
					m_drivetrain->SetLimit(0.40);
					m_drivetrain->SetAngleHeading(0.);
					m_drivetrain->EnableDistance();
					if(m_drivetrain->GetDistancePID() > 1.5)
					{

						if (m_autonLoop == 0)
							m_drivetrain->SetLimit(0.5);
						else
							m_drivetrain->SetLimit(0.55);
					}
					if (m_drivetrain->GetDistancePID() > 3)
					{
						m_drivetrain->SetAngleHeading(0.);
						m_arm->clearCans(false);
						m_arm->intakeSet(-1.0);
					}
					if(m_drivetrain->DistanceAtSetpoint())
					{
						m_drivetrain-> DisableDistance();
						m_autonCase++;
					}
					break;
			//elevator picks up next tote
			//if autonLoop is less than two, then it will reset encoders then go to case 1
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
			//if robot reaches setpoint, encoders will reset, distance and heading will change, setting down totes and goes to next case
				case 6:
					if(m_drivetrain->TurnPIDatSetpoint())
					{
						m_drivetrain->DisableAngle();
						m_drivetrain->ResetEncoders();
						m_drivetrain->SetLimit(0.7);
						m_drivetrain->SetDistance(-5.75);
						m_drivetrain->SetAngleHeading(-90.0);
						m_drivetrain->SetCorrLimit(0.3);
						m_drivetrain->EnableDistance();
						m_autonCase++;
					}
					break;
				case 7:
					if (m_drivetrain->DistanceAtSetpoint())
					{
						m_drivetrain->DisableDistance();
						m_drivetrain->ResetEncoders();
						m_drivetrain->SetTurnPIDHeading(0.);
						m_drivetrain->EnableAngle();
						m_autonCase++;
					}
					break;
				case 8:
					if(m_drivetrain->TurnPIDatSetpoint())
					{
						m_drivetrain->DisableAngle();
						m_drivetrain->SetDistance(-6.0);
						m_drivetrain->SetAngleHeading(0.0);
						m_drivetrain->SetCorrLimit(0.3);
						m_drivetrain->EnableDistance();
						m_autonCase++;
					}
					break;
				case 9:
					m_elev->Set(kBottom);
					m_arm->intakeSet(1);
					if (m_drivetrain->DistanceAtSetpoint())
					{
						m_drivetrain->DisableDistance();
						m_drivetrain->ArcadeDrive(0.,0.);
						SmartDashboard::PutNumber("Auton Time",DriverStation::GetInstance()->GetMatchTime());
						m_autonCase++;
					}
					break;
				}
		}

	void AutonThreeToteBack ()
	{
/*	if (m_autonTime->Get()>14 && m_autonCase < 9)
	{
		m_drivetrain->DisableAngle();
		m_drivetrain->ResetEncoders();
		m_drivetrain->SetDistance(-6.0);
		m_drivetrain->SetAngleHeading(0.0);
		m_drivetrain->SetCorrLimit(0.3);
		m_drivetrain->EnableDistance();
		m_autonCase = 9;
	}*/
	switch (m_autonCase)
			{
		//Waits until all pids are 0
			case 0:
				if (f_elevReset && f_shoulderReset && f_wristReset)
					m_autonCase++;
				break;
		//Sets all pids to driving position. If elevator is at setpoint, it moves on to the next case.
			case 1:
				m_elev->Set(kCarry);
				m_arm->shoulderSetPos(ksDriving);
				m_arm->wristSetPos(kwAuton);

				if (!m_arm->sIsEnabled())
					m_arm->sEnable();
				if (!m_arm->wIsEnabled())
					m_arm->wEnable();

				if (m_elev->AtSetpoint())
				{
					if (m_autonLoop == 0)
						m_autonCase = 2;
					else
						m_autonCase = 3;
				}
				break;
			//Robot backs up. Goes to the next case when the distance is at setpoint.
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
		//if auton loop is less than two, then elevator is raised. Will go on to next case if elevator is at midpoint and resets encoders
		//else carries tote, resets encoders, changes turning heading, and allows it to turn quickly, also starts driving and skips to case six
			case 3:
					if (m_autonLoop < 2)
					{
						m_elev->Set(kUMid);
						m_drivetrain->ResetEncoders();
						if(m_autonLoop == 0)
						{
							if(m_elev->AtSetpoint())
								m_autonCase++;
						}
						else
							m_autonCase++;
					}
					else
					{
						m_elev->Set(kCarry);
						m_drivetrain->ResetEncoders();
						//m_drivetrain->SetAngleHeading(-90.0);
						//m_drivetrain->SetLimit(0.85);
						//m_drivetrain->SetCorrLimit(0.5);
						//m_drivetrain->SetDistance(-7.);
						//m_drivetrain->EnableDistance();
						m_drivetrain->SetTurnPIDHeading(60.);
						m_drivetrain->EnableAngle();
						m_autonCase = 6;
					}
				break;
		//clears can while driving to next tote and speeds up after clearing can. Then begins to take in next tote.
		//once robot is around next tote, disables pid, and goes to next case
			case 4:
				m_drivetrain->SetDistance(5.7);
				m_arm->clearCans(true);
				m_drivetrain->SetLimit(0.40);
				m_drivetrain->SetAngleHeading(0.);
				m_drivetrain->EnableDistance();
				if(m_drivetrain->GetDistancePID() > 2)
				{
						m_drivetrain->SetLimit(0.6);
				}
				if (m_drivetrain->GetDistancePID() > 3)
				{
					m_drivetrain->SetAngleHeading(0.);
					m_arm->clearCans(false);
					m_arm->intakeSet(-1.0);
				}
				if(m_drivetrain->DistanceAtSetpoint())
				{
					m_drivetrain-> DisableDistance();
					m_autonCase++;
				}
				break;
		//elevator picks up next tote
		//if autonLoop is less than two, then it will reset encoders then go to case 1
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
		//if robot reaches setpoint, encoders will reset, distance and heading will change, setting down totes and goes to next case
			case 6:
				if(m_drivetrain->TurnPIDatSetpoint())
				{
					m_drivetrain->DisableAngle();
					m_drivetrain->ResetEncoders();
					m_drivetrain->SetLimit(0.7);
					m_drivetrain->SetDistance(11.75);
					m_drivetrain->SetAngleHeading(60.0);
					m_drivetrain->SetCorrLimit(0.2);
					m_drivetrain->EnableDistance();

					m_arm->wristSetPos(kwGround);
					m_arm->shoulderSetPos(ksGround);
					m_autonCase++;
				}
				break;
			case 7:
				if (m_drivetrain->DistanceAtSetpoint())
				{

					m_drivetrain->DisableDistance();
					m_drivetrain->ResetEncoders();
//					m_drivetrain->SetTurnPIDHeading(72.5);
//					m_drivetrain->EnableAngle();
					m_autonCase=9;
				}
				break;
			case 8:
			if (m_drivetrain->TurnPIDatSetpoint())
			{
				m_drivetrain->DisableAngle();
				m_drivetrain->ResetEncoders();

				m_autonCase++;
			}
			break;

			case 9:
				if (m_arm->ShoulderAtSetpoint() && m_arm->WristAtSetpoint())
				{
					m_arm->rollerSet(1);
					m_drivetrain->SetDistance(-4.0);
					m_drivetrain->SetAngleHeading(72.5);
					m_drivetrain->SetCorrLimit(0.25);
					m_drivetrain->SetLimit(0.5);
					m_drivetrain->EnableDistance();
					m_autonCase++;
				}
				break;
			case 10:
				m_elev->Set(kBottom);
				if (m_drivetrain->GetDistancePID() < -0.5)
					m_arm->intakeSet(1);
				if (m_drivetrain->DistanceAtSetpoint())
				{
					m_drivetrain->DisableDistance();
					m_drivetrain->ArcadeDrive(0.,0.);
					SmartDashboard::PutNumber("Auton Time",DriverStation::GetInstance()->GetMatchTime());
					m_autonCase++;
				}
				break;
			}
	}

	void AutonThreeToteBackFaster ()
	{
/*	if (m_autonTime->Get()>14 && m_autonCase < 9)
	{
		m_drivetrain->DisableAngle();
		m_drivetrain->ResetEncoders();
		m_drivetrain->SetDistance(-6.0);
		m_drivetrain->SetAngleHeading(0.0);
		m_drivetrain->SetCorrLimit(0.3);
		m_drivetrain->EnableDistance();
		m_autonCase = 9;
	}*/
	switch (m_autonCase)
			{
		//Waits until all pids are 0
			case 0:
				if (f_elevReset && f_shoulderReset && f_wristReset)
					m_autonCase++;
				break;
		//Sets all pids to driving position. If elevator is at setpoint, it moves on to the next case.
			case 1:
				m_elev->Set(kUMid);
				m_arm->shoulderSetPos(ksDriving);
				m_arm->wristSetPos(kwAuton);

				if (!m_arm->sIsEnabled())
					m_arm->sEnable();
				if (!m_arm->wIsEnabled())
					m_arm->wEnable();

				if (m_autonLoop == 0)
					m_autonCase = 2;
				else
					m_autonCase = 3;

				break;
			//Robot backs up. Goes to the next case when the distance is at setpoint.
			case 2:
				m_drivetrain->SetDistance(-0.45);

				if (!m_drivetrain->IsEnabledDistance())
					m_drivetrain->EnableDistance();

				if(m_drivetrain->DistanceAtSetpoint())
				{
					m_drivetrain-> DisableDistance();
					m_autonCase++;
				}
					break;
		//if auton loop is less than two, then elevator is raised. Will go on to next case if elevator is at midpoint and resets encoders
		//else carries tote, resets encoders, changes turning heading, and allows it to turn quickly, also starts driving and skips to case six
			case 3:
					if (m_autonLoop < 2)
					{
						m_drivetrain->ResetEncoders();
						if(m_autonLoop == 0)
						{
							if(m_elev->AtSetpoint())
								m_autonCase++;
						}
						else
							m_autonCase++;
					}
					else
					{
						m_elev->Set(kCarry);
						m_arm->intakeSet(0.0);
						m_drivetrain->ResetEncoders();
						//m_drivetrain->SetAngleHeading(-90.0);
						//m_drivetrain->SetLimit(0.85);
						//m_drivetrain->SetCorrLimit(0.5);
						//m_drivetrain->SetDistance(-7.);
						//m_drivetrain->EnableDistance();
						m_drivetrain->SetTurnPIDHeading(60.);
						m_drivetrain->EnableAngle();
						m_autonCase = 6;
					}
				break;
		//clears can while driving to next tote and speeds up after clearing can. Then begins to take in next tote.
		//once robot is around next tote, disables pid, and goes to next case
			case 4:
				m_drivetrain->SetDistance(5.7);
				m_arm->clearCans(true);
				m_drivetrain->SetLimit(0.425);
				m_drivetrain->SetAngleHeading(0.);
				m_drivetrain->EnableDistance();
				if(m_drivetrain->GetDistancePID() > 2)
				{
						m_drivetrain->SetLimit(0.6);
				}
				if (m_drivetrain->GetDistancePID() > 3)
				{
					m_drivetrain->SetAngleHeading(0.);
					m_arm->clearCans(false);
					m_arm->intakeSet(-1.0);
				}
				if(m_drivetrain->DistanceAtSetpoint())
				{
					m_drivetrain-> DisableDistance();
					m_autonCase++;
				}
				break;
		//elevator picks up next tote
		//if autonLoop is less than two, then it will reset encoders then go to case 1
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
		//if robot reaches setpoint, encoders will reset, distance and heading will change, setting down totes and goes to next case
			case 6:
				if(m_drivetrain->TurnPIDatSetpoint())
				{
					m_drivetrain->DisableAngle();
					m_drivetrain->ResetEncoders();
					m_drivetrain->SetLimit(0.8);
					m_drivetrain->SetDistance(7.50);
					m_drivetrain->SetAngleHeading(60.0);
					m_drivetrain->SetCorrLimit(0.2);
					m_drivetrain->EnableDistance();

					m_arm->wristSetPos(kwGround);
					m_arm->shoulderSetPos(ksGround);
					m_autonCase++;
				}
				break;
			case 7:
				if (m_drivetrain->DistanceAtSetpoint())
				{

					m_drivetrain->DisableDistance();
					m_drivetrain->ResetEncoders();
					m_elev->Set(kBottom);
//					m_drivetrain->SetTurnPIDHeading(72.5);
//					m_drivetrain->EnableAngle();
					m_autonCase=9;
				}
				break;
			case 8:
			if (m_drivetrain->TurnPIDatSetpoint())
			{
				m_drivetrain->DisableAngle();
				m_drivetrain->ResetEncoders();
				m_autonCase++;
			}
			break;

			case 9:
				if (m_arm->ShoulderAtSetpoint() && m_arm->WristAtSetpoint())
				{
					m_arm->rollerSet(1);
					m_arm->intakeSet(1);
					m_drivetrain->SetDistance(-4.0);
					m_drivetrain->SetAngleHeading(72.5);
					m_drivetrain->SetCorrLimit(0.25);
					m_drivetrain->SetLimit(0.5);
					m_drivetrain->EnableDistance();
					m_autonCase++;
				}
				break;
			case 10:
				if (m_drivetrain->DistanceAtSetpoint())
				{
					m_drivetrain->DisableDistance();
					m_drivetrain->ArcadeDrive(0.,0.);
					SmartDashboard::PutNumber("Auton Time",DriverStation::GetInstance()->GetMatchTime());
					m_autonCase++;
				}
				break;
			}
	}

	void AutonThreeToteBackwTurns ()
	{
	switch (m_autonCase)
			{
		//Waits until all pids are 0
			case 0:
				if (f_elevReset && f_shoulderReset && f_wristReset)
					m_autonCase++;
				break;
		//Sets all pids to driving position. If elevator is at setpoint, it moves on to the next case.
			case 1:
				m_elev->Set(kUMid);
				m_arm->shoulderSetPos(ksDriving);
				m_arm->wristSetPos(kwAuton);

				if (!m_arm->sIsEnabled())
					m_arm->sEnable();
				if (!m_arm->wIsEnabled())
					m_arm->wEnable();

				if (m_autonLoop == 0)
					m_autonCase = 2;
				else
					m_autonCase = 3;

				break;
			//Robot backs up. Goes to the next case when the distance is at setpoint.
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
		//if auton loop is less than two, then elevator is raised. Will go on to next case if elevator is at midpoint and resets encoders
		//else carries tote, resets encoders, changes turning heading, and allows it to turn quickly, also starts driving and skips to case six
			case 3:
					if (m_autonLoop < 2)
					{
						m_drivetrain->ResetEncoders();
						if(m_autonLoop == 0)
						{
							if(m_elev->AtSetpoint())
								m_autonCase++;
						}
						else
							m_autonCase++;
					}
					else
					{
						m_elev->Set(kCarry);
						m_drivetrain->ResetEncoders();
						//m_drivetrain->SetAngleHeading(-90.0);
						//m_drivetrain->SetLimit(0.85);
						//m_drivetrain->SetCorrLimit(0.5);
						//m_drivetrain->SetDistance(-7.);
						//m_drivetrain->EnableDistance();
						m_drivetrain->SetTurnPIDHeading(60.);
						m_drivetrain->EnableAngle();
						m_autonCase = 6;
					}
				break;
		//clears can while driving to next tote and speeds up after clearing can. Then begins to take in next tote.
		//once robot is around next tote, disables pid, and goes to next case
			case 4:
				m_drivetrain->SetDistance(5.7);
				m_arm->clearCans(true);
				m_drivetrain->SetLimit(0.50);
				m_drivetrain->SetAngleHeading(5);
				m_drivetrain->EnableDistance();
				if(m_drivetrain->GetDistancePID() > 2)
				{
						m_drivetrain->SetAngleHeading(-10);
				}
				if (m_drivetrain->GetDistancePID() > 4)
				{
					m_drivetrain->SetAngleHeading(0.);
					m_arm->clearCans(false);
					m_arm->intakeSet(-1.0);
				}
				if(m_drivetrain->DistanceAtSetpoint())
				{
					m_drivetrain-> DisableDistance();
					m_autonCase++;
				}
				break;
		//elevator picks up next tote
		//if autonLoop is less than two, then it will reset encoders then go to case 1
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
		//if robot reaches setpoint, encoders will reset, distance and heading will change, setting down totes and goes to next case
			case 6:
				if(m_drivetrain->TurnPIDatSetpoint())
				{
					m_drivetrain->DisableAngle();
					m_drivetrain->ResetEncoders();
					m_drivetrain->SetLimit(0.8);
					m_drivetrain->SetDistance(11.75);
					m_drivetrain->SetAngleHeading(60.0);
					m_drivetrain->SetCorrLimit(0.2);
					m_drivetrain->EnableDistance();

					m_arm->wristSetPos(kwGround);
					m_arm->shoulderSetPos(ksGround);
					m_autonCase++;
				}
				break;
			case 7:
				if (m_drivetrain->DistanceAtSetpoint())
				{

					m_drivetrain->DisableDistance();
					m_drivetrain->ResetEncoders();
					m_elev->Set(kBottom);
//					m_drivetrain->SetTurnPIDHeading(72.5);
//					m_drivetrain->EnableAngle();
					m_autonCase=9;
				}
				break;
			case 8:
			if (m_drivetrain->TurnPIDatSetpoint())
			{
				m_drivetrain->DisableAngle();
				m_drivetrain->ResetEncoders();
				m_autonCase++;
			}
			break;

			case 9:
				if(m_elev->AtSetpoint())
					m_arm->intakeSet(1);
				if (m_arm->ShoulderAtSetpoint() && m_arm->WristAtSetpoint())
				{
					m_arm->rollerSet(1);
					m_drivetrain->SetDistance(-4.0);
					m_drivetrain->SetAngleHeading(72.5);
					m_drivetrain->SetCorrLimit(0.25);
					m_drivetrain->SetLimit(0.5);
					m_drivetrain->EnableDistance();
					m_autonCase++;
				}
				break;
			case 10:
				if (m_drivetrain->DistanceAtSetpoint())
				{
					m_drivetrain->DisableDistance();
					m_drivetrain->ArcadeDrive(0.,0.);
					m_autonCase++;
				}
				break;
			}
	}



	void AutonThreeToteSmack ()
	{
/*	if (m_autonTime->Get()>14 && m_autonCase < 9)
	{
		m_drivetrain->DisableAngle();
		m_drivetrain->ResetEncoders();
		m_drivetrain->SetDistance(-6.0);
		m_drivetrain->SetAngleHeading(0.0);
		m_drivetrain->SetCorrLimit(0.3);
		m_drivetrain->EnableDistance();
		m_autonCase = 9;
	}*/
	switch (m_autonCase)
			{
		//Waits until all pids are 0
			case 0:
				if (f_elevReset && f_shoulderReset && f_wristReset)
					m_autonCase++;
				break;
		//Sets all pids to driving position. If elevator is at setpoint, it moves on to the next case.
			case 1:
				m_elev->Set(kCarry);
				m_arm->shoulderSetPos(ksDriving);
				m_arm->wristSetPos(kwAuton);

				if (!m_arm->sIsEnabled())
					m_arm->sEnable();
				if (!m_arm->wIsEnabled())
					m_arm->wEnable();

				if (m_elev->AtSetpoint())
				{
					m_autonCase++;
				}
				break;
		//if auton loop is less than two, then elevator is raised. Will go on to next case if elevator is at midpoint and resets encoders
		//else carries tote, resets encoders, changes turning heading, and allows it to turn quickly, also starts driving and skips to case six
			case 2:
					if (m_autonLoop < 2)
					{
						m_elev->Set(kTop);
						m_drivetrain->ResetEncoders();
						m_drivetrain->SetTurnPIDHeading(15);
						m_arm->clearCans(true);
						m_drivetrain->EnableAngle();
						m_autonCase++;
					}
					else
					{
						m_elev->Set(kCarry);
						m_drivetrain->ResetEncoders();
						//m_drivetrain->SetAngleHeading(-90.0);
						//m_drivetrain->SetLimit(0.85);
						//m_drivetrain->SetCorrLimit(0.5);
						//m_drivetrain->SetDistance(-7.);
						//m_drivetrain->EnableDistance();
						m_drivetrain->SetTurnPIDHeading(60.);
						m_drivetrain->EnableAngle();
						m_autonCase = 6;
					}
				break;
		//clears can while driving to next tote and speeds up after clearing can. Then begins to take in next tote.
		//once robot is around next tote, disables pid, and goes to next case
			case 3:
				if (m_drivetrain->AtAngleHeading())
				{
					m_drivetrain->DisableAngle();
					m_drivetrain->ResetEncoders();
					m_drivetrain->SetDistance(.25);
					m_drivetrain->SetAngleHeading(15);
					m_drivetrain->EnableDistance();
				}
				if (m_drivetrain->DistanceAtSetpoint())
				{
					m_drivetrain->DisableDistance();
					m_drivetrain->ResetEncoders();

					if ((fabs(m_drivetrain->GetAverageRate())) < 0.5)
						{
							m_drivetrain->SetTurnPIDHeading(0);
							m_drivetrain->EnableAngle();
							m_autonCase++;
						}
				}
				break;
			case 4:
				if (m_drivetrain->AtAngleHeading())
				{
					m_drivetrain->SetDistance(5.5);
					if (m_autonLoop == 0)
						m_drivetrain->SetLimit(0.5);
					else
						m_drivetrain->SetLimit(0.55);
					m_drivetrain->SetAngleHeading(0.);
					m_drivetrain->EnableDistance();
				}
				if (m_drivetrain->GetDistancePID() > 2)
				{
					m_drivetrain->SetAngleHeading(0.0);
					m_arm->clearCans(false);
					m_arm->intakeSet(-1.0);
				}
				if(m_drivetrain->DistanceAtSetpoint())
				{
					m_drivetrain-> DisableDistance();
					m_autonCase++;
				}
				break;
		//elevator picks up next tote
		//if autonLoop is less than two, then it will reset encoders then go to case 1
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
		//if robot reaches setpoint, encoders will reset, distance and heading will change, setting down totes and goes to next case
			case 6:
				if(m_drivetrain->TurnPIDatSetpoint())
				{
					m_drivetrain->DisableAngle();
					m_drivetrain->ResetEncoders();
					m_drivetrain->SetLimit(0.7);
					m_drivetrain->SetDistance(11.75);
					m_drivetrain->SetAngleHeading(60.0);
					m_drivetrain->SetCorrLimit(0.2);
					m_drivetrain->EnableDistance();

					m_arm->wristSetPos(kwGround);
					m_arm->shoulderSetPos(ksGround);
					m_autonCase++;
				}
				break;
			case 7:
				if (m_drivetrain->DistanceAtSetpoint())
				{

					m_drivetrain->DisableDistance();
					m_drivetrain->ResetEncoders();
//					m_drivetrain->SetTurnPIDHeading(72.5);
//					m_drivetrain->EnableAngle();
					m_autonCase=9;
				}
				break;
			case 8:
			if (m_drivetrain->TurnPIDatSetpoint())
			{
				m_drivetrain->DisableAngle();
				m_drivetrain->ResetEncoders();

				m_autonCase++;
			}
			break;

			case 9:
				if (m_arm->ShoulderAtSetpoint() && m_arm->WristAtSetpoint())
				{
					m_arm->rollerSet(1);
					m_drivetrain->SetDistance(-4.0);
					m_drivetrain->SetAngleHeading(72.5);
					m_drivetrain->SetCorrLimit(0.25);
					m_drivetrain->SetLimit(0.5);
					m_drivetrain->EnableDistance();
					m_autonCase++;
				}
				break;
			case 10:
				m_elev->Set(kBottom);
				if (m_drivetrain->GetDistancePID() < -0.5)
					m_arm->intakeSet(0.5);
				if (m_drivetrain->DistanceAtSetpoint())
				{
					m_drivetrain->DisableDistance();
					m_drivetrain->ArcadeDrive(0.,0.);
					SmartDashboard::PutNumber("Auton Time",DriverStation::GetInstance()->GetMatchTime());
					m_autonCase++;
				}
				break;
			}
	}

	void AutonTwoCan()
	{
		switch (m_autonCase)
		{
		case 0:
			if (f_elevReset && f_shoulderReset && f_wristReset)
				m_autonCase++;
			break;
		case 1:
			m_arm->shoulderSetPos(ksDriving);
			m_arm->wristSetPos(kwDriving);

			if (!m_arm->sIsEnabled())
				m_arm->sEnable();
			if (!m_arm->wIsEnabled())
				m_arm->wEnable();

			m_autonCase++;
			break;
		case 2:
			m_drivetrain->SetDistance(-4.);
			if (!m_drivetrain->IsEnabledDistance())
				m_drivetrain->EnableDistance();

			if (m_drivetrain->DistanceAtSetpoint())
			{
				m_arm->shoulderSetPos(ksTwoTote);
				m_arm->wristSetPos(kwTwoTote);
				m_autonCase++;
			}
			break;
		case 3:
			if (m_arm->ShoulderAtSetpoint())
				m_autonCase++;
			break;
		case 4:
			m_drivetrain->SetAngleHeading(90.0);

			if (m_drivetrain->AtAngleHeading())
			{
				m_drivetrain->SetAngleHeading(-90.0);
				m_autonCase++;
			}
			break;
		case 5:
			if (m_drivetrain->AtAngleHeading())
			{
				m_drivetrain->SetAngleHeading(0.);
				m_drivetrain->SetDistance(4.0);
				m_arm->wristSetPos(kwDriving);
				m_arm->shoulderSetPos(ksDriving);
				m_autonCase++;
			}
			break;
		case 6:
			if (m_drivetrain->DistanceAtSetpoint())
			{
				m_arm->wDisable();
				m_arm->sDisable();
				m_autonCase++;
			}
		}
	}

	void AutonDriveForward()
	{
		switch (m_autonCase)
					{
				//Waits until all pids are 0
					case 0:
						if (f_elevReset && f_shoulderReset && f_wristReset)
							m_autonCase++;
						break;
				//Sets all pids to driving position. If elevator is at setpoint, it moves on to the next case.
					case 1:
						m_arm->shoulderSetPos(ksDriving);
						m_arm->wristSetPos(kwDriving);

						if (!m_arm->sIsEnabled())
							m_arm->sEnable();
						if (!m_arm->wIsEnabled())
							m_arm->wEnable();
						m_autonCase++;
						break;
					//Robot backs up. Goes to the next case when the distance is at setpoint.
					case 2:
						m_drivetrain->SetDistance(5.0);
						m_drivetrain->SetLimit(0.5);

						if (!m_drivetrain->IsEnabledDistance())
							m_drivetrain->EnableDistance();

						if(m_drivetrain->DistanceAtSetpoint())
						{
							m_drivetrain-> DisableDistance();
							m_autonCase++;
						}
							break;
					}
	}

	void AutonKnockCanGoAutoZone()
	{
		switch (m_autonCase)
			{
		//Waits until all pids are 0
			case 0:
				if (f_elevReset && f_shoulderReset && f_wristReset)
					m_autonCase++;
				break;
		//Sets all pids to driving position. If elevator is at setpoint, it moves on to the next case.
			case 1:
				m_arm->shoulderSetPos(ksDriving);
				m_arm->wristSetPos(kwDriving);

				if (!m_arm->sIsEnabled())
					m_arm->sEnable();
				if (!m_arm->wIsEnabled())
					m_arm->wEnable();
				m_autonCase++;
				break;
			//Robot backs up. Goes to the next case when the distance is at setpoint.
			case 2:
				m_drivetrain->SetDistance(4.0);
				m_drivetrain->SetLimit(0.5);

				if (!m_drivetrain->IsEnabledDistance())
					m_drivetrain->EnableDistance();

				if(m_drivetrain->DistanceAtSetpoint())
				{
					m_drivetrain->ResetEncoders();
					m_drivetrain-> DisableDistance();
					m_autonCase++;
				}
				break;
			case 3:
				m_arm->shoulderSetPos(ksCanKnock);
				m_arm->sEnable();
				m_arm->wristSetPos(kwCanKnock);
				m_arm->wEnable();

				if (m_arm->ShoulderAtSetpoint() && m_arm->WristAtSetpoint())
				{
					m_drivetrain->SetDistance(-5.0);
					m_drivetrain->EnableDistance();
					m_autonCase++;
				}
				break;
			case 4:
				if(m_drivetrain->DistanceAtSetpoint())
				{
					m_drivetrain->ResetEncoders();
					m_drivetrain-> DisableDistance();
					m_autonCase++;
				}
				break;
			case 5:
				m_arm->shoulderSetPos(ksDriving);
				m_arm->wristSetPos(kwDriving);

				if (m_arm->ShoulderAtSetpoint() && m_arm->WristAtSetpoint())
				{
					m_drivetrain->SetDistance(-1.0);
					m_drivetrain->EnableDistance();
					m_autonCase++;
				}
				break;
			case 6:
				m_drivetrain->SetDistance(6.0);
				m_drivetrain->SetLimit(0.5);

				if (!m_drivetrain->IsEnabledDistance())
					m_drivetrain->EnableDistance();

				if(m_drivetrain->DistanceAtSetpoint())
				{
					m_drivetrain-> DisableDistance();
					m_drivetrain->ResetEncoders();
					m_autonCase++;
				}
				break;
			}

	}

	void AutonCanBurglar()
	{
		switch(m_autonCase)
		{
		case 0:
			m_burgleTime->Stop();
			m_burgleTime->Start();
			m_burgleTime->Reset();
			m_arm->setBurgle(1.0);
			m_autonCase++;
			break;
		case 1:
			if (m_burgleTime->Get() > 0.05){
				m_drivetrain->SetAngleHeading(0);
				m_drivetrain->SetCorrLimit(0.1);
				m_drivetrain->SetLimit(0.95);
				m_autonCase++;
			}
			break;
		case 2:
			if (m_burgleTime->Get() > 0.4){
				m_arm->setBurgle(0);
			}
			m_drivetrain->SetDistance(7.0);
			if (!m_drivetrain->IsEnabledDistance())
				m_drivetrain->EnableDistance();
			if(m_drivetrain->DistanceAtSetpoint())
			{
				m_drivetrain-> DisableDistance();
				m_autonCase++;
			}
			break;
		}
	}

	void AutonCanBurglarStay()
		{
			switch(m_autonCase)
			{
			case 0:
				m_burgleTime->Stop();
				m_burgleTime->Start();
				m_burgleTime->Reset();
				m_arm->setBurgle(1.0);
				m_autonCase++;
				break;
			case 1:
				if (m_burgleTime->Get() > 0.4){
					m_arm->setBurgle(0);
					m_autonCase++;
				}
				break;
			}
		}

	void AutonCanBurglarDelayDrive()
		{
			switch(m_autonCase)
			{
			case 0:
				m_burgleTime->Stop();
				m_burgleTime->Start();
				m_burgleTime->Reset();
				m_arm->setBurgle(1.0);
				m_autonCase++;
				break;
			case 1:
				if (m_burgleTime->Get() > 0.3){ //0.3
					m_drivetrain->SetAngleHeading(0);
					m_drivetrain->SetCorrLimit(0.1);
					m_drivetrain->SetLimit(0.95);
					m_autonCase++;
				}
				break;
			case 2:
				if (m_burgleTime->Get() > 0.4){
					m_arm->setBurgle(0);
				}
				m_drivetrain->SetDistance(7.0);
				if (!m_drivetrain->IsEnabledDistance())
					m_drivetrain->EnableDistance();
				if(m_drivetrain->DistanceAtSetpoint())
				{
					m_drivetrain-> DisableDistance();
					m_autonCase++;
				}
				break;
			}
		}

	void TeleopInit()
	{
		m_drivetrain->DisableDistance();
		m_elev->Disable();
		m_arm->wDisable();
		m_arm->sDisable();

		m_drivetrain->DisableAngle();

		m_arm->intakeSet(0.0);

		if (!m_debug->Running())
			m_debug->StartRun();

		m_debug->EnableWatch(false);
		m_autonCase = 0;
	}

	void TeleopPeriodic()
	{
		TeleopElevator();
		TeleopDrive();
		TeleopArm();
		//PrintData();

		if (m_driver->GetButtonPress(AdvancedJoystick::kButtonA))
		{
			m_debug->SetTempMessage("Driver-noted debug point");
			m_debug->LogData("Debug point","Driver-noted");
		}
	}

	void TestInit ()
	{
	}

	void TestPeriodic()
	{

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
		if (m_operator->GetRawButton(AdvancedJoystick::kButtonBack))
		{
			m_elev->Reset();
			m_arm->wReset();
			m_arm->sReset();
		}

		m_elev->SetPID(m_operator->GetRawButton(AdvancedJoystick::kButtonBack));
		m_arm->GetPID(m_operator->GetRawButton(AdvancedJoystick::kButtonBack));

		if (m_operator->GetRawButton(AdvancedJoystick::kButtonRB))
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
		{
			m_elev->Disable();

			//Manual Control
			if (m_operator->GetRawAxis(AdvancedJoystick::kLeftTrigger) > 0.1)
				m_elev->Set(-m_operator->GetRawAxis(AdvancedJoystick::kLeftTrigger));
			else if (m_operator->GetRawAxis(AdvancedJoystick::kRightTrigger) > 0.1)
				m_elev->Set(m_operator->GetRawAxis(AdvancedJoystick::kRightTrigger));
			else
				m_elev->Set(0);
		}
	}

	void TeleopDrive() {
		/*
		if (m_driver->GetRawButton(AdvancedJoystick::kButtonX)){
			m_drivetrain->PIDWrite(-.5);
			//This function is DriveStraightForward
		}
		else if (m_driver->GetRawButton(AdvancedJoystick::kButtonB)){
			m_drivetrain->PIDWrite(.5);
			//This function is DriveStraightBackward)
		}
		else*/

		if (m_driver->GetRawButton(AdvancedJoystick::kButtonStart) && m_driver->GetRawButton(AdvancedJoystick::kButtonBack)){
			m_drivetrain->ResetFlags();
			m_drivetrain->ResetGyroAngle();
			m_drivetrain->ResetEncoders();
		}

		m_drivetrain->ArcadeDrive(m_driver->GetRawAxis(AdvancedJoystick::kLeftY), m_driver->GetRawAxis(AdvancedJoystick::kRightX));


	}

	void TeleopArm(){

		if (m_operator->GetRawButton(AdvancedJoystick::kButtonLB) && m_operator->GetRawButton(AdvancedJoystick::kButtonStart))
		{
			m_arm->shoulderSetPos(ksFourCanPlace);
			m_arm->wristSetPos(kwFourCanPlace);
		}
		else if (m_operator->GetRawButton(AdvancedJoystick::kButtonLB))
		{
			m_arm->shoulderSetPos(ksAutoPlace);
			m_arm->wristSetPos(kwAutoPlace);
			m_arm->rollerSet(-0.2);
		}
		else if (m_operator->GetPOV() == 180){
			m_arm->shoulderSetPos(ksGround);
			m_arm->wristSetPos(kwGround);
			m_arm->sEnable();
			m_arm->wEnable();
		}
		/*else if(m_operator ->GetPOV() == 270 && m_operator->GetRawButton(AdvancedJoystick::kButtonStart)){
			m_arm->shoulderSetPos(ksCanKnock);
			m_arm->wristSetPos(kwCanKnock);
			m_arm->sEnable();
			m_arm->wEnable();
		} */
		else if(m_operator->GetPOV() == 270 && m_operator->GetRawButton(AdvancedJoystick::kButtonStart)){
			m_arm->shoulderSetPos(ksGroundM);
			m_arm->wristSetPos(kwGroundM);
			m_arm->sEnable();
			m_arm->wEnable();
		}
		else if(m_operator ->GetPOV() == 270){
			m_arm->shoulderSetPos(ksDriving);
			m_arm->wristSetPos(kwDriving);
			m_arm->sEnable();
			m_arm->wEnable();
		}
		else if(m_operator ->GetPOV() == 0){
			m_arm->shoulderSetPos(ksFiveCan);
			m_arm->wristSetPos(kwFiveCan);
			m_arm->sEnable();
			m_arm->wEnable();
		}
		else if(m_operator ->GetPOV() == 90 && m_operator->GetRawButton(AdvancedJoystick::kButtonStart)){
			m_arm->shoulderSetPos(ksPackage);
			m_arm->wristSetPos(kwPackage);
			m_arm->sEnable();
			m_arm->wEnable();
		}
		else if(m_operator ->GetPOV() == 90){
			m_arm->shoulderSetPos(ksFourCanPlace);
			m_arm->wristSetPos(kwFourCanPlace);
			m_arm->sEnable();
			m_arm->wEnable();
		}
		else if((m_driver->GetRawAxis(AdvancedJoystick::kLeftTrigger) < 0.2) && ((m_operator->GetPOV() % 45) != 0))
		{
			m_arm->sDisable();
			m_arm->wDisable();
			m_arm->shoulderSet(m_operator->GetRawAxis(AdvancedJoystick::kRightY));
			m_arm->wristSet(-m_operator->GetRawAxis(AdvancedJoystick::kLeftY));
		}


		if (m_driver->GetRawButton(AdvancedJoystick::kButtonRB)){
			m_arm->rollerSet(1);
		}
		else if (m_driver->GetRawButton(AdvancedJoystick::kButtonLB)){
			if (m_operator->GetRawButton(AdvancedJoystick::kButtonLB) && m_operator->GetRawButton(AdvancedJoystick::kButtonStart))
				m_arm->rollerSet(-0.25);
			else
				m_arm->rollerSet(-0.5);
		}
		else if (m_driver->GetRawButton(AdvancedJoystick::kButtonB))
			m_arm->canRotate(true);
		else if (m_driver->GetRawButton(AdvancedJoystick::kButtonX))
			m_arm->canRotate(false);
		else if(!m_operator->GetRawButton(AdvancedJoystick::kButtonLB) || m_operator->GetRawButton(AdvancedJoystick::kButtonStart)){
			m_arm->rollerSet(0);
		}


		if (m_driver->GetRawAxis(AdvancedJoystick::kRightTrigger) > 0.2){
			m_arm->intakeSet(-m_driver->GetRawAxis(AdvancedJoystick::kRightTrigger));
		}
		else if (m_driver->GetRawAxis(AdvancedJoystick::kLeftTrigger) > 0.2){
			if (m_driver->GetRawAxis(AdvancedJoystick::kLeftY) > 0.0)
			{
				m_arm->intakeSet(m_driver->GetRawAxis(AdvancedJoystick::kLeftTrigger)*(fabs(m_driver->GetRawAxis(AdvancedJoystick::kLeftY))));
				m_arm->wristSetSetpoint(-0.225);
				m_arm->rollerSet(-0.1);
				m_arm->wEnable();
			}
			else
				m_arm->intakeSet(m_driver->GetRawAxis(AdvancedJoystick::kLeftTrigger));
		}
		else
		{
			m_arm->intakeSet(0);
		}

		if (m_driver->GetRawButton(AdvancedJoystick::kButtonStart)){
			if (m_driver->GetPOV() == 0)
				m_arm->setBurgle(-0.3);
			else if (m_driver->GetPOV() == 180)
				m_arm->setBurgle(1.0);
			else if (m_driver->GetPOV() == 90){
				if (m_burgleTime->Get() == 0){
					m_burgleTime->Stop();
					m_burgleTime->Start();
					m_burgleTime->Reset();
					m_arm->setBurgle(1.0);
				}
				if (m_burgleTime->Get() > 0.4){
					m_arm->setBurgle(0);
				}

			}
			else
				m_arm->setBurgle(0.0);
		}
		else
		{
			m_arm->setBurgle(0.0);
			m_burgleTime->Stop();
			m_burgleTime->Reset();
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

		SmartDashboard::PutNumber("Total Current",m_pdp->GetTotalCurrent());

		SmartDashboard::PutNumber("Auton Case",m_autonCase);
		SmartDashboard::PutNumber("Auton Loop",m_autonLoop);

		SmartDashboard::PutBoolean("Shoulder Reset", f_shoulderReset);
		SmartDashboard::PutBoolean("Wrist Reset", f_wristReset);
		SmartDashboard::PutBoolean("Elevator Reset", f_elevReset);
		SmartDashboard::PutNumber("Burgle Time", m_burgleTime->Get());
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
