/*
 * Drivetrain.h
 *
 *  Created on: Feb 3, 2015
 *      Author: ROBO6
 */

#ifndef DRIVETRAIN_H_
#define DRIVETRAIN_H_

#define RATIO_1 26.11001704
#define RATIO_2 2.986676441

#define GYRO_P 0.015
#define GYRO_I 0.0
#define GYRO_D 0.01

#define DISTANCE_P -2.5
#define DISTANCE_I 0.0
#define DISTANCE_D -0.1

#define TURN_P -0.1
#define TURN_I 0.0
#define TURN_D -0.1

#define FEEDBACK_P 0.1
#define FEEDBACK_I 0.0
#define FEEDBACK_D 0.0

#define NAVX_ENABLED

#include "RobotUtils/HotSubsystem.h"
#include "navx/AHRS.h"
#include "WPILib.h"
#include "FeedbackWrapper.h"
#include "DistancePIDWrapper.h"
#include "AngleOutputWrapper.h"
#include "ADXRS453Z.h"
#include <cmath>

class Drivetrain: public HotSubsystem, public PIDOutput {
public:

	friend class HotSubsystemHandler;
	Drivetrain(int lDrive1, int lDrive2, int rDrive1, int rDrive2, int lEncode, int rEncode);
	virtual ~Drivetrain();

	void ArcadeDrive(double speed, double angle, bool squaredinputs=false) { m_drive->ArcadeDrive(speed, angle, squaredinputs); }

	void PIDWrite (float output);
	void SetPID (bool set) { f_setPID = set; }

	void SetDistance (float distance) { m_distancePID->SetSetpoint(distance); }

	void SetTurnPIDHeading (float angle);
	float GetTurnPIDHeading () {return m_turnPID->GetSetpoint();}
	bool TurnPIDatSetpoint () { return fabs(GetGyroAngle() - m_turnPID->GetSetpoint()) < 7; }


	void SetAngleHeading (float angle) { m_angleHeading = (angle - m_gyroOffset) ; }
	float GetAngleHeading () {return (m_angleHeading); }
	bool AtAngleHeading () { return fabs(GetGyroAngle() - m_angleHeading) < 1.; }

#ifdef NAVX_ENABLED
	void ResetGyroAngle() { m_gyro->ZeroYaw(); m_gyroOffset = 0.0; }
#else
	void ResetGyroAngle() {m_gyro->Reset(); }
#endif

	bool DistanceAtSetpoint () { return fabs(GetDistancePID() - m_distancePID->GetSetpoint()) < 0.2; }

	void EnableDistance () {m_distancePID->Enable(); }
	void DisableDistance () {m_distancePID->Disable(); }
	bool IsEnabledDistance () {return (m_distancePID->IsEnabled()); }
	double GetDistancePID () {return (m_distancePIDWrapper->PIDGet()); }

	void EnableAngle () {m_turnPID->Enable(); }
	void DisableAngle () {m_turnPID->Disable(); }
	bool IsEnabledAngle () { return (m_turnPID->IsEnabled()); }
#ifdef NAVX_ENABLED
	double GetGyroAngle () { return m_gyro->GetYaw(); }
	double GetGyroRoll () { return m_gyro->GetPitch(); }
#else
	double GetGyroAngle () { return (m_gyro->GetAngle()); }
#endif

	void SetGyroOffset() { m_gyroOffset = GetGyroAngle(); }
	float GetGyroOffset () { return m_gyroOffset; }

	void ResetEncoders () {m_lEncode->Reset(); m_rEncode->Reset(); }
	void ResetFlags () {f_setPID = false; f_DisabledDistance = false; }

	void ResetPIDs () {DisableDistance(); DisableAngle(); }

	void SetLimit (float lim) { m_speedLimit = lim; }
	float GetLimit () { return m_speedLimit; }
	void SetCorrLimit (float lim) { m_correctLimit = lim; }
	float GetCorrLimit () { return m_correctLimit; }

	bool GetTipping() { return f_tipping; }
	void ResetTipFlag() { f_tipping = false; }

#ifdef NAVX_ENABLED
	AHRS* GetGyro() { return m_gyro; }
#else
	ADXRS453Z* GetGyro() { return m_gyro; }
#endif

	double GetAverageRate () { return (m_lEncode->GetRate() + m_rEncode->GetRate())/2; }

	DistancePIDWrapper* GetEncoders() { return m_distancePIDWrapper; }
	Encoder* GetLEncode() { return m_lEncode; }
	Encoder* GetREncode() { return m_rEncode; }

protected:

	void Update();
	void PrintData();

private:
	void GyroCal();
	void TipCheck();

	Talon* m_lDrive1;
	Talon* m_lDrive2;
	Talon* m_rDrive1;
	Talon* m_rDrive2;

	Encoder* m_lEncode;
	Encoder* m_rEncode;

	RobotDrive* m_drive;

	PIDController* m_turnPID;
	PIDController* m_distancePID;

	DistancePIDWrapper* m_distancePIDWrapper;
	AngleOutputWrapper* m_angleOut;

	Timer* m_timer;

#ifdef NAVX_ENABLED
	AHRS* m_gyro;
	bool m_firstGyroIt;
#else
	ADXRS453Z* m_gyro;
#endif
	float m_speedLimit;
	float m_correctLimit;
	float m_angleHeading;
	float m_gyroOffset;

	bool f_setPID;
	bool f_DisabledDistance;
	bool f_tipping;
};

#endif /* DRIVETRAIN_H_ */
