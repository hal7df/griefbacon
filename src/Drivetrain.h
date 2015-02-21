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

#define FEEDBACK_P 0.1
#define FEEDBACK_I 0.0
#define FEEDBACK_D 0.0

#define DRIVE_MAX_THROTTLE 0.2

#include "RobotUtils/HotSubsystem.h"
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

	void SetDistance (float distance);

	void SetAngleHeading (float angle) {m_angleHeading = angle; }
	float GetAngleHeading () {return (m_angleHeading); }
	bool AtAngleHeading () { return fabs(m_gyro->GetAngle() - m_angleHeading) < 0.1; }

	bool DistanceAtSetpoint () { return fabs(GetDistancePID() - m_distancePID->GetSetpoint()) < 0.2; }

	void EnableDistance () {m_distancePID->Enable(); }
	void DisableDistance () {m_distancePID->Disable(); }
	bool IsEnabledDistance () {return (m_distancePID->IsEnabled()); }
	double GetDistancePID () {return (m_distancePIDWrapper->PIDGet()); }

	void EnableAngle () {m_turnPID->Enable(); }
	void DisableAngle () {m_turnPID->Disable(); }
	bool IsEnabledAngle () {return (m_turnPID->IsEnabled()); }
	double GetAnglePID () {return (m_gyro->GetAngle()); }

	void ResetEncoders () {m_lEncode->Reset(); m_rEncode->Reset(); }
	void ResetFlags () {f_setPID = false; }
	void ResetPIDs () {DisableDistance(); DisableAngle(); }
	void ResetGyroAngle() {m_gyro->Reset(); }
	void ResetRamp () { m_lastMultiplier = 0.0; }

	void SetLimit (float lim) { m_speedLimit = lim; }
	float GetLimit () { return m_speedLimit; }
	void SetCorrLimit (float lim) { m_correctLimit = lim; }
	float GetCorrLimit () { return m_correctLimit; }
	void SetMaxThrottleDelta (float delta) { m_maxThrottle = delta; }
	float GetMaxThrottleDelta () { return m_maxThrottle; }

protected:

	void Update();
	void PrintData();

private:

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

	ADXRS453Z* m_gyro;

	float m_speedLimit;
	float m_correctLimit;
	float m_angleHeading;

	float m_lastMultiplier;
	float m_maxThrottle;

	bool f_setPID;
};

#endif /* DRIVETRAIN_H_ */
