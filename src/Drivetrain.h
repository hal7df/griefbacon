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

#define DISTANCE_P 0.16
#define DISTANCE_I 0.0
#define DISTANCE_D 0.0

#define FEEDBACK_P 0.1
#define FEEDBACK_I 0.0
#define FEEDBACK_D 0.0

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
	Drivetrain(int lDrive1, int lDrive2, int rDrive1, int rDrive2, int lEncode, int rEncode, int gyro);
	virtual ~Drivetrain();

	void ArcadeDrive(double speed, double angle, bool squaredinputs=false) { m_drive->ArcadeDrive(speed, angle, squaredinputs); }

	void PIDWrite (float output);
	void SetPID (bool set) { f_setPID = set; }

	void SetDistance (float distance) { m_distancePID->SetSetpoint(distance); }
	void SetAngle (float angle) {m_turnPID->SetSetpoint(angle); }

	bool DistanceAtSetpoint () { return fabs(GetDistancePID() - m_distancePID->GetSetpoint()) < 0.01; }

	void EnableDistance () {m_distancePID->Enable(); }
	void DisableDistance () {m_distancePID->Disable(); }
	bool IsEnabledDistance () {return (m_distancePID->IsEnabled()); }
	double GetDistancePID () {return (m_distancePIDWrapper->PIDGet()); }

	void EnableAngle () {m_turnPID->Enable(); }
	void DisableAngle () {m_turnPID->Disable(); }
	bool IsEnabledAngle () {return (m_turnPID->IsEnabled()); }
	double GetAnglePID () {return (m_gyro->GetAngle()); }

	void ResetEncoders () {m_lEncode->Reset(); m_rEncode->Reset(); }
	void ResetFlags () {f_setPID = false; f_DisabledDistance = false; }

	void ResetPIDs () {DisableDistance(); DisableAngle(); }

protected:

	void Update();
	void PrintData();

private:

	Talon* m_lDrive1;
	Talon* m_lDrive2;
	Talon* m_rDrive1;
	Talon* m_rDrive2;
	Talon* m_dummy;

	Encoder* m_lEncode;
	Encoder* m_rEncode;

	RobotDrive* m_drive;

	PIDController* m_turnPID;
	PIDController* m_distancePID;

	DistancePIDWrapper* m_distancePIDWrapper;
	AngleOutputWrapper* m_angleOut;

	Timer* m_timer;

	ADXRS453Z* m_gyro;

	int m_etaFlag;
	int m_StraightDistanceCase;

	float m_distancePIDSet;
	float m_anglePIDSet;

	bool f_setPID;
	bool f_DisabledDistance;
};

#endif /* DRIVETRAIN_H_ */
