/*
 * Drivetrain.h
 *
 *  Created on: Feb 3, 2015
 *      Author: ROBO6
 */

#ifndef DRIVETRAIN_H_
#define DRIVETRAIN_H_

#include <RobotUtils/HotSubsystem.h>
#include "WPILib.h"

#define DISTANCE_P 0.1
#define DISTANCE_I 0.0
#define DISTANCE_D 0.0

class Drivetrain: public HotSubsystem, public PIDSource, public PIDOutput {
public:

	friend class HotSubsystemHandler;
	Drivetrain(int lDrive1, int lDrive2, int rDrive1, int rDrive2, int lEncode, int rEncode);
	virtual ~Drivetrain();

	void ArcadeDrive(double speed, double angle, bool squaredinputs=false) { m_drive->ArcadeDrive(speed, angle, squaredinputs); }

	void PIDWrite(float input);
	double PIDGet();


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
	PIDController* m_dStraight;
};

#endif /* DRIVETRAIN_H_ */
