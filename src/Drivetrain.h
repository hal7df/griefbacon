/*
 * Drivetrain.h
 *
 *  Created on: Feb 3, 2015
 *      Author: ROBO6
 */

#ifndef DRIVETRAIN_H_
#define DRIVETRAIN_H_

#define RATIO_1 26.11001704;
#define RATIO_2 2.986676441;

#include <RobotUtils/HotSubsystem.h>
#include "WPILib.h"
#include <cmath>

class Drivetrain: public HotSubsystem {
public:

	friend class HotSubsystemHandler;
	Drivetrain(int lDrive1, int lDrive2, int rDrive1, int rDrive2, int lEncode1, int lEncode2, int rEncode1, int rEncode2);
	virtual ~Drivetrain();

	void ArcadeDrive(double speed, double angle, bool squaredinputs=false) { m_drive->ArcadeDrive(speed, angle, squaredinputs); }
	void ETA();

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
	int m_etaFlag;
};

#endif /* DRIVETRAIN_H_ */
