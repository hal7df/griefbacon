/*
 * Drivetrain.cpp
 *
 *  Created on: Feb 3, 2015
 *      Author: ROBO6
 */

#include "Drivetrain.h"

Drivetrain::Drivetrain(int lDrive1, int lDrive2, int rDrive1, int rDrive2) :
	HotSubsystem("drivetrain")
{
	// TODO Auto-generated constructor stub
	m_lDrive1 = new Talon (lDrive1);
	m_lDrive2 = new Talon (lDrive2);
	m_rDrive1 = new Talon (rDrive1);
	m_rDrive2 = new Talon (rDrive2);

	m_drive = new RobotDrive (m_lDrive1, m_lDrive2, m_rDrive1, m_rDrive2);
	m_drive->SetSafetyEnabled(false);

}

Drivetrain::~Drivetrain() {
	// TODO Auto-generated destructor stub
}

void Drivetrain::Update() {

}

void Drivetrain::PrintData() {

}
