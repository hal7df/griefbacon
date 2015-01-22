/*
 * AutonWrapper.h
 *
 *  Created on: Jan 20, 2015
 *      Author: ROBO6
 */

#ifndef AUTONWRAPPER_H_
#define AUTONWRAPPER_H_

#define AUTO_P 0.0
#define AUTO_I 0.0
#define AUTO_D 0.0
#define ENCODE_CONVERT 0.0

#include "WPILib.h"

class AutonWrapper: public PIDOutput, public PIDSource {
public:
	AutonWrapper(RobotDrive* drive, Encoder* encodeL, Encoder* encodeR);
	virtual ~AutonWrapper();

	//PIDOutput and PIDSource functions
	double PIDGet();
	void PIDWrite(float output);

	//PIDController passthrough functions
	void SetSetpoint (float setpoint) { m_PID->SetSetpoint(setpoint); }
	float GetSetpoint () {return m_PID->GetSetpoint();}

	void Enable () {m_PID->Enable(); }
	void Disable () {m_PID->Disable(); }
	bool IsEnabled () {return m_PID->IsEnabled(); }

	float GetPIDOutput () {return m_PID->Get();}

	void SetPID (float p, float i, float d) { m_PID->SetPID(p,i,d); }
	float GetP () {return m_PID->GetP();}
	float GetI () {return m_PID->GetI();}
	float GetD () {return m_PID->GetD();}

private:
	PIDController* m_PID;
	RobotDrive* m_drive;
	Encoder* m_encodeL;
	Encoder* m_encodeR;
};

#endif /* AUTONWRAPPER_H_ */
