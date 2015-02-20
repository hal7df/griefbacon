/*
 * Arm.h
 *
 *  Created on: Feb 7, 2015
 *      Author: Robo10
 */

#ifndef SRC_ARM_H_
#define SRC_ARM_H_

#include "RobotUtils/HotSubsystem.h"
#include "WPILib.h"
#include <semaphore.h>

#define WRIST_P -16.0
#define WRIST_I 0.01
#define WRIST_D 0.0

#define SHOULDER_P -8.0
#define SHOULDER_I 0.01
#define SHOULDER_D 0.0

#define SHOULDER_GROUND -0.919
#define SHOULDER_TWOTOTE -0.831
#define SHOULDER_DRIVING -0.109
#define SHOULDER_CANSTACK -0.2
#define SHOULDER_PACKAGE 0.0

#define WRIST_GROUND -0.216
#define WRIST_TWOTOTE -0.293
#define WRIST_DRIVING -0.488
#define WRIST_CANSTACK 0.0
#define WRIST_PACKAGE 0.0

enum sPos_t {
	ksGround,
	ksTwoTote,
	ksDriving,
	ksCanStack,
	ksPackage
};

enum wPos_t {
	kwGround,
	kwTwoTote,
	kwDriving,
	kwCanStack,
	kwPackage
};

class Arm: public HotSubsystem, public PIDOutput {
public:
	friend class HotSubsystemHandler;
	Arm(int pickSL, int pickSR, int pickW, int pickRL, int pickRR, int intakeL, int intakeR);
	virtual ~Arm();

	void shoulderSet(double speed);
	void wristSet(double speed);
	void shoulderSetPos (sPos_t position);
	void wristSetPos (wPos_t position);
	void rollerSet(double speed);
	void intakeSet(double speed);

	void clearCans (bool on);

	bool WristAtSetpoint ();
	bool ShoulderAtSetpoint ();

	void PIDWrite(float input);
	void GetPID (bool get) { f_getPID = get; }
	void sEnable ();
	void sDisable () { if (sIsEnabled()) m_shoulderPid->Disable(); }
	void wEnable ();
	void wDisable () { if (wIsEnabled()) m_wristPid->Disable(); }

	void sReset () {  m_shoulderEncode->Reset(); }
	void wReset () {  m_wristEncode->Reset(); }

	bool sIsEnabled(){return m_shoulderPid -> IsEnabled();}
	bool wIsEnabled(){return m_wristPid -> IsEnabled();}

	void shoulderSetSetpoint(int point);
	void wristSetSetpoint(int point);
	double GetWristRate(){ return m_wristEncode -> GetRate(); }
	double GetShoulderRate(){ return m_shoulderEncode -> GetRate(); }

	bool GetEStop() { return f_wEStop || f_sEStop; }
	bool GetWEstop(){ return f_wEStop; }
	bool GetSEstop(){ return f_sEStop; }

	void ResetEStop();

protected:
	void Update();
	void PrintData();

private:
	void EStopCheck();

	CANTalon* m_pickSL;
	CANTalon* m_pickSR;
	CANTalon* m_pickW;
	CANTalon* m_pickRL;
	CANTalon* m_pickRR;
	CANTalon* m_intakeL;
	CANTalon* m_intakeR;

	Encoder* m_shoulderEncode;
	Encoder* m_wristEncode;

	PIDController* m_shoulderPid;
	PIDController* m_wristPid;
	bool f_getPID;

	bool f_eStopRunning;
	bool f_wSetpointChanged;
	bool f_sSetpointChanged;
	bool f_wEStop;
	bool f_sEStop;
	Timer* m_wStopTime;
	Timer* m_sStopTime;
	sem_t m_semaphore;
};

#endif /* SRC_ARM_H_ */
