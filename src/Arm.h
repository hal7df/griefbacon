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

//#define PRACTICE_BOT
#define COMPETITION_BOT

#ifdef PRACTICE_BOT
#define ARM_ENCODER_REVERSE false
#endif

#ifdef COMPETITION_BOT
#define ARM_ENCODER_REVERSE true
#endif

#define WRIST_P -16.0
#define WRIST_I 0.015 //0.01
#define WRIST_D 0.0

#define SHOULDER_P -8.00
#define SHOULDER_I -0.01
#define SHOULDER_D 0.0


#define SHOULDER_GROUND -0.892 //-0.931
#define SHOULDER_TWOTOTE -0.831
#define SHOULDER_DRIVING -0.072 //-0.109
#define SHOULDER_CANSTACK -0.120
#define SHOULDER_FIVECAN -0.167
#define SHOULDER_PACKAGE 0.0
#define SHOULDER_CANKNOCK -0.777
#define SHOULDER_AUTOPLACE -0.046 //-0.042 //-0.046 //-0.214
#define SHOULDER_GROUND_M -0.546
#define SHOULDER_FOURCAN -0.183

#define WRIST_GROUND -0.239 //-0.206
#define WRIST_TWOTOTE -0.293
#define WRIST_DRIVING -0.971 //-0.488
#define WRIST_AUTON -0.488
#define WRIST_CANSTACK -0.045
#define WRIST_FIVECAN -0.192
#define WRIST_PACKAGE 0.0
#define WRIST_CANKNOCK -0.079
#define WRIST_AUTOPLACE -0.191 //-0.183 //-0.017
#define WRIST_GROUND_M -0.973
#define WRIST_FOURCAN -0.109

enum sPos_t {
	ksGround,
	ksTwoTote,
	ksDriving,
	ksCanStack,
	ksPackage,
	ksFiveCan,
	ksCanKnock,
	ksAutoPlace,
	ksGroundM,
	ksFourCanPlace
};

enum wPos_t {
	kwGround,
	kwTwoTote,
	kwDriving,
	kwCanStack,
	kwPackage,
	kwFiveCan,
	kwCanKnock,
	kwAutoPlace,
	kwGroundM,
	kwAuton,
	kwFourCanPlace
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
	void canRotate(bool speed);
	void intakeSet(double speed);

	void clearCans (bool on);

	void setBurgle (float speed) { m_lCanBurgle->Set(-speed); m_rCanBurgle->Set(speed); }

	bool WristAtSetpoint ();
	bool ShoulderAtSetpoint ();

	void PIDWrite(float input);
	void GetPID (bool get) { f_getPID = get; }
	void sEnable ();
	void sDisable ();
	void wEnable ();
	void wDisable ();

	void sReset () {  m_shoulderEncode->Reset(); }
	void wReset () {  m_wristEncode->Reset(); }

	bool sIsEnabled(){return m_shoulderPid -> IsEnabled();}
	bool wIsEnabled(){return m_wristPid -> IsEnabled();}

	void shoulderSetSetpoint(int point);
	void wristSetSetpoint(float point);
	double GetWristRate(){ return m_wristEncode -> GetRate(); }
	double GetShoulderRate(){ return m_shoulderEncode -> GetRate(); }

	bool GetEStop() { return f_wEStop || f_sEStop; }
	bool GetWEstop(){ return f_wEStop; }
	bool GetSEstop(){ return f_sEStop; }

	void ResetEStop();

	Encoder* GetShoulder() { return m_shoulderEncode; }
	Encoder* GetWrist() { return m_wristEncode; }

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

	Victor* m_lCanBurgle;
	Victor* m_rCanBurgle;

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
	bool f_shoulderStop;

	Timer* m_wStopTime;
	Timer* m_sStopTime;
	sem_t m_semaphore;
};

#endif /* SRC_ARM_H_ */
