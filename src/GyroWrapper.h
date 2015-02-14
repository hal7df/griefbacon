/*
 * GyroWrapper.h
 *
 *  Created on: Feb 12, 2015
 *      Author: ROBO6
 */

#ifndef GYROWRAPPER_H_
#define GYROWRAPPER_H_

#include "WPILib.h"
#include "PIDSource.h"
#include "RobotUtils/HotSubsystem.h"
#include "RobotUtils/RobotUtils.h"

class GyroWrapper: public HotSubsystem, public PIDSource {
public:
	friend class HotSubsystemHandler;
	GyroWrapper(int gyro);
	virtual ~GyroWrapper();

	double PIDGet();
	double GetRatio() { return m_driftRatio; }
	void Reset();

	void GyroRatio();
protected:
	void Update();
	void PrintData();
private:
	Gyro* m_gyro;

	Timer* m_driftTime;
	Timer* m_gyroTime;

	double m_driftRatio;

	int m_driftRatioCase;
};

#endif /* GYROWRAPPER_H_ */
