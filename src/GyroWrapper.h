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

class GyroWrapper: public PIDSource {
public:
	GyroWrapper(Gyro* gyro, Timer* gyroTime);
	virtual ~GyroWrapper();

	double PIDGet();
	double GetRatio() { return m_driftRatio; }

	void GyroRatio();

private:
	Gyro* m_gyro;

	Timer* m_driftTime;

	Timer* m_gyroTime;

	double m_driftRatio;

	int m_driftRatioCase;
};

#endif /* GYROWRAPPER_H_ */
