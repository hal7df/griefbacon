/*
 * DistancePIDWrapper.h
 *
 *  Created on: Jan 20, 2015
 *      Author: ROBO6
 */

#ifndef DistancePIDWrapper_H_
#define DistancePIDWrapper_H_

#define AUTO_P 0.1
#define AUTO_I 0.0
#define AUTO_D 0.0
#define ENCODE_CONVERT 0.0

#include "WPILib.h"

class DistancePIDWrapper: public PIDSource {
public:
	DistancePIDWrapper(Encoder* lEncode, Encoder* rEncode);
	virtual ~DistancePIDWrapper();

	//PIDOutput and PIDSource functions
	double PIDGet();

private:
	Encoder* m_lEncode;
	Encoder* m_rEncode;
};

#endif /* DistancePIDWrapper_H_ */
