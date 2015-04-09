/*
 * BurgleWrapper.h
 *
 *  Created on: Apr 8, 2015
 *      Author: paul
 */

#ifndef BURGLEWRAPPER_H_
#define BURGLEWRAPPER_H_

#include "WPILib.h"

class BurgleWrapper: public PIDSource, public PIDOutput {
public:
	BurgleWrapper(Victor* canburgleL, Victor* canburgleR, AnalogPotentiometer* potL,  AnalogPotentiometer* potR);
	virtual ~BurgleWrapper();

	double PIDGet() { return ((m_potL->Get() + m_potR->Get())/2); }
	void PIDWrite (float output);

private:
	Victor* m_canburgleL;
	Victor* m_canburgleR;
	AnalogPotentiometer* m_potL;
	AnalogPotentiometer* m_potR;
};

#endif /* BURGLEWRAPPER_H_ */
