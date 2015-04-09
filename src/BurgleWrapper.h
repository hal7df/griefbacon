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
	BurgleWrapper(Talon* canburgleL, Talon* canburgleR, AnalogPotentiometer* potL,  AnalogPotentiometer* potR);
	virtual ~BurgleWrapper();

	double PIDGet() { return ((m_potL->Get() + m_potR->Get())/2); }
	void PIDWrite (float output);

private:
	Talon* m_canburgleL;
	Talon* m_canburgleR;
	AnalogPotentiometer* m_potL;
	AnalogPotentiometer* m_potR;
};

#endif /* BURGLEWRAPPER_H_ */
