/*
 * BurgleWrapper.cpp
 *
 *  Created on: Apr 8, 2015
 *      Author: paul
 */

#include "BurgleWrapper.h"

BurgleWrapper::BurgleWrapper(Talon* canburgleL, Talon* canburgleR, AnalogPotentiometer* potL,  AnalogPotentiometer* potR) {
	m_canburgleL = canburgleL;
	m_canburgleR = canburgleR;
	m_potL = potL;
	m_potR = potR;
}

BurgleWrapper::~BurgleWrapper() {
	// TODO Auto-generated destructor stub
}

void BurgleWrapper::PIDWrite (float output) {
	m_canburgleL->Set(output);
	m_canburgleR->Set(output);
}
