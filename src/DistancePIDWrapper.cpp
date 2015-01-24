/*
 * DistancePIDWrapper.cpp
 *
 *  Created on: Jan 20, 2015
 *      Author: ROBO6
 */

#include "DistancePIDWrapper.h"

DistancePIDWrapper::DistancePIDWrapper(Encoder* encodeL, Encoder* encodeR) {
	// TODO Auto-generated constructor stub
	m_encodeL = encodeL;
	m_encodeR = encodeR;

}

DistancePIDWrapper::~DistancePIDWrapper() {
	// TODO Auto-generated destructor stub
}

double DistancePIDWrapper::PIDGet () {
	return((m_encodeL->GetDistance() - m_encodeR->GetDistance()) / 2);
}
