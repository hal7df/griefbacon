/*
 * DistancePIDWrapper.cpp
 *
 *  Created on: Jan 20, 2015
 *      Author: ROBO6
 */

#include "DistancePIDWrapper.h"

DistancePIDWrapper::DistancePIDWrapper(Encoder* lEncode, Encoder* rEncode) {
	// TODO Auto-generated constructor stub
	m_lEncode = lEncode;
	m_rEncode = rEncode;

}

DistancePIDWrapper::~DistancePIDWrapper() {
	// TODO Auto-generated destructor stub
}

double DistancePIDWrapper::PIDGet () {
	return((m_lEncode->GetDistance() + m_rEncode->GetDistance()) / 2);
}
