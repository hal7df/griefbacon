/*
 * FeedbackWrapper.cpp
 *
 *  Created on: Jan 31, 2015
 *      Author: ROBO6
 */

#include "FeedbackWrapper.h"

FeedbackWrapper::FeedbackWrapper(Encoder* lEncode, Encoder* rEncode) {
	// TODO Auto-generated constructor stub
	m_lEncode = lEncode;
	m_rEncode = rEncode;
}

FeedbackWrapper::~FeedbackWrapper() {
	// TODO Auto-generated destructor stub
}

double FeedbackWrapper::PIDGet() {
	return((m_lEncode->GetRate() + (m_rEncode->GetRate())) / 2);
}
