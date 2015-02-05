/*
 * FeedbackWrapper.cpp
 *
 *  Created on: Jan 31, 2015
 *      Author: ROBO6
 */

#include "FeedbackWrapper.h"

FeedbackWrapper::FeedbackWrapper(Encoder* encodeL, Encoder* encodeR) {
	// TODO Auto-generated constructor stub
	m_encodeL = encodeL;
	m_encodeR = encodeR;
}

FeedbackWrapper::~FeedbackWrapper() {
	// TODO Auto-generated destructor stub
}

double FeedbackWrapper::PIDGet() {
	return((m_encodeL->GetRate() - m_encodeR->GetRate()) / 2);
}
