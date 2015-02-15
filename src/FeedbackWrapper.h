/*
 * FeedbackWrapper.h
 *
 *  Created on: Jan 31, 2015
 *      Author: ROBO6
 */

#ifndef FEEDBACKWRAPPER_H_
#define FEEDBACKWRAPPER_H_

#define FEED_P 0.1
#define FEED_I 0.0
#define FEED_D 0.0

#include "WPILib.h"

class FeedbackWrapper: public PIDSource {
public:
	FeedbackWrapper(Encoder* lEncode, Encoder* rEncode);
	virtual ~FeedbackWrapper();

	double PIDGet();



private:
	Encoder* m_lEncode;
	Encoder* m_rEncode;
};

#endif /* FEEDBACKWRAPPER_H_ */
