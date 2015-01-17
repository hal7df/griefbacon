/*
 * EuroWrapper.h
 *
 *  Created on: Jan 17, 2015
 *      Author: ROBO6
 */

#ifndef EUROWRAPPER_H_
#define EUROWRAPPER_H_

#include <PIDOutput.h>

class EuroWrapper: public PIDOutput {
public:
	EuroWrapper();
	virtual ~EuroWrapper();
};

#endif /* EUROWRAPPER_H_ */
