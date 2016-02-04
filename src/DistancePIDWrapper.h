/*
 * DistancePIDWrapper.h
 *
 *  Created on: Feb 4, 2016
 *      Author: Chad
 */

#ifndef SRC_DISTANCEPIDWRAPPER_H_
#define SRC_DISTANCEPIDWRAPPER_H_

#include "WPILib.h"

class DistancePIDWrapper: public PIDSource, public PIDOutput{
public:
	DistancePIDWrapper(Encoder* lEncode, Encoder* rEncode);
	virtual ~DistancePIDWrapper();

	double PIDGet();
	void PIDWrite(float output);

private:
	Encoder* m_lEncode;
	Encoder* m_rEncode;
};

#endif /* SRC_DISTANCEPIDWRAPPER_H_ */
