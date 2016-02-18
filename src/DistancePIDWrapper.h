/*
 * DistancePIDWrapper.h
 *
 *  Created on: Feb 4, 2016
 *      Author: Chad
 */

#ifndef SRC_DISTANCEPIDWRAPPER_H_
#define SRC_DISTANCEPIDWRAPPER_H_

#include "WPILib.h"
#include "Drivetrain.h"

class Drivetrain;

class DistancePIDWrapper: public PIDSource, public PIDOutput{
public:
	DistancePIDWrapper(Drivetrain* drivetrain);
	virtual ~DistancePIDWrapper();

	double PIDGet();
	void PIDWrite(float output);

private:
	Drivetrain* m_drivetrain;
};

#endif /* SRC_DISTANCEPIDWRAPPER_H_ */
