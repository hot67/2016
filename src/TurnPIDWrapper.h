/*
 * TurnPIDWrapper.h
 *
 *  Created on: Feb 4, 2016
 *      Author: Chad
 */

#ifndef SRC_TURNPIDWRAPPER_H_
#define SRC_TURNPIDWRAPPER_H_

#include "WPILib.h"
#include "Drivetrain.h"

class Drivetrain;

class TurnPIDWrapper: public PIDOutput, public PIDSource {
public:
	TurnPIDWrapper(Drivetrain* drivetrain);
	virtual ~TurnPIDWrapper();

	void PIDWrite (float output);
	double PIDGet();

private:
	Drivetrain* m_drivetrain;
};

#endif /* SRC_TURNPIDWRAPPER_H_ */
