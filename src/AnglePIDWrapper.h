/*
 * TurnPIDWrapper.h
 *
 *  Created on: Mar 9, 2016
 *      Author: ROBO6
 */

#ifndef SRC_ANGLEPIDWRAPPER_H_
#define SRC_ANGLEPIDWRAPPER_H_

#include "WPILib.h"
#include "Drivetrain.h"

class Drivetrain;

class AnglePIDWrapper : public PIDOutput, public PIDSource {
public:
	AnglePIDWrapper(Drivetrain *drivetrain);
	virtual ~AnglePIDWrapper();

	void PIDWrite(float output);
	double PIDGet();

private:
	Drivetrain *m_drivetrain;
};

#endif /* SRC_ANGLEPIDWRAPPER_H_ */
