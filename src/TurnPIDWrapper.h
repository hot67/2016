/*
 * TurnPIDWrapper.h
 *
 *  Created on: Mar 9, 2016
 *      Author: ROBO6
 */

#ifndef SRC_TURNPIDWRAPPER_H_
#define SRC_TURNPIDWRAPPER_H_

#include "WPILib.h"
#include "Drivetrain.h"

class Drivetrain;

class TurnPIDWrapper : public PIDOutput, public PIDSource {
public:
	TurnPIDWrapper(Drivetrain *drivetrain);
	virtual ~TurnPIDWrapper();

	void PIDWrite(float output);
	double PIDGet();

private:
	Drivetrain *m_drivetrain;
};

#endif /* SRC_TURNPIDWRAPPER_H_ */
