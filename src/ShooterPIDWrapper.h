/*
 * ShooterPIDWrapper.h
 *
 *  Created on: Feb 13, 2016
 *      Author: Marlina
 */

#ifndef SRC_SHOOTERPIDWRAPPER_H_
#define SRC_SHOOTERPIDWRAPPER_H_

#include "Intake.h"

class Intake;

class ShooterPIDWrapper : public PIDSource, public PIDOutput {
private:
	Intake *m_intake;
public:
	ShooterPIDWrapper(Intake *intake);
	virtual ~ShooterPIDWrapper();

	void PIDWrite(float output);
	double PIDGet();
};

#endif /* SRC_SHOOTERPIDWRAPPER_H_ */
