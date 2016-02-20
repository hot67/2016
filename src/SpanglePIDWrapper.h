/*
 * SpanglePIDWrapper.h
 *
 *  Created on: Feb 4, 2016
 *      Author: Chad
 */

#ifndef SRC_SpanglePIDWrapper_H_
#define SRC_SpanglePIDWrapper_H_

#include "WPILib.h"
#include "Drivetrain.h"

class Drivetrain;

class SpanglePIDWrapper: public PIDSource, public PIDOutput{
public:
	SpanglePIDWrapper(Drivetrain* drivetrain);
	virtual ~SpanglePIDWrapper();

	double PIDGet();
	void PIDWrite(float output);

private:
	Drivetrain* m_drivetrain;
};

#endif /* SRC_SpanglePIDWrapper_H_ */
