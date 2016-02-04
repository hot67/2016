/*
 * TurnPIDWrapper.h
 *
 *  Created on: Feb 4, 2016
 *      Author: Chad
 */

#ifndef SRC_TURNPIDWRAPPER_H_
#define SRC_TURNPIDWRAPPER_H_

#include "WPILib.h"

class TurnPIDWrapper: public PIDOutput {
public:
	TurnPIDWrapper(RobotDrive* drive);
	virtual ~TurnPIDWrapper();

	void PIDWrite (float output);

private:
	RobotDrive* m_drive;
};

#endif /* SRC_TURNPIDWRAPPER_H_ */
