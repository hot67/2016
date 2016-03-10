/*
 * CameraHandler.h
 *
 *  Created on: Mar 1, 2016
 *      Author: ROBO6
 */

#ifndef SRC_CAMERAHANDLER_H_
#define SRC_CAMERAHANDLER_H_

#include <math.h>
#include "WPILib.h"

class CameraHandler {
public:
	CameraHandler();
	virtual ~CameraHandler();

	double GetTargetNormalizedCenter();

	bool AtTarget();
	bool SeeTargetRight();
	bool SeeTargetLeft();
	bool SeeTarget();
	double GetX();
};

#endif /* SRC_CAMERAHANDLER_H_ */
