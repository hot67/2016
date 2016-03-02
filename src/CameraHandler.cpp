/*
 * CameraHandler.cpp
 *
 *  Created on: Mar 1, 2016
 *      Author: ROBO6
 */

#include <CameraHandler.h>

CameraHandler::CameraHandler() {
}

CameraHandler::~CameraHandler() {
}

double CameraHandler::GetTargetNormalizedCenter() {
	int targetInd = 0;
	float closestDist = SmartDashboard::GetNumber("distanceToTarget0", 0.0);

	if (SmartDashboard::GetNumber("distanceToTarget1", 0.0) != 0.0 && SmartDashboard::GetNumber("distanceToTarget1", 0.0) < closestDist) {
		targetInd = 1;
		closestDist = SmartDashboard::GetNumber("distanceToTarget1", 0.0);
	}

	if (SmartDashboard::GetNumber("distanceToTarget2", 0.0) != 0.0 && SmartDashboard::GetNumber("distanceToTarget2", 0.0) < closestDist) {
		targetInd = 2;
		closestDist = SmartDashboard::GetNumber("distanceToTarget2", 0.0);
	}

	return SmartDashboard::GetNumber("ImageXCenter0", 0.0);
}

bool CameraHandler::AtTarget() {
	return fabs(GetTargetNormalizedCenter()) < 0.1;
}

bool CameraHandler::SeeTargetLeft() {
	return GetTargetNormalizedCenter() <= -0.1;
}

bool CameraHandler::SeeTargetRight() {
	return GetTargetNormalizedCenter() >= 0.1;
}

bool CameraHandler::SeeTarget() {
	return GetTargetNormalizedCenter() != 0.0;
}
