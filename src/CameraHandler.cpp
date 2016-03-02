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
