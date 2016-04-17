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

	double dist0 = SmartDashboard::GetNumber("distanceToTarget0", 0.0);
	double dist1 = SmartDashboard::GetNumber("distanceToTarget1", 0.0);
	double dist2 = SmartDashboard::GetNumber("distanceToTarget2", 0.0);

	double x0 = SmartDashboard::GetNumber("ImageXCenter0", 0.0);
	double x1 = SmartDashboard::GetNumber("ImageXCenter1", 0.0);
	double x2 = SmartDashboard::GetNumber("ImageXCenter2", 0.0);

	double dist = dist0;
	double x = x0;

	if (dist1 != 0.0 && dist1 < dist) {
		dist = dist1;
		x = x1;
	}

	if (dist2 != 0.0 && dist2 < dist) {
		dist = dist2;
		x = x2;
	}

	return x;
}

bool CameraHandler::AtTarget() {
	return SeeTarget() && fabs(GetX()) < 2.5;
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

double CameraHandler::GetX() {
	return GetTargetNormalizedCenter() * 33.5 ;// * 33.5 + 10;//(0.2 * 33.5);
}
