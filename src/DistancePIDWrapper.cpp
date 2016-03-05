/*
 * DistancePIDWrapper.cpp
 *
 *  Created on: Feb 4, 2016
 *      Author: Chad
 */

#include <DistancePIDWrapper.h>

DistancePIDWrapper::DistancePIDWrapper(Drivetrain* drivetrain) {
	m_drivetrain = drivetrain;
}

DistancePIDWrapper::~DistancePIDWrapper() {
}


void DistancePIDWrapper::PIDWrite(float output) {
	SmartDashboard::PutNumber("* Drive PID Write", output * 0.8);
	m_drivetrain->SetSpeed(output * 0.8);
}


double DistancePIDWrapper::PIDGet () {
	return(m_drivetrain->GetAverageDistance());
}
