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
	// TODO Auto-generated destructor stub
}


void DistancePIDWrapper::PIDWrite(float output) {
	m_drivetrain->SetSpeed(output);
}


double DistancePIDWrapper::PIDGet () {
	return(m_drivetrain->GetAverageDistance());
}
