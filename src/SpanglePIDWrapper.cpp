/*
 * SpanglePIDWrapper.cpp
 *
 *  Created on: Feb 4, 2016
 *      Author: Chad
 */

#include <SpanglePIDWrapper.h>

SpanglePIDWrapper::SpanglePIDWrapper(Drivetrain* drivetrain) {
	m_drivetrain = drivetrain;
}

SpanglePIDWrapper::~SpanglePIDWrapper() {
	// TODO Auto-generated destructor stub
}


void SpanglePIDWrapper::PIDWrite(float output) {
	output += m_drivetrain->GetTurn();

	if (output > 1.0) {
		output = 1.0;
	} else if (output < -1.0) {
		output = -1.0;
	}

	m_drivetrain->SetTurn(output);
}

double SpanglePIDWrapper::PIDGet () {
	return m_drivetrain->GetLSpeed() - m_drivetrain->GetRSpeed();
}
