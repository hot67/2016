/*
 * TurnPIDWrapper.cpp
 *
 *  Created on: Mar 9, 2016
 *      Author: ROBO6
 */

#include <TurnPIDWrapper.h>

TurnPIDWrapper::TurnPIDWrapper(Drivetrain *drivetrain) {
	m_drivetrain = drivetrain;
}

TurnPIDWrapper::~TurnPIDWrapper() {
	// TODO Auto-generated destructor stub
}

void TurnPIDWrapper::PIDWrite(float output) {
	SmartDashboard::PutNumber("Turn PID Output", output);
	m_drivetrain->ArcadeDrive(0.0, output * 0.7);
}

double TurnPIDWrapper::PIDGet() {
	return m_drivetrain->GetAngle();
}
