/*
 * TurnPIDWrapper.cpp
 *
 *  Created on: Mar 9, 2016
 *      Author: ROBO6
 */

#include <AnglePIDWrapper.h>

AnglePIDWrapper::AnglePIDWrapper(Drivetrain *drivetrain) {
	m_drivetrain = drivetrain;
}

AnglePIDWrapper::~AnglePIDWrapper() {
}

void AnglePIDWrapper::PIDWrite(float output) {
	SmartDashboard::PutNumber("Turn PID Output", output);
	m_drivetrain->SetTurn(output);
}

double AnglePIDWrapper::PIDGet() {
	return m_drivetrain->GetAngle();
}
