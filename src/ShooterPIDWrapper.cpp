/*
 * ShooterPIDWrapper.cpp
 *
 *  Created on: Feb 13, 2016
 *      Author: Marlina
 */

#include <ShooterPIDWrapper.h>

ShooterPIDWrapper::ShooterPIDWrapper(Intake *intake) {
	m_intake = intake;
}

ShooterPIDWrapper::~ShooterPIDWrapper() {
	// TODO Auto-generated destructor stub
}

void ShooterPIDWrapper::PIDWrite(float output) {
	float speed = m_intake->GetShooterPIDSetPoint() + output;

	speed = (speed > 1.0) ? 1.0 : speed;
	speed = (speed < 0.0) ? 0.0 : speed;

	m_intake->SetShooter(speed);
}

double ShooterPIDWrapper::PIDGet() {
	return m_intake->GetShooterSpeed() / SHOOTER_MAX_SPEED;
}
