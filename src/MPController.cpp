/*
 * MPController.cpp
 *
 *  Created on: Apr 18, 2016
 *      Author: Jin
 */

#include <MPController.h>

MPController::MPController(double Kp, double Ki, double Kd, double Vmax, double Amax, double Amin,
		PIDSource *source, PIDOutput *output) : PIDController(Kp, Ki, Kd, source, output) {
	m_Vmax = Vmax;
	m_Amax = Amax;
	m_Amin = Amin;
	m_timer = new Timer();
}

void MPController::SetSetpoint(double target) {
	if (!IsEnabled()) {
		m_path = new MPPath(m_Vmax, m_Amax, m_Amin, target);
	}
}

void MPController::Enable() {
	if (!IsEnabled() && m_path != NULL) {
		m_timer->Stop();
		m_timer->Reset();
		m_timer->Start();
		PIDController::Enable();
	}
}

void MPController::Disable() {
	if (IsEnabled()) {
		PIDController::Disable();
	}
	delete m_path;
}

void MPController::Update() {
	if (m_path != NULL) {
		PIDController::SetSetpoint(m_path->GetP(m_timer->Get()));
	}
}
