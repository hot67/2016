/*
 * Intake.cpp
 *
 *  Created on: Jan 31, 2016
 *      Author: Marlina
 */

#include <Intake.h>


Intake::Intake(HotBot* bot) : HotSubsystem(bot, "Intake") {
	/*
	 * 	Talons
	 */
	m_rollerTalon = new CANTalon(ROLLER_ID);
	m_shooterTalon = new CANTalon(SHOOTER_ID);

	/*
	 * 	Buffers
	 */
	m_rollerBuf = new DoubleBuffer(BufferPriority::kAbsMax);
	m_shooterBuf = new DoubleBuffer(BufferPriority::kAbsMax);
	m_shooterPIDBuf = new BooleanBuffer(false);

	/*
	 * 	Sensors
	 */
	DigitalInput *m_shooterLight = new DigitalInput(6);
	m_shooterEncoder = new Encoder(m_shooterLight, m_shooterLight, true);
	m_shooterEncoder->SetDistancePerPulse(1);

	/*
	 * 	PID
	 */
	ShooterPIDWrapper *wrapper = new ShooterPIDWrapper(this);
	m_shooterPID = new PIDController(SHOOTER_SPEED_P, SHOOTER_SPEED_I, SHOOTER_SPEED_D,
			wrapper, wrapper);
	m_shooterPID->SetPercentTolerance(0.05);
}

/******************************
 * MOTORS
 ******************************/
//	Send signal to motor
//		All signal to motor mast go through this function
void Intake::_SetRoller(float speed) {
	m_rollerTalon->Set(speed);
}
void Intake::_SetShooter(float speed) {
	m_shooterTalon->Set(speed);
}

//	Request signal to motor
void Intake::SetRoller(float speed, int priority) {
	m_rollerBuf->Write(speed, priority);
}
void Intake::SetShooter(float speed, int priority) {
	m_shooterBuf->Write(speed, priority);
}

//	Update
void Intake::Update() {
	if (!m_rollerBuf->IsLocked()) {
		_SetRoller(m_rollerBuf->Read());
	}

	if (!m_shooterBuf->IsLocked()) {
		_SetShooter(m_shooterBuf->Read());
	}

	if (!m_shooterPIDBuf->IsLocked()) {
		if (m_shooterPIDBuf->Read()) {
			_EnableShooterPID();
		} else {
			_DisableShooterPID();
		}
	}
}

/******************************
 * SENSORS
 ******************************/
/*
 * encoder picks each reflective thing however many times per rotation (defined as SHOOTER_PULSE_PER_ROTATION) and is then divided by shooter pulse per rotation
 */
double Intake::GetShooterSpeed(){
	return m_shooterEncoder->GetRate() * 60;
}

/******************************
 * SHOOTER	PID
 ******************************/
void Intake::_EnableShooterPID() {
	if (!IsShooterPIDEnabled()) {
		m_shooterPID->Enable();
		m_shooterPIDBuf->Lock();
	}
}
void Intake::_DisableShooterPID() {
	if (IsShooterPIDEnabled()) {
		m_shooterPID->Disable();
		m_shooterPIDBuf->Unlock();
	}
}
void Intake::EnableShooterPID(int priority){
	m_shooterPIDBuf->Write(true, priority);
}

void Intake::DisableShooterPID(int priority){
	m_shooterPIDBuf->Write(false, priority);
}

bool Intake::IsShooterPIDEnabled(){
	return m_shooterPID->IsEnabled();
}

void Intake::SetShooterPIDSetPoint(float speed){
	m_shooterPID->SetSetpoint(speed);
}

double Intake::GetShooterPIDSetPoint(){
	return m_shooterPID->GetSetpoint();
}

bool Intake::ShooterAtSetPoint(){
	return m_shooterPID->OnTarget();
}

/******************************
 * 	Shooter PID Wrapper
 ******************************/
Intake::ShooterPIDWrapper::ShooterPIDWrapper(Intake *intake) {
	m_intake = intake;
}
void Intake::ShooterPIDWrapper::PIDWrite(float output) {
	m_speed += output;

	m_speed = (m_speed > 1.0) ? 1.0 : m_speed;
	m_speed = (m_speed < -1.0) ? -1.0 : m_speed;

	m_intake->_SetShooter(m_speed);
}
double Intake::ShooterPIDWrapper::PIDGet() {
	return m_intake->GetShooterSpeed();
}
