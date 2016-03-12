/*
 * Drivetrain.cpp
 *
 *  Created on: Jan 23, 2016
 *      Author: Chad
 */

#include "Drivetrain.h"

Drivetrain::Drivetrain(HotBot* bot) : HotSubsystem(bot, "Drivetrain") {
	/*
	 * 	Drive
	 */
	m_lfDrive = new CANTalon(TALON_DRIVE_LF);
	m_lrDrive = new CANTalon(TALON_DRIVE_LR);
	m_rfDrive = new CANTalon(TALON_DRIVE_RF);
	m_rfDrive = new CANTalon(TALON_DRIVE_RR);

	m_drive = new RobotDrive(m_lfDrive, m_lrDrive, m_rfDrive, m_rrDrive);
	m_drive->SetSafetyEnabled(false);

	/*
	 * 	Buffer For Driving
	 */
	m_speedBuf = new DoubleBuffer(BufferPriority::kAbsMax);
	m_turnBuf = new DoubleBuffer(BufferPriority::kAbsMax);

	/*
	 *	Shifting
	 */
	m_shift = new Solenoid(SOLENOID_SHIFT);
	m_shiftBuf = new BooleanBuffer(false);

	/*
	 * 	Encoders
	 */
	m_lEncoder = new Encoder(DRIVE_ENCODER_LF, DRIVE_ENCODER_LR, true);
	m_rEncoder = new Encoder(DRIVE_ENCODER_RF, DRIVE_ENCODER_RR, false);

	/*
	 * 	Gyro
	 */
	m_gyro = new AHRS(SPI::Port::kMXP);

	DistancePIDWrapper *distancePIDWrapper = new DistancePIDWrapper(this);
    m_distancePID = new PIDController(DISTANCE_SHIFTL_P, DISTANCE_SHIFTL_I, DISTANCE_SHIFTL_D,
    		distancePIDWrapper, distancePIDWrapper);
    m_distancePID->SetAbsoluteTolerance(5.2);
    m_distancePIDBuf = new BooleanBuffer(false);

	AnglePIDWrapper *anglePIDWrapper = new AnglePIDWrapper (this);
    m_anglePID = new PIDController(0.086, 0.0071, 0.0,
    		anglePIDWrapper, anglePIDWrapper);
    m_anglePID->SetAbsoluteTolerance(1.0);
    m_anglePIDBuf = new BooleanBuffer(false);
}

/******************************
 * 	Motor
 ******************************/
void Drivetrain::_ArcadeDrive(float speed, float turn) {
	m_drive->ArcadeDrive(speed, turn);
}
void Drivetrain::_SetShift(bool on) {
	m_shift->Set(on);
}

void Drivetrain::ArcadeDrive(float speed, float turn, int priority) {
	m_speedBuf->Write(speed, priority);
	m_turnBuf->Write(turn, priority);
}
void Drivetrain::SetShift(bool on, int priority) {
	m_shiftBuf->Write(on, priority);
}
void Drivetrain::ShiftHigh(int priority) {
	SetShift(false, priority);
}
void Drivetrain::ShiftLow(int priority) {
	SetShift(true, priority);
}

void Drivetrain::Update() {
	if (!m_speedBuf->IsLocked() && !m_turnBuf->IsLocked()) {
		_ArcadeDrive(m_speedBuf->Read(), m_turnBuf->Read());
	}

	if (!m_shiftBuf->IsLocked()) {
		_SetShift(m_shiftBuf->Read());
	}

	if (!m_distancePIDBuf->IsLocked()) {
		if (m_distancePIDBuf->Read()) {
			_EnableDistancePID();
		} else {
			_DisableDistancePID();
		}
	}

	if (!m_anglePIDBuf->IsLocked()) {
		if (m_anglePIDBuf->Read()) {
			_EnableAnglePID();
		} else {
			_DisableAnglePID();
		}
	}
}
/******************************
 * Sensors
 ******************************/
double Drivetrain::GetLDistance() {
	return m_lEncoder->GetDistance() * .0084275;
}
double Drivetrain::GetRDistance() {
	return m_rEncoder->GetDistance() * .0084275;
}
double Drivetrain::GetDistance() {
	return (GetLDistance() + GetRDistance()) / 2;
}

double Drivetrain::GetLSpeed() {
	return m_lEncoder->GetRate() * .0084275;
}
double Drivetrain::GetRSpeed() {
	return m_rEncoder->GetRate() * .0084275;
}
double Drivetrain::GetSpeed() {
	return (GetLSpeed() + GetRSpeed()) / 2;
}

double Drivetrain::GetAngle() {
	if (m_gyro->GetAngle() > 180.0) {
		return m_gyro->GetAngle() - 360.0;
	} else {
		return m_gyro->GetAngle();
	}
}
double Drivetrain::GetAngularSpeed() {
	return m_gyro->GetRate();
}

/******************************
 * 	Distance PID
 ******************************/
void Drivetrain::_EnableDistancePID() {
	if (!IsDistancePIDEnabled()) {
		m_distancePID->Enable();
	}
}
void Drivetrain::_DisableDistancePID() {
	if (IsDistancePIDEnabled()) {
		m_distancePID->Disable();
	}
}
void Drivetrain::EnableDistancePID(int priority){
	if (!IsDistancePIDEnabled()) {
		m_distancePIDBuf->Write(true, priority);
	}
}

void Drivetrain::DisableDistancePID(int priority){
	if (IsDistancePIDEnabled()) {
		m_distancePIDBuf->Write(false, priority);
	}
}

bool Drivetrain::IsDistancePIDEnabled(){
	return m_distancePID->IsEnabled();
}

void Drivetrain::SetDistancePIDSetpoint(double inches) {
	m_distancePID->SetSetpoint(inches);
}

double Drivetrain::GetDistancePIDSetpoint() {
	return m_distancePID->GetSetpoint();
}

bool Drivetrain::DistanceAtPIDSetpoint () {
	return m_distancePID->OnTarget();
}

Drivetrain::DistancePIDWrapper::DistancePIDWrapper(Drivetrain *drivetrain) {
	m_drivetrain = drivetrain;
}
void Drivetrain::DistancePIDWrapper::PIDWrite(float output) {
	m_drivetrain->_ArcadeDrive(output, 0.0);
}
double Drivetrain::DistancePIDWrapper::PIDGet() {
	return m_drivetrain->GetDistance();
}

/******************************
 * 	Angle PID
 ******************************/
void Drivetrain::_EnableAnglePID() {
	if (!IsAnglePIDEnabled()) {
		m_anglePID->Enable();
	}
}
void Drivetrain::_DisableAnglePID() {
	if (IsAnglePIDEnabled()) {
		m_anglePID->Disable();
	}
}
void Drivetrain::EnableAnglePID(int priority){
	if (!IsAnglePIDEnabled()) {
		m_anglePIDBuf->Write(true, priority);
	}
}

void Drivetrain::DisableAnglePID(int priority){
	if (IsAnglePIDEnabled()) {
		m_anglePIDBuf->Write(false, priority);
	}
}

bool Drivetrain::IsAnglePIDEnabled(){
	return m_anglePID->IsEnabled();
}

void Drivetrain::SetAnglePIDSetpoint(double inches) {
	m_anglePID->SetSetpoint(inches);
}

double Drivetrain::GetAnglePIDSetpoint() {
	return m_anglePID->GetSetpoint();
}

bool Drivetrain::AngleAtPIDSetpoint () {
	return m_anglePID->OnTarget();
}

Drivetrain::AnglePIDWrapper::AnglePIDWrapper(Drivetrain *drivetrain) {
	m_drivetrain = drivetrain;
}
void Drivetrain::AnglePIDWrapper::PIDWrite(float output) {
	m_drivetrain->ArcadeDrive(0.0, output);
}
double Drivetrain::AnglePIDWrapper::PIDGet() {
	return m_drivetrain->GetAngle();
}
