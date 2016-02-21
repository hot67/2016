/*
 * Drivetrain.cpp
 *
 *  Created on: Jan 23, 2016
 *      Author: Chad
 */

#include "Drivetrain.h"

Drivetrain::Drivetrain(HotBot* bot)
	: HotSubsystem(bot, "Drivetrain") {
	m_lDriveF = new CANTalon(TALON_DRIVE_LF);
	m_lDriveR = new CANTalon(TALON_DRIVE_LR);
	m_rDriveF = new CANTalon(TALON_DRIVE_RF);
	m_rDriveR = new CANTalon(TALON_DRIVE_RR);

	m_shift = new CANTalon(TALON_SHIFT);

	m_lEncode = new Encoder(DRIVE_ENCODER_LF, DRIVE_ENCODER_LR);
	m_rEncode = new Encoder(DRIVE_ENCODER_RF, DRIVE_ENCODER_RR);

	m_timer = new Timer;

	m_drive = new RobotDrive(m_lDriveF, m_lDriveR, m_rDriveF, m_rDriveR);
	m_drive->SetSafetyEnabled(false);

	m_distancePIDWrapper = new DistancePIDWrapper(this);
	m_turnPIDWrapper = new TurnPIDWrapper (this);

    m_drive->SetExpiration(0.1);

    m_euro = new AHRS(SPI::Port::kMXP);

    m_distancePID = new PIDController(distanceP,distanceI,distanceD,m_distancePIDWrapper, m_distancePIDWrapper);

	m_turnPID = new PIDController(turnP, turnI, turnD, turnF, m_euro, m_turnPIDWrapper);
	m_turnPID->SetInputRange(-180.0f,  180.0f);
	m_turnPID->SetOutputRange(-1.0, 1.0);
	m_turnPID->SetAbsoluteTolerance(ToleranceDegrees);
	m_turnPID->SetContinuous(true);

	/**
	 * 	Initialize
	 */
	m_turning = m_speed = 0.0;

	/**
	 * 	Initial value of shift is High
	 */
	f_shift = HIGH;
}

/******************************
 * Sensors
 ******************************/

double Drivetrain::GetAngle(){
	return(m_euro->GetAngle());
}

double Drivetrain::GetAverageDistance(){
	return((GetLDistance() + GetRDistance()) / 2);
}

double Drivetrain::GetLDistance(){
	return(m_lEncode->GetDistance());
}

double Drivetrain::GetRDistance(){
	return(m_rEncode->GetDistance());
}

double Drivetrain::GetLSpeed(){
	return(m_lEncode->GetRate());
}

double Drivetrain::GetRSpeed(){
	return(m_rEncode->GetRate());
}

double Drivetrain::GetAverageSpeed(){
	return((GetLSpeed() + GetRSpeed()) / 2);
}

bool Drivetrain::GetShift() {
	return f_shift;
}

bool Drivetrain::IsShiftHight() {
	return f_shift == HIGH;
}

bool Drivetrain::IsShiftLow() {
	return f_shift = LOW;
}

/******************************
 * Motors
 ******************************/
void Drivetrain::ArcadeDrive(double speed, double angle){
	m_speed = speed;
	m_turning = angle;
	m_drive->ArcadeDrive(m_speed, m_turning);

	SmartDashboard::PutNumber("Drivetrain speed", m_speed);
	SmartDashboard::PutNumber("Drivetrain turn", m_turning);
	SmartDashboard::PutNumber("Drive LF", m_lDriveF->Get());
	SmartDashboard::PutNumber("Drive LR", m_lDriveR->Get());
	SmartDashboard::PutNumber("Drive RF", m_rDriveF->Get());
	SmartDashboard::PutNumber("Drive RR", m_rDriveR->Get());
}

void Drivetrain::SetSpeed(double speed) {
	ArcadeDrive(speed, m_turning);
}

void Drivetrain::SetTurn(double turn) {
	ArcadeDrive(m_speed, turn);
}

void Drivetrain::SetShift(bool on){
	m_shift->Set(on);
}

void Drivetrain::ShiftLow() {
	f_shift = LOW;	//	False is low gear
	m_shift->Set(1.0);
}

void Drivetrain::ShiftHigh() {
	if (f_shift == LOW) {
		/**
		 * 	This code only runs first time this function is called
		 */
		m_timer->Stop();
		m_timer->Reset();
		m_timer->Start();
		f_shift = HIGH;	//	True is high gear
	}

	if (m_timer->Get() < 1.0) {
		m_shift->Set(-1.0);
	} else {
		m_shift->Set(0.0);
	}
}

/******************************
 * 	Distance PID
 ******************************/

void Drivetrain::EnableDistance(){
	m_distancePID->Enable();
}

void Drivetrain::DisableDistance(){
	m_distancePID->Disable();
}

bool Drivetrain::IsEnabledDistance(){
	return (m_distancePID->IsEnabled());
}

void Drivetrain::SetDistance(double distance) {
	m_distancePID->SetSetpoint(distance);
}

double Drivetrain::GetDistancePIDSetPoint() {
	return m_distancePID->GetSetpoint();
}

bool Drivetrain::DistanceAtSetPoint () {
	return m_distancePID->OnTarget();
}

double Drivetrain::GetDistancePID () {
	return (m_distancePIDWrapper->PIDGet());
}


/******************************
 * Turn PID
 ******************************/
void Drivetrain::EnableAngle(){
	m_turnPID->Enable();
}

void Drivetrain::DisableAngle(){
	m_turnPID->Disable();
}

bool Drivetrain::IsEnabledAngle() {
	return m_turnPID->IsEnabled();
}

void Drivetrain::SetAngle(double angle) {
	m_turnPID->SetSetpoint(angle);
}

double Drivetrain::GetAnglePIDSetPoint() {
	return m_turnPID->GetSetpoint();
}

bool Drivetrain::AngleAtSetPoint() {
	return m_turnPID->OnTarget();
}

void Drivetrain::ResetGyroAngle(){
	m_euro->Reset();
}

void Drivetrain::DisableBothPIDs(){
	DisableDistance();
	DisableAngle();
}


Drivetrain::~Drivetrain() {

}
