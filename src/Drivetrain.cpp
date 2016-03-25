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

	m_shift = new Solenoid(SOLENOID_SHIFT);

	m_lEncode = new Encoder(DRIVE_ENCODER_LF, DRIVE_ENCODER_LR, true);
	m_rEncode = new Encoder(DRIVE_ENCODER_RF, DRIVE_ENCODER_RR, false);

	m_gyro = new AHRS(SPI::Port::kMXP);

	m_accel = new BuiltInAccelerometer();

	m_timer = new Timer;

	m_drive = new RobotDrive(m_lDriveF, m_lDriveR, m_rDriveF, m_rDriveR);
	m_drive->SetSafetyEnabled(false);
    m_drive->SetExpiration(0.1);

	m_distancePIDWrapper = new DistancePIDWrapper(this);

	m_anglePIDWrapper = new AnglePIDWrapper (this);

    m_distancePID = new PIDController(DISTANCE_SHIFTL_P, DISTANCE_SHIFTL_I, DISTANCE_SHIFTL_D, m_distancePIDWrapper, m_distancePIDWrapper);
    m_distancePID->SetAbsoluteTolerance(5.2);

    m_anglePID = new PIDController(ANGLE_P, ANGLE_I, ANGLE_D, m_anglePIDWrapper, m_anglePIDWrapper);
    m_anglePID->SetAbsoluteTolerance(2.0);

    /*
     *  Initialize turning and speed
     */
    m_turn = m_speed = 0.0;

    for (int i = 0; i < 50; i++) {
    	m_xRing[i] = 0.0;
    	m_yRing[i] = 0.0;
    	m_zRing[i] = 0.0;
    }
}

/******************************
 * Sensors
 ******************************/

/*

double Drivetrain::GetAngle(){
	return (m_euro->GetAngle());
}

*/

double Drivetrain::GetAverageDistance(){
	return((GetLDistance() + GetRDistance()) / 2 * -1);
}

double Drivetrain::GetLDistance(){
	return(m_lEncode->GetDistance() * .0084275);
}

double Drivetrain::GetRDistance(){
	return(m_rEncode->GetDistance() * .0084275);
}

double Drivetrain::GetLSpeed(){
	return(m_lEncode->GetRate() * .0084275);
}

double Drivetrain::GetRSpeed(){
	return(m_rEncode->GetRate() * .0084275);
}

double Drivetrain::GetAverageSpeed(){
	return((GetLSpeed() + GetRSpeed()) / 2 * -1);
}

void Drivetrain::ResetEncoder(){
	m_lEncode->Reset();
	m_rEncode->Reset();
}

double Drivetrain::GetAngle() {
	//credit to mark the true genius
	if (m_gyro->GetAngle() > 180.0) {
		return m_gyro->GetAngle() - 360.0;
	} else {
		return m_gyro->GetAngle();
	}
}

void Drivetrain::ResetGyro() {
	m_gyro->Reset();
}

void Drivetrain::UpdateAccelArray() {
	m_xRing[m_index++ % 50] = fabs(m_accel->GetX());
	m_yRing[m_index++ % 50] = fabs(m_accel->GetY());
	m_zRing[m_index++ % 50] = fabs(m_accel->GetZ() - 1);

	double averageX = 0;
	double averageY = 0;
	double averageZ = 0;

	for (int i = 0; i < 50; i++) {
		averageX += m_xRing[i];
		averageY += m_yRing[i];
		averageZ += m_zRing[i];
	}

	oldAccelX = averageX /50;
	oldAccelY = averageY /50;
	oldAccelZ = averageZ /50;
}

double Drivetrain::GetAccelX() {
	return oldAccelX;
}
double Drivetrain::GetAccelY() {
	return oldAccelY;
}
double Drivetrain::GetAccelZ() {
	return oldAccelZ;
}

/******************************
 * Motors
 ******************************/
void Drivetrain::ArcadeDrive(double speed, double angle){
	m_speed = speed;
	m_turn = angle;
	m_drive->ArcadeDrive(speed, angle);

//	SmartDashboard::PutBoolean("TurnPID Enabled", m_turnPID->IsEnabled());
	SmartDashboard::PutBoolean("DistancePID Enabled", m_distancePID->IsEnabled());

//	SmartDashboard::PutBoolean("SpanglePID Enabled", m_spanglePID->IsEnabled());

//	SmartDashboard::PutNumber("m_lEncode Distance", m_lEncode->GetDistance());
//	SmartDashboard::PutNumber("m_rEncode Distance", m_rEncode->GetDistance());
//	SmartDashboard::PutNumber("m_lEncode Rate", m_lEncode->GetRate());
//	SmartDashboard::PutNumber("m_rEncode Rate", m_rEncode->GetRate());
}

void Drivetrain::SetSpeed(double speed) {
	ArcadeDrive(speed, m_turn);
}

void Drivetrain::SetTurn(double turn) {
	ArcadeDrive(m_speed, turn);
}

void Drivetrain::SetShift(bool on){
	m_shift->Set(on);
}

void Drivetrain::ShiftHigh(){
	SetShift(false);
	m_distancePID->SetPID(DISTANCE_SHIFTH_P, DISTANCE_SHIFTH_I, DISTANCE_SHIFTH_D);
}

void Drivetrain::ShiftLow(){
	SetShift(true);
	m_distancePID->SetPID(DISTANCE_SHIFTL_P, DISTANCE_SHIFTL_I, DISTANCE_SHIFTL_D);
}

float Drivetrain::GetSpeed(){
	return(m_speed);
}

float Drivetrain::GetTurn(){
	return(m_turn);
}

/******************************
 * 	Distance PID
 ******************************/

void Drivetrain::EnableDistance(){
	if (!IsEnabledDistance()) {
		m_distancePID->Enable();
	}

	if (!IsEnabledAngle()) {
		SetAngle(GetAngle());
		EnableAngle();
	}
}

void Drivetrain::DisableDistance(){
	if (IsEnabledDistance()) {
		m_distancePID->Disable();
		DisableAngle();
	}
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
	if (fabs(GetDistancePIDSetPoint() - GetAverageDistance()) < 4) {
		return true;
	} else {
		return false;
	}
}

double Drivetrain::GetDistancePID () {
	return (m_distancePIDWrapper->PIDGet());
}


/******************************
 * Turn PID
 ******************************/

void Drivetrain::EnableAngle() {
	if (!IsEnabledAngle()) {
		m_anglePID->Enable();
	}
}

void Drivetrain::DisableAngle(){
	if (IsEnabledAngle()) {
		m_anglePID->Disable();
	}
}

bool Drivetrain::IsEnabledAngle() {
	return m_anglePID->IsEnabled();
}

void Drivetrain::SetAngle(double angle) {
	m_anglePID->SetSetpoint(angle);
}

double Drivetrain::GetAnglePIDSetPoint() {
	return m_anglePID->GetSetpoint();
}

bool Drivetrain::AngleAtSetPoint() {
	if (fabs(GetAnglePIDSetPoint() - GetAngle()) < 2) {
		return true;
	}
	else {
		return false;
	}
}

/*
void Drivetrain::DisableBothPIDs(){
	DisableDistance();
//	DisableAngle();
}

*/

/******************************
 * Spangle PID
 ******************************/
/*
void Drivetrain::EnableSpangle(){
	m_spanglePID->Enable();
}

void Drivetrain::DisableSpangle(){
	m_spanglePID->Disable();
}

void Drivetrain::SetSpangle(float angle){
	m_spanglePID->SetSetpoint(angle);
}
bool Drivetrain::SpangleAtSetPoint() {
	return m_spanglePID->OnTarget();
}

double Drivetrain::GetSpanglePIDSetPoint(){
	return m_spanglePID->GetSetpoint();
}

bool Drivetrain::IsEnabledSpangle() {
	return m_spanglePID->IsEnabled();
}
 */

Drivetrain::~Drivetrain() {

}
