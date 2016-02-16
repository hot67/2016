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

	m_lEncode = new Encoder(DRIVE_ENCODER_LF, DRIVE_ENCODER_LR);
	m_rEncode = new Encoder(DRIVE_ENCODER_RF, DRIVE_ENCODER_RR);

	m_timer = new Timer;

	//m_gyro = new Gyro;

	m_drive = new RobotDrive(m_lDriveF, m_lDriveR, m_rDriveF, m_rDriveR);
	m_drive->SetSafetyEnabled(false);

	m_distancePIDWrapper = new DistancePIDWrapper(this);
	m_turnPIDWrapper = new TurnPIDWrapper (this);

    m_drive->SetExpiration(0.1);

    //m_gyro = new AHRS(SPI::Port::kMXP);

    m_distancePID = new PIDController(distanceP,distanceI,distanceD,m_distancePIDWrapper, m_distancePIDWrapper);

	/*m_turnPID = new PIDController(turnP, turnI, turnD, turnF, m_gyro, m_turnPIDWrapper);
	m_turnPID->SetInputRange(-180.0f,  180.0f);
	m_turnPID->SetOutputRange(-1.0, 1.0);
	m_turnPID->SetAbsoluteTolerance(kToleranceDegrees);
	m_turnPID->SetContinuous(true); */
}

void Drivetrain::ArcadeDrive(double speed, double angle){
	m_speed = speed;
	m_turning = angle;
	m_drive->ArcadeDrive(speed, angle);

	/*SmartDashboard::PutNumber("m_lEncode Distance", m_lEncode->GetDistance());
	SmartDashboard::PutNumber("m_rEncode Distance", m_rEncode->GetDistance());
	SmartDashboard::PutNumber("m_lEncode Rate", m_lEncode->GetRate());
	SmartDashboard::PutNumber("m_rEncode Rate", m_rEncode->GetRate());*/
}

double Drivetrain::GetAngle(){
	return 0; //(m_gyro->GetAngle());
}

double Drivetrain::GetDistancePos(){
	return((m_lEncode->GetDistance() + m_rEncode->GetDistance()) / 2);
}

double Drivetrain::GetDistanceL(){
	return(m_lEncode->GetDistance());
}

double Drivetrain::GetDistanceR(){
	return(m_rEncode->GetDistance());
}

double Drivetrain::GetSpeedL(){
	return(m_lEncode->GetRate());
}

double Drivetrain::GetSpeedR(){
	return(m_rEncode->GetRate());
}

double Drivetrain::GetAverageSpeed(){
	return((m_lEncode->GetRate() + m_rEncode->GetRate()) / 2);
}

void Drivetrain::SetSpeed(double speed) {
	ArcadeDrive(speed, m_turning);
}

void Drivetrain::SetTurn(double turn) {
	ArcadeDrive(m_speed, turn);
}

Drivetrain::~Drivetrain() {

}
