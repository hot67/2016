/*
 * Drivetrain.cpp
 *
 *  Created on: Jan 23, 2016
 *      Author: Chad
 */

#include "Drivetrain.h"

Drivetrain::Drivetrain(HotBot* bot)
	: HotSubsystem(bot, "Drivetrain") {
	m_lDriveF = new CANTalon(20);
	m_lDriveR = new CANTalon(21);
	m_rDriveF = new CANTalon(22);
	m_rDriveR = new CANTalon(23);

	m_lEncode = new Encoder(0, 1);
	m_rEncode = new Encoder(2, 3);

	m_timer = new Timer;

	m_drive = new RobotDrive(m_lDriveF, m_lDriveR, m_rDriveF, m_rDriveR);

	m_distancePIDWrapper = new DistancePIDWrapper(m_lEncode, m_rEncode);
	m_turnPIDWrapper = new TurnPIDWrapper (m_drive);

    m_drive->SetExpiration(0.1);
    m_drive->SetInvertedMotor(RobotDrive::kFrontLeftMotor, true); //Inverts left side motors
    m_drive->SetInvertedMotor(RobotDrive::kRearLeftMotor, true);
    try {
        m_gyro = new AHRS(SPI::Port::kMXP); // This is the Gyro.
    } catch (std::exception ex ) {
        std::string err_string = "Error instantiating navX MXP:  ";
        err_string += ex.what();
        DriverStation::ReportError(err_string.c_str());
    }

    m_distancePID = new PIDController(distanceP,distanceI,distanceD,m_distancePIDWrapper, m_distancePIDWrapper);

	m_turnPID = new PIDController(turnP, turnI, turnD, turnF, m_gyro, m_turnPIDWrapper);
	        m_turnPID->SetInputRange(-180.0f,  180.0f);
	        m_turnPID->SetOutputRange(-1.0, 1.0);
	        m_turnPID->SetAbsoluteTolerance(kToleranceDegrees);
	        m_turnPID->SetContinuous(true);
}

Drivetrain::~Drivetrain() {

}
