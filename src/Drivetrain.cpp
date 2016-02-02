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

}

Drivetrain::~Drivetrain() {

}
