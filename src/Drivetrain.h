/*
 * Drivetrain.h
 *
 *  Created on: Jan 23, 2016
 *      Author: Chad
 */

#ifndef SRC_DRIVETRAIN_H_
#define SRC_DRIVETRAIN_H_

#include "WPILib.h"
#include "RobotUtils/HotSubsystem.h"
#include <cmath>

class Drivetrain : public HotSubsystem {
public:
	Drivetrain(HotBot* bot);

	friend class HotSubsystemHandler;
	virtual ~Drivetrain();

	void ArcadeDrive(double speed, double angle, bool squaredinputs=true) {
		m_drive->ArcadeDrive(speed, angle, squaredinputs);

		SmartDashboard::PutNumber("m_lEncode Distance", m_lEncode->GetDistance());
		SmartDashboard::PutNumber("m_rEncode Distance", m_rEncode->GetDistance());
		SmartDashboard::PutNumber("m_lEncode Rate", m_lEncode->GetRate());
		SmartDashboard::PutNumber("m_rEncode Rate", m_rEncode->GetRate());
	}

private:
	CANTalon* m_lDriveF;
	CANTalon* m_lDriveR;
	CANTalon* m_rDriveF;
	CANTalon* m_rDriveR;

	Encoder* m_lEncode;
	Encoder* m_rEncode;

	Timer* m_timer;

	RobotDrive* m_drive;

};

#endif /* SRC_DRIVETRAIN_H_ */
