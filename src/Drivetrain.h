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
#include "AHRS.h"
#include "DistancePIDWrapper.h"
#include "TurnPIDWrapper.h"

const static double turnP = 0.03f;
const static double turnI = 0.00f;
const static double turnD = 0.00f;
const static double turnF = 0.00f;
const static double kToleranceDegrees = 2.0f;

const static double distanceP = -2.5;
const static double distanceI = 0.0;
const static double distanceD = -0.1;

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

	DistancePIDWrapper* m_distancePIDWrapper;
	TurnPIDWrapper* m_turnPIDWrapper;

	AHRS* m_gyro;

	PIDController* m_turnPID;
	PIDController* m_distancePID;

};

#endif /* SRC_DRIVETRAIN_H_ */
