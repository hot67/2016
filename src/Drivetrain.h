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
//#include "AHRS.h"
#include "DistancePIDWrapper.h"
#include "TurnPIDWrapper.h"

#define TALON_DRIVE_LF 20
#define TALON_DRIVE_LR 21
#define TALON_DRIVE_RF 22
#define TALON_DRIVE_RR 23

#define DRIVE_ENCODER_LF 0
#define DRIVE_ENCODER_LR 1
#define DRIVE_ENCODER_RF 2
#define DRIVE_ENCODER_RR 3

const static double turnP = 0.03f; //PID Variables
const static double turnI = 0.00f;
const static double turnD = 0.00f;
const static double turnF = 0.00f;
const static double kToleranceDegrees = 2.0f;

const static double distanceP = -2.5;
const static double distanceI = 0.0;
const static double distanceD = -0.1;

class TurnPIDWrapper;
class DistancePIDWrapper;

class Drivetrain : public HotSubsystem {
public:
	Drivetrain(HotBot* bot);

	friend class HotSubsystemHandler;
	virtual ~Drivetrain();

	double GetAngle(); //Encoder Functions
	double GetDistancePos();
	double GetDistanceL();
	double GetDistanceR();
	double GetSpeedL();
	double GetSpeedR();
	double GetAverageSpeed();

	void SetTurn(double turn);
	void SetSpeed(double speed);
	void ArcadeDrive(double speed, double angle);

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

	//AHRS* m_gyro;

	PIDController* m_turnPID;
	PIDController* m_distancePID;

	float m_turning, m_speed;

};

#endif /* SRC_DRIVETRAIN_H_ */
