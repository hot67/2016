/*
 * Drivetrain.h
 *
 *  Created on: Jan 23, 2016
 *      Author: Chad
 */


/*
 * CHAD'S TO DO LIST
 *
 * add in tolerance for distance
 * 			you have tolerance for turning but not for distance so figure it out on your own
 * get distance value -> going to be called GetDistancePIDSetPoint()
 * 			what is the goal for the target distance?
 * 			example: we need to move 5 units so we SetDistance(5), so how can we get live data to pull that '5' to the dashboard
 * what the heck is setangle
 * 			make a turnPID function for setting the turnPID. thanks
 * what the heck is resetgyro
 * 			???????
 * 			tell jin/marlina when you figure it out
 *
 *
 *
 */
#ifndef SRC_DRIVETRAIN_H_
#define SRC_DRIVETRAIN_H_

#include "WPILib.h"
#include <AHRS.h>
#include "RobotUtils/HotSubsystem.h"
#include "DistancePIDWrapper.h"
#include "TurnPIDWrapper.h"

/*
 * Talons' Can Bus Location
 */

#define TALON_DRIVE_LF 20
#define TALON_DRIVE_LR 21
#define TALON_DRIVE_RF 22
#define TALON_DRIVE_RR 23

/*
 * Drive Encoders' DIO Location
 */

#define DRIVE_ENCODER_LF 0
#define DRIVE_ENCODER_LR 1
#define DRIVE_ENCODER_RF 2
#define DRIVE_ENCODER_RR 3

/*
 *  Gear Shift Solenoid Location
 *  	Changed from CANTalon
 */

#define SOLENOID_SHIFT 1

/*
 * PID coefficients for turning
 */

const static double turnP = 0.03f;
const static double turnI = 0.00f;
const static double turnD = 0.00f;
const static double turnF = 0.00f;

/*
 * Allowance for gyro turning
 */

const static double ToleranceDegrees = 2.0f;

/*
 * PID coefficients for distance
 */

const static double distanceP = 0.01;
const static double distanceI = 0.0;
const static double distanceD = 0.0;

/**
 * 	Shifting
 */
#define HIGH false
#define LOW true

class TurnPIDWrapper;
class DistancePIDWrapper;

class Drivetrain : public HotSubsystem {
public:
	Drivetrain(HotBot* bot);

	friend class HotSubsystemHandler;
	virtual ~Drivetrain();

	/******************************
	 * 	Sensors
	 ******************************/

	/*
	 * Gets current angle of gyro
	 */
	double GetAngle();

	/*
	 * ????? this is not in the cpp...
	 */
	void ResetGyro();

	/*
	 * Resets angle to zero (yaw?)
	 */
	void ResetGyroAngle();

	/*
	 * Gets average encoder distance
	 */
	double GetAverageDistance();

	/*
	 * Gets left encoder distance
	 */
	double GetLDistance();

	/*
	 * Gets right encoder distance
	 */
	double GetRDistance();

	/*
	 * Gets left encoder speed
	 */
	double GetLSpeed();

	/*
	 * Gets right encoder speed
	 */
	double GetRSpeed();

	/*
	 * Gets average encoder speed
	 */
	double GetAverageSpeed();

	/**
	 * 	Check Shifting
	 */
	bool GetShift();
	bool IsShiftHight();
	bool IsShiftLow();

	/******************************
	 * Motor Control
	 ******************************/
	void ArcadeDrive(double speed, double turn);
	void SetTurn(double turn);
	void SetSpeed(double speed);

	/*
	 * Shifting
	 * 	true:	LOW GEAR
	 * 	false:	HIGH GEAR
	 */
	void SetShift(bool on);

	/**
	 * 	You cannot call the following two functions together
	 */
	void ShiftLow();
	void ShiftHigh();

	/******************************
	 * Distance PID
	 ******************************/

	/*
	 * Enable and Disable
	 */

	void EnableDistance();
	void DisableDistance();

	/*
	 *  Is Enabled?
	 */
	bool IsEnabledDistance();

	/*
	 * Set Setpoint
	 */
	void SetDistance(double distance);

	/*
	 * What is goal now?
	 */
	double GetDistancePIDSetPoint();

	/*
	 * Have we arrived to the set point?
	 */

	bool DistanceAtSetPoint();

	/*
	 * Current encoder value to send to PID
	 */

	double GetDistancePID();

	/******************************
	 * Angle PID
	 ******************************/

	/*
	 * Enabling and Disabling
	 */
	void EnableAngle();
	void DisableAngle();

	/*
	 * Is Enabled?
	 */
	bool IsEnabledAngle();

	/*
	 * set turn set point
	 */
	void SetAngle(double angle);

	/*
	 * What is goal now?
	 */
	double GetAnglePIDSetPoint();

	/**
	 * 	Have we arrived to the set point?
	 */
	bool AngleAtSetPoint();


	void DisableBothPIDs();

private:
	CANTalon* m_lDriveF;
	CANTalon* m_lDriveR;
	CANTalon* m_rDriveF;
	CANTalon* m_rDriveR;

	Solenoid* m_shift;

	Encoder* m_lEncode;
	Encoder* m_rEncode;

	Timer* m_timer;

	RobotDrive* m_drive;

	DistancePIDWrapper* m_distancePIDWrapper;
	TurnPIDWrapper* m_turnPIDWrapper;

	AHRS* m_euro;

	PIDController* m_turnPID;
	PIDController* m_distancePID;

	float m_turning, m_speed;
};

#endif /* SRC_DRIVETRAIN_H_ */
