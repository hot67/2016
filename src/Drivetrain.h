/*
 * Drivetrain.h
 *
 *  Created on: Jan 23, 2016
 *      Author: Chad
 */


/*
 * CHAD'S TO DO LIST
 *
 * get distance value -> going to be called GetDistancePIDSetPoint()
 * 			what is the goal for the target distance?
 * 			example: we need to move 5 units so we SetDistance(5), so how can we get live data to pull that '5' to the dashboard
 * what the heck is setangle
 * 			make a turnPID function for setting the turnPID. thanks
 *
 *
 */
#ifndef SRC_DRIVETRAIN_H_
#define SRC_DRIVETRAIN_H_

#include <AnglePIDWrapper.h>
#include "WPILib.h"
#include "AHRS.h"
#include "RobotUtils/HotSubsystem.h"
#include <cmath>

//#include "AHRS.h"

#include "DistancePIDWrapper.h"

//#include "TurnPIDWrapper.h"

#include "SpanglePIDWrapper.h"

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
 *  Gear Shift CAN Location
 */

#define SOLENOID_SHIFT 0

/*
 * PID coefficients for turning
 */
#define ANGLE_P 0.12 //0.086
#define ANGLE_I 0.0035 //0.0071
#define ANGLE_D 0

#define SPANGLE_P 0
#define SPANGLE_I 0
#define SPANGLE_D 0

#define DISTANCE_SHIFTL_P 0.12
#define DISTANCE_SHIFTL_I 0
#define DISTANCE_SHIFTL_D 0

#define DISTANCE_SHIFTH_P 0.045
#define DISTANCE_SHIFTH_I 0
#define DISTANCE_SHIFTH_D 0

//class TurnPIDWrapper;
class DistancePIDWrapper;
class AnglePIDWrapper;

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
//	double GetAngle();

	/*
	 * Resets angle to zero (yaw?)
	 */
//	void ResetGyroAngle();

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

	void ResetEncoder();

	double GetAngle();
	void ResetGyro();

	void UpdateAccelArray();

	double GetAccelX();
	double GetAccelY();
	double GetAccelZ();

	/******************************
	 * Motor Control
	 ******************************/
	void ArcadeDrive(double speed, double turn);
	void SetTurn(double turn);
	void SetSpeed(double speed);
	float GetSpeed();
	float GetTurn();
	/*
	 * Shifting
	 */
	void SetShift(bool on);

	void ShiftHigh();
	void ShiftLow();

	/*****************************
	 * 	PID
	 *****************************/
	void EnablePID();
	void DisablePID();

	bool IsPIDEnabled();

	void SetPIDSetpoint(double distance, double angle);

	double GetDistancePIDSetpoint();
	double GetAnglePIDSetpoint();

	bool DistanceAtSetpoint();
	bool AngleAtSetpoint();

private:
	CANTalon* m_lDriveF;
	CANTalon* m_lDriveR;
	CANTalon* m_rDriveF;
	CANTalon* m_rDriveR;

	Encoder* m_lEncode;
	Encoder* m_rEncode;

	Solenoid* m_shift;

	AHRS *m_gyro;
	BuiltInAccelerometer *m_accel;

	Timer* m_timer;

	RobotDrive* m_drive;

	DistancePIDWrapper* m_distancePIDWrapper;

	AnglePIDWrapper* m_anglePIDWrapper;

	PIDController* m_anglePID;
	PIDController* m_distancePID;
	//PIDController* m_spanglePID;

	float m_turn, m_speed;

	double m_xRing[50];
	double m_yRing[50];
	double m_zRing[50];

	int m_index = 0;

	double oldAccelX = 0;
	double oldAccelY = 0;
	double oldAccelZ = 0;

};

#endif /* SRC_DRIVETRAIN_H_ */
