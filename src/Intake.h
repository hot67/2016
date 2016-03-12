/*
 * Intake.h
 *
 *  Created on: Jan 31, 2016
 *      Author: Marlina
 */

#ifndef SRC_INTAKE_H_
#define SRC_INTAKE_H_

#include "RobotUtils/RobotUtils.h"

//#define COMPETITION_BOT
#define PRACTICE_BOT

#ifdef PRACTICE_BOT
//shooter PID
#define SHOOTER_SPEED_P 0.01 //code from 2012 was 0.01
#define SHOOTER_SPEED_I 0.0 //code from 2012 was 0.0
#define SHOOTER_SPEED_D 0.0 //code from 2012 was 0.00
#endif

#ifdef COMPETITION_BOT
//shooter PID info
#define SHOOTER_SPEED_P 0.0
#define SHOOTER_SPEED_I 0.0
#define SHOOTER_SPEED_D 0.0
#endif


//roller and shooter CAN locations
#define ROLLER_ID 13
#define SHOOTER_ID 16

//encoder DIO location
#define SHOOTER_ENCODER 8
//will be adding in white-black sensor, but leaving encoder initialization for now

#define DEFAULT_SHOOTER_SPEED 0.8

// To do: what is it actually
#define SHOOTER_PULSE_PER_ROTATION 3

// Shooter Speeds

class ShooterPIDWrapper;

class Intake: public HotSubsystem {
public:
	enum ShooterStatus {
		kShooterStopped = 0,
		kShooterSpeeding = 1,
		kShooterAtSpeed = 2
	};

	Intake(HotBot* bot);

	/******************************
	 * MOTORS
	 ******************************/
	/*
	 * 	Request signal to motor
	 * 		speed must be in [-1, 1]
	 * 		higher priority signal will go
	 */
	void SetRoller(float speed, int priority = 0);
	void SetShooter(float speed, int priority = 0);

	/*
	 * 	Update system
	 * 		Reflect request to motor
	 */
	void Update();

	/******************************
	 * SENSORS
	 ******************************/
	/*
	 * 	Measure shooter speed in RPM
	 */
	double GetShooterSpeed();

	/******************************
	 * 	SHOOTER PID
	 ******************************/
	/*
	 * Enable and Disable Shooter PID
	 */
	void EnableShooterPID(int priority = 0);
	void DisableShooterPID(int priority = 0);

	/*
	 * Is enabled?
	 */
	bool IsShooterPIDEnabled();

	/*
	 * Set Setpoint
	 */
	void SetShooterPIDSetPoint(float speed);

	/*
	 * Get set point
	 */
	double GetShooterPIDSetPoint();

	/*
	 * At set point?
	 */
	bool ShooterAtSetPoint();

	/*
	 * LOGGING
	 */
	void IntakePrintData();

	/******************************
	 * 	Shooter PID Wrapper
	 ******************************/
	class ShooterPIDWrapper : public PIDSource, public PIDOutput {
	public:
		ShooterPIDWrapper(Intake *intake);

		void PIDWrite(float output);
		double PIDGet();

	private:
		Intake *m_intake;
		float m_speed = 0.0;
	};
private:

	/*
	 * 	Functions to actually send signal to motor
	 */
	void _SetRoller(float speed);
	void _SetShooter(float speed);

	/*
	 * 	Enable Disable
	 */
	void _EnableShooterPID();
	void _DisableShooterPID();

	/*
	 * 	Talons
	 */
	CANTalon* m_rollerTalon;
	CANTalon* m_shooterTalon;

	/*
	 * 	Buffers
	 */
	DoubleBuffer *m_rollerBuf, *m_shooterBuf;
	BooleanBuffer *m_shooterPIDBuf;

	/*
	 * 	Shooter Encoder
	 */
	Encoder* m_shooterEncoder;

	/*
	 * 	PID
	 */
	PIDController* m_shooterPID;
};


#endif /* SRC_INTAKE_H_ */
