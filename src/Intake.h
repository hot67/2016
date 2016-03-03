/*
 * Intake.h
 *
 *  Created on: Jan 31, 2016
 *      Author: Marlina
 */

#ifndef SRC_INTAKE_H_
#define SRC_INTAKE_H_

#include "RobotUtils/HotSubsystem.h"

#include "ShooterPIDWrapper.h"

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
private:

	CANTalon* m_rollerTalon;
	CANTalon* m_shooterTalon;

	Encoder* m_shooterEncoder;

	ShooterPIDWrapper *m_shooterPIDWrapper;
	PIDController* m_shooterSpeedPID;

	Timer *m_pulseOutTimer;
	bool f_rollingIn;

public:
	enum ShooterStatus {
		kShooterStopped = 0,
		kShooterSpeeding = 1,
		kShooterAtSpeed = 2
	};

	Intake(HotBot* bot);

	virtual ~Intake();

	float m_desiredShooterSpeed = DEFAULT_SHOOTER_SPEED; // sets initial shooter speed to default speed


	/******************************
	 * SENSORS
	 ******************************/

	/*
	 * get encoder rate
	 * 			the black-white sensor
	 */
	double GetShooterSpeed();
	double GetShooterPeriod();

	/******************************
	 * MOTORS
	 ******************************/

	/*
	 * Setting raw roller speed
	 * 		on a scale of -1 to 1
	 */
	void SetRoller(float speed); //set roller speed

	/*
	 * Setting raw shooter speed
	 * 		on a scale of -1 to 1
	 */
	void SetShooter(float speed); //set shooter speed

	float GetShooter();

	void ResetRollerStatus();


	/******************************
	 * Shooter specifics
	 ******************************/

	/*
	 * setting shooter default to 0.8
	 */
	void SetShooterDefault();

	/*
	 * rolls the ball into the shooter if the shooter PID is on target (if the shooter is up to speed)
	 */
	void Shoot();

	/*
	 * Shooter status function
	 */
	ShooterStatus GetShooterStatus();

	/*
	 * Increase Shooter Speed by 0.01
	 */
	void IncreaseShooterSpeed();

	/*
	 * Decrease shooter speed by 0.01
	 */
	void DecreaseShooterSpeed();

	/*
	 * sets shooter speed to m_desiredshooterspeed
	 * 			which is the adjustable variable by the IncreaseShooterSpeed() and the DecreaseShooterSpeed()
	 */
	void SetDesiredShooterSpeed();



	/******************************
	 * 	SHOOTER PID
	 ******************************/

	/*
	 * Enable and Disable Shooter PID
	 */
	void EnableShooterPID();
	void DisableShooterPID();

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


};


#endif /* SRC_INTAKE_H_ */
