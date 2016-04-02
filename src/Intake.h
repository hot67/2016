/*
 * Intake.h
 *
 *  Created on: Jan 31, 2016
 *      Author: Marlina
 */

#ifndef SRC_INTAKE_H_
#define SRC_INTAKE_H_

#include "RobotUtils/HotSubsystem.h"


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

#define LEFT_SHOOTER_ID 16
#define RIGHT_SHOOTER_ID 17

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

	CANTalon* m_lShooterTalon;
	CANTalon* m_rShooterTalon;

	Encoder* m_shooterEncoder;

	ShooterPIDWrapper *m_shooterPIDWrapper;
	PIDController* m_shooterSpeedPID;

	Timer *m_pulseOutTimer;

	Timer *m_shootingTimer;
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

	float GetLeftShooter();
	float GetRightShooter();

	void ResetRollerStatus();


	/******************************
	 * Shooter specifics
	 ******************************/

	/*
	 * rolls the ball into the shooter if the shooter PID is on target (if the shooter is up to speed)
	 */
	void Shoot();

	/*
	 * Shooter status function
	 */
	ShooterStatus GetShooterStatus();


};


#endif /* SRC_INTAKE_H_ */
