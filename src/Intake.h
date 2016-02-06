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
#define SHOOTER_SPEED_P 0.0 //code from 2012 was 0.01
#define SHOOTER_SPEED_I 0.0 //code from 2012 was 0.0
#define SHOOTER_SPEED_D 0.0 //code from 2012 was 0.05
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
#define ENCODER_CHANNEL1 8
//will be adding in white-black sensor, but leaving encoder initialization for now

#define DEFAULT_SHOOTER_SPEED 0.8

class Intake: public HotSubsystem {
private:
	CANTalon* m_rollerTalon;
	CANTalon* m_shooterTalon;

	Encoder* m_shooterEncoder;

	PIDController* m_shooterSpeedPID;

public:
	Intake(HotBot* bot);

	virtual ~Intake();

	float m_desiredShooterSpeed = DEFAULT_SHOOTER_SPEED; // sets initial shooter speed to default speed


	void SetRoller(float speed); //set roller speed
	void SetShooter(float speed); //set shooter speed

	void SetShooterDefault(); //sets default shooter speed

	void Shoot(); //if shooter speed is at 95% of what the shooter talon asks it to be, then roll in

	void IncreaseShooterSpeed(); // increase shooter speed by 0.01
	void DecreaseShooterSpeed(); // decrease shooter speed by 0.01

	float GetShooterSpeed();
};


#endif /* SRC_INTAKE_H_ */
