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
#define SHOOTER_SPEED_P 0.0
#define SHOOTER_SPEED_I 0.0
#define SHOOTER_SPEED_D 0.0
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

class Intake: public HotSubsystem {
private:
	CANTalon* m_rollerTalon;
	CANTalon* m_shooterTalon;

	Encoder* m_shooterEncoder;

	PIDController* m_shooterSpeedPID;

	//sets default speed of roller to 0.8
	float DEFAULT_SHOOTER_SPEED = 0.8;

	//sets minimum shooter speed to 95% of the speed ordered by the talon
	float MINIMUM_SHOOTER_SPEED = ((0.95 * m_shooterTalon->Get()));

public:
	Intake(HotBot* bot);
	virtual ~Intake();

	void SetRoller(float speed); //set roller speed
	void SetShooter(float speed); //set shooter speed

	void SetShooterDefault(); //sets default shooter speed

	void Shoot(); //if shooter speed is at 95% of what the shooter talon asks it to be, then roll in
};


#endif /* SRC_INTAKE_H_ */
