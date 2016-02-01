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
#endif

#ifdef COMPETITION_BOT
#endif

#define ROLLER_ID 13
#define SHOOTER_ID 16

#define DEFAULT_SHOOTER_SPEED 0.8

#define ENCODER_CHANNEL1 8 //will be adding in white-black sensor, but leaving encoder initializations for now
#define ENCODER_CHANNEL2 9

class Intake: public HotSubsystem {
private:
	CANTalon* m_rollerTalon;
	CANTalon* m_shooterTalon;

	Encoder* m_shooterEncoder;

public:
	Intake(HotBot* bot);
	virtual ~Intake();

	void SetRoller(float speed); //set roller speed
	void SetShooter(float speed); //set shooter speed

	void SetShooterDefault(); //sets default shooter speed

};


#endif /* SRC_INTAKE_H_ */
