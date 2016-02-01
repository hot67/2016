/*
 * Intake.cpp
 *
 *  Created on: Jan 31, 2016
 *      Author: Marlina
 */

#include <Intake.h>


Intake::Intake(HotBot* bot) : HotSubsystem(bot, "Intake") {

	m_rollerTalon = new CANTalon(ROLLER_ID);
	m_shooterTalon = new CANTalon(SHOOTER_ID);

	m_shooterEncoder = new Encoder(ENCODER_CHANNEL1, ENCODER_CHANNEL2); //will need to be changed to white-black sensor

}

Intake::~Intake() {

}

void Intake::SetRoller(float speed){
	//negative values roll in
	//positive values roll out
	m_rollerTalon->Set(speed);
}

void Intake::SetShooter(float speed){
	m_shooterTalon->Set(speed);
}

void Intake::SetShooterDefault(){
	//default speed for shooting before changed by DPAD

	m_shooterTalon->Set(DEFAULT_SHOOTER_SPEED);
}
