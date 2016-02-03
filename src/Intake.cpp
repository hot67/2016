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

	m_shooterEncoder = new Encoder(ENCODER_CHANNEL1, ENCODER_CHANNEL1, true);
	//seems like encoder but is actually white-black sensor

	m_shooterSpeedPID = new PIDController(SHOOTER_SPEED_P, SHOOTER_SPEED_I, SHOOTER_SPEED_D, m_shooterEncoder, m_shooterTalon);
}

Intake::~Intake() {

}

void Intake::SetRoller(float speed){
	//negative values roll in
	//positive values roll out
	m_rollerTalon->Set(speed);
}

void Intake::SetShooter(float speed){ //set speed of shooter
	m_shooterTalon->Set(speed);
}

void Intake::SetShooterDefault(){
	//default speed for shooting before changed by DPAD
	m_shooterTalon->Set(DEFAULT_SHOOTER_SPEED);
}

void Intake::Shoot(){
	//if minimum shooter speed is 95% of the speed that the talon is saying, then roll in
	if (m_shooterEncoder->GetRate() > MINIMUM_SHOOTER_SPEED)
		m_rollerTalon->Set(-0.3);
	//in main cpp, this must go with 'if arm is 95% to setpoint'
}
