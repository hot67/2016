/*
 * Intake.cpp
 *
 *  Created on: Jan 31, 2016
 *      Author: Marlina
 */

#include <Intake.h>


Intake::Intake(HotBot* bot) : HotSubsystem(bot, "Intake") {

	m_rollerTalon = new CANTalon(ROLLER_ID);

	m_lShooterTalon = new CANTalon(LEFT_SHOOTER_ID);
	m_rShooterTalon = new CANTalon(RIGHT_SHOOTER_ID);

	DigitalInput *m_shooterLight = new DigitalInput(6);
	m_shooterEncoder = new Encoder(m_shooterLight, m_shooterLight, true);
	m_shooterEncoder->SetDistancePerPulse(1);
	//m_shooterEncoder->SetSamplesToAverage(127);

	//what is distance per pulse (ask jim/rodney)

	//seems like encoder but is actually white-black sensor

	m_pulseOutTimer = new Timer();
	m_shootingTimer = new Timer();

	f_rollingIn = false;
}

Intake::~Intake() {

}

/******************************
 * MOTORS
 ******************************/

void Intake::SetRoller(float speed){
	//negative values roll in
	//positive values roll out
//	m_rollerTalon->Set(speed);

	SmartDashboard::PutNumber("Roller Get", speed);
	if (speed > 0.0) {
		SmartDashboard::PutNumber("Roller Get", speed);
		m_rollerTalon->Set(speed);
		f_rollingIn = true;
	} else if (speed == 0.0) {
		if (f_rollingIn == true) {
			m_pulseOutTimer->Stop();
			m_pulseOutTimer->Reset();
			m_pulseOutTimer->Start();

			SmartDashboard::PutNumber("Roller Get", -1.0);
			m_rollerTalon->Set(-0.6);
			f_rollingIn = false;
		}
		else if (f_rollingIn == false) {
			if (m_pulseOutTimer->Get() < 0.1) {
			} else {
				SmartDashboard::PutNumber("Roller Get", 0.0);
				m_rollerTalon->Set(0.0);
			}
		}
	} else {
		SmartDashboard::PutNumber("Roller Get", speed);
		m_rollerTalon->Set(speed);
		f_rollingIn = false;
	}
}

void Intake::ResetRollerStatus() {
	f_rollingIn = false;
}

void Intake::SetShooter(float speed){ //set speed of shooter
	float time = m_shootingTimer->Get();
	m_shootingTimer->Stop();
	m_shootingTimer->Reset();
	m_shootingTimer->Start();

	if ((speed - m_shooterPrev) / time > 1.0) {
		m_shooterPrev = m_shooterPrev + time;
	} else if ((speed - m_shooterPrev) / time > -1.0) {
		m_shooterPrev = speed;
	} else {
		m_shooterPrev = m_shooterPrev - time;
	}

	m_lShooterTalon->Set(m_shooterPrev);
	m_rShooterTalon->Set(-m_shooterPrev);

	/*
	//positive values roll out
	//negative values will destroy the robot

	SmartDashboard::PutNumber("* ShootingTimer Get", m_shootingTimer->Get());
	if (speed == 0) {
		m_shootingTimer->Stop();
		m_shootingTimer->Reset();
		m_shootingTimer->Start();
	}

	if (m_shootingTimer->Get() < 1.0) {

		//y-int is 0.5, m is 0.5, time is x, speed is
		m_lShooterTalon->Set((0.5 + 0.5*m_shootingTimer->Get())*(speed));
		m_rShooterTalon->Set((0.5 + 0.5*m_shootingTimer->Get())*(-speed));
	}
	else {
		m_lShooterTalon->Set(speed);
		m_rShooterTalon->Set(-speed);
	}

	// we will never accidently destroy the robot
	//if there's a negative value, it won't run
	 */
}

float Intake::GetLeftShooter(){
	return m_lShooterTalon->Get();
}

float Intake::GetRightShooter(){
	return m_rShooterTalon->Get();
}

/******************************
 * SHOOTER SPECIFICS
 ******************************/


Intake::ShooterStatus Intake::GetShooterStatus() {
	if (GetLeftShooter() == 0 && GetRightShooter() == 0) {
		return ShooterStatus::kShooterStopped;
	}else {
		return ShooterStatus::kShooterAtSpeed;
		//return ShooterStatus::kShooterSpeeding;
	}
}

void Intake::Shoot(){
	//if minimum shooter speed is 95% of the speed that the talon is saying, then roll in
	if (m_shooterSpeedPID->OnTarget()) {
		SetRoller(1.0);
	}
}
