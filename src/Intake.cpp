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

	DigitalInput *m_shooterLight = new DigitalInput(6);
	m_shooterEncoder = new Encoder(m_shooterLight, m_shooterLight, true);
	m_shooterEncoder->SetDistancePerPulse(1);
	//m_shooterEncoder->SetSamplesToAverage(127);

	//what is distance per pulse (ask jim/rodney)

	//seems like encoder but is actually white-black sensor

	m_shooterPIDWrapper = new ShooterPIDWrapper(this);

	m_shooterSpeedPID = new PIDController(SHOOTER_SPEED_P, SHOOTER_SPEED_I, SHOOTER_SPEED_D,
			m_shooterPIDWrapper, m_shooterPIDWrapper);
	m_shooterSpeedPID->SetPercentTolerance(0.05);

	m_pulseOutTimer = new Timer();
	f_rollingIn = false;
}

Intake::~Intake() {

}

/******************************
 * SENSORS
 ******************************/

/*
 * encoder picks each reflective thing however many times per rotation (defined as SHOOTER_PULSE_PER_ROTATION) and is then divided by shooter pulse per rotation
 */
double Intake::GetShooterSpeed(){
	return m_shooterEncoder->GetRate() * 60;
}

double Intake::GetShooterPeriod(){
	return m_shooterEncoder->GetPeriod();
}


/******************************
 * MOTORS
 ******************************/

void Intake::SetRoller(float speed){
	//negative values roll in
	//positive values roll out
	m_rollerTalon->Set(speed);
/*
	if (speed > 0.0) {
		SmartDashboard::PutNumber("Roller Get", speed);
//		m_rollerTalon->Set(speed);
		f_rollingIn = true;
	} else if (speed == 0.0) {
		if (f_rollingIn == true) {
			m_pulseOutTimer->Stop();
			m_pulseOutTimer->Reset();
			m_pulseOutTimer->Start();

			SmartDashboard::PutNumber("Roller Get", -1.0);
//			m_rollerTalon->Set(-1.0);
			f_rollingIn = false;
		}
		else if (f_rollingIn == false) {
			if (m_pulseOutTimer->Get() < 1.0) {
			} else {
				SmartDashboard::PutNumber("Roller Get", 0.0);
//				m_rollerTalon->Set(0.0);
			}
		}
	} else {
		SmartDashboard::PutNumber("Roller Get", speed);
//		m_rollerTalon->Set(speed);
		f_rollingIn = false;
	}
*/
}


void Intake::SetShooter(float speed){ //set speed of shooter
	//positive values roll out
	//negative values will destroy the robot

	m_shooterTalon->Set(speed);

	// we will never accidently destroy the robot
	//if there's a negative value, it won't run
}

float Intake::GetShooter(){
	return m_shooterTalon->Get();
}

/******************************
 * SHOOTER SPECIFICS
 ******************************/

void Intake::SetShooterDefault(){
	//default speed for shooting before changed by DPAD
	SetShooter(DEFAULT_SHOOTER_SPEED);
}

Intake::ShooterStatus Intake::GetShooterStatus() {
	if (GetShooter() == 0) {
		return ShooterStatus::kShooterStopped;
	}
	else if (ShooterAtSetPoint() == true) {
		return ShooterStatus::kShooterAtSpeed;
	}
	else {
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

void Intake::IncreaseShooterSpeed(){
	//updates desired shooter speed by a positive 0.01
	m_desiredShooterSpeed += 0.01;
}

void Intake::DecreaseShooterSpeed(){
	//updates desired shooter speed by a negative 0.01
	m_desiredShooterSpeed -= 0.01;
}

void Intake::SetDesiredShooterSpeed(){
	SetShooter(-m_desiredShooterSpeed);
}



/******************************
 * SHOOTER	PID
 ******************************/
void Intake::EnableShooterPID(){
	m_shooterSpeedPID->Enable();
}

void Intake::DisableShooterPID(){
	m_shooterSpeedPID->Disable();
}

bool Intake::IsShooterPIDEnabled(){
	return m_shooterSpeedPID->IsEnabled();
}

void Intake::SetShooterPIDSetPoint(float speed){
	m_shooterSpeedPID->SetSetpoint(speed);
}

double Intake::GetShooterPIDSetPoint(){
	return m_shooterSpeedPID->GetSetpoint();
}

bool Intake::ShooterAtSetPoint(){
	return m_shooterSpeedPID->OnTarget();
}
