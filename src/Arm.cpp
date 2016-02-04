#include <Arm.h>


Arm::Arm(HotBot* bot) : HotSubsystem(bot, "Arm") { //A robot


/*
 * Super simple init code. see Arm.h for constant declarations
 */
	m_armRightTalon = new CANTalon(ARM_ID_LEFT); //Arm Right Talon
	m_armLeftTalon = new CANTalon(ARM_ID_RIGHT); //Arm Left Talon

	m_screwRightTalon = new CANTalon(SCREW_DRIVE_ID_RIGHT); //Screw Right Talon
	m_screwLeftTalon = new CANTalon(SCREW_DRIVE_ID_LEFT); //Screw Left Talon

	m_screwEncoder = new Encoder(ENCODER_CHANNEL1_SCREW,ENCODER_CHANNEL2_SCREW); //Screw Encoder
	m_armEncoder = new Encoder(ENCODER_CHANNEL1_ARM,ENCODER_CHANNEL2_ARM); //Arm Encoder

	m_armEncoder->SetDistancePerPulse(1); //360 pulses per revolution
	m_screwEncoder->SetDistancePerPulse(1); //Same


	m_armPIDController = new PIDController(ARM_P,ARM_I,ARM_D,m_armEncoder,m_armLeftTalon); //Arm PID Controller


	m_screwPIDController = new PIDController(SCREW_P,SCREW_I,SCREW_D,m_screwEncoder,m_screwLeftTalon); //Screw PID Controller
/*
 * Slave the right motors to the left ones which will be controlled by PIDs and Teleop
 */
	m_screwRightTalon->SetControlMode(CANSpeedController::kFollower); //Slave the right motor to the left
	m_screwRightTalon->Set(SCREW_DRIVE_ID_LEFT);
	m_screwRightTalon->SetClosedLoopOutputDirection(true); //Maybe invert??? (we don't know yet)

	m_armRightTalon->SetControlMode(CANSpeedController::kFollower); //Slave the right motor to the left
	m_armRightTalon->Set(ARM_ID_LEFT);
	m_armRightTalon->SetClosedLoopOutputDirection(true); //Invert direction of this motor, as it will be facing the other direction


}

Arm::~Arm() {
	//Empty destructor. Also probably not used
}

void Arm::SetArm(float speed) {
	m_armLeftTalon->Set(speed);
}

void Arm::SetScrew(float speed) {
	m_screwLeftTalon->Set(speed);
}

void Arm::EnableArmPID() {
	m_armPIDController->Enable();
}

void Arm::DisableArmPID() {
	m_armPIDController->Disable();
}

void Arm::EnableScrewPID() {
	m_screwPIDController->Enable();
}

void Arm::DisableScrewPID() {
	m_screwPIDController->Disable();
}

void Arm::SetArmPIDPoint(ArmSetPoint setpoint) {


	switch (setpoint) {
	case kFarHighGoal: //Far away high goal
		m_armPIDController->SetSetpoint(FAR_HIGH_GOAL);
		break;
	case kMediumLowGoal: //Medium away low goal
		m_armPIDController->SetSetpoint(MEDIUM_LOW_GOAL);
		break;
	case kCloseHighGoal: //Close high goal
		m_armPIDController->SetSetpoint(CLOSE_HIGH_GOAL);
		break;
	case kCarry: //Carry position
		m_armPIDController->SetSetpoint(CARRY);
		break;
	case kCloseLowGoal: //Close low goal
		m_armPIDController->SetSetpoint(CLOSE_LOW_GOAL);
		break;
	case kPickup: //Pickup position
		m_armPIDController->SetSetpoint(PICKUP);
		break;
	case kObstacle: //Obstacle self-lift position
		m_armPIDController->SetSetpoint(OBSTACLE);
		break;
	case kClimbArm: //Climb position
		m_armPIDController->SetSetpoint(CLIMB_ARM);
	}


}

void Arm::SetScrewPIDPoint(ScrewSetPoint point) {


	switch (point) {
	case kClimbScrew: //Climb position
		m_screwPIDController->SetSetpoint(CLIMB_SCREW);
		break;
	case kRetractScrew: //Retract the screw
		m_screwPIDController->SetSetpoint(RETRACT_SCREW);
		break;
	}


}


float Arm::GetArmSetPoint() {
	return m_armPIDController->GetSetpoint(); //returns setpoint
}

float Arm::GetScrewSetPoint() {
	return m_screwPIDController->GetSetpoint(); //returns setpoint
}

bool Arm::ArmAtSetPoint() { //If arm is at the given set point
	return m_armPIDController->OnTarget();
}

bool Arm::ScrewAtSetPoint() { //If screw is at the given set point
	return m_screwPIDController->OnTarget();
}

float Arm::RC(float degrees){return((degrees/180)*3.14159265358979323846);} //Radian Convertifier
