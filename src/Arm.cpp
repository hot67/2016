#include <Arm.h>


Arm::Arm(HotBot* bot) : HotSubsystem(bot, "Arm") {


/*
 * Super simple init code. see Arm.h for constant declarations
 */
	m_armLeftTalon = new CANTalon(ARM_ID_LEFT);
	m_armRightTalon = new CANTalon(ARM_ID_RIGHT);

	m_screwRightTalon = new CANTalon(SCREW_DRIVE_ID_RIGHT);
	m_screwLeftTalon = new CANTalon(SCREW_DRIVE_ID_LEFT);

	m_screwEncoder = new Encoder(ENCODER_CHANNEL1_SCREW,ENCODER_CHANNEL2_SCREW);
	m_armEncoder = new Encoder(ENCODER_CHANNEL1_ARM,ENCODER_CHANNEL2_ARM);

	m_armEncoder->SetDistancePerPulse(1); //360 pulses per revolution
	// ToDo: SetDistancePerPulse for screw


	m_armPIDController = new PIDController(ARM_P,ARM_I,ARM_D,m_armEncoder,m_armLeftTalon);

	m_screwPIDController = new PIDController(SCREW_P,SCREW_I,SCREW_D,m_screwEncoder,m_screwLeftTalon);
/*
 * Slave the right motors to the left ones which will be controlled by PIDs and Teleop
 */
	m_screwRightTalon->SetControlMode(CANSpeedController::kFollower); //Slavery is good
	m_screwRightTalon->Set(SCREW_DRIVE_ID_LEFT);
	m_screwRightTalon->SetClosedLoopOutputDirection(true); //Maybe invert???

	m_armRightTalon->SetControlMode(CANSpeedController::kFollower); // See Above
	m_armRightTalon->Set(ARM_ID_LEFT);
	m_armRightTalon->SetClosedLoopOutputDirection(true); //Invert direction of this motor, as it will be facing the other direction


}

Arm::~Arm() {
	// TODO Auto-generated destructor stub // i thinks me will leave this heres
}

void Arm::SetArm(float speed) {
	m_armLeftTalon->Set(speed);
}

void Arm::SetExtend(float speed) {
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
	case kClimb: //Climb position
		m_armPIDController->SetSetpoint(CLIMB_ARM);
	}


}

void Arm::SetScrewPIDPoint(ScrewSetPoint point) {


	switch (point) {
	case kClimb: //Climb position
		m_screwPIDController->SetSetpoint(CLIMB_SCREW);
		break;
	case kRetractScrew: //Retract the screw
		m_screwPIDController->SetSetpoint(RETRACT_SCREW);
		break;
	}


}

bool Arm::ArmAtSetPoint() { //If arm is at the given set point
	return m_armPIDController->OnTarget();
}

bool Arm::ScrewAtSetPoint() { //If screw is at the given set point
	return m_screwPIDController->OnTarget();
}

float Arm::RC(float degrees) {return ( (degrees/180) * 3.14159265358979323846);} //Radian Convertifier
