#include <Arm.h>



Arm::Arm(HotBot* bot) : HotSubsystem(bot, "Arm") { //A robot


/*
 * Super simple init code. see Arm.h for constant declarations
 */
	m_armRightTalon = new CANTalon(TALON_ARM_L); //Arm Right Talon
	m_armLeftTalon = new CANTalon(TALON_ARM_R); //Arm Left Talon

	m_screwRightTalon = new CANTalon(TALON_SCREW_R); //Screw Right Talon
	m_screwLeftTalon = new CANTalon(TALON_SCREW_L); //Screw Left Talon


	//m_screwEncoder = new Encoder(ENCODER_SCREW1,ENCODER_SCREW2); //Screw Encoder REMOVED FOR NOW
	//m_armEncoder = new Encoder(ENCODER_ARM1,ENCODER_ARM2); //Arm Encoder REMOVED FOR NOW

	//m_armEncoder->SetDistancePerPulse(ARM_ENCODER_PULSE_PER_REVOLUTION); //360 pulses per revolution REMOVED FOR NOW
	//m_screwEncoder->SetDistancePerPulse(SCREW_ENCODER_PULSE_PER_REVOLUTION); //Same REMOVED FOR NOW


	//m_armPIDController = new PIDController(ARM_P,ARM_I,ARM_D,m_armEncoder,m_armLeftTalon); //Arm PID Controller REMOVED FOR NOW

	m_armLeftTalon->SetFeedbackDevice(CANTalon::QuadEncoder); //using a digital encoder.
	m_screwLeftTalon->SetFeedbackDevice(CANTalon::QuadEncoder); //using a digital encoder.

	m_armLeftTalon->ConfigEncoderCodesPerRev(360);
	m_screwLeftTalon->ConfigEncoderCodesPerRev(360);

/*	m_armLeftTalon->SetP(ARM_P); //set the p, i and d
	m_armLeftTalon->SetI(ARM_I);
	m_armLeftTalon->SetD(ARM_D);


	m_screwLeftTalon->SetP(SCREW_P); //set the p, i and d
	m_screwLeftTalon->SetI(SCREW_I);
	m_screwLeftTalon->SetD(SCREW_D);


	//m_screwPIDController = new PIDController(SCREW_P,SCREW_I,SCREW_D,m_screwEncoder,m_screwLeftTalon); //Screw PID Controller REMOVED FOR NOW */


/*
 * Slave the right motors to the left ones which will be controlled by PIDs and Teleop
 */


	m_screwRightTalon->SetControlMode(CANSpeedController::kFollower); //Slave the right motor to the left
	m_screwRightTalon->Set(TALON_SCREW_L);
	m_screwRightTalon->SetClosedLoopOutputDirection(true); //Maybe invert??? (we don't know yet)

	// m_armRightTalon->SetControlMode(CANSpeedController::kFollower); //Slave the right motor to the left
	// m_armRightTalon->Set(TALON_ARM_L);
	// m_armRightTalon->SetClosedLoopOutputDirection(true); //Invert direction of this motor, as it will be facing the other direction


}




Arm::~Arm() {
	//Empty destructor. Also probably not used
}




void Arm::SetArm(float speed) {
	m_armLeftTalon->Set(-speed);
	m_armRightTalon->Set(speed);
}




void Arm::SetScrew(float speed) {
	m_screwLeftTalon->Set(speed);
}




void Arm::EnableArmPID() {
	//m_armPIDController->Enable(); REMOVED FOR NOW
//	m_armLeftTalon->Enable(); //i think this is how we enable pid controllers.
}



void Arm::DisableArmPID() {
	//m_armPIDController->Disable(); REMOVED FOR NOW
	//m_armLeftTalon->Disable(); //disable the pidcontroller
}




void Arm::EnableScrewPID() {
	//m_screwPIDController->Enable(); REMOVED FOR NOW
//	m_screwLeftTalon->Enable();
}




void Arm::DisableScrewPID() {
	//m_screwPIDController->Disable(); REMOVED FOR NOW
//	m_screwLeftTalon->Disable();
}




void Arm::SetArmPIDPoint(ArmSetPoint setpoint) {


	switch (setpoint) {
	case kFarHighGoal: //Far away high goal
		//m_armPIDController->SetSetpoint(FAR_HIGH_GOAL); REMOVED FOR NOW
		m_armLeftTalon->SetSetpoint(FAR_HIGH_GOAL);
		break;
	case kMediumLowGoal: //Medium away low goal
		//m_armPIDController->SetSetpoint(MEDIUM_LOW_GOAL); REMOVED FOR NOW
		m_armLeftTalon->SetSetpoint(MEDIUM_LOW_GOAL);
		break;
	case kCloseHighGoal: //Close high goal
		//m_armPIDController->SetSetpoint(CLOSE_HIGH_GOAL); REMOVED FOR NOW
		m_armLeftTalon->SetSetpoint(CLOSE_HIGH_GOAL);
		break;
	case kCarry: //Carry position
		//m_armPIDController->SetSetpoint(CARRY); REMOVED FOR NOW
		m_armLeftTalon->SetSetpoint(CARRY);
		break;
	case kCloseLowGoal: //Close low goal
		//m_armPIDController->SetSetpoint(CLOSE_LOW_GOAL); REMOVED FOR NOW
		m_armLeftTalon->SetSetpoint(CLOSE_LOW_GOAL);
		break;
	case kPickup: //Pickup position
		//m_armPIDController->SetSetpoint(PICKUP); REMOVED FOR NOW
		m_armLeftTalon->SetSetpoint(PICKUP);
		break;
	case kObstacle: //Obstacle self-lift position
		//m_armPIDController->SetSetpoint(OBSTACLE); REMOVED FOR NOW
		m_armLeftTalon->SetSetpoint(OBSTACLE);
		break;
	case kClimbArm: //Climb position
		//m_armPIDController->SetSetpoint(CLIMB_ARM); REMOVED FOR NOW
		m_armLeftTalon->SetSetpoint(CLIMB_ARM);
		break;
	case kResetArm: //put it back in the starting position
		//m_armPIDController->SetSetpoint(0); REMOVED FOR NOW
		m_armLeftTalon->SetSetpoint(0);
	}


}




void Arm::SetScrewPIDPoint(ScrewSetPoint point) {


	switch (point) {
	case kClimbScrew: //Climb position
		//m_screwPIDController->SetSetpoint(CLIMB_SCREW); REMOVED FOR NOW
		m_screwLeftTalon->SetSetpoint(CLIMB_SCREW);
		break;
	case kRetractScrew: //Retract the screw
		//m_screwPIDController->SetSetpoint(RETRACT_SCREW); REMOVED FOR NOW
		m_screwLeftTalon->SetSetpoint(RETRACT_SCREW);
		break;
	case kResetScrew: //put it back in the starting position
		//m_screwPIDController->SetSetpoint(0); REMOVED FOR NOW
		m_screwLeftTalon->SetSetpoint(0);
	}


}




float Arm::GetArmEncoderRate() {
	//return m_armEncoder->GetRate(); REMOVED FOR NOW
	return m_armLeftTalon->GetSpeed(); //Might have to be divided by four. returns the encoder rate.
}




float Arm::GetScrewEncoderRate() {
	//return m_screwEncoder->GetRate(); REMOVED FOR NOW
	return m_screwLeftTalon->GetSpeed();
}




void Arm::ZeroArmEncoder() {
	//m_armEncoder->Reset(); REMOVED FOR NOW
	m_armLeftTalon->SetPosition(0);
}




void Arm::ZeroScrewEncoder() {
	//m_screwEncoder->Reset(); REMOVED FOR NOW
	m_screwLeftTalon->SetPosition(0);
}




float Arm::GetArmSetPoint() {
	//return m_armPIDController->GetSetpoint(); //returns setpoint REMOVED FOR NOW
	return m_armLeftTalon->GetSetpoint();
}




float Arm::GetScrewSetPoint() {
	//return m_screwPIDController->GetSetpoint(); //returns setpoint REMOVED FOR NOW
	return m_screwLeftTalon->GetSetpoint();
}


float Arm::GetArmRate() {
	return m_screwLeftTalon->GetSpeed();
}

float Arm::GetScrewPos() {
	//return m_screwEncoder->Get(); //returns the current encoder value REMOVED FOR NOW
	return m_screwLeftTalon->GetPosition(); //may need to be divided by four.
}




float Arm::GetArmPos() {
	//return m_armEncoder->Get(); //returns the current encoder value REMOVED FOR NOW
	return m_screwLeftTalon->GetPosition(); //may need to be divided by four
}




bool Arm::ArmAtSetPoint() { //If arm is at the given set point
	//return m_armPIDController->OnTarget(); REMOVED FOR NOW
/*	int error = m_armLeftTalon->GetClosedLoopError();
	switch (error) { //check the error
	case 0: //if it's zero, we are there
		return true;
	default:
		return false;
	} */


}




bool Arm::ScrewAtSetPoint() { //If screw is at the given set point
	//return m_screwPIDController->OnTarget(); REMOVED FOR NOW
	/* int error = m_armLeftTalon->GetClosedLoopError();
	switch (error) { //check the error
	case 0: //if it's zero, we are there
		return true;
	default:
		return false;
	} */
}



void Arm::ArmPrintData() {
	//SmartDashboard::PutNumber("Arm Encoder", m_armEncoder->GetDistance()); //Brandon told me to write these. idk what they do. REMOVED FOR NOW
	//SmartDashboard::PutNumber("Screw Encoder", m_screwEncoder->GetDistance()); REMOVED FOR NOW
}

void Arm::EnableArmMotionProfiling() {


	/* delete m_armTrajectoryPoints;
	delete m_armMotionProfile;
	float current_velocity = (m_armLeftTalon->GetSpeed/4)*10; //initial velocity in degrees per second
	float position = m_armLeftTalon/4; //position in degrees
	m_armTrajectoryPoints = new Trajectory(current_velocity, position, m_armTargetPos, ARM_MAX_V, ARM_MAX_A); //setup the trajectory class
	m_armMotionProfile = new MotionProfiling((*m_armTrajectoryPoints), m_armLeftTalon,ARM_DELTA_TIME); //setup the actual motion profiling
	m_armMotionProfile->BeginProfiling(); */
}

void Arm::SetArmMotionProfilePoint(float target) {
	m_armTargetPos = target; //temporary code, sets the profile target to target
}

void Arm::PeriodicArmTask() {
	/* m_armMotionProfile->Iterate(); //call this at about half the delta time. */
}


float Arm::RC(float degrees){return((degrees/180)*3.14159265358979323846);} //Radian Convertifier
