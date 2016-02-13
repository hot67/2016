#include <Arm.h>




Arm::Arm(HotBot* bot) : HotSubsystem(bot, "Arm") { //A robot


/*
 * Super simple init code. see Arm.h for constant declarations
 */
	m_armRightTalon = new CANTalon(TALON_ARM_L); //Arm Right Talon
	m_armLeftTalon = new CANTalon(TALON_ARM_R); //Arm Left Talon

	m_screwRightTalon = new CANTalon(TALON_SCREW_R); //Screw Right Talon
	m_screwLeftTalon = new CANTalon(TALON_SCREW_L); //Screw Left Talon

	m_armLightSensor = new DigitalInput(LIGHT_ARM);


	//m_screwEncoder = new Encoder(ENCODER_SCREW1,ENCODER_SCREW2); //Screw Encoder REMOVED FOR NOW
	//m_armEncoder = new Encoder(ENCODER_ARM1,ENCODER_ARM2); //Arm Encoder REMOVED FOR NOW

	//m_armEncoder->SetDistancePerPulse(ARM_ENCODER_PULSE_PER_REVOLUTION); //360 pulses per revolution REMOVED FOR NOW
	//m_screwEncoder->SetDistancePerPulse(SCREW_ENCODER_PULSE_PER_REVOLUTION); //Same REMOVED FOR NOW


	//m_armPIDController = new PIDController(ARM_P,ARM_I,ARM_D,m_armEncoder,m_armLeftTalon); //Arm PID Controller REMOVED FOR NOW

	m_armLeftTalon->SetFeedbackDevice(CANTalon::QuadEncoder); //using a digital encoder.
	m_screwLeftTalon->SetFeedbackDevice(CANTalon::QuadEncoder); //using a digital encoder.


	m_armLeftTalon->ConfigEncoderCodesPerRev(.1); //Math might be off here?  We are setting how many times the encoder ticks per motor revolution.
	m_screwLeftTalon->ConfigEncoderCodesPerRev(360);

	m_armLeftTalon->SetP(ARM_P); //set the p, i and d
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

	m_armRightTalon->SetControlMode(CANSpeedController::kFollower); //Slave the right motor to the left
	m_armRightTalon->Set(TALON_ARM_L);
	m_armRightTalon->SetClosedLoopOutputDirection(true); //Invert direction of this motor, as it will be facing the other direction


	m_armController = new ARMPIDController(m_armLeftTalon, m_armRightTalon); //Initialize our output controller
	m_screwController = new ARMPIDController(m_screwLeftTalon, m_screwRightTalon); //Initialize the other one

	m_armMPController = new ArmMotionProfiling(m_armLeftTalon); //pass it ot the motion profile.

	m_screwMPController = new ArmMotionProfiling(m_screwLeftTalon); //pass it to the motion profile.

}




Arm::~Arm() {
	//Empty destructor. Also probably not used
}




void Arm::SetArm(float speed) {
	//m_armController->PIDWrite(speed);
	m_armLeftTalon->Set(speed);
}




void Arm::SetScrew(float speed) {
	//m_screwController->PIDWrite(speed);
	m_screwLeftTalon->Set(speed);
}




void Arm::EnableArmPID() {
	//m_armPIDController->Enable(); REMOVED FOR NOW
	m_armLeftTalon->Enable(); //i think this is how we enable pid controllers.
}



void Arm::DisableArmPID() {
	//m_armPIDController->Disable(); REMOVED FOR NOW
	m_armLeftTalon->Disable(); //disable the pidcontroller
}




void Arm::EnableScrewPID() {
	//m_screwPIDController->Enable(); REMOVED FOR NOW
	m_screwLeftTalon->Enable();
}




void Arm::DisableScrewPID() {
	//m_screwPIDController->Disable(); REMOVED FOR NOW
	m_screwLeftTalon->Disable();
}



void Arm::SetArmMPPoint(ArmSetPoint setpoint) {

	switch (setpoint) {
		case kFarHighGoal: //Far away high goal
			//m_armPIDController->SetSetpoint(FAR_HIGH_GOAL); REMOVED FOR NOW
			//m_armLeftTalon->SetSetpoint(FAR_HIGH_GOAL);
			m_armMPTargetPos = FAR_HIGH_GOAL;
			break;
		case kMediumLowGoal: //Medium away low goal
			//m_armPIDController->SetSetpoint(MEDIUM_LOW_GOAL); REMOVED FOR NOW
			//m_armLeftTalon->SetSetpoint(MEDIUM_LOW_GOAL);
			m_armMPTargetPos = MEDIUM_LOW_GOAL;
			break;
		case kCloseHighGoal: //Close high goal
			//m_armPIDController->SetSetpoint(CLOSE_HIGH_GOAL); REMOVED FOR NOW
			//m_armLeftTalon->SetSetpoint(CLOSE_HIGH_GOAL);
			m_armMPTargetPos = CLOSE_HIGH_GOAL;
			break;
		case kCarry: //Carry position
			//m_armPIDController->SetSetpoint(CARRY); REMOVED FOR NOW
			//m_armLeftTalon->SetSetpoint(CARRY);
			m_armMPTargetPos = CARRY;
			break;
		case kCloseLowGoal: //Close low goal
			//m_armPIDController->SetSetpoint(CLOSE_LOW_GOAL); REMOVED FOR NOW
			//m_armLeftTalon->SetSetpoint(CLOSE_LOW_GOAL);
			m_armMPTargetPos = CLOSE_LOW_GOAL;
			break;
		case kPickup: //Pickup position
			//m_armPIDController->SetSetpoint(PICKUP); REMOVED FOR NOW
			//m_armLeftTalon->SetSetpoint(PICKUP);
			m_armMPTargetPos = PICKUP;
			break;
		case kObstacle: //Obstacle self-lift position
			//m_armPIDController->SetSetpoint(OBSTACLE); REMOVED FOR NOW
			//m_armLeftTalon->SetSetpoint(OBSTACLE);
			m_armMPTargetPos = OBSTACLE;

			break;
		case kClimbArm: //Climb position
			//m_armPIDController->SetSetpoint(CLIMB_ARM); REMOVED FOR NOW
			//m_armLeftTalon->SetSetpoint(CLIMB_ARM);
			m_armMPTargetPos = CLIMB_ARM;
			break;
		case kResetArm: //put it back in the starting position
			//m_armPIDController->SetSetpoint(0); REMOVED FOR NOW
			//m_armLeftTalon->SetSetpoint(0);
			m_armMPTargetPos = 0;
		}

}


void Arm::SetArmPIDPoint(ArmSetPoint setpoint) { //CURRENTLY DOES MOTION PROFILING POINTS


	switch (setpoint) {
	case kFarHighGoal: //Far away high goal
		//m_armPIDController->SetSetpoint(FAR_HIGH_GOAL); REMOVED FOR NOW
		m_armLeftTalon->SetSetpoint(FAR_HIGH_GOAL);
		//m_armTargetPos = FAR_HIGH_GOAL;
		break;
	case kMediumLowGoal: //Medium away low goal
		//m_armPIDController->SetSetpoint(MEDIUM_LOW_GOAL); REMOVED FOR NOW
		m_armLeftTalon->SetSetpoint(MEDIUM_LOW_GOAL);
		//m_armTargetPos = MEDIUM_LOW_GOAL;
		break;
	case kCloseHighGoal: //Close high goal
		//m_armPIDController->SetSetpoint(CLOSE_HIGH_GOAL); REMOVED FOR NOW
		m_armLeftTalon->SetSetpoint(CLOSE_HIGH_GOAL);
		//m_armTargetPos = CLOSE_HIGH_GOAL;
		break;
	case kCarry: //Carry position
		//m_armPIDController->SetSetpoint(CARRY); REMOVED FOR NOW
		m_armLeftTalon->SetSetpoint(CARRY);
		//m_armTargetPos = CARRY;
		break;
	case kCloseLowGoal: //Close low goal
		//m_armPIDController->SetSetpoint(CLOSE_LOW_GOAL); REMOVED FOR NOW
		m_armLeftTalon->SetSetpoint(CLOSE_LOW_GOAL);
		//m_armTargetPos = CLOSE_LOW_GOAL;
		break;
	case kPickup: //Pickup position
		//m_armPIDController->SetSetpoint(PICKUP); REMOVED FOR NOW
		m_armLeftTalon->SetSetpoint(PICKUP);
		//m_armTargetPos = PICKUP;
		break;
	case kObstacle: //Obstacle self-lift position
		//m_armPIDController->SetSetpoint(OBSTACLE); REMOVED FOR NOW
		m_armLeftTalon->SetSetpoint(OBSTACLE);
		//m_armTargetPos = OBSTACLE;

		break;
	case kClimbArm: //Climb position
		//m_armPIDController->SetSetpoint(CLIMB_ARM); REMOVED FOR NOW
		m_armLeftTalon->SetSetpoint(CLIMB_ARM);
		//m_armTargetPos = CLIMB_ARM;
		break;
	case kResetArm: //put it back in the starting position
		//m_armPIDController->SetSetpoint(0); REMOVED FOR NOW
		m_armLeftTalon->SetSetpoint(0);
		//m_armTargetPos = 0;
	}


}




void Arm::SetScrewPIDPoint(ScrewSetPoint point) { //CURRENTLY DOES MOTION PROFILING POINTS


	switch (point) {
	case kClimbScrew: //Climb position
		//m_screwPIDController->SetSetpoint(CLIMB_SCREW); REMOVED FOR NOW
		m_armLeftTalon->SetSetpoint(CLIMB_SCREW);
		//m_screwTargetPos = CLIMB_SCREW;
		break;
	case kRetractScrew: //Retract the screw
		//m_screwPIDController->SetSetpoint(RETRACT_SCREW); REMOVED FOR NOW
		m_armLeftTalon->SetSetpoint(RETRACT_SCREW);
		//m_screwTargetPos = RETRACT_SCREW;
		break;
	case kResetScrew: //put it back in the starting position
		//m_screwPIDController->SetSetpoint(0); REMOVED FOR NOW
		m_armLeftTalon->SetSetpoint(0);
		//m_screwTargetPos = 0;

	}


}

void Arm::SetScrewMpPoint(ScrewSetPoint point) {


	switch (point) {
	case kClimbScrew: //Climb position
		//m_screwPIDController->SetSetpoint(CLIMB_SCREW); REMOVED FOR NOW
		//m_armLeftTalon->SetSetpoint(CLIMB_SCREW);
		m_screwMPTargetPos = CLIMB_SCREW;
		break;
	case kRetractScrew: //Retract the screw
		//m_screwPIDController->SetSetpoint(RETRACT_SCREW); REMOVED FOR NOW
		//m_armLeftTalon->SetSetpoint(RETRACT_SCREW);
		m_screwMPTargetPos = RETRACT_SCREW;
		break;
	case kResetScrew: //put it back in the starting position
		//m_screwPIDController->SetSetpoint(0); REMOVED FOR NOW
		//m_armLeftTalon->SetSetpoint(0);
		m_screwMPTargetPos = 0;

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




float Arm::GetArmPIDSetPoint() {
	//return m_armPIDController->GetSetpoint(); //returns setpoint REMOVED FOR NOW
	return m_armLeftTalon->GetSetpoint();
}




float Arm::GetScrewPIDSetPoint() {
	//return m_screwPIDController->GetSetpoint(); //returns setpoint REMOVED FOR NOW
	return m_screwLeftTalon->GetSetpoint();
}




float Arm::GetScrewPos() {
	//return m_screwEncoder->Get(); //returns the current encoder value REMOVED FOR NOW
	return m_screwLeftTalon->GetPosition(); //may need to be divided by four. Going to be in distance eventually
}




float Arm::GetArmPos() {
	//return m_armEncoder->Get(); //returns the current encoder value REMOVED FOR NOW
	return m_screwLeftTalon->GetPosition() - LIGHT_SENSOR_POS; //may need to be divided by four. adjusts for light sensor pos
}




bool Arm::ArmAtPIDSetPoint() { //If arm is at the given set point
	//return m_armPIDController->OnTarget(); REMOVED FOR NOW
	int error = m_armLeftTalon->GetClosedLoopError();
	switch (error) { //check the error
	case 0: //if it's zero, we are there
		return true;
	default:
		return false;
	}


}




bool Arm::ScrewAtPIDSetPoint() { //If screw is at the given set point
	//return m_screwPIDController->OnTarget(); REMOVED FOR NOW
	int error = m_armLeftTalon->GetClosedLoopError();
	switch (error) { //check the error
	case 0: //if it's zero, we are there
		return true;
	default:
		return false;
	}
}



void Arm::ArmPrintData() {
	//SmartDashboard::PutNumber("Arm Encoder", m_armEncoder->GetDistance()); //Brandon told me to write these. idk what they do. REMOVED FOR NOW
	//SmartDashboard::PutNumber("Screw Encoder", m_screwEncoder->GetDistance()); REMOVED FOR NOW
}



void Arm::EnableScrewMotionProfiling() {

	if (!(m_screwMPController->IsEnabled())) {

		float current_position = (m_screwLeftTalon->GetPosition()/4)*9.52; //position in units of the motor shaft.
		float current_velocity = (m_screwLeftTalon->GetSpeed()/4)*9.52;
		m_screwMPController->Generate(
				current_position,
				current_velocity,
				m_screwMPTargetPos,
				SCREW_MAX_V,
				SCREW_MAX_A,
				SCREW_DELTA_TIME);
		m_screwMPController->BeginProfiling();

	}

}



void Arm::EnableArmMotionProfiling() {

	if (!(m_armMPController->IsEnabled())) {

		float current_position = m_armLeftTalon->GetPosition();
		float current_velocity = m_armLeftTalon->GetSpeed();
		m_screwMPController->Generate(
				current_position,
				current_velocity,
				m_armMPTargetPos,
				ARM_MAX_V,
				ARM_MAX_A,
				ARM_DELTA_TIME);
		m_armMPController->BeginProfiling(); //Starts moving

	}

}

void Arm::DisableScrewMotionProfiling() {

	if (m_screwMPController->IsEnabled()) {
		m_screwMPController->EndProfiling(); //End movement.
	}

}



void Arm::DisableArmMotionProfiling() {

	if (m_armMPController->IsEnabled()) {
		m_armMPController->EndProfiling(); //End the motion profiling
	}

}



void Arm::PauseArmMotionProfiling() {

	m_armMPController->Pause();

}


void Arm::PauseScrewMotionProfiling() {

	m_screwMPController->Pause();

}


void Arm::ResumeScrewMotionProfiling() {

	m_screwMPController->UnPause();

}


void Arm::ResumeArmMotionProfiling() {

	m_armMPController->UnPause();

}



void Arm::SetArmMotionProfilePoint(float target) {
	m_armMPTargetPos = target; //temporary code, sets the profile target to target
}



void Arm::SetScrewMotionProfilePoint(float target) {
	m_screwMPTargetPos = target; //same
}



void Arm::PeriodicArmTask() {
	m_armMPController->Iterate(); //call this at about half the delta time.
}

void Arm::PeriodicScrewTask() {
	m_screwMPController->Iterate();
}

bool Arm::IsLightSensorTriggered() {

	return m_armLightSensor->Get();

}


Arm::ARMPIDController::ARMPIDController(CANTalon * talonLeft, CANTalon * talonRight) {

	m_talonLeft = talonLeft;
	m_talonRight = talonRight;

}
/*
void Arm::ARMPIDController::PIDWrite(float output) {

	m_talonLeft->Set(-output);
	m_talonRight->Set(output);

}


float Arm::RC(float degrees){return((degrees/180)*3.14159265358979323846);} //Radian Convertifier
*/
