#include <Arm.h>

/*
 * The Arm Subsystem
 */
Arm::Arm(HotBot* bot) : HotSubsystem(bot, "Arm") { //A robot


	/*
	 * Setup the arm talons
	 */
	m_armRightTalon = new CANTalon(TALON_ARM_L);
	m_armLeftTalon = new CANTalon(TALON_ARM_R);

	/*
	 * Setup the screw talons
	 */
	m_screwRightTalon = new CANTalon(TALON_SCREW_R);
	m_screwLeftTalon = new CANTalon(TALON_SCREW_L);

	/*
	 * Initialize the light sensor for the arm.
	 */
	m_armLightSensor = new DigitalInput(LIGHT_ARM);

	/*
	 * Tell the talon it is using digital encoders. These counts willbe multiplied by four,
	 * so in later use we need to divide them by four.
	 */
	m_armLeftTalon->SetFeedbackDevice(CANTalon::QuadEncoder);
	m_screwLeftTalon->SetFeedbackDevice(CANTalon::QuadEncoder);

	/*
	 * Configure the encoder counts per revolution
	 */
	m_armLeftTalon->ConfigEncoderCodesPerRev(1636.363636);
	m_screwLeftTalon->ConfigEncoderCodesPerRev(360);

	/*
	 * Set P, I, and D
	 */
	m_armLeftTalon->SetP(ARM_P);
	m_armLeftTalon->SetI(ARM_I);
	m_armLeftTalon->SetD(ARM_D);
	m_screwLeftTalon->SetP(SCREW_P);
	m_screwLeftTalon->SetI(SCREW_I);
	m_screwLeftTalon->SetD(SCREW_D);

	/*
	 * Slave the right motors to the left ones which will be controlled by PIDs and Teleop
	 */
	m_screwRightTalon->SetControlMode(CANSpeedController::kFollower);
	m_screwRightTalon->Set(TALON_SCREW_L);

	/*
	* Slave the right motors to the left ones which will be controlled by PIDs and Teleop
	*/
	m_armRightTalon->SetControlMode(CANSpeedController::kFollower);
	m_armRightTalon->Set(TALON_ARM_L);
	m_armRightTalon->SetClosedLoopOutputDirection(true);

	/*
	 * Initialize the Motion Profiling Controllers
	 * We just need to give them a Talon.
	 */
	m_armMPController = new ArmMotionProfiling(m_armLeftTalon);
	m_screwMPController = new ArmMotionProfiling(m_screwLeftTalon);

}

/*
 * Set the Arm Speed
 */
void Arm::SetArm(float speed) {
	//m_armController->PIDWrite(speed);
	m_armLeftTalon->Set(speed);
}

/*
 * Set the Screw Speed
 */
void Arm::SetScrew(float speed) {
	m_screwLeftTalon->Set(speed);
}

/*
 * Enable control of the Arm PID.
 */
void Arm::EnableArmPID() {
	m_armLeftTalon->Enable();
}

/*
 * Disable control of the Arm PID
 */
void Arm::DisableArmPID() {
	m_armLeftTalon->Disable();
}

/*
 * Enable control of the Screw PID.
 */
void Arm::EnableScrewPID() {
	//m_screwPIDController->Enable(); REMOVED FOR NOW
	m_screwLeftTalon->Enable();
}

/*
 * Disable control of the Screw PID.
 */
void Arm::DisableScrewPID() {
	//m_screwPIDController->Disable(); REMOVED FOR NOW
	m_screwLeftTalon->Disable();
}

/*
 * Set Motion Profiling Point.
 */
void Arm::SetArmMPPoint(ArmSetPoint setpoint) {

	switch (setpoint) {
		case kFarHighGoal:

			/*
			 * Far Away High Goal
			 */
			m_armMPTargetPos = FAR_HIGH_GOAL-LIGHT_SENSOR_POS;
			break;
		case kMediumLowGoal:

			/*
			 * Medium Away Low Goal
			 */
			m_armMPTargetPos = MEDIUM_LOW_GOAL-LIGHT_SENSOR_POS;
			break;
		case kCloseHighGoal:

			/*
			 * Close High Goal
			 */
			m_armMPTargetPos = CLOSE_HIGH_GOAL-LIGHT_SENSOR_POS;
			break;
		case kCarry:

			/*
			 * Carry Position
			 */
			m_armMPTargetPos = CARRY-LIGHT_SENSOR_POS;
			break;
		case kCloseLowGoal:

			/*
			 * Close Low Goal
			 */
			m_armMPTargetPos = CLOSE_LOW_GOAL-LIGHT_SENSOR_POS;
			break;
		case kPickup:

			/*
			 * Pickup Position
			 */
			m_armMPTargetPos = PICKUP-LIGHT_SENSOR_POS;
			break;
		case kObstacle:

			/*
			 * Obstacle Self-Lift Position
			 */
			m_armMPTargetPos = OBSTACLE-LIGHT_SENSOR_POS;

			break;
		case kClimbArm:

			/*
			 * Climb Position
			 */
			m_armMPTargetPos = CLIMB_ARM-LIGHT_SENSOR_POS;
			break;
		case kResetArm:

			/*
			 * Back to Starting Position
			 */
			m_armMPTargetPos = -LIGHT_SENSOR_POS;
		}

}

/*
 * Set PID Point
 */
void Arm::SetArmPIDPoint(ArmSetPoint setpoint) {

	/*
	 * All of these adjust for the Light Sensor Position.
	 * Otherwise known as the zero position.
	 * so SetSetpoint(0) is not true 0, SetSetpoint(-LIGHT_SENSOR_POS) is true 0
	 */
	switch (setpoint) {
	case kFarHighGoal:

		/*
		 * Far Away High Goal
		 */
		m_armLeftTalon->SetSetpoint(FAR_HIGH_GOAL-LIGHT_SENSOR_POS);
		break;
	case kMediumLowGoal:

		/*
		 * Medium Away Low Goal
		 */
		m_armLeftTalon->SetSetpoint(MEDIUM_LOW_GOAL-LIGHT_SENSOR_POS);
		break;
	case kCloseHighGoal:

		/*
		 * Close High Goal
		 */
		m_armLeftTalon->SetSetpoint(CLOSE_HIGH_GOAL-LIGHT_SENSOR_POS);
		break;
	case kCarry:

		/*
		 * Carry Position
		 */
		m_armLeftTalon->SetSetpoint(CARRY-LIGHT_SENSOR_POS);
		break;
	case kCloseLowGoal:

		/*
		 * Close Low Goal
		 */
		m_armLeftTalon->SetSetpoint(CLOSE_LOW_GOAL-LIGHT_SENSOR_POS);
		break;
	case kPickup:

		/*
		 * Pickup Position
		 */
		m_armLeftTalon->SetSetpoint(PICKUP-LIGHT_SENSOR_POS);
		break;
	case kObstacle:

		/*
		 * Obstacle Self-Lift Position
		 */
		m_armLeftTalon->SetSetpoint(OBSTACLE-LIGHT_SENSOR_POS);
		break;
	case kClimbArm:

		/*
		 * Climb Position
		 */
		m_armLeftTalon->SetSetpoint(CLIMB_ARM-LIGHT_SENSOR_POS);
		break;
	case kResetArm:
		/*
		 * Back to the Starting Position
		 */
		m_armLeftTalon->SetSetpoint(-LIGHT_SENSOR_POS);
	}


}

void Arm::SetScrewPIDPoint(double setpoint) {

	/*
	 * Set the target, adjusting for gearbox ratios.
	 */
	m_screwLeftTalon->SetSetpoint(setpoint * 4 * 360);

}

void Arm::SetArmPIDPoint(double setpoint) {

	/*
	 * Set the target, adjusting for true 0.
	 */
	m_armLeftTalon->SetSetpoint(setpoint-LIGHT_SENSOR_POS);

}



void Arm::SetScrewPIDPoint(ScrewSetPoint point) {


	switch (point) {
	case kClimbScrew:

		/*
		 * Climb Position
		 */
		m_armLeftTalon->SetSetpoint(CLIMB_SCREW);
		break;
	case kRetractScrew:

		/*
		 * Retract the Screw
		 */
		m_armLeftTalon->SetSetpoint(RETRACT_SCREW);
		break;
	case kResetScrew:

		/*
		 * Back to the Starting Position
		 */
		m_armLeftTalon->SetSetpoint(0);

	}


}

void Arm::SetScrewMPPoint(ScrewSetPoint point) {


	switch (point) {
	case kClimbScrew:

		/*
		 * Climb Position
		 */
		m_screwMPTargetPos = CLIMB_SCREW;
		break;
	case kRetractScrew:

		/*
		 * Retract the Screw
		 */
		m_screwMPTargetPos = RETRACT_SCREW;
		break;
	case kResetScrew:

		/*
		 * Back to the Starting Position
		 */
		m_screwMPTargetPos = 0;

	}


}



/*
 * Sensors
 */
float Arm::GetArmEncoderRate() {
	return m_armLeftTalon->GetSpeed()/4;
}




float Arm::GetScrewEncoderRate() {
	return m_screwLeftTalon->GetSpeed();
}




void Arm::ZeroArmEncoder() {
	m_armLeftTalon->SetPosition(0);
}




void Arm::ZeroScrewEncoder() {
	m_screwLeftTalon->SetPosition(0);
}




float Arm::GetArmPIDSetPoint() {
	return m_armLeftTalon->GetSetpoint();
}




float Arm::GetScrewPIDSetPoint() {
	return m_screwLeftTalon->GetSetpoint();
}




float Arm::GetScrewPos() {
	return m_screwLeftTalon->GetPosition();
}



float Arm::GetArmPos() {
	return m_screwLeftTalon->GetPosition() - LIGHT_SENSOR_POS;
}



bool Arm::ArmAtPIDSetPoint() {
	int error = m_armLeftTalon->GetClosedLoopError();

	/*
	 * Check the error, how far we are from the
	 * destination.
	 */
	switch (error) {
	case 0:
		return true;
	default:
		return false;
	}


}




bool Arm::ScrewAtPIDSetPoint() { //If screw is at the given set point
	int error = m_armLeftTalon->GetClosedLoopError();
	/*
	 * Check the error, how far we are from the
	 * destination.
	 */
	switch (error) { //check the error
	case 0:
		return true;
	default:
		return false;
	}
}



void Arm::ArmPrintData() {

	/*
	 * Write to SmartDashboard
	 */
	SmartDashboard::PutNumber("Arm Encoder", m_armLeftTalon->GetPosition()/4);
	SmartDashboard::PutNumber("Screw Encoder", m_armLeftTalon->GetPosition()/4);
}

/*
 * Motion Profiling
 */
void Arm::EnableScrewMotionProfiling() {

	if (!(m_screwMPController->IsEnabled())) {

		float current_position = m_screwLeftTalon->GetPosition();
		float current_velocity = m_screwLeftTalon->GetSpeed();
		m_screwMPController->BeginProfiling(
				current_position,
				current_velocity,
				m_screwMPTargetPos,
				SCREW_MAX_V,
				SCREW_MAX_A,
				SCREW_DELTA_TIME);

	}

}



void Arm::EnableArmMotionProfiling() {

	if (!(m_armMPController->IsEnabled())) {

		float current_position = m_armLeftTalon->GetPosition();
		float current_velocity = m_armLeftTalon->GetSpeed();
		m_screwMPController->BeginProfiling( //start the moving!!
				current_position,
				current_velocity,
				m_armMPTargetPos,
				ARM_MAX_V,
				ARM_MAX_A,
				ARM_DELTA_TIME);

	}

}

void Arm::DisableScrewMotionProfiling() {

	if (m_screwMPController->IsEnabled()) {
		m_screwMPController->EndProfiling();
	}

}



void Arm::DisableArmMotionProfiling() {

	if (m_armMPController->IsEnabled()) {
		m_armMPController->EndProfiling();
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


/*
 * Motion Profiling, Set the point
 */
void Arm::SetArmMotionProfilePoint(float target) {
	m_armMPTargetPos = target;
}



void Arm::SetScrewMotionProfilePoint(float target) {
	m_screwMPTargetPos = target;
}



void Arm::PeriodicArmTask() { //call me every half delta time. probably 10 ms
	m_screwMPController->Iterate();
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
