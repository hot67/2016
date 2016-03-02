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
	m_armLeftTalon->ConfigEncoderCodesPerRev(360.0);
	m_screwLeftTalon->ConfigEncoderCodesPerRev(1024.0);

	/*
	 * Set P, I, and D, in the Controllers using our wrapper
	 */
	m_armPIDWrapper = new ArmPIDWrapper(this);
	m_screwPIDWrapper = new ScrewPIDWrapper(this);

	m_armPIDController = new PIDController(ARM_UP_P, ARM_UP_I, ARM_UP_D, m_armPIDWrapper, m_armPIDWrapper);
	m_armPIDController->SetAbsoluteTolerance(1);


	m_screwPIDController = new PIDController(SCREW_P, SCREW_I, SCREW_D, m_screwPIDWrapper, m_screwPIDWrapper);

	/*
	 * Initialize the Motion Profiling Controllers
	 * We just need to give them a Talon.
	 */
	m_armMPTargetPos = 0;
	m_screwMPTargetPos = 0;
	m_armMPController = new ArmMotionProfiling(m_armLeftTalon);
	m_screwMPController = new ArmMotionProfiling(m_screwLeftTalon);

}

/*
 * No overextending of Talons!
 *
 * Won't work bc if the arm position meets the requirement, we would be setting arm twice in the same loop
 * meaning that we would have two things controlling the same talon at that point.
 *
 */
/* void Arm::PeriodicTask() {

	int currentArmPos = GetArmPos();
	if (currentArmPos <= -15) {
		DisableArmMotionProfiling();
		DisableArmPID();
		SetArm(0);
	}
	else if (currentArmPos >= 98) {
		DisableArmMotionProfiling();
		DisableArmPID();
		SetArm(0);
	}
	int currentScrewPos = GetScrewPos();
	if (currentScrewPos <= 0) {
		DisableScrewMotionProfiling();
		DisableArmPID();
		SetScrew(0);
	}
	else if (currentScrewPos >= 37440) {
		DisableScrewMotionProfiling();
		DisableArmPID();
		SetScrew(0);
	}

} */


void Arm::SetArm(float speed) {
	m_armLeftTalon->Set(-speed);
	m_armRightTalon->Set(speed);
}

void Arm::SetScrew(float speed) {

	/*
	 * go up when screwpos is less than 30
	 *
	 * go down when screwpos is more than 0
	 */

	if (GetScrewPos() < 22.13 && speed < 0) {
		//going up and screw position is less than 30
		m_screwLeftTalon->Set(-speed);
		m_screwRightTalon->Set(-speed);
	}

	else if (GetScrewPos() > 0 && speed > 0) {
		m_screwLeftTalon->Set(-speed);
		m_screwRightTalon->Set(-speed);
	}
	else {
		m_screwLeftTalon->Set(0);
		m_screwRightTalon->Set(0);
	}
}

float Arm::GetArmPos() {
	return - m_armLeftTalon->GetPosition() * 79.2;
}

float Arm::GetScrewPos() {
	return - m_screwLeftTalon->GetPosition() / 4;
}

float Arm::GetArmSpeed() {
	return - m_armLeftTalon->GetSpeed() * 79.2;
}

float Arm::GetScrewSpeed() {
	return - m_screwLeftTalon->GetSpeed() / 4;
}



void Arm::ZeroArmEncoder() {
	m_armLeftTalon->SetPosition(LIGHT_SENSOR_POS / 79.2);
}

void Arm::ZeroScrewEncoder() {
	m_screwLeftTalon->SetPosition(0);
}

bool Arm::IsLightSensorTriggered() {
	return m_armLightSensor->Get();
}


void Arm::ArmPrintData() {

	/*
	 * Write to SmartDashboard
	 */
	SmartDashboard::PutNumber("Arm Encoder", GetArmPos());
	SmartDashboard::PutNumber("Screw Encoder", GetArmPos());
}


Arm::ArmPIDWrapper::ArmPIDWrapper(Arm *arm) {
	m_arm = arm;
}

void Arm::ArmPIDWrapper::PIDWrite(float output) {
	m_arm->SetArm(-output);
}

double Arm::ArmPIDWrapper::PIDGet() {
	return m_arm->GetArmPos();
}

/*
 * Enable control of the Arm PID.
 */
void Arm::EnableArmPID() {
	if (!m_armPIDController->IsEnabled()) {
		m_armPIDController->Enable();
	}
}

/*
 * Disable control of the Arm PID
 */
void Arm::DisableArmPID() {
	if (m_armPIDController->IsEnabled()) {
		m_armPIDController->Disable();
	}
}

/*
 * Is Arm Control Enabled?
 */
bool Arm::IsArmPIDEnabled() {
	return m_armPIDController->IsEnabled();
}

/*
 * PID Update
 * because gravity is bringing the arm down too hard
 */

void Arm::ArmPIDUpdate() {

	//to decrease PID coming down so it doesn't slam

	if (GetArmSpeed() > 0) {
		//if arm is going up
		m_armPIDController->SetPID(ARM_UP_P, ARM_UP_I, ARM_UP_D);
	}
	else {
		m_armPIDController->SetPID(ARM_DOWN_P, ARM_DOWN_I, ARM_DOWN_D);
	}
}

/*
 * Set PID Point
 */
void Arm::SetArmPIDPoint(ArmSetPoint setpoint) {

	/*
	 * Setting ArmPIDPoint with enums though we don't always use the enums
	 */
	switch (setpoint) {
	case kFarHighGoal:

		/*
		 * Far Away High Goal
		 */
		m_armPIDController->SetSetpoint(FAR_HIGH_GOAL);
		break;
	case kMediumLowGoal:

		/*
		 * Medium Away Low Goal
		 */
		m_armPIDController->SetSetpoint(MEDIUM_HIGH_GOAL);
		break;
	case kCloseHighGoal:

		/*
		 * Close High Goal
		 */
		m_armPIDController->SetSetpoint(CLOSE_HIGH_GOAL);
		break;
	case kCarry:

		/*
		 * Carry Position
		 */
		m_armPIDController->SetSetpoint(CARRY);
		break;
	case kCloseLowGoal:

		/*
		 * Close Low Goal
		 */
		m_armPIDController->SetSetpoint(CLOSE_LOW_GOAL);
		break;
	case kPickup:

		/*
		 * Pickup Position
		 */
		m_armPIDController->SetSetpoint(PICKUP);
		break;
	case kObstacle:

		/*
		 * Obstacle Self-Lift Position
		 */
		m_armPIDController->SetSetpoint(OBSTACLE);
		break;
	case kClimbArm:

		/*
		 * Climb Position
		 */
		m_armPIDController->SetSetpoint(CLIMB_ARM);
		break;
	case kBatter:
		m_armPIDController->SetSetpoint(BATTER_HIGH_GOAL);
		break;
	}
}


void Arm::SetArmPIDPoint(double setpoint) {

	/*
	 * Set the target, adjusting for true 0.
	 */
	m_armPIDController->SetSetpoint(setpoint);

}


float Arm::GetArmPIDSetPoint() {
	return m_armPIDController->GetSetpoint();
}


Arm::ScrewPIDWrapper::ScrewPIDWrapper(Arm *arm) {
	m_arm = arm;
}

void Arm::ScrewPIDWrapper::PIDWrite(float output) {
	m_arm->SetScrew(-output);

	SmartDashboard::PutNumber("ScrewPID PIDWrite output Value", output);
}

double Arm::ScrewPIDWrapper::PIDGet() {
	return m_arm->GetScrewPos();
}

/*
 * Enable control of the Screw PID.
 */
void Arm::EnableScrewPID() {
	if (!m_screwPIDController->IsEnabled()) {
		m_screwPIDController->Enable();
	}
}


/*
 * Disable control of the Screw PID.
 */
void Arm::DisableScrewPID() {
	if (m_screwPIDController->IsEnabled()) {
		m_screwPIDController->Disable();
	}
}



bool Arm::IsScrewPIDEnabled() {

	return m_armPIDController->IsEnabled();

}


void Arm::SetScrewPIDPoint(ScrewSetPoint point) {


	switch (point) {
	case kClimbScrew:

		/*
		 * Climb Position
		 */
		m_screwPIDController->SetSetpoint(CLIMB_SCREW);
		break;
	case kRetractScrew:

		/*
		 * Retract the Screw
		 */
		m_screwPIDController->SetSetpoint(RETRACT_SCREW);
		break;
	case kResetScrew:

		/*
		 * Back to the Starting Position
		 */
		m_screwPIDController->SetSetpoint(0);

	}


}


void Arm::SetScrewPIDPoint(double setpoint) {

	/*
	 * Set the target, adjusting for gearbox ratios.
	 */
	m_screwPIDController->SetSetpoint(setpoint);

}


float Arm::GetScrewPIDSetPoint() {
	return m_screwPIDController->GetSetpoint();
}


bool Arm::ArmAtPIDSetPoint() {
	int error = m_armPIDController->GetError();

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
	int error = m_screwPIDController->GetError();
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
