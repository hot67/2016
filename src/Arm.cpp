#include <Arm.h>



Arm::Arm(HotBot* bot) : HotSubsystem(bot, "Arm") {
	/**
	 * 	Initialize Arm Talons
	 */
	m_armRightTalon = new CANTalon(TALON_ARM_L); //Arm Right Talon
	m_armLeftTalon = new CANTalon(TALON_ARM_R); //Arm Left Talon

	/**
	 * 	Slave Right Motor To Left
	 */
	m_armRightTalon->SetControlMode(CANSpeedController::kFollower);
	m_armRightTalon->Set(TALON_ARM_L);
	m_armRightTalon->SetClosedLoopOutputDirection(true);

	/**
	 * 	Initialize Screw Talons
	 */
	m_screwRightTalon = new CANTalon(TALON_SCREW_R); //Screw Right Talon
	m_screwLeftTalon = new CANTalon(TALON_SCREW_L); //Screw Left Talon

	/**
	 * 	Slave Right Motor To Left
	 */
	m_screwRightTalon->SetControlMode(CANSpeedController::kFollower);
	m_screwRightTalon->Set(TALON_SCREW_L);
	m_screwRightTalon->SetClosedLoopOutputDirection(true);

	/**
	 * 	Encoders
	 */
	m_armEncoder = new Encoder(ENCODER_ARM1,ENCODER_ARM2);
	m_armEncoder->SetDistancePerPulse(ARM_ENCODER_DISTANCE_PER_PULSE);

	m_screwEncoder = new Encoder(ENCODER_SCREW1, ENCODER_SCREW2);
	m_screwEncoder->SetDistancePerPulse(SCREW_ENCODER_DISTANCE_PER_PULSE);

	/**
	 * 	Light Sensor
	 */
	m_armLightSensor = new DigitalInput(LIGHT_ARM);

	/**
	 * 	PID Controllers
	 */
	m_armPID = new PIDController(ARM_P, ARM_I, ARM_D, m_armEncoder, m_armLeftTalon);
	m_screwPID = new PIDController(SCREW_P, SCREW_I, SCREW_D, m_screwEncoder, m_screwLeftTalon);
}


Arm::~Arm() {
	//Empty destructor. Also probably not used
}


/**
 * 	Raw Access To Talons
 */
void Arm::SetArm(float speed) {
	/**
	 * 	We set only to left talon because right talon is slave to left
	 */
	m_armLeftTalon->Set(speed);
}


void Arm::SetScrew(float speed) {
	/**
	 * 	We set only to left talon because right talon is slave to left
	 */
	m_screwLeftTalon->Set(speed);
}

/**
 * 	Raw Access To Encoder
 */
double Arm::GetScrewPos() {
	return m_screwLeftTalon->GetPosition();
}

double Arm::GetArmPos() {
	return m_screwLeftTalon->GetPosition() + LIGHT_SENSOR_POS;
}


/**
 * 	Raw Access to Encoder Speed
 */
double Arm::GetArmEncoderRate() {
	return m_armLeftTalon->GetSpeed(); //Might have to be divided by four. returns the encoder rate.
}

double Arm::GetScrewEncoderRate() {
	return m_screwLeftTalon->GetSpeed();
}

/**
 * 	Reset Encoder
 */
void Arm::ZeroArmEncoder() {
	m_armEncoder->Reset();
}

void Arm::ZeroScrewEncoder() {
	m_screwEncoder->Reset();
}

/**
 * 	Raw access to light sensor
 */
bool Arm::IsLightSensorTriggered() {
	return m_armLightSensor->Get();
}

/******************************
 * 	Arm PID
 ******************************/
/**
 * 	Enable and Disable
 */
void Arm::EnableArmPID() {
	if (!IsArmPIDEnabled()) {
		m_armPID->Enable();
	}
}

void Arm::DisableArmPID() {
	if (IsArmPIDEnabled()) {
		m_armPID->Disable();
	}
}

/**
 * 	Is Enabled?
 */
bool Arm::IsArmPIDEnabled() {
	return m_armPID->IsEnabled();
}

/**
 * 	Set Arm PID Point
 */
void Arm::SetArmPIDSetPoint(double setpoint) {
	m_armPID->SetSetpoint(setpoint - LIGHT_SENSOR_POS);
}

/**
 * 	Set Pre-defined Set Point
 */
void Arm::SetArmPIDSetPoint(ArmSetPoint setpoint) {
	switch (setpoint) {
	case kFarHighGoal: //Far away high goal
		SetArmPIDSetPoint(FAR_HIGH_GOAL);
		break;
	case kMediumLowGoal: //Medium away low goal
		SetArmPIDSetPoint(MEDIUM_LOW_GOAL);
		break;
	case kCloseHighGoal: //Close high goal
		SetArmPIDSetPoint(CLOSE_HIGH_GOAL);
		break;
	case kCarry: //Carry position
		SetArmPIDSetPoint(CARRY);
		break;
	case kCloseLowGoal: //Close low goal
		SetArmPIDSetPoint(CLOSE_LOW_GOAL);
		break;
	case kPickup: //Pickup position
		SetArmPIDSetPoint(PICKUP);
		break;
	case kObstacle: //Obstacle self-lift position
		SetArmPIDSetPoint(OBSTACLE);
		break;
	case kClimbArm: //Climb position
		SetArmPIDSetPoint(CLIMB_ARM);
		break;
	case kResetArm: //put it back in the starting position
		SetArmPIDSetPoint(0);
	}
}

/**
 * 	What is current goal?
 */
double Arm::GetArmPIDSetPoint() {
	return m_armPID->GetSetpoint();
}

/**
 * 	Have arrived to the goal?
 */
bool Arm::ArmAtPIDSetPoint() {
	return m_armPID->OnTarget();
}

/******************************
 * 	Screw PID
 ******************************/
/**
 * 	Enable and Disable
 */
void Arm::EnableScrewPID() {
	if (!IsScrewPIDEnabled()) {
		m_screwPID->Enable();
	}
}

void Arm::DisableScrewPID() {
	if (IsScrewPIDEnabled()) {
		m_screwPID->Disable();
	}
}

/**
 * 	Is Enabled?
 */
bool Arm::IsScrewPIDEnabled() {
	return m_screwPID->IsEnabled();
}

/**
 * 	Set Set Point
 */
void Arm::SetScrewPIDPoint(double setpoint) {
	/**
	 * 	We multiply by 4 * 360 because one pulse of encoder corresponds with 1 degree of rotation and 1 rotation is 1/4 inches.
	 * 	Probably, change encoder distance per pulse in future.
	 */
	m_screwPID->SetSetpoint(setpoint * 4 * 360);
}

/**
 * 	Set Pre-defined Set Points
 */
void Arm::SetScrewPIDPoint(ScrewSetPoint point) { //CURRENTLY DOES MOTION PROFILING POINTS
	switch (point) {
	case kClimbScrew: //Climb position
		SetScrewPIDPoint(CLIMB_SCREW);
		break;
	case kRetractScrew: //Retract the screw
		SetScrewPIDPoint(RETRACT_SCREW);
		break;
	case kResetScrew: //put it back in the starting position
		SetScrewPIDPoint(0);
	}
}

/**
 * 	What is current goal
 */
double Arm::GetScrewPIDSetPoint() {
	return m_screwPID->GetSetpoint();
}

/**
 * 	Have we arrived?
 */
bool Arm::ScrewAtPIDSetPoint() {
	return m_screwPID->OnTarget();
}

void Arm::ArmPrintData() {
	//SmartDashboard::PutNumber("Arm Encoder", m_armEncoder->GetDistance()); //Brandon told me to write these. idk what they do. REMOVED FOR NOW
	//SmartDashboard::PutNumber("Screw Encoder", m_screwEncoder->GetDistance()); REMOVED FOR NOW
}
