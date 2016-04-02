#include <Arm.h>

/*
 * The Arm Subsystem
 */
Arm::Arm(HotBot* bot) : HotSubsystem(bot, "Arm") { //A robot


	/*
	 * Setup Arm Tilt Sensor
	 */
	m_armAccelerometer = new ADXL345_I2C(I2C::kOnboard, ADXL345_I2C::kRange_2G);
	m_armGyro = new AnalogGyro(0);
	m_armAngle = new TiltSensor(m_armAccelerometer, m_armGyro, 0.15);

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

	/**
	 *  Initialize Brake
	 */
	m_brake = new Solenoid(SOLENOID_BRAKE);

	/*
	 * Portcullis
	 */
	m_portcullisWedge = new Solenoid(SOLENOID_PORTCULLIS_WEDGE);

	/*
	 * Initialize the light sensor for the arm.
	 */
	m_armLightSensor = new DigitalInput(LIGHT_ARM);

	/*
	 * Tell the talon it is using digital encoders. These counts will be multiplied by four,
	 * so in later use we need to divide them by four.
	 */
	m_armLeftTalon->SetFeedbackDevice(CANTalon::QuadEncoder);
	m_armRightTalon->SetFeedbackDevice(CANTalon::QuadEncoder);

	m_screwLeftTalon->SetFeedbackDevice(CANTalon::QuadEncoder);
	m_screwRightTalon->SetFeedbackDevice(CANTalon::QuadEncoder);

	/*
	 * Configure the encoder counts per revolution
	 */
	m_armLeftTalon->ConfigEncoderCodesPerRev(360);
	m_armRightTalon->ConfigEncoderCodesPerRev(360);
	m_screwLeftTalon->ConfigEncoderCodesPerRev(360);
	m_screwRightTalon->ConfigEncoderCodesPerRev(360);

	/*
	 * Set P, I, and D, in the Controllers using our wrapper
	 */
	m_armPIDWrapper = new ArmPIDWrapper(this);
	m_screwPIDWrapper = new ScrewPIDWrapper(this);

	m_armPIDController = new PIDController(ARM_UP_P, ARM_UP_I, ARM_UP_D, m_armPIDWrapper, m_armPIDWrapper);
	m_armPIDController->SetAbsoluteTolerance(3.0); //4


	m_screwPIDController = new PIDController(SCREW_P, SCREW_I, SCREW_D, m_screwPIDWrapper, m_screwPIDWrapper);

	m_offset = 8.41; //7.75

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
	SmartDashboard::PutNumber("Screw speed", speed);

	if (GetScrewPos() < 75 && speed < 0) {
		//going up and screw position is less than 30
		m_screwLeftTalon->Set(-speed);
		m_screwRightTalon->Set(-speed);
	}
	else if (GetScrewPos() > -5 && speed > 0) {
		m_screwLeftTalon->Set(-speed);
		m_screwRightTalon->Set(-speed);
	}
	else {
		m_screwLeftTalon->Set(0);
		m_screwRightTalon->Set(0);
	}
}

void Arm::SetBrake(bool on) {
	m_brake->Set(on);
}

void Arm::ApplyBrake() {
	m_brake->Set(false);
}

void Arm::ReleaseBrake() {
	m_brake->Set(true);
}

void Arm::WedgeOut() {
	m_portcullisWedge->Set(true);
}

void Arm::WedgeIn() {
	m_portcullisWedge->Set(false);
}

float Arm::GetArmPos() {
	return - m_armLeftTalon->GetPosition() * 79.2 + m_offset;
}

double Arm::GetArmAngle() {
	return - m_armAngle->GetAngle();
}

float Arm::GetRightArmPos() {
	return - m_armRightTalon->GetPosition() * 79.2 + m_offset;
}

float Arm::GetScrewPos() {
	return m_screwLeftTalon->GetPosition() / 4 * SCREW_ENCODER_VALUE;
	//the screw encoder value for practice bot is -1, while for competition bot is 1
}

float Arm::GetRightScrewPos() {
	return - m_screwRightTalon->GetPosition() / 4;
}


float Arm::GetArmSpeed() {
	return - m_armLeftTalon->GetSpeed() * 79.2;
}

float Arm::GetScrewSpeed() {
	return - m_screwLeftTalon->GetSpeed() / 4;
}

void Arm::ZeroArmEncoder() {
	m_armLeftTalon->SetPosition(0.0);
	m_armRightTalon->SetPosition(0.0);

	m_offset = 8.41; //7.75
}

void Arm::ZeroScrewEncoder() {
	m_screwLeftTalon->SetPosition(0);
	m_screwRightTalon->SetPosition(0);
}

void Arm::ZeroLightSensorArmEncoder() {
	m_armLeftTalon->SetPosition(0);
	m_armRightTalon->SetPosition(0);

	m_offset = 58.73;
}

void Arm::ZeroAccelerometerArmEncoder() {
	m_armLeftTalon->SetPosition(0);
	m_armRightTalon->SetPosition(0);

	m_offset = GetArmAngle();
}

void Arm::CalibrateArm(double offset) {
	m_armLeftTalon->SetPosition(0.0);
	m_armRightTalon->SetPosition(0.0);

	m_offset = offset;
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
		SetArmPIDPoint(FAR_HIGH_GOAL);
		break;
	case kMediumLowGoal:

		/*
		 * Medium Away Low Goal
		 */
		SetArmPIDPoint(MEDIUM_HIGH_GOAL);
		break;
	case kCloseHighGoal:

		/*
		 * Close High Goal
		 */
		SetArmPIDPoint(CLOSE_HIGH_GOAL);
		break;
	case kCarry:

		/*
		 * Carry Position
		 */
		SetArmPIDPoint(CARRY);
		break;
	case kCloseLowGoal:

		/*
		 * Close Low Goal
		 */
		SetArmPIDPoint(CLOSE_LOW_GOAL);
		break;
	case kPickup:

		/*
		 * Pickup Position
		 */
		SetArmPIDPoint(PICKUP);
		break;
	case kObstacle:

		/*
		 * Obstacle Self-Lift Position
		 */
		SetArmPIDPoint(OBSTACLE);
		break;
	case kClimbArm:

		/*
		 * Climb Position
		 */
		SetArmPIDPoint(CLIMB_ARM);
		break;
	case kBatter:
		SetArmPIDPoint(BATTER_HIGH_GOAL);
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
	//return m_armPIDController->OnTarget();

	if (fabs(GetArmPos() - GetArmPIDSetPoint()) <= 4) {
		return true;
	}
	else {
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
