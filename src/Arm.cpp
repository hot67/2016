#include <Arm.h>

Arm::Arm(HotBot* bot) : HotSubsystem(bot, "Arm") {
	/*
	 * 	Control
	 */
	m_lArm = new CANTalon(TALON_ARM_L);
	m_rArm = new CANTalon(TALON_ARM_R);
	m_lScrew = new CANTalon(TALON_SCREW_L);
	m_rScrew = new CANTalon(TALON_SCREW_R);

	/*
	 * 	CANTalon Configuration
	 */
	m_lArm->SetFeedbackDevice(CANTalon::QuadEncoder);
	m_rArm->SetFeedbackDevice(CANTalon::QuadEncoder);
	m_lScrew->SetFeedbackDevice(CANTalon::QuadEncoder);
	m_rScrew->SetFeedbackDevice(CANTalon::QuadEncoder);
	m_lArm->ConfigEncoderCodesPerRev(360);
	m_rArm->ConfigEncoderCodesPerRev(360);
	m_lScrew->ConfigEncoderCodesPerRev(360);
	m_rScrew->ConfigEncoderCodesPerRev(360);

	m_armBuf = new DoubleBuffer(BufferPriority::kAbsMax);
	m_screwBuf = new DoubleBuffer(BufferPriority::kAbsMax);

	/*
	 * 	Braking
	 */
	m_brake = new Solenoid(SOLENOID_BRAKE);

	m_brakeBuf = new BooleanBuffer(true);

	/*
	 * 	Light sensor
	 */
	m_light = new DigitalInput(LIGHT_ARM);

	/*
	 * 	Arm PID
	 */
	ArmPIDWrapper *m_wrapper = new ArmPIDWrapper(this);
	m_armPID = new PIDController(ARM_UP_P, ARM_UP_I, ARM_UP_D, m_wrapper, m_wrapper);
	m_armPIDBuf = new BooleanBuffer(false);
}

/******************************
 * 	Motor
 ******************************/
void Arm::_SetArm(float speed) {
	m_lArm->Set(speed);
	m_rArm->Set(-speed);
}
void Arm::_SetScrew(float speed) {
	//	ToDo:	Safety
	m_lScrew->Set(speed);
	m_rScrew->Set(speed);
}
void Arm::_SetBrake(bool on) {
	m_brake->Set(on);
}

void Arm::SetArm(float speed, int priority) {
	m_armBuf->Write(speed, priority);
}
void Arm::SetScrew(float speed, int priority) {
	m_screwBuf->Write(speed, priority);
}
void Arm::SetBrake(bool on, int priority) {
	m_brakeBuf->Write(on, priority);
}
void Arm::ApplyBrake(int priority) {
	m_brakeBuf->Write(false, priority);
}
void Arm::ReleaseBrake(int priority) {
	m_brakeBuf->Write(true, priority);
}

void Arm::Update() {
	if (!m_armBuf->IsLocked()) {
		_SetArm(m_armBuf->Read());
	}

	if (!m_screwBuf->IsLocked()) {
		_SetScrew(m_screwBuf->Read());
	}

	if (!m_brakeBuf->IsLocked()) {
		_SetBrake(m_brakeBuf->Read());
	}

	if (!m_armPIDBuf->IsLocked()) {
		if (m_armPIDBuf->Read()) {
			_EnableArmPID();
		} else {
			_DisableArmPID();
		}
	}
}

/******************************
 * 	Sensor
 ******************************/
double Arm::GetArmLAngle() {
	return - m_lArm->GetPosition() * 79.2;
}
double Arm::GetArmRAngle() {
	return -m_rArm->GetPosition() * 79.2;
}
double Arm::GetArmAngle() {
	return GetArmLAngle();
}

double Arm::GetArmLSpeed() {
	return - m_lArm->GetSpeed() * 79.2;
}
double Arm::GetArmRSpeed() {
	return - m_rArm->GetSpeed() * 79.2;
}
double Arm::GetArmSpeed() {
	return GetArmLSpeed();
}

double Arm::GetScrewLPosition() {
	return m_lScrew->GetPosition() / 4;
}
double Arm::GetScrewRPosition() {
	return m_rScrew->GetPosition() / 4;
}
double Arm::GetScrewPosition() {
	return GetScrewLPosition();
}

double Arm::GetScrewLSpeed() {
	return m_lScrew->GetSpeed() / 4;
}
double Arm::GetScrewRSpeed() {
	return m_rScrew->GetSpeed() / 4;
}
double Arm::GetScrewSpeed() {
	return GetScrewLSpeed();
}

bool Arm::GetLight() {
	return m_light->Get();
}

/******************************
 * 	Arm PID
 ******************************/
void Arm::_EnableArmPID() {
	if (!IsArmPIDEnabled()) {
		m_armPID->Enable();
	}
}
void Arm::_DisableArmPID() {
	if (IsArmPIDEnabled()) {
		m_armPID->Disable();
	}
}
void Arm::EnableArmPID(int priority) {
	m_armPIDBuf->Write(true, priority);
}
void Arm::DisableArmPID(int priority) {
	m_armPIDBuf->Write(false, priority);
}

bool Arm::IsArmPIDEnabled() {
	return m_armPID->IsEnabled();
}

void Arm::SetArmPIDSetpoint(double angle) {
	m_armPID->SetSetpoint(angle);
}

double Arm::GetArmPIDSetpoint() {
	return m_armPID->GetSetpoint();
}

bool Arm::ArmAtPIDSetpoint() {
	return m_armPID->OnTarget();
}

/******************************
 * 	Arm PID Wrapper
 ******************************/
Arm::ArmPIDWrapper::ArmPIDWrapper(Arm *arm) {
	m_arm = arm;
}

void Arm::ArmPIDWrapper::PIDWrite(float output) {
	m_arm->SetArm(-output);
}

double Arm::ArmPIDWrapper::PIDGet() {
	return m_arm->GetArmAngle();
}
