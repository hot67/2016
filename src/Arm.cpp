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

	m_armPIDController = new PIDController(ARM_P,ARM_I,ARM_D,m_armEncoder,m_armLeftTalon);

	m_screwPIDController = new PIDController(SCREW_P,SCREW_I,SCREW_D,m_screwEncoder,m_screwLeftTalon);
/*
 * Slave the right motors to the left ones which will be controlled by PIDs and Teleop
 */
	m_screwRightTalon->SetControlMode(CANSpeedController::kFollower); //Slavery is good
	m_screwRightTalon->Set(SCREW_DRIVE_ID_LEFT);

	m_armRightTalon->SetControlMode(CANSpeedController::kFollower); // See Above
	m_armRightTalon->Set(ARM_ID_LEFT);
}

Arm::~Arm() {
	// TODO Auto-generated destructor stub // i thinks me will leave this heres
}

void Arm::SetExtend(float speed) {

}
