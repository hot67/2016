#ifndef SRC_ARM_H_
#define SRC_ARM_H_

#include <RobotUtils/HotSubsystem.h>

//#define COMPETITION_BOT
#define PRACTICE_BOT

#ifdef PRACTICE_BOT
//Do some practicing. stop reading my comments
#endif

#ifdef COMPETITION_BOT
//Do some competeting. why are you still reading these
#endif

//PID values
#define ARM_P 0
#define ARM_I 0
#define ARM_D 0
#define SCREW_P 0
#define SCREW_I 0
#define SCREW_D 0

//Values for angles of Arm Positioning (degrees)
#define FAR_HIGH_GOAL 45
#define CLIMB_ARM 97.126
#define MEDIUM_LOW_GOAL 50
#define CLOSE_HIGH_GOAL 60
#define CARRY 10
#define CLOSE_LOW_GOAL 15
#define PICKUP 0
#define OBSTACLE -10

//Values of distance to reach, in feet
#define CLIMB_SCREW 0
#define RETRACT_SCREW 0

//Encoder ids
#define ENCODER_CHANNEL1_ARM 4
#define ENCODER_CHANNEL2_ARM 5
#define ENCODER_CHANNEL1_SCREW 6
#define ENCODER_CHANNEL2_SCREW 7

//Motor ids
#define SCREW_DRIVE_ID_LEFT 14
#define SCREW_DRIVE_ID_RIGHT 15
#define ARM_ID_RIGHT 12
#define ARM_ID_LEFT 11

enum ArmSetPoint {
	kFarHighGoal = 1,
	kMediumLowGoal = 2, //I think. 50 degrees pulse roller for .1 seconds
	kCloseHighGoal = 3,
	kCarry = 4,
	kCloseLowGoal = 5,
	kPickup = 6,
	kObstacle = 7,
	kClimb = 8
};

enum ScrewSetPoint {
	kClimb = 1,
	kRetractScrew = 2
};

class Arm: public HotSubsystem {

	CANTalon* m_armLeftTalon; //Initializes Talons for Arm
	CANTalon* m_armRightTalon;

	CANTalon* m_screwLeftTalon; //Initializes Talons for Screwdrive
	CANTalon* m_screwRightTalon;

	Encoder* m_screwEncoder; //Initializes Encoders
	Encoder* m_armEncoder;

	PIDController* m_armPIDController; //Initializes PID Controllers
	PIDController* m_screwPIDController;

public:

	Arm(HotBot* bot); //Constructor
	virtual ~Arm();

	void SetArm(float speed); //Set the Speed of the Arm
	void SetExtend(float speed); //Set the Speed of the Screw Drive / Arm Extender

	void SetArmPIDPoint(ArmSetPoint setpoint);
	/*
	 * Set the desired pidcontroller setpoint, such as close_low_goal
	 */
	void SetScrewPIDPoint(ScrewSetPoint setpoint);

	bool ArmAtSetPoint(); //Checks if arm is at given set point
	bool ScrewAtSetPoint(); //Checks if screw is at given set point

	void EnableArmPID(); //Enable the Pid for the arm
	void DisableArmPID(); //Disable the Pid for the arm

	void EnableScrewPID(); //Enable the Pid for the screw
	void DisableScrewPID(); //Disable the Pid for the screw

private:


	class ShoulderPIDController : public PIDOutput {
	private:
		CANTalon* m_right;
		CANTalon* m_left;
		Encoder* m_encoder;
	public:
		ShoulderPIDController(CANTalon* motorRight,CANTalon* motorLeft, Encoder* encoder);
		void Set(float speed); //Set speed of shoulder
		void PIDWrite(float speed); //Write pid value
	};


};

#endif /* SRC_ARM_H_ */