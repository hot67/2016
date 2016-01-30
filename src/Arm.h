#ifndef SRC_ARM_H_
#define SRC_ARM_H_

#include <RobotUtils/HotSubsystem.h>

#define ARM_P 0
#define ARM_I 0
#define ARM_D 0
#define SCREW_P 0
#define SCREW_I 0
#define SCREW_D 0

#define FAR_HIGH_GOAL 0
#define CLIMB 0
#define MEDIUM_LOW_GOAL 0
#define CLOSE_HIGH_GOAL 0
#define CARRY 0
#define CLOSE_LOW_GOAL 0
#define PICKUP 0
#define RETRACT_SCREW 0
#define OBSTACLE 0

#define ENCODER_CHANNEL1_ARM 4
#define ENCODER_CHANNEL2_ARM 5
#define ENCODER_CHANNEL1_SCREW 6
#define ENCODER_CHANNEL2_SCREW 7

#define SCREW_DRIVE_ID_LEFT 14
#define SCREW_DRIVE_ID_RIGHT 15
#define ARM_ID_RIGHT 12
#define ARM_ID_LEFT 11

enum ArmSetPoint {
	kFarHighGoal = 1,
	kClimb = 2,
	kMediumLowGoal = 3, //I think. 50 degrees pulse roller for .1 seconds
	kCloseHighGoal = 4,
	kCarry = 5,
	kCloseLowgoal = 6,
	kPickup = 7,
	kRetractScrew = 8,
	kObstacle= 9
};

class Arm: public HotSubsystem {

	CANTalon* m_armLeftTalon;
	CANTalon* m_armRightTalon;

	CANTalon* m_screwLeftTalon;
	CANTalon* m_screwRightTalon;

	Encoder* m_screwEncoder;
	Encoder* m_armEncoder;

	PIDController* m_armPIDController;
	PIDController* m_screwPIDController;

public:

	Arm(HotBot* bot); //Constructor
	virtual ~Arm();

	void Set(float speed); //Set the Speed of the Arm
	void SetExtend(float speed); //Set the Speed of the Screw Drive / Arm Extender

	void SetPIDPoint(ArmSetPoint setpoint); //Set the desired pidcontroller setpoint, such as close_low_goal

	void EnableArmPID(); //Enable the Pids
	void DisableArmPID(); //Disable the Pids

	void EnableScrewPID();
	void DisableScrewPID();

};

#endif /* SRC_ARM_H_ */
