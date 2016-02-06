#ifndef SRC_ARM_H_
#define SRC_ARM_H_

#include <RobotUtils/HotSubsystem.h>

//#define COMPETITION_BOT
#define PRACTICE_BOT

#ifdef PRACTICE_BOT
//PID values
#define ARM_P 0
#define ARM_I 0
#define ARM_D 0
#define SCREW_P 0
#define SCREW_I 0
#define SCREW_D 0
#endif

#ifdef COMPETITION_BOT
//PID values
#define ARM_P 0
#define ARM_I 0
#define ARM_D 0
#define SCREW_P 0
#define SCREW_I 0
#define SCREW_D 0
#endif

//PPRs
#define ARM_ENCODER_PULSE_PER_REVOLUTION 1
#define SCREW_ENCODER_PULSE_PER_REVOLUTION 1

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
	kFarHighGoal = 1, //45 degrees
	kMediumLowGoal = 2, //50 degrees
	kCloseHighGoal = 3, //60 degrees
	kCarry = 4, //10 degrees
	kCloseLowGoal = 5, //15 degrees
	kPickup = 6, //unknown
	kObstacle = 7, //-10 degrees
	kClimbArm = 8, //97 degrees
	kResetArm = 0
};

enum ScrewSetPoint {
	kClimbScrew = 1, //extend
	kRetractScrew = 2, //retract
	kResetScrew = 0
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
	void SetScrew(float speed); //Set the Speed of the Screw Drive / Arm Extender

	void SetArmPIDPoint(ArmSetPoint setpoint);
	/*
	 * Set the desired pidcontroller setpoint, such as close_low_goal
	 */
	void SetScrewPIDPoint(ScrewSetPoint setpoint);

	float GetScrewSetPoint(); //Returns the Setpoint of the Screw PIDController
	float GetArmSetPoint(); //Returns the Setpoint of the Arm PIDController

	float GetScrewPos(); //Returns the current encoder value of the screw
	float GetArmPos(); //Returns the current encoder value of the arm

	bool ArmAtSetPoint(); //Checks if arm is at given set point
	bool ScrewAtSetPoint(); //Checks if screw is at given set point

	void ZeroArmEncoder(); //zero the arm encoder
	void ZeroScrewEncoder(); //zero the screw encoder

	float GetArmEncoderRate(); //Returns the arm encoder rate
	float GetScrewEncoderRate(); //Returns the screw encoder rate

	void EnableArmPID(); //Enable the PID for the arm
	void DisableArmPID(); //Disable the PID for the arm

	void EnableScrewPID(); //Enable the PID for the screw
	void DisableScrewPID(); //Disable the PID for the screw
	float RC(float degrees); //Radian Convertifier. May not end up being used


};

#endif /* SRC_ARM_H_ */
