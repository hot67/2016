#ifndef SRC_ARM_H_
#define SRC_ARM_H_

#include <RobotUtils/HotSubsystem.h>
#include "MotionProfiling.h"

/*
 * Notice: Currently a lot of PIDs, and simple math, is commented out.
 * This is due to the fact that encoders will most likely be wired directly
 * into the talon srx.
 */


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

#define ARM_MAX_A 1
#define ARM_MAX_V 1
#define ARM_DELTA_TIME 20

#endif

#ifdef COMPETITION_BOT
//PID values
#define ARM_P 0
#define ARM_I 0
#define ARM_D 0
#define SCREW_P 0
#define SCREW_I 0
#define SCREW_D 0

#define ARM_MAX_A 1
#define ARM_MAX_V 1
#define ARM_DELTA_TIME 20

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
//#define ENCODER_ARM1 4 REMOVED FOR NOW
//#define ENCODER_ARM2 5 REMOVED FOR NOW
//#define ENCODER_SCREW1 6 REMOVED FOR NOW
//#define ENCODER_SCREW2 7 REMOVED FOR NOW

//Motor ids
#define TALON_SCREW_L 14
#define TALON_SCREW_R 15
#define TALON_ARM_R 12
#define TALON_ARM_L 11

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

	//Encoder* m_screwEncoder; //Initializes Encoders. REMOVED FOR NOW
	//Encoder* m_armEncoder;

	//PIDController* m_armPIDController; //Initializes PID Controllers REMOVED FOR NOW
	//PIDController* m_screwPIDController;

	MotionProfiling m_armMotionProfile;
	Trajectory m_armTrajectoryPoints;
	float m_armTargetPos;

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

	void EnableScrewMotionProfiling();
	void SetScrewMotionProfilePoint(float target);
	void DisableScrewMotionProfiling();
	void PeriodicScrewTask();
	void PauseScrewMotionProfiling();
	void ResumeScrewMotionProfiling();

	void EnableArmMotionProfiling();
	void SetArmMotionProfilePoint(float target);
	void DisableArmMotionProfiling();
	void PeriodicArmTask();
	void PauseArmMotionProfiling();
	void ResumeArmMotionProfiling();

protected:
	void ArmPrintData();
public:
	float GetArmEncoderRate(); //Returns the arm encoder rate
	float GetScrewEncoderRate(); //Returns the screw encoder rate

	void EnableArmPID(); //Enable the PID for the arm
	void DisableArmPID(); //Disable the PID for the arm

	void EnableScrewPID(); //Enable the PID for the screw
	void DisableScrewPID(); //Disable the PID for the screw
	float RC(float degrees); //Radian Convertifier. May not end up being used


};

#endif /* SRC_ARM_H_ */
