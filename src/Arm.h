#ifndef SRC_ARM_H_
#define SRC_ARM_H_

#include <ArmMotionProfiling.h>
#include <RobotUtils/HotSubsystem.h>

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

//THESE CONSTANTS MUST BE IN ENCODER TICKS!!
#define SCREW_MAX_A 1
#define SCREW_MAX_V 1
#define SCREW_DELTA_TIME 20

#define LIGHT_SENSOR_POS 1

#endif

#ifdef COMPETITION_BOT
//PID values
#define ARM_P 0
#define ARM_I 0
#define ARM_D 0
#define SCREW_P 0
#define SCREW_I 0
#define SCREW_D 0

//THESE CONSTANTS MUST BE IN ENCODER TICKS!!
#define ARM_MAX_A 1
#define ARM_MAX_V 1
#define ARM_DELTA_TIME 20

#define SCREW_MAX_A 1
#define SCREW_MAX_V 1
#define SCREW_DELTA_TIME 1
#define LIGHT_SENSOR_POS 1

#endif

//PPRs
#define ARM_ENCODER_PULSE_PER_REVOLUTION 1
#define SCREW_ENCODER_PULSE_PER_REVOLUTION 1

//Values for angles of Arm Positioning (ENCODER TICKS)
#define FAR_HIGH_GOAL 45
#define CLIMB_ARM 97.126
#define MEDIUM_LOW_GOAL 50
#define CLOSE_HIGH_GOAL 60
#define CARRY 10
#define CLOSE_LOW_GOAL 15
#define PICKUP 0
#define OBSTACLE -10

//Values of distance to reach, in feet
#define CLIMB_SCREW 37440
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

//Light sensor id
#define LIGHT_ARM 9

enum ArmSetPoint {
	kFarHighGoal = 1, //45 degrees 			(relative, we need this to be finally in ENCODER TICKS)
	kMediumLowGoal = 2, //50 degrees 		(relative, we need this to be finally in ENCODER TICKS)
	kCloseHighGoal = 3, //60 degrees 		(relative, we need this to be finally in ENCODER TICKS)
	kCarry = 4, //10 degrees 				(relative, we need this to be finally in ENCODER TICKS)
	kCloseLowGoal = 5, //15 degrees 		(relative, we need this to be finally in ENCODER TICKS)
	kPickup = 6, //unknown 					(relative, we need this to be finally in ENCODER TICKS)
	kObstacle = 7, //-10 degrees 			(relative, we need this to be finally in ENCODER TICKS)
	kClimbArm = 8, //97 degrees				(relative, we need this to be finally in ENCODER TICKS)
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

	DigitalInput* m_armLightSensor;

	//Encoder* m_screwEncoder; //Initializes Encoders. REMOVED FOR NOW
	//Encoder* m_armEncoder;

	//PIDController* m_armPIDController; //Initializes PID Controllers REMOVED FOR NOW
	//PIDController* m_screwPIDController;

	ArmMotionProfiling *m_armMPController; //Initialize the motion profile variables
	float m_armMPTargetPos;

	ArmMotionProfiling *m_screwMPController; //Initialize the motion profile variables
	float m_screwMPTargetPos;


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

	void SetArmMPPoint(ArmSetPoint setpoint);
	/*
	 * same functions, but using motion profiling!
	 */
	void SetScrewMPPoint(ScrewSetPoint setpoint);

	void SetArmPIDPoint(double setpoint);

	void SetScrewPIDPoint(double setpoint);

	float GetScrewPIDSetPoint(); //Returns the Setpoint of the Screw PIDController
	float GetArmPIDSetPoint(); //Returns the Setpoint of the Arm PIDController

	float GetArmMPSetPoint(); //same functions, but for motion profiling.
	float GetScrewMPSetPoint(); //same functions, but for motion profiling.

	float GetScrewPos(); //Returns the current encoder value of the screw
	float GetArmPos(); //Returns the current encoder value of the arm

	bool ArmAtPIDSetPoint(); //Checks if arm is at given set point (pid)
	bool ScrewAtPIDSetPoint(); //Checks if screw is at given set point (pid)

	bool ArmAtMPSetPoint(); //checks if the arm is at the given set point (motion profiling)
	bool ScrewAtMPSetPoint(); //checks if the screw is at the given set point (motion profiling)

	void ZeroArmEncoder(); //zero the arm encoder
	void ZeroScrewEncoder(); //zero the screw encoder

	void EnableScrewMotionProfiling(); //A series of functions to enable, disable, and manage motion profiling.
	void SetScrewMotionProfilePoint(float target);
	void DisableScrewMotionProfiling();
	void PeriodicScrewTask(); //call this every half of delta time.
	void PauseScrewMotionProfiling();
	void ResumeScrewMotionProfiling();

	void EnableArmMotionProfiling(); //A series of functions to enable, disable, and manage motion profiling.
	void SetArmMotionProfilePoint(float target);
	void DisableArmMotionProfiling();
	void PeriodicArmTask(); //call this 1/2 of delta time.
	void PauseArmMotionProfiling();
	void ResumeArmMotionProfiling();

	void ArmPrintData(); //Print the encoder values to smart dashboard.

	float GetArmEncoderRate(); //Returns the arm encoder rate
	float GetScrewEncoderRate(); //Returns the screw encoder rate

	void EnableArmPID(); //Enable the PID for the arm
	void DisableArmPID(); //Disable the PID for the arm

	void EnableScrewPID(); //Enable the PID for the screw
	void DisableScrewPID(); //Disable the PID for the screw
	float RC(float degrees); //Radian Convertifier. May not end up being used

	bool IsLightSensorTriggered();

	class ARMPIDController : PIDOutput { //A PIDOutput wrapper to handle all autonomous output to the motors.

		CANTalon * m_talonLeft;
		CANTalon* m_talonRight;
	public:

		ARMPIDController(CANTalon * talonLeft, CANTalon * talonRight); //just going to set the talons
		void PIDWrite(float output); //actually does the output handling.
	};
private:
	ARMPIDController * m_armController; //The shoulder output wrapper
	ARMPIDController * m_screwController; //The screwdrive output wrapper

};

#endif /* SRC_ARM_H_ */
