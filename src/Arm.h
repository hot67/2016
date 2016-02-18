#ifndef SRC_ARM_H_
#define SRC_ARM_H_

#include <RobotUtils/HotSubsystem.h>

/*
 * Notice: Currently a lot of PIDs, and simple math, is commented out.
 * This is due to the fact that encoders will most likely be wired directly
 * into the talon srx.
 */

/******************************
 * 	Robot Dependent Values
 ******************************/
/**
 * 	Decide if we are working with competition bot or practice bot
 */
//#define COMPETITION_BOT
#define PRACTICE_BOT

/**
 * 	For Practice Bot
 */
#ifdef PRACTICE_BOT
/**
 * 	PID Values for Arm
 */
#define ARM_P 0
#define ARM_I 0
#define ARM_D 0

/**
 * 	PID Values for Screw
 */
#define SCREW_P 0
#define SCREW_I 0
#define SCREW_D 0

/**
 * 	Arm Motion Profiling Configuration (Calibration)
 */
#define ARM_MAX_A 1
#define ARM_MAX_V 1
#define ARM_DELTA_TIME 20

/**
 * 	Screw Motion Profiling Configuration (Calibration)
 */
#define SCREW_MAX_A 1
#define SCREW_MAX_V 1
#define SCREW_DELTA_TIME 20

/**
 * 	Position of Light Sensor
 * 		Really robot depended?
 */
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

#define ARM_MAX_A 1
#define ARM_MAX_V 1
#define ARM_DELTA_TIME 20

#define SCREW_MAX_A 1
#define SCREW_MAX_V 1
#define SCREW_DELTA_TIME 1
#define LIGHT_SENSOR_POS 1

#endif

/******************************
 * 	Robot Independent Values
 ******************************/
/**
 * 	Distance Per Pulse
 * 	ARM:
 * 		100 rotation of encoder is 22 rotation of arm
 * 		1 rotation of encoder is 0.22 rotation of arm
 * 		360 degree of encoder is 0.22 * 360 degree of arm
 * 		1 degree of encoder is 0.22 degree of arm
 * 		1 tick of encoder is 0.22 degree of arm
 *
 * 	SCREW:
 * 		1 rotation of encoder is 1/4 inches of extension
 * 		360 degree of encoder is 1/4 inches of extension
 * 		1 degree of encoder is 1 / 4 / 360 inches of extension
 * 		1 tick of encoder is 1 / 4 / 360 inches of extension
 */
#define ARM_ENCODER_DISTANCE_PER_PULSE 0.22	//	22/100
#define SCREW_ENCODER_DISTANCE_PER_PULSE 0.00069444444	// 1/4 / 360

/**
 * 	Arm PID Set Points (in degree)
 */
#define FAR_HIGH_GOAL 45
#define CLIMB_ARM 97.126
#define MEDIUM_LOW_GOAL 50
#define CLOSE_HIGH_GOAL 60
#define CARRY 10
#define CLOSE_LOW_GOAL 15
#define PICKUP 0
#define OBSTACLE -10

/**
 * 	Screw PID Set Points (in inches)
 */
#define CLIMB_SCREW 0
#define RETRACT_SCREW 0

/******************************
 * 	Wiring
 ******************************/
/**
 * 	Encoders
 */
#define ENCODER_ARM1 4
#define ENCODER_ARM2 5
#define ENCODER_SCREW1 6
#define ENCODER_SCREW2 7

/**
 * 	Talons
 */
#define TALON_SCREW_L 14
#define TALON_SCREW_R 15
#define TALON_ARM_R 12
#define TALON_ARM_L 11

/**
 * 	Light Sensor
 */
#define LIGHT_ARM 9

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
	/**
	 * 	Talons for Arm
	 */
	CANTalon* m_armLeftTalon; //Initializes Talons for Arm
	CANTalon* m_armRightTalon;

	/**
	 * 	Talons for Screw
	 */
	CANTalon* m_screwLeftTalon; //Initializes Talons for Screwdrive
	CANTalon* m_screwRightTalon;

	/**
	 * 	Encoder for Arm
	 */
	Encoder *m_armEncoder;

	/**
	 * 	Encoder for Screw
	 */
	Encoder *m_screwEncoder;

	/**
	 *	Light Sensor for Arm
	 */
	DigitalInput* m_armLightSensor;

	/**
	 * 	PID Controllers
	 */
	PIDController *m_armPID;
	PIDController *m_screwPID;

public:
	Arm(HotBot* bot); //Constructor
	virtual ~Arm();

	/**
	 *	Raw Access To Talons
	 */
	void SetArm(float speed); //Set the Speed of the Arm
	void SetScrew(float speed); //Set the Speed of the Screw Drive / Arm Extender

	/**
	 * 	Raw Access to Encoder Values
	 */
	double GetArmPos();
	double GetScrewPos();

	/**
	 * 	Raw Access to Encoder Speed
	 */
	double GetArmEncoderRate(); //Returns the arm encoder rate
	double GetScrewEncoderRate(); //Returns the screw encoder rate

	/**
	 * 	Encoder Resetter
	 */
	void ZeroArmEncoder();
	void ZeroScrewEncoder();

	/**
	 * 	Raw access to light sensor
	 */
	bool IsLightSensorTriggered();

	/******************************
	 * 	Arm PID
	 ******************************/
	/**
	 * 	Enable and Disable
	 */
	void EnableArmPID();
	void DisableArmPID();

	/**
	 * 	Is Enabled?
	 */
	bool IsArmPIDEnabled();

	/**
	 * 	Set Set Point
	 */
	void SetArmPIDSetPoint(double setpoint);

	/**
	 * 	Set Pre-defined Set Point
	 */
	void SetArmPIDSetPoint(ArmSetPoint setpoint);

	/**
	 * 	What is goal now?
	 */
	double GetArmPIDSetPoint();

	/**
	 * 	Have arrive to the set point?
	 */
	bool ArmAtPIDSetPoint();

	/******************************
	 * 	Screw PID
	 ******************************/
	/**
	 * 	Enable and Disable
	 */
	void EnableScrewPID();
	void DisableScrewPID();

	/**
	 * 	Is Enabled?
	 */
	bool IsScrewPIDEnabled();

	/**
	 * 	Set Set Point
	 */
	void SetScrewPIDPoint(double setpoint);

	/**
	 * 	Set Pre-defined Set Point
	 */
	void SetScrewPIDPoint(ScrewSetPoint setpoint);

	/**
	 * 	What is the goal now?
	 */
	double GetScrewPIDSetPoint();

	/**
	 * 	Have arrived to the set point?
	 */
	bool ScrewAtPIDSetPoint();


	/******************************
	 * 	Utility Functions
	 ******************************/
	/**
	 * 	Log:
	 *
	 */
	void ArmPrintData(); //Print the encoder values to smart dashboard.

	float RC(float degrees); //Radian Convertifier. May not end up being used
};

#endif /* SRC_ARM_H_ */
