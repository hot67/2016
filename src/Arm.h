#ifndef SRC_ARM_H_
#define SRC_ARM_H_

#include <ArmMotionProfiling.h>
#include <RobotUtils/HotSubsystem.h>

//#define COMPETITION_BOT
#define PRACTICE_BOT

#ifdef PRACTICE_BOT

/*
 * PID Values
 */
#define ARM_P 0
#define ARM_I 0
#define ARM_D 0
#define SCREW_P 0
#define SCREW_I 0
#define SCREW_D 0

/*
 * Motion Profiling Constants,
 * need to get these from the motors.
 */
#define ARM_MAX_A 1
#define ARM_MAX_V 1
#define ARM_DELTA_TIME 20


#define SCREW_MAX_A 1
#define SCREW_MAX_V 1
#define SCREW_DELTA_TIME 20

#define LIGHT_SENSOR_POS 1

#endif

#ifdef COMPETITION_BOT

/*
 * PID Values
 */
#define ARM_P 0
#define ARM_I 0
#define ARM_D 0
#define SCREW_P 0
#define SCREW_I 0
#define SCREW_D 0

/*
 * Motion Profiling Constants,
 * need to get these from the motors.
 */
#define ARM_MAX_A 1
#define ARM_MAX_V 1
#define ARM_DELTA_TIME 20

#define SCREW_MAX_A 1
#define SCREW_MAX_V 1
#define SCREW_DELTA_TIME 1
#define LIGHT_SENSOR_POS 1

#endif

/*
 * Encoder Pulses per Revolution
 */
#define ARM_ENCODER_PULSE_PER_REVOLUTION 1
#define SCREW_ENCODER_PULSE_PER_REVOLUTION 1

/*
 * PID Setpoints
 */
#define FAR_HIGH_GOAL 45
#define CLIMB_ARM 97.126
#define MEDIUM_LOW_GOAL 50
#define CLOSE_HIGH_GOAL 60
#define CARRY 10
#define CLOSE_LOW_GOAL 15
#define PICKUP 0
#define OBSTACLE -10

/*
 * PID Setpoints
 */
#define CLIMB_SCREW 37440
#define RETRACT_SCREW 0

/*
 * CANTalon Motor Locations
 */
#define TALON_SCREW_L 14
#define TALON_SCREW_R 15
#define TALON_ARM_R 12
#define TALON_ARM_L 11

/*
 * Light Sensor Location
 */
#define LIGHT_ARM 9

/*
 * Arm Setpoint Enums
 */
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

/*
 * Screw Setpoint Enums
 */
enum ScrewSetPoint {
	kClimbScrew = 1, //extend
	kRetractScrew = 2, //retract
	kResetScrew = 0
};

class Arm: public HotSubsystem {

	/*
	 * CANTalons
	 */
	CANTalon* m_armLeftTalon;
	CANTalon* m_armRightTalon;
	CANTalon* m_screwLeftTalon;
	CANTalon* m_screwRightTalon;

	/*
	 * Light Sensor
	 */
	DigitalInput* m_armLightSensor;

	/*
	 * Motion Profiling Controllers and Variables
	 */
	ArmMotionProfiling *m_armMPController;
	float m_armMPTargetPos;
	ArmMotionProfiling *m_screwMPController;
	float m_screwMPTargetPos;


public:

	/*
	 * Arm Constructor
	 */
	Arm(HotBot* bot);
	virtual ~Arm();

	void SetArm(float speed);
	void SetScrew(float speed);

	/*
	 * PID Setpoints and MP Setpoints
	 */
	void SetScrewPIDPoint(ScrewSetPoint setpoint);
	void SetScrewMPPoint(ScrewSetPoint setpoint);
	void SetScrewPIDPoint(double setpoint);

	void SetArmMPPoint(ArmSetPoint setpoint);
	void SetArmPIDPoint(double setpoint);
	void SetArmPIDPoint(ArmSetPoint setpoint);

	/*
	 * Returns the setpoint
	 */
	float GetScrewPIDSetPoint();
	float GetArmPIDSetPoint();
	float GetArmMPSetPoint();
	float GetScrewMPSetPoint();

	/*
	 * Sensors
	 */

	/*
	 * Has the Light Sensor been Triggered?
	 */
	bool IsLightSensorTriggered();

	/*
	 * Print out encoder data to Smart Dashboard.
	 */
	void ArmPrintData();

	/*
	 * Return the Sensor Positions
	 */
	float GetScrewPos();
	float GetArmPos();

	/*
	 * Get the Speed Encoder is Reading
	 */
	float GetArmEncoderRate();
	float GetScrewEncoderRate();

	/*
	 * AtSetpoint? (Arm)
	 */
	bool ArmAtPIDSetPoint();

	/*
	 * AtSetpoint? (Screw)
	 */
	bool ScrewAtPIDSetPoint();

	/*
	 * AtSetpoint? (Arm MP)
	 */
	bool ArmAtMPSetPoint();

	/*
	 * AtSetpoint? (Screw MP)
	 */
	bool ScrewAtMPSetPoint();

	/*
	 * Reset the Encoder
	 */
	void ZeroArmEncoder();
	void ZeroScrewEncoder();

	/*
	 * Motion Profiling.
	 */
	void EnableScrewMotionProfiling();

	/*
	 * Set the target manually
	 */
	void SetScrewMotionProfilePoint(float target);

	/*
	 * Disable Motion Profiling
	 */
	void DisableScrewMotionProfiling();

	/*
	 * Periodic Task.
	 * Call me about every 10 ms
	 */
	void PeriodicScrewTask();

	/*
	 * Pause and resume Profiling
	 */
	void PauseScrewMotionProfiling();
	void ResumeScrewMotionProfiling();

	/*
	 * Enable Screw Motion Profiling
	 */
	void EnableArmMotionProfiling();

	/*
	 * Set the target manually
	 */
	void SetArmMotionProfilePoint(float target);

	/*
	 * Disable Motion Profiling
	 */
	void DisableArmMotionProfiling();

	/*
	 * Periodic Task.
	 * Call me about every 10 ms
	 */
	void PeriodicArmTask();

	/*
	 * Pause and resume Profiling
	 */
	void PauseArmMotionProfiling();
	void ResumeArmMotionProfiling();

	/*
	 * Enable the PIDs
	 */
	void EnableArmPID();
	void EnableScrewPID();

	/*
	 * Disable the PIDs
	 */
	void DisableScrewPID();
	void DisableArmPID();
};

#endif /* SRC_ARM_H_ */
