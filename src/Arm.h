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
#define ARM_UP_P 0.05
#define ARM_UP_I 0
#define ARM_UP_D 0

#define ARM_DOWN_P 0.025
#define ARM_DOWN_I 0
#define ARM_DOWN_D 0

#define SCREW_P 0.2
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
#define ARM_UP_P 0.05
#define ARM_UP_I 0
#define ARM_UP_D 0

#define ARM_DOWN_P 0.025
#define ARM_DOWN_I 0
#define ARM_DOWN_D 0

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
 * PID Setpoints for arm
 */
#define FAR_HIGH_GOAL 45
#define MEDIUM_HIGH_GOAL 50
#define CLOSE_HIGH_GOAL 65 //54.8 actually sets 51.
#define BATTER_HIGH_GOAL 35

#define CLOSE_LOW_GOAL 15

#define CARRY 20
#define PICKUP 0
#define OBSTACLE -10
#define CLIMB_ARM 97.126

/**
 * 	Shooter Speed for Different Set points
 */
#define FAR_HIGH_GOAL_SHOOTER 1.0
#define MEDIUM_HIGH_GOAL_SHOOTER 1.0
#define CLOSE_HIGH_GOAL_SHOOTER 1.0
#define BATTER_HIGH_GOAL_SHOOTER 0.6

/*
 * PID Setpoints for screw
 */
#define CLIMB_SCREW 5
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
	kBatter = 9, //35 degrees
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
	 * PID Controllers
	 */
	PIDController * m_armPIDController;
	PIDController * m_screwPIDController;
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

	/*
	 * Raw Access to Talons
	 * Uses m_armPIDWrapper, and m_screwPIDWrapper
	 */
	void SetArm(float speed);
	void SetScrew(float speed);

	/*
	* Raw access to Encoder Values.
	*/
	float GetScrewPos();
	float GetArmPos();

	/*
	 * Raw access to Encoder Speed
	 */
	float GetArmSpeed();
	float GetScrewSpeed();

	/*
	 * Encoder Resetter
	 */
	void ZeroArmEncoder();
	void ZeroScrewEncoder();

	/*
	 * Raw Access to Light Sensor
	 */
	bool IsLightSensorTriggered();

	/*
	 * Print Encoder Data to Smart Dashboard
	 */
	void ArmPrintData();

	/*
	 * Make sure nothing over extends.
	 */
	void PeriodicTask();

	/*****************************
	 *		Arm PID
	 *****************************/

	class ArmPIDWrapper : public PIDSource , public PIDOutput {
		Arm *m_arm;
	public:
		double PIDGet();
		void PIDWrite(float output);
		ArmPIDWrapper(Arm *arm);
	};
private:
	ArmPIDWrapper * m_armPIDWrapper;
public:
	/*
	 * Enable and Disable
	 */
	void EnableArmPID();
	void DisableArmPID();

	/*
	 * Is Enabled?
	 */
	bool IsArmPIDEnabled();

	/*
	 * Set Setpoint
	 */
	void SetArmPIDPoint(double setpoint);
	void SetArmPIDPoint(ArmSetPoint setpoint);

	/*
	 * What is goal now?
	 */
	float GetArmPIDSetPoint();

	/*
	 * Have we arrived at the Set Point?
	 */
	bool ArmAtPIDSetPoint();

	/*
	 * PID Update
	 * because gravity is bringing arm down too hard
	 */

	void ArmPIDUpdate();

	/*****************************
	 * 		Screw PID
	 ******************************/

	class ScrewPIDWrapper : public PIDSource, public PIDOutput {

		Arm * m_arm;
	public:
		double PIDGet();
		void PIDWrite(float output);

		ScrewPIDWrapper(Arm *arm);
	};
private:
	ScrewPIDWrapper * m_screwPIDWrapper;
public:
	/*
	 * Enable and Disable
	 */
	void EnableScrewPID();
	void DisableScrewPID();

	/*
	 * Is Enabled?
	 */
	bool IsScrewPIDEnabled();

	/*
	 * Set Setpoint
	 */
	void SetScrewPIDPoint(ScrewSetPoint setpoint);
	void SetScrewPIDPoint(double setpoint);

	/*
	 * What is the goal now?
	 */
	float GetScrewPIDSetPoint();

	/*
	 * Have we arrived at the Set Point?
	 */
	bool ScrewAtPIDSetPoint();

	/******************************
	 * 		Arm Motion Profiling
	 ******************************/
	/*
	 * Enable and Disable.
	 */
	void EnableArmMotionProfiling();
	void DisableArmMotionProfiling();
	void PauseArmMotionProfiling();
	void ResumeArmMotionProfiling();

	/*
	 * Is Enabled?
	 */
	bool IsArmMPEnabled();

	/*
	 * Set Setpoint.
	 */
	void SetArmMotionProfilePoint(ArmSetPoint target);
	void SetArmMotionProfilePoint(float target);

	/*
	 * Periodic Task, every 10ms
	 */
	void PeriodicArmTask();


	/******************************
	 * 		Screw Motion Profiling
	 ******************************/
	/*
	 * Enable and Disable.
	 */
	void EnableScrewMotionProfiling();
	void DisableScrewMotionProfiling();
	void PauseScrewMotionProfiling();
	void ResumeScrewMotionProfiling();

	/*
	 * Is Enabled?
	 */
	bool IsScrewMPEnabled();

	/*
	 * Set Setpoint.
	 */
	void SetScrewMotionProfilePoint(ScrewSetPoint target);
	void SetScrewMotionProfilePoint(float target);

	/*
	 * Periodic Task, every 10ms
	 */
	void PeriodicScrewTask();

	/*
	 * Have we arrived at the Set Point?
	 */
	bool ScrewAtMPSetPoint();
};

#endif /* SRC_ARM_H_ */
