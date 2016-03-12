#ifndef SRC_ARM_H_
#define SRC_ARM_H_

#include <RobotUtils/RobotUtils.h>

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

#define LIGHT_SENSOR_POS 0

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
#define FAR_HIGH_GOAL 42
#define MEDIUM_HIGH_GOAL 50
#define CLOSE_HIGH_GOAL 65 //54.8 actually sets 51.
#define BATTER_HIGH_GOAL 35

#define CLOSE_LOW_GOAL 15

#define CARRY 15.06
#define PICKUP 0
#define OBSTACLE -10
#define CLIMB_ARM 92.

#define CLIMBING_ARM 63.

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
#define CLIMB_SCREW 22.13
#define RETRACT_SCREW 0

/*
 * CANTalon Motor Locations
 */
#define TALON_SCREW_L 14
#define TALON_SCREW_R 15
#define TALON_ARM_R 12
#define TALON_ARM_L 11

/*
 * 	Solenoid for brake
 */
#define SOLENOID_BRAKE 1

/*
 * Light Sensor Location
 */
#define LIGHT_ARM 9

#define SOLENOID_ARM_BRAKE 1

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
	kBatter = 9 //35 degrees
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
public:
	Arm(HotBot* bot);

	/*********************************
	 * 	Motor
	 *********************************/
	void SetArm(float speed, int priority = 0);
	void SetScrew(float speed, int priority = 0);

	void SetBrake(bool on, int priority = 0);
	void ApplyBrake(int priority = 0);
	void ReleaseBrake(int priority = 0);

	void Update();

	/*********************************
	 * 	Sensor
	 *********************************/
	/*
	 * 	Arm
	 */
	double GetArmLAngle();
	double GetArmRAngle();
	double GetArmAngle();

	double GetArmLSpeed();
	double GetArmRSpeed();
	double GetArmSpeed();

	bool GetLight();

	/*
	 * 	Screw
	 */
	double GetScrewLPosition();
	double GetScrewRPosition();
	double GetScrewPosition();

	double GetScrewLSpeed();
	double GetScrewRSpeed();
	double GetScrewSpeed();

	/*********************************
	 * 	Arm PID
	 *********************************/
	void EnableArmPID(int priority = 0);
	void DisableArmPID(int priority = 0);

	bool IsArmPIDEnabled();

	void SetArmPIDSetpoint(double angle);

	double GetArmPIDSetpoint();

	bool ArmAtPIDSetpoint();

	/*********************************
	 * 	Arm PID Wrapper
	 *********************************/
	class ArmPIDWrapper : public PIDSource, public PIDOutput {
	public:
		ArmPIDWrapper(Arm *arm);

		void PIDWrite(float output);
		double PIDGet();
	private:
		Arm *m_arm;
	};


private:
	/*
	 * 	Function to send signal
	 * 		all signal to motor/solenoid/etc.. must go through here
	 */
	void _SetArm(float speed);
	void _SetScrew(float speed);
	void _SetBrake(bool on);

	/*
	 * 	Arm PID
	 */
	void _EnableArmPID();
	void _DisableArmPID();

	/*
	 * 	Motors
	 */
	CANTalon *m_lArm, *m_rArm;
	CANTalon *m_lScrew, *m_rScrew;
	DoubleBuffer *m_armBuf, *m_screwBuf;

	/*
	 * 	Brake Solenoid
	 */
	Solenoid *m_brake;
	BooleanBuffer *m_brakeBuf;

	/*
	 * 	Light Sensor
	 */
	DigitalInput *m_light;

	/*
	 * 	PID
	 */
	PIDController *m_armPID;
	BooleanBuffer *m_armPIDBuf;
};

#endif /* SRC_ARM_H_ */
