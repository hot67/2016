/*
 * Drivetrain.h
 *
 *  Created on: Jan 23, 2016
 *      Author: Chad
 */


/*
 * CHAD'S TO DO LIST
 *
 * get distance value -> going to be called GetDistancePIDSetPoint()
 * 			what is the goal for the target distance?
 * 			example: we need to move 5 units so we SetDistance(5), so how can we get live data to pull that '5' to the dashboard
 * what the heck is setangle
 * 			make a turnPID function for setting the turnPID. thanks
 *
 *
 */
#ifndef SRC_DRIVETRAIN_H_
#define SRC_DRIVETRAIN_H_

#include <cmath>
#include "WPILib.h"
#include "AHRS.h"
#include "RobotUtils/RobotUtils.h"

/*
 * Talons' Can Bus Location
 */

#define TALON_DRIVE_LF 20
#define TALON_DRIVE_LR 21
#define TALON_DRIVE_RF 22
#define TALON_DRIVE_RR 23

/*
 * Drive Encoders' DIO Location
 */

#define DRIVE_ENCODER_LF 0
#define DRIVE_ENCODER_LR 1
#define DRIVE_ENCODER_RF 2
#define DRIVE_ENCODER_RR 3

/*
 *  Gear Shift CAN Location
 */

#define SOLENOID_SHIFT 0

/*
 * PID coefficients for turning
 */
#define TURN_P 0
#define TURN_I 0
#define TURN_D 0

#define SPANGLE_P 0
#define SPANGLE_I 0
#define SPANGLE_D 0

#define DISTANCE_SHIFTL_P 0.12
#define DISTANCE_SHIFTL_I 0
#define DISTANCE_SHIFTL_D 0

#define DISTANCE_SHIFTH_P 0.045
#define DISTANCE_SHIFTH_I 0
#define DISTANCE_SHIFTH_D 0

//class TurnPIDWrapper;
class DistancePIDWrapper;
class SpanglePIDWrapper;
class AnglePIDWrapper;

class Drivetrain : public HotSubsystem {
public:
	Drivetrain(HotBot* bot);

	/******************************
	 * 	Motor
	 ******************************/
	void ArcadeDrive(float speed, float turn, int priority = 0);

	void SetShift(bool on, int priority = 0);
	void ShiftHigh(int priority = 0);
	void ShiftLow(int priority = 0);

	void Update();

	/******************************
	 *	Sensors
	 ******************************/
	double GetLDistance();
	double GetRDistance();
	double GetDistance();

	double GetLSpeed();
	double GetRSpeed();
	double GetSpeed();

	double GetAngle();
	double GetAngularSpeed();

	/******************************
	 * 	Distance PID
	 ******************************/
	void EnableDistancePID(int priority = 0);
	void DisableDistancePID(int priority = 0);

	bool IsDistancePIDEnabled();

	void SetDistancePIDSetpoint(double inches);

	double GetDistancePIDSetpoint();

	bool DistanceAtPIDSetpoint();

	/******************************
	 * 	Distance PID Wrapper
	 ******************************/
	class DistancePIDWrapper : public PIDSource, public PIDOutput {
	public:
		DistancePIDWrapper(Drivetrain *drivetrain);

		void PIDWrite(float output);
		double PIDGet();
	private:
		Drivetrain *m_drivetrain;
	};

	/******************************
	 * 	Angle PID
	 ******************************/
	void EnableAnglePID(int priority = 0);
	void DisableAnglePID(int priority = 0);

	bool IsAnglePIDEnabled();

	void SetAnglePIDSetpoint(double degree);

	double GetAnglePIDSetpoint();

	bool AngleAtPIDSetpoint();

	/******************************
	 * 	Angle PID Wrapper
	 ******************************/
	class AnglePIDWrapper : public PIDSource, public PIDOutput {
	public:
		AnglePIDWrapper(Drivetrain *drivetrain);

		void PIDWrite(float output);
		double PIDGet();
	private:
		Drivetrain *m_drivetrain;
	};

private:
	void _ArcadeDrive(float speed, float turn);
	void _SetShift(bool on);
	void _EnableDistancePID();
	void _DisableDistancePID();
	void _EnableAnglePID();
	void _DisableAnglePID();

	/*
	 * 	Talons
	 */
	CANTalon *m_lfDrive, *m_lrDrive, *m_rfDrive, *m_rrDrive;
	RobotDrive* m_drive;
	DoubleBuffer *m_speedBuf, *m_turnBuf;

	/*
	 * 	Shifting
	 */
	Solenoid *m_shift;
	BooleanBuffer *m_shiftBuf;

	/*
	 * 	Encoders
	 */
	Encoder *m_lEncoder, *m_rEncoder;

	AHRS *m_gyro;

	/*
	 * 	PID
	 */
	PIDController* m_anglePID;
	PIDController* m_distancePID;
	BooleanBuffer *m_anglePIDBuf, *m_distancePIDBuf;
};

#endif /* SRC_DRIVETRAIN_H_ */
