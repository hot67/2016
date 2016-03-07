/*
 * SpeedControllerWrapper.h
 *
 *  Created on: Mar 7, 2016
 *      Author: Jakob
 */

#ifndef SPEEDCONTROLLERWRAPPER_H_
#define SPEEDCONTROLLERWRAPPER_H_

#include <PIDOutput.h>
#include <PIDSource.h>

/*
 * This is my first attempt at the all in one wrapper!
 * It handles internally the setting of all motor functions
 * for up to two motors, along with PIDController functions
 */
class SpeedControllerWrapper: public PIDController, public PIDOutput, public PIDSource {
	// The main speed controller
	SpeedController *m_controllerMain;

	// A slave speed controller, if it is provided
	SpeedController *m_controllerSlave;

	// The last set value given to the controller
	int lastValue;

	// Whether we are using a slave motor
	bool isFollowing;

	// Whether slave motor is inverted
	bool isInverted;

	// The encoder / sensor
	PIDSource * m_sensor;

public:

	/*
	 * Constructor
	 * This takes two speedcontroller,
	 * so they can be a Talon, CanTalon, Jaguar, etc.
	 */
	SpeedControllerWrapper(SpeedController *motor, SpeedController *slave);

	/*
	 * Constructor
	 * This takes a speedcontroller,
	 * so it can be a Talon, CanTalon,Jaguar, etc.
	 */
	SpeedControllerWrapper(SpeedController *motor);

	/*
	 * Constructor
	 * In addition to controllers,
	 * this takes the default PID
	 */
	SpeedControllerWrapper(SpeedController *motor, int p, int i, int d);

	/*
	 * Constructor
	 * In addition to controllers,
	 * this takes the default PID
	 */
	SpeedControllerWrapper(SpeedController *motor, SpeedController *slave, int p, int i, int d);

	/*
	 * Constructors with sensors
	 */
	SpeedControllerWrapper(SpeedController *motor, int p, int i, int d, PIDSource *sensor);
	SpeedControllerWrapper(SpeedController *motor, SpeedController *slave, int p, int i, int d, PIDSource *sensor);

	/*
	 * Manual control of the motors.
	 */
	void Set(int amount);
	void PIDWrite(int amount);

	/*
	 * Follower motor operations
	 */
	void SetFollower(SpeedController *follower);
	void EnableFollowing();
	void DisableFollowing();
	void EnableInverting();
	void DisableInverting();

	/*
	 * Overridden methods for enabling and disabling
	 * When disabled, the latest buffered Set value will be given to the talon
	 */
	void Enable();
	void Disable();

	/*
	 * Sensors
	 */
	int PIDGet();
	int GetSensorValue();
	void SetSensor(PIDSource *encoder);
};

#endif /* SPEEDCONTROLLERWRAPPER_H_ */
