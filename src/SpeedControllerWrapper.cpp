/*
 * SpeedControllerWrapper.cpp
 *
 *  Created on: Mar 7, 2016
 *      Author: Jakob
 */

#include "SpeedControllerWrapper.h"

/*
 * Constructor
 */
SpeedControllerWrapper::SpeedControllerWrapper(SpeedController *motor, SpeedController *slave)
{

	m_controllerMain = motor;
	m_controllerSlave = slave;
	isFollowing = true;
	isInverted = false;
	lastValue = 0;
}



SpeedControllerWrapper::SpeedControllerWrapper(SpeedController *motor)
{
	m_controllerMain = motor;
	isFollowing = false;
	isInverted = true;
	lastValue = 0;
}



SpeedControllerWrapper::SpeedControllerWrapper(SpeedController *motor, int p, int i, int d) :
	PIDController(p, i, d, this, this)
{

	m_controllerMain = motor;
	isFollowing = false;
	isInverted = false;
	lastValue = 0;
}



SpeedControllerWrapper::SpeedControllerWrapper(SpeedController *motor, SpeedController *slave, int p, int i, int d) :
		PIDController(p, i, d, this, this)
{

	m_controllerMain = motor;
	m_controllerSlave= slave;
	isFollowing = true;
	isInverted = false;
	lastValue = 0;
}



SpeedControllerWrapper::SpeedControllerWrapper(SpeedController *motor, int p, int i, int d, PIDSource *sensor) :
	PIDController(p, i, d, this, this)
{

	m_controllerMain = motor;
	m_sensor = sensor;
	isFollowing = false;
	isInverted = false;
	lastValue = 0;
}



SpeedControllerWrapper::SpeedControllerWrapper(SpeedController *motor, SpeedController *slave, int p, int i, int d, PIDSource *sensor) :
		PIDController(p, i, d, this, this)
{

	m_controllerMain = motor;
	m_sensor = sensor;
	m_controllerSlave= slave;
	isFollowing = true;
	isInverted = false;
	lastValue = 0;
}



void SpeedControllerWrapper::Set(int amount)
{
	if (!PIDController::IsEnabled())
	{
		PIDWrite(amount);
	}
	else
	{
		lastValue = amount;
	}
}



void SpeedControllerWrapper::PIDWrite(int amount)
{
	if (isFollowing)
	{
		if (isInverted)
		{
			m_controllerSlave->Set(-amount);
		}
		else
		{
			m_controllerSlave->Set(amount);
		}
	}
	m_controllerMain->Set(amount);
}


void SpeedControllerWrapper::SetFollower(SpeedController *follower)
{
	m_controllerSlave = follower;
}



void SpeedControllerWrapper::EnableFollowing()
{
	isFollowing = true;
}



void SpeedControllerWrapper::DisableFollowing()
{
	isFollowing = false;
}

void SpeedControllerWrapper::EnableInverting()
{
	isInverted = true;
}



void SpeedControllerWrapper::DisableInverting()
{
	isInverted = false;
}



void SpeedControllerWrapper::Enable()
{
	Set(0);
	PIDController::Enable();
}



void SpeedControllerWrapper::Disable()
{
	PIDController::Disable();
	Set(lastValue);
}



int SpeedControllerWrapper::PIDGet()
{
	return m_sensor->PIDGet();
}



int SpeedControllerWrapper::GetSensorValue()
{
	return PIDGet();
}



void SpeedControllerWrapper::SetSensor(PIDSource *sensor)
{
	m_sensor = sensor;
}
