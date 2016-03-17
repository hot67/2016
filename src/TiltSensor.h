/*
 * TiltSensor.h
 *
 *  Created on: Mar 9, 2016
 *      Author: Rodney
 */

#ifndef TILTSENSOR_H_
#define TILTSENSOR_H_
#include "WPILib.h"

class TiltSensor : public PIDSource
{
public:
	TiltSensor(ADXL345_I2C* accelerometer, AnalogGyro* gyro, double tau);
	virtual ~TiltSensor();

	double UpdateAngle();
	double GetAngle();
	double PIDGet();

private:
	void InitTiltSensor();
	double GetNewAccelAngle();
	double GetNewGyroRate();

	ADXL345_I2C* m_accelerometer;
	AnalogGyro* m_gyro;
	double m_tau;		// response time
	double m_alpha;		// = tau / (tau + dT)

	Timer *m_deltaT;
	double m_angle;
};
#endif
