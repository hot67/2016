/*
 * MPController.h
 *
 *  Created on: Apr 18, 2016
 *      Author: Jin
 */

#ifndef SRC_MPCONTROLLER_H_
#define SRC_MPCONTROLLER_H_

#include "WPILib.h"
#include "MPPath.h"

class MPController : public PIDController {
private:
	double m_Vmax, m_Amax, m_Amin;
	MPPath *m_path = NULL;
	Timer *m_timer;

public:
	MPController(double Kp, double Ki, double Kd, double Vmax, double Amax, double Amin,
			PIDSource *source, PIDOutput *output);

	void SetSetpoint(double target);
	void Enable();
	void Disable();
	void Update();
};

#endif /* SRC_MPCONTROLLER_H_ */
