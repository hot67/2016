/*
 * MotionProfiling.h
 *
 *  Created on: Feb 6, 2016
 *      Author: Jakob
 */

#ifndef MOTIONPROFILING_H_
#define MOTIONPROFILING_H_

#include "WPILib.h"

class MotionProfiling {

	CANTalon * m_Talon; //The talon we are going to profile

	float talonStatus; //Used for checking status of talon in rest of code. dont feed it points when its busy!

	int pointsLen; //Length of the array that points points to

	float * points; //Motion Profile Trajectory points (should point to the beginning of an array)

	bool startMP; //Start motion profiling yet?


public:

	void BeginProfiling(); //Begin the motion profiling

	void GiveBuffer(); //Pass the motion profile points to the talon's buffer

	void EndProfiling(); //End the motion profiling

	void Iterate(); //A control function to happen every teleop iteration

	MotionProfiling(CANTalon* inputTalon, int * inputPoints, int inputLength) : m_Talon(inputTalon), points(inputPoints), pointsLen(inputLength) {
		talonStatus = 0;
		startMP = false;
	}



};



#endif /* MOTIONPROFILING_H_ */
