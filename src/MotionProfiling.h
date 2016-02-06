/*
 * MotionProfiling.h
 *
 *  Created on: Feb 6, 2016
 *      Author: Jakob
 */

#ifndef MOTIONPROFILING_H_
#define MOTIONPROFILING_H_

#include "WPILib.h"

enum MotionProfileStates {
	kBuffered = 0,
	kUnBuffered = 1,
	kStopped = 2,
	kPaused = 3
};

class MotionProfiling {

	CANTalon * m_Talon; //The talon we are going to profile

	float talonStatus; //Used for checking status of talon in rest of code. dont feed it points when its busy!

	int pointsLen; //Length of the array that points points to

	float * points; //Motion Profile Trajectory points (should point to the beginning of an array)

	MotionProfileStates mpState; //State of the motion profiling


public:

	bool MP; //Want to profile yet?

	void BeginProfiling(); //Begin the motion profiling

	void GiveBuffer(); //Pass the motion profile points to the talon's buffer

	void EndProfiling(); //End the motion profiling

	void Iterate(); //A control function to happen every teleop iteration

	void Pause(); //Pause the motion profiling
	void UnPause(); //Unpause the motion profiling

	void Process(); //Process the motion profile buffer

	MotionProfiling(CANTalon* inputTalon, int * inputPoints, int inputLength) : m_Talon(inputTalon), points(inputPoints), pointsLen(inputLength) {
		talonStatus = 0;
		mpState = kStopped;
		MP = false;
	}



};



#endif /* MOTIONPROFILING_H_ */
