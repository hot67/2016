/*
 * ArmMotionProfiling.h
 *
 *  Created on: Feb 6, 2016
 *      Author: Jakob
 */

#ifndef ArmMotionProfiling_H_
#define ArmMotionProfiling_H_

#include "WPILib.h"
#include "TestMotionProfile.h"
#include "Trajectory.h"

#define PRACTICE_BOT
//#define COMPETITION_BOT

#ifdef PRACTICE_BOT

#define ARM_MAX_VELOCITY 0
#define ARM_MAX_ACCELERATION 0
#define ARM_DELTA_TIME 1 //ms

#endif

#ifdef COMPETITION_BOT
#endif

enum MotionProfileStates {
	kBuffered = 0,
	kUnBuffered = 1,
	kStopped = 2,
	kPaused = 3
};

class ArmMotionProfiling {

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

	void GenerateMotionProfiles(float targetPoint, float initialVelocity);
	/*
	 * Generate the motion profile for said point.
	 * Can be used instead of passing an array to the constructor
	 */

	ArmMotionProfiling(CANTalon* inputTalon, int * inputPoints, int inputLength) : m_Talon(inputTalon), points(inputPoints), pointsLen(inputLength) {
		talonStatus = 0;
		mpState = kStopped;
		MP = false;
	}



};



#endif /* ArmMotionProfiling_H_ */
