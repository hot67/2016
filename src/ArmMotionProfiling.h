/*
 * MotionProfiling.h
 *
 *  Created on: Feb 6, 2016
 *      Author: Jakob
 */

#ifndef MotionProfiling_H_
#define MotionProfiling_H_

#include "WPILib.h"
#include "Trajectory.h"

#define PRACTICE_BOT
//#define COMPETITION_BOT

#ifdef PRACTICE_BOT

#define ARM_DELTA_TIME 20 //ms

#endif

#ifdef COMPETITION_BOT

#define ARM_DELTA_TIME 20 //ms

#endif

enum MotionProfileStates { //States of the motion profile. used during iteration to see what we should be doing.
	kRunning = 0,
	kStopped = 1,
	kPaused = 2
};

class ArmMotionProfiling {

	CANTalon * m_Talon; //The talon we are going to profile

	float talonStatus; //Used for checking status of talon in rest of code. dont feed it points when its busy!

	MotionProfileStates mpState; //State of the motion profiling

	Trajectory* m_trajectory;

	float m_deltaTime; //Delta Time!! Change in time over time.

	bool MP; //Want to profile yet?

	void GeneratePoints(); //Generate the motion profile points

	void PrepProfiling(); //Begin the motion profiling.
public:

	void EndProfiling(); //End the motion profiling

	void Iterate(); //A control function to happen half the delta time

	void Pause(); //Pause the motion profiling
	void UnPause(); //Unpause the motion profiling

	void Process(); //Process the motion profile buffer

	bool IsEnabled();


	void BeginProfiling( //Actually set up creation. Calls PrepProfiling() and GeneratePoints()
			float current_position,
			float current_velocity,
			float target_position,
			float max_V,
			float max_A,
			float deltaTime);

	/*
	 * Generate the motion profile for said point.
	 * Can be used instead of passing an array to the constructor
	 */
	ArmMotionProfiling(CANTalon* inputTalon);
	/*
	 * Constructor. maxA, maxV, and deltaTime will be defaulted to the constants at the top of this header
	 */

};



#endif /* MotionProfiling_H_ */
