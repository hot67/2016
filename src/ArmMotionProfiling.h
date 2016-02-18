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

/*
 * An Enum used to handle the state of the motion profiling of the talon.
 */
enum MotionProfileStates {
	kRunning = 0,
	kStopped = 1,
	kPaused = 2
};

class ArmMotionProfiling {

	/*
	 * The Talon
	 */
	CANTalon * m_Talon;

	/*
	 * Utilizes the previou Enums.
	 * handle the state of motion profiling
	 */
	MotionProfileStates mpState;

	/*
	 * A class for our trajectory points.
	 * Will be handled in dynamic memory
	 */
	Trajectory* m_trajectory;

	/*
	 * The delta time. Change in time over time.
	 */
	float m_deltaTime;

	/*
	 * A simpler form of mpState,
	 * for IsEnabled();
	 */
	bool MP;

	/*
	 * Internal Functions for generating points
	 * and for giving them to the talon
	 */
	void GeneratePoints();
	void PrepProfiling();

public:

	/*
	 * Setup the points and begin profiling
	 */
	void BeginProfiling(
			float current_position,
			float current_velocity,
			float target_position,
			float max_V,
			float max_A,
			float deltaTime);

	/*
	 * Stop Motion Profiling
	 */
	void EndProfiling();

	/*
	 * A control function to happen
	 * half of the delta time.
	 * Must use threading, is threading safe.
	 */
	void Iterate();

	/*
	 * Pause motion profiling
	 */
	void Pause();

	/*
	 * Resume Motion Profiling
	 */
	void UnPause();

	/*
	 *	Another Internal function, may be called manually.
	 *	Moves the used points of the Talon to the back of it's buffer.
	 */
	void Process();

	/*
	 * Is motion profiling running? returns a bool.
	 */
	bool IsEnabled();

	/*
	 * Constructor. Only takes a talon.
	 * Everything else is passed later.
	 */
	ArmMotionProfiling(CANTalon* inputTalon);

};



#endif /* MotionProfiling_H_ */
