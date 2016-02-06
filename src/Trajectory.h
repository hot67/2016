/*
 * Trajectory.h
 *
 *  Created on: Feb 6, 2016
 *      Author: Jakob
 */

#ifndef SRC_TRAJECTORY_H_
#define SRC_TRAJECTORY_H_

#define DELTA_TIME 0


class Trajectory {

	float m_initialV;
	float m_initialPos;
	float m_goal;
	float m_maxV;
	float m_maxA;

	float distance_1; //Distance it is accelerating
	float distance_2; //Distance it is at maximum velocity
	float distance_3; //Distance it is deccelerating

	float time_1; //Time it is accelerating
	float time_2; //Time it is at maximum velocity
	float time_3; //Time it is deccelerating

public:
	Trajectory(float initialV, float initialPos, float goal, float maxV, float maxA); //initializes the requirements for calculating profile

	float Velocity(float time); //Returns velocity
	float Position(float time); //Returns position
};

#endif /* SRC_TRAJECTORY_H_ */
