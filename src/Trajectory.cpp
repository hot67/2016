/*
 * Trajectory.cpp
 *
 *  Created on: Feb 6, 2016
 *      Author: Jakob
 */

#include <Trajectory.h>


/*
Trajectory::Trajectory(float maxVelocity, float maxAcceleration, float initialVelocity, float time, int graphPos, bool last) {

	switch (graphPos) {

	case 0: //We are still accelerating
		VelocityVar = ( 2 * maxAcceleration * time) + initialVelocity;

		PositionVar = ( maxAcceleration * (time * time)) + ( initialVelocity * time );
		if (last) {tmp = PositionVar;}
		break;

	case 1: //We have reached maximum velocity
		VelocityVar = maxVelocity;

		PositionVar = ( maxVelocity * time) + tmp;
		if (last) {tmp = PositionVar;}
		break;

	case 2: //We are deccelerating
		VelocityVar = ( 2 * -maxAcceleration * time) + maxVelocity;

		PositionVar = ( -maxAcceleration * (time * time)) + ( maxVelocity * time ) + tmp;
	}

}
*/

Trajectory::Trajectory(float initialV, float initialPos, float goal, float maxV, float maxA) :
		m_initialV(initialV),
		m_initialPos(initialPos),
		m_goal(goal),
		m_maxV(maxV),
		m_maxA(maxA) {

	time_1 = ( m_maxV - m_initialV ) / m_maxA;
	time_2 = ( ( m_goal - m_initialPos ) - (m_maxA * ( time_1 * time_1 ) + ( m_maxV * time_1 ) ) );
	time_3 = distance_2 + (m_maxV / m_maxA);

	distance_1 = ( ( m_maxA * time_1 ) / 2 ) + ( m_initialV * time_1 );
	distance_2 = ( distance_1 + ( m_maxV * ( time_2 - time_1 ) ) );
	distance_3 = m_goal - m_initialPos;

}

float Trajectory::Velocity(float time) {

}

float Trajectory::Position(float time) {

}
