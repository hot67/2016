/*
 * Trajectory.cpp
 *
 *  Created on: Feb 6, 2016
 *      Author: Jakob
 */

#include <Trajectory.h>


/*
 * A class to handle the calculation of Trajectory Points.
 */
Trajectory::Trajectory(float initialV, float initialPos, float goal, float maxV, float maxA) :
		m_initialV(initialV),
		m_initialPos(initialPos),
		m_goal(goal),
		m_maxV(maxV),
		m_maxA(maxA) {

	time_1 = ( m_maxV - m_initialV ) / m_maxA; //computes time for acceleration
	time_2 = ( ( m_goal - m_initialPos ) - (m_maxA * ( time_1 * time_1 ) + ( m_maxV * time_1 ) ) ); //computes time for max velocity
	time_3 = distance_2 + (m_maxV / m_maxA); //computes time for decceleration

	distance_1 = ( ( m_maxA * time_1 ) / 2 ) + ( m_initialV * time_1 ); //computes distance for acceleration
	distance_2 = ( distance_1 + ( m_maxV * ( time_2 - time_1 ) ) ); //computes distance for max velocity
	distance_3 = m_goal - m_initialPos; //computes distance for decceleration

}

/*
 * Returns the velocity given a time.
 */
float Trajectory::Velocity(float time) { //returns the velocity given a time.

	if (time<time_1) { //Accelerating

		return (m_maxA * time) + m_initialV;

	}

	else if (time<time_2) { //Max Velocity

		return (m_maxV);

	}

	else if (time<time_3) { //Deccelerating

		return m_maxV - ( m_maxA * ( time - time_2 ) );

	}

	return 0.0;

}

/*
 * Returns the Position given a time.
 */
float Trajectory::Position(float time) {

	if (time<time_1) {
		/*
		 * Accelerating
		 */

		return ( ( m_maxA * ( time * time) ) / 2) + ( m_initialV * time);

	}

	else if (time<time_2) {
		/*
		 * Maximum Velocity
		 */

		return distance_1 + ( m_maxV * ( time - time_1) );

	}

	else if (time<time_3) {
		/*
		 * Deccelerating
		 */

		return distance_2 - ( ( m_maxA * ( (time - time_2) * (time - time_2 ) ) ) / 2);

	}

	return distance_3;

}

/*
 * Amount of trajectory points it will take to reach the goal,
 * given a delta time in Ms.
 */
float Trajectory::DataLen(float deltatime) {
	return (time_3/1000) / deltatime;
}
