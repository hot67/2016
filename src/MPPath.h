/*
 * MPPath.h
 *
 *  Created on: Apr 18, 2016
 *      Author: Jin
 */

#ifndef MPPATH_H_
#define MPPATH_H_

class MPPath {
private:
	double m_Vmax, m_Amax, m_Amin, m_target;
	double m_t1, m_t2, m_t3;
	double m_p1, m_p2, m_p3;
public:
	MPPath(double Vmax, double Amax, double Amin, double target);

	double GetP(double t);
};

#endif /* MPPATH_H_ */
