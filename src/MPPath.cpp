/*
 * MPPath.cpp
 *
 *  Created on: Apr 18, 2016
 *      Author: Jin
 */
#include<math.h>
#include "MPPath.h"

MPPath::MPPath(double Vmax, double Amax, double Amin, double target) {
	m_Vmax = Vmax;
	m_Amax = Amax;
	m_Amin = Amin;
	m_target = target;

    double dt1 = Vmax / Amax;
    double dt3 = -Vmax / Amin;
    double tailP = (dt1+dt3)*Vmax/2;

    if (target > tailP) {
		m_t1 = dt1;
		m_p1 = dt1 * Vmax / 2;
		//m_v1 = Vmax;

		m_t2 = m_t1 + (target-tailP) / Vmax;
		m_p2 = target - dt3*Vmax/2;
		//m_v2 = Vmax;

		m_t3 = m_t2+dt3;
		m_p3 = target;
		//m_v3 = 0;
    } else {
    	double W = (-Amax+Amin)/(Amax*Amin);
    	double v = sqrt(2*target/W);
    	m_t1 = m_t2 = v/Amax;
        m_p1 = m_p2 = Amax * m_t1 * m_t1 / 2;

        m_t3 = sqrt(2*target*W);
        m_p3 = target;
    }
}

double MPPath::GetP(double t) {
    if (t < m_t1) {
    	return m_Amax * t * t / 2;
    } else if (t < m_t2) {
    	return m_p1 + m_Vmax * (t-m_t1);
    } else if (t < m_t3) {
    	return m_p2 + m_Amin * (t-m_t2) * (t-m_t2) / 2 + m_Vmax * (t-m_t2);
    } else {
    	return m_p3;
    }
}

