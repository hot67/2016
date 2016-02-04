/*
 * TurnPIDWrapper.cpp
 *
 *  Created on: Feb 4, 2016
 *      Author: Chad
 */

#include <TurnPIDWrapper.h>

TurnPIDWrapper::TurnPIDWrapper(RobotDrive* drive) {
	// TODO Auto-generated constructor stub
	m_drive = drive;

}

TurnPIDWrapper::~TurnPIDWrapper() {
	// TODO Auto-generated destructor stub
}

void TurnPIDWrapper::PIDWrite(float output) {
		m_drive->TankDrive(output,-output);
}
