/*
 * TurnPIDWrapper.cpp
 *
 *  Created on: Feb 4, 2016
 *      Author: Chad
 */

#include <TurnPIDWrapper.h>

TurnPIDWrapper::TurnPIDWrapper(Drivetrain* drivetrain){
	m_drivetrain = drivetrain;

}

TurnPIDWrapper::~TurnPIDWrapper() {
	// TODO Auto-generated destructor stub
}

void TurnPIDWrapper::PIDWrite(float output) {
	m_drivetrain->SetTurn(output);
}

double TurnPIDWrapper::PIDGet(){
	return(m_drivetrain->GetAngle());
}
