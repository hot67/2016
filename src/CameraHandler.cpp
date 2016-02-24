/*
 * CameraHandler.cpp
 *
 *  Created on: Feb 18, 2016
 *      Author: Robo08
 */
#include "WPIlib.h"
#include "CameraHandler.h"
#include "Drivetrain.h"


using namespace std;

CameraHandler::CameraHandler() {
	// TODO Auto-generated constructor stub
	m_driver = new Joystick(1);
	m_operator = new Joystick(2);
	m_camTimer = new Timer;
	Drivetrain::m_turnPIDWrapper = new TurnPIDWrapper(this);

}

CameraHandler::~CameraHandler() {

}
//All the values one could ever wish for (maybe):
double CameraHandler::GetNormalizedX() {
	return SmartDashboard::GetNumber("normalizedX", 0.0);
}
double CameraHandler::GetNormalizedY(){
	return SmartDashboard::GetNumber("normalizedY", 0.0);
}
double CameraHandler::GetDistance(){
	return SmartDashboard::GetNumber("distance", 0.0);
}
double CameraHandler::GetAveNormalizedX(){
	for (int x = 0; x < 10; x++){
		m_camera->GetInstance();
		float normX = SmartDashboard::GetNumber("normalizedX", 0.0);
		normX = m_camArray [x];
	}
	return m_dataSum/9;
}

double CameraHandler::GetWidth(){
	return SmartDashboard::GetNumber("width", 0.0);
}

double CameraHandler::GetHeight(){
	return SmartDashboard::GetNumber("height", 0.0);
}

void CameraHandler::ToggleFirstCamera(){ //Toggle cam1
			m_camera = CameraServer::GetInstance();
			m_camera->StartAutomaticCapture("cam0");
			m_camera->SetQuality(75);
		}

void CameraHandler::ToggleSecondCamera(){ // Toggle cam2
			m_camera = CameraServer::GetInstance();
			m_camera->StartAutomaticCapture("cam1");
			m_camera->SetQuality(75);
		}
void CameraHandler::AutoLineUp(){ //Needs actual field values. Zero's used as placeholders
	if (0 > CameraHandler::GetNormalizedX() > 0 ){
	Drivetrain::m_turnPID->SetSetpoint(1);
	}
	else if (0 > CameraHandler::GetNormalizedX() > 0 ){ //Gonna have quite a few of these. Or Switch/Case.
	Drivetrain::m_turnPID->SetSetpoint(1);
	}
	else {
		Drivetrain::m_turnPID->SetSetpoint(0);
	}
}
double CameraHandler::GetAveRate(){ //Testing
	double widthO = SmartDashboard::GetNumber("width", 0.0);
	m_camTimer->Reset();
	m_camTimer->Start();
	Wait(1.00);
	m_camTimer->Stop();
	m_camTimer->Reset();
	double widthF = SmartDashboard::GetNumber("width", 0.0);
	double AveRate = (widthO-widthF);
	return AveRate;
}
void CameraHandler::EnablePID(){
	Drivetrain::m_turnPID->Enable;

}
void CameraHandler::DisablePID(){
	Drivetrain::m_turnPID->Disable;
}


