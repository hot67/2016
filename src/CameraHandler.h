/*
 * CameraHandler.h
 *
 *  Created on: Feb 18, 2016
 *      Author: Robo08
 */
#include "TurnPIDWrapper.h"
#include "Drivetrain.h"
#ifndef SRC_CAMERAHANDLER_H_
#define SRC_CAMERAHANDLER_H_

class CameraHandler {
public:
	CameraServer *m_camera;
	Joystick *m_driver;
	Joystick *m_operator;
	Timer *m_camTimer;
	int m_camArray [9];
	float m_dataSum = (m_camArray [0] + m_camArray [1] + m_camArray [2] + m_camArray [3] + m_camArray [4]
		+ m_camArray [5] + m_camArray [6] + m_camArray [7] + m_camArray [8] + m_camArray [9]);
	CameraHandler();

	virtual ~CameraHandler();

	/******************************
	 * 	Simple Reading
	 ******************************/
	/**
	 * 	Get Normalized X (-1.0, 1.0)
	 * 		This value is written as "normalizedX" on SmartDashboard
	 */
	double GetNormalizedX();

	/**
	 * 	Get Normalized Y (-1.0, 1.0)
	 * 		This value is written as "normalizedY" on SmartDashboard
	 */
	double GetNormalizedY();

	/**
	 * 	Get Distance
	 * 		This value is written as "distance" on SmartDashboard
	 */
	double GetDistance();

	/**
	 * 	Average Normalized X
	 * 		Take average of previous 10 values
	 */
	double GetAveNormalizedX();

	/**
	 * 	Average Normalized Y
	 */
	double GetAveNormalizedY();

	double GetWidth();

	double GetHeight();

	void ToggleFirstCamera();

	void ToggleSecondCamera();

	void AutoLineUp();

	void DisablePID();

	void EnablePID();

	double GetAveRate();
};

#endif /* SRC_CAMERAHANDLER_H_ */
