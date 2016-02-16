#include "WPILib.h"
#include "RobotUtils/RobotUtils.h"

class Johncena : public HotBot {
private:
	//	Joysticks
	HotJoystick *m_driver, *m_operator;
public:
	std::shared_ptr<CameraServer> camera;

	Johncena () {
		//	Initialize Joysticks
		m_driver = new HotJoystick(0);
		m_operator = new HotJoystick(1);
		camera = CameraServer::GetInstance();
	}
void TeleopPeriodic(){
	if (m_driver->GetRawButton(0)){
		camera->StartAutomaticCapture("cam0");
	}
	else if (m_driver->GetRawButton(1)){
		camera->StartAutomaticCapture("cam1");
	}

};

};

START_ROBOT_CLASS(Johncena);
