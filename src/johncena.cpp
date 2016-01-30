#include "WPILib.h"
#include "RobotUtils/RobotUtils.h"

class Johncena : public HotBot {
private:
	//	Joysticks
	HotJoystick *m_driver, *m_operator;

public:
	Johncena () {
		//	Initialize Joysticks
		m_driver = new HotJoystick(0);
		m_operator = new HotJoystick(1);
	}

	/**
	 * 	Initializations
	 */

	/**
	 * 	Periods
	 */

};

START_ROBOT_CLASS(Johncena);
