#include "WPILib.h"
#include "RobotUtils/RobotUtils.h"


enum obstacle {

	kLowBar,
	kMoat,
	kPortCullis,
	kRamparts,
	kChevalDeFris,
	kDrawbridge,
	kSallyPort,
	kRoughTerrain

};

struct auton_choice {

	bool high;
	bool low;
	obstacle Obstacle;

};

class Johncena : public HotBot {
private:
	//	Joysticks
	HotJoystick *m_driver, *m_operator;
	auton_choice m_autonChoice;

public:
	Johncena () {
		//	Initialize Joysticks
		m_driver = new HotJoystick(0);
		m_operator = new HotJoystick(1);

		m_autonChoice = {true, false, kLowBar};

		bool m_autonWorking = false;
	}

	/**
	 * 	Initializations
	 */

	void RobotInit() {
		/*
		 * Do Robot Init
		 */
	}

	void DisabledPeriodic() {
		/*
		 * Auton Choices
		 */

		if (m_operator->GetRawButton(1)) { //High Goal Low Bar. A button
			m_autonChoice = {true, false, kLowBar};
		} else if (m_operator->GetRawButton(2)) { //Low Goal Low Bar. B button
			m_autonChoice = {false, true, kLowBar};
		} else if (m_operator->GetRawButton(3)) { //High Goal Moat. X button
			m_autonChoice = {true, false, kMoat};
		} else if (m_operator->GetRawButton(4)) { //Low Goal Moat. Y button
			m_autonChoice = {false, true, kMoat};
		} else if (m_operator->GetRawButton(5)) { //High Goal Portcullis. Left Bumper.
			m_autonChoice = {true, false, kPortCullis};
		} else if (m_operator->GetRawButton(6)) { //Low Goal Portcullis. Right Bumper.
			m_autonChoice = {false, true, kPortCullis};
		} else if (m_operator->GetRawButton(7)) { //High Goal Ramparts. Back.
			m_autonChoice = {true, false, kRamparts};
		} else if (m_operator->GetRawButton(8)) { //Low Goal Ramparts. Start.
			m_autonChoice = {false, true, kRamparts};
		} else if (m_operator->GetRawButton(9)) { //High Goal Rough Terrain. Left Stick.
			m_autonChoice = {true, false, kRoughTerrain};
		} else if (m_operator->GetRawButton(10)) { //Low Goal Rough Terrain. Right Stick.
			m_autonChoice = {false, true, kRoughTerrain};
		}

	}

	void AutonomousInit() {
		/*
		 * Initialize auton stuff.
		 */
		switch (m_autonChoice.Obstacle);
	}

	/**
	 * 	Periods
	 */

};

START_ROBOT_CLASS(Johncena);
