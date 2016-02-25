#include "WPILib.h"
#include "RobotUtils/RobotUtils.h"
#include "Drivetrain.h"
#include "Arm.h"
#include "Intake.h"
#include "AHRS.h"

#define DISTANCE_TO_FIRST_OBSTACLE 7.5 //feet
#define DISTANCE_TO_GOAL_SHOOT_POSITION 2 //feet
#define DISTANCE_TO_CROSS_OBSTACLE 4 //feet

/*
 * Time to run intake motors outwards
 * to score in the respective goal.
 */
#define INTAKE_FIRE_TIME_HIGH 1 //second
#define INTAKE_FIRE_TIME_LOw 1 //second

/*
 * Aiming angles for when we are actually off of the obstacle.
 * 1 is the far left
 * 5 is the far right
 */
#define HIGH_SHOOT_ANGLE_OBSTACLE1 45
#define HIGH_SHOOT_ANGLE_OBSTACLE2 35
#define HIGH_SHOOT_ANGLE_OBSTACLE3 0
#define HIGH_SHOOT_ANGLE_OBSTACLE4 -35
#define HIGH_SHOOT_ANGLE_OBSTACLE5 -45

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

enum autonStage {
	kBeforeObstacle,
	kOnObstacle,
	kPastObstacle,
	kShooting
};

enum side {
	kLeft,
	kRight,
	kCenter
};

struct auton_choice {

	bool high;
	bool low;
	obstacle Obstacle;
	side Side;
};

class Johncena : public HotBot {
private:
	//	Joysticks
	HotJoystick *m_driver, *m_operator;

	auton_choice m_autonChoice;
	autonStage m_autonStage;
	int m_autonAngle; //Angle to turn after passing the obstacle should be one of the HIGH_SHOOT_ANGLE constants

	Drivetrain * m_drivetrain;
	Arm * m_arm;
	Intake * m_intake;
	int m_shootSeconds; //how long the shooter has been running;

public:
	Johncena () {
		//	Initialize Joysticks
		m_driver = new HotJoystick(0);
		m_operator = new HotJoystick(1);
		m_drivetrain = new Drivetrain(this);
		m_arm = new Arm(this);
		m_intake = new Intake(this);
		m_shootSeconds = 0;

		m_autonChoice = {true, false, kLowBar};
		m_autonStage = kBeforeObstacle;

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
		 * Auton Choices. We may not need these (as far as I know)
		 * This is because right now my code for traversing all of the obstacles is the same.
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

		if (m_driver->GetRawButton(1)) { //Obstacle 1. Far left. A button
			m_autonAngle = HIGH_SHOOT_ANGLE_OBSTACLE1;
		} else if (m_driver->GetRawButton(2)) { //Obstacle 2. Second from the left. B button
			m_autonAngle = HIGH_SHOOT_ANGLE_OBSTACLE2;
		} else if (m_driver->GetRawButton(3)) { //Obstacle 3. Middle. X Button
			m_autonAngle = HIGH_SHOOT_ANGLE_OBSTACLE3;
		} else if (m_driver->GetRawButton(4)) { //Obstacle 4. Second from the right. Y Button.
			m_autonAngle = HIGH_SHOOT_ANGLE_OBSTACLE4;
		} else if (m_driver->GetRawButton(5)) { //Obstacle 5. Far right. Left Bumper.
			m_autonAngle = HIGH_SHOOT_ANGLE_OBSTACLE5;
		}

	}

	void AutonomousInit() {
		/*
		 * Initialize auton stuff.
		 */
	}

	/**
	 * 	Periodic
	 */

	void AutonomousPeriodic() {

		switch (m_autonStage) {
		case kBeforeObstacle:
			BeforeObstaclePeriodic();
			break;

		case kOnObstacle:
			OnObstaclePeriodic();
			break;
		case kPastObstacle:
			PastObstaclePeriodic();
		}

	}

	/*
	 * Called before the robot hits the obstacle
	 */
	void BeforeObstaclePeriodic() {

		if ( !m_drivetrain->DistanceAtSetPoint() ) {

			if ( !m_drivetrain->IsEnabledDistance() ) {

				m_drivetrain->SetDistance(DISTANCE_TO_FIRST_OBSTACLE); //assumes 4" distance from robot to midpoint.
				m_drivetrain->EnableDistance();


				if ( !m_drivetrain->DistanceAtSetPoint() ) {

					if ( !m_drivetrain->IsEnabledDistance() ) {
						m_drivetrain->SetDistance(7.5); //assumes 4" distance from robot to midpoint.
						m_drivetrain->EnableDistance();
					}
				}
			}
		}
		else {

			m_drivetrain->DisableDistance();
			m_autonStage = kOnObstacle;
		}

	}

	/*
	 * Called while the robot is on the obstacle
	 */
	void OnObstaclePeriodic() {

		if ( !m_drivetrain->DistanceAtSetPoint() ) {
			if ( !m_drivetrain->IsEnabledDistance() ) {

				m_drivetrain->SetShift(true);
				m_drivetrain->SetDistance(DISTANCE_TO_CROSS_OBSTACLE);
				m_drivetrain->EnableDistance();
			}
		}
		else {

			m_drivetrain->DisableDistance();
			m_drivetrain->SetShift(false);
			m_autonStage = kPastObstacle;
		}
	}

	void PastObstaclePeriodic() {
		if ( !m_drivetrain->AngleAtSetPoint() ) {
			if ( !m_drivetrain->IsEnabledAngle() ) {
				m_drivetrain->SetAngle(m_autonAngle);
				m_drivetrain->EnableAngle();
			}
		}
		else {
			m_drivetrain->DisableAngle();
		}

	}

};

START_ROBOT_CLASS(Johncena);
