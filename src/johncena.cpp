#include <memory>
#include "WPILib.h"
#include "RobotUtils/RobotUtils.h"
#include "Intake.h"
#include "Drivetrain.h"
#include "Arm.h"
#include "CameraHandler.h"
#include <string>

 /*
  * ToDo:
  *
  * RB, Y - bring arm away from tower, screw out, then move arm towards tower with fully etended screw
  * RB, B - start retracting screw and once its like 1/4 way in, move arm up (bringing up the drivetrain) and lock break
  *
  * full speed on the screw
  *
  */

/*
 * ARM MAPPING
 *
 * operator right bumper & y-button - arm to climb, screw extend
 * operator right bumper & b-button - arm does nothing, screw retract
 *
 * operator left bumper & y-button - arm to medium low goal, prepares to shoot
 * operator left bumper & x-button - arm to obstacle
 * operator left button & a-button - arm to close low goal, prepares to shoot
 *
 * operator y-button - arm to far high goal
 * operator x-button - arm to carry
 * operator a-button - arm to floor pickup
 * operator b-button - arm to close high goal
 *
 * operator left joystick - manual screw
 * operator right joystick - manual pivot
 *
 */

/*
 * INTAKE MAPPING
 *
 * operator left trigger - intake roll out
 * operator right trigger - intake roll in
 *
 * operator DPAD up - shoot speed increases by 1%
 * operator DPAD down - shoot speed decreases by 1%
 *
 * driver right trigger - shoots (runs into the shooter)
 * driver left trigger - roll out
 *
 * operator back button - runs up the shooter
 *
 */

//#define COMPETITION_BOT
#define PRACTICE_BOT

#define CAMERA_TO_ENCODE_COUNT 500

using namespace std;

enum autonDefenseType {
	kNoDefense = 0,
	kRamparts = 1,
	kMoat = 2,
	kRockWall = 3,
	kRoughTerrain = 4,
	kLowBar = 5,
	kChiliFries = 6 //Cheval de Frise
};

enum autonDefenseLocation {
	kNoLocation = 0,
	k1 = 1, //low bar
	k2 = 2,
	k3 = 3,
	k4 = 4,
	k5 = 5
};

class johncena: public HotBot
{
private:
	Relay *m_light;

	/*
	 * Joysticks
	 */
	HotJoystick* m_driver;
	HotJoystick* m_operator;

	/*
	 * Subsystems!
	 */
	Drivetrain* m_drivetrain;
	Intake* m_intake;
	Arm* m_arm;
	CameraHandler *m_camera;
	std::shared_ptr<USBCamera> m_USBCamera;

	/*
	 * Power Distribution Panel
	 */
	PowerDistributionPanel* m_pdp;

	/*
	 * Timer for rolling out of the arm
	 */
	Timer* m_rollForShootTime;

	Timer* m_autonTimer;

	/*
	 * Flag for checking if we have rolled out or not
	 */
	bool f_rollingIn;

	/*
	 * for intake to see if we rolled out to get it away from the shooter yet or not
	 */
	bool m_rollLoop = false;

	/**
	 *  First in for shooter
	 */
	bool f_shooterDriverHasControl;
	bool f_shooterOperatorHasControl;

	bool f_autonRan;

	bool f_shootingOrNot;
	/*
	 * Auton choice/case selection initializations
	 */
	autonDefenseType m_autonDefenseType;
	autonDefenseLocation m_autonDefenseLocation;

	unsigned m_autonCase;
	unsigned m_autonLoop;

	unsigned m_lineUpCase;

	int m_autonDuringDriveCase = 0;

	int m_gyroAutonLineUpStep = 0;
	int m_gyroAutonLineUpCount = 0;

	int m_autonArmToKickstandCase = 0;
	int m_autonArmToGroundCase = 0;

public:
	johncena()
	{
		m_light = new Relay(0);

		/*
		 * Joystick initialization
		 */
		m_driver = new HotJoystick(0);
		m_operator = new HotJoystick(1);

		/*
		 * Joystick deadband
		 */
		m_driver->SetDeadband(HotJoystick::kAxisALL, 0.2);
		m_operator->SetDeadband(HotJoystick::kAxisALL, 0.2);

		/*
		 * Subsystem initialization
		 */
		m_drivetrain = new Drivetrain(this);
		m_intake = new Intake(this);
		m_arm = new Arm(this);
		m_camera = new CameraHandler();

		m_USBCamera = std::make_shared<USBCamera>("cam0", true);

		/*
		 * Power Distribution Panel initialization
		 */
		m_pdp = new PowerDistributionPanel;

		/*
		 * Timer for rolling out the arm
		 */
		m_rollForShootTime = new Timer;

		m_autonTimer = new Timer;

		/*
		 * Default auton choice is nothing
		 */
		//SmartDashboard::PutString("Auton Choice", "Do Nothing Auton");

		/*
		 * Sets auton case to 0
		 */
		m_autonCase = 0;
		m_autonLoop = 0;

		m_lineUpCase = 0;

		m_autonDefenseLocation = kNoLocation;
		m_autonDefenseType = kNoDefense;

		/**
		 *  First no one has control
		 */
		f_shooterDriverHasControl = f_shooterOperatorHasControl = false;

		f_autonRan = false;
		f_rollingIn = false;

		f_shootingOrNot = false;

		m_autonTimer = new Timer();
	}

	void RobotInit()
	{
		m_USBCamera->SetExposureManual(0);
		m_USBCamera->SetExposureHoldCurrent();

		/*
		 * Configure camera server
		 */
		CameraServer::GetInstance()->SetQuality(50);
		CameraServer::GetInstance()->StartAutomaticCapture("cam1");

		m_arm->ZeroArmEncoder();
		m_arm->ZeroScrewEncoder();
		m_drivetrain->ResetEncoder();
		m_drivetrain->ResetGyro();
	}

	void DisabledInit()
	{
		/*
		 * Sets the flag for rolling out to false because we don't start the robot by rolling out
		 */
		m_intake->ResetRollerStatus();
	}

	void DisabledPeriodic()
	{

		//m_camera->SetExposureManual(100);
		//m_camera->SetExposureHoldCurrent();

		SmartDashboard::PutNumber("Recieved Image X 0 ", 1);

		m_light->Set(Relay::kForward);
		/*

		std::string dType = SmartDashboard::GetString("Defense Type", "None");

		if (dType.compare("Low Bar")) {
			m_autonDefenseType = kLowBar;
		}
		else if (dType.compare("Ramparts")) {
			m_autonDefenseType = kRamparts;
		}
		else if (dType.compare("Moat")) {
			m_autonDefenseType = kMoat;
		}
		else if (dType.compare("Rock Wall")) {
			m_autonDefenseType = kRockWall;
		}
		else if (dType.compare("Rough Terrain")) {
			m_autonDefenseType = kRoughTerrain;
		}
		else if (dType.compare("Chili Fries")) {
			m_autonDefenseType = kChiliFries;
		}
		else if (dType.compare("None")) {
			m_autonDefenseType = kNoDefense;
		} */


		m_autonDefenseType = (autonDefenseType)SmartDashboard::GetNumber("Defense Type", 0);
		m_autonDefenseLocation = (autonDefenseLocation)SmartDashboard::GetNumber("Defense Location", 0);

		f_shootingOrNot = SmartDashboard::GetBoolean("Shoot", false);

		PrintData();
	}

	void AutonomousInit()
	{
		/*
		 * Sets default auton case to 0 just in case
		 */
		m_autonCase = 0;
		m_autonLoop = 0;

		m_autonArmToKickstandCase = 0;
		m_autonArmToGroundCase = 0;
		m_autonDuringDriveCase = 0;


		f_autonRan = false;

		m_autonTimer->Stop();
		m_autonTimer->Reset();
		m_autonTimer->Start();
	}

	void AutonomousPeriodic()
	{
		m_arm->ReleaseBrake();

		switch (m_autonCase) {
			case 0:
				/*if (AutonBeforeDrive() == true) {
					m_autonCase++;
				}
				break; */
			case 1:
				//if the auton arm initial flag is going up
				if (AutonDuringDrive() == true && f_shootingOrNot == true) {
					m_autonCase++;
				}
				break;
			case 2:
				//if the auton drive initial flag is going up
				if (AutonBeforeShoot() == true) {
					m_autonCase++;
				}
				break;
			case 3:
				if (AutonShooting() == true) {
					m_autonCase++;
				}
				break;
			case 4:
				//zero the arm encoder
				m_autonCase++;
				break;
			case 5:
				f_autonRan = true;
		}

		PrintData();

		m_arm->ArmPIDUpdate(); //to decrease PID coming down so it doesn't slam
	}

	bool AutonBeforeDrive() {
		//arm to desired location depending on defense type
		//only is true when we're at the desired location

		//the purpose of the truth is to move into next case in auton periodic

		//pid is disabled when it returns true

		switch (m_autonDefenseType) {
			case kRamparts:
				//if (AutonArmToKickstand() == true) {
					return true;
				//}
				//arm to kickstand
				break;
			case kMoat:
				//if (AutonArmToKickstand() == true) {
					return true;
				//}
				//arm kickstand
				break;
			case kRockWall:
				//if (AutonArmToKickstand() == true){
					return true;
				//}
				//arm kickstand
				break;
			case kRoughTerrain:
				//if (AutonArmToKickstand() == true) {
					return true;
				//}
				//arm kickstand
				break;
			case kLowBar:
				//if (AutonArmToGround() == true) {
					return true;
				//}
				//arm ground
				//you cannot use autonarmtokickstand function bc its to ground
				break;
			case kChiliFries:
				/*if (AutonArmToKickstand() == true) {
					return true;
				} */


				return true;

				//arm kickstand ???
				break;
		}
	}

	bool AutonDuringDrive() {
		//drive across the defense
		//distance may vary
		//all should be done in low gear

		//drive pid should be disabled after returning true

		//purpose of the truth is to move into next case in auton periodic

		//robot is already over the defense

		switch (m_autonDefenseType) {
			case kRamparts:
				/*switch(m_autonDuringDriveCase) {
					case 0:
						m_drivetrain->SetDistance(180);
						m_drivetrain->EnableDistance();
						m_autonDuringDriveCase++;
						return false;
						break;
					case 1:
						if (m_drivetrain->DistanceAtSetPoint()) {
							m_drivetrain->DisableDistance();
							return true;
							m_autonDuringDriveCase++;
						}
						break;
				}
				*/

				//driveX amount of feet
				break;

			case kMoat:
				/*switch (m_autonDuringDriveCase) {
					case 0:
						m_drivetrain->SetDistance(84);
						m_arm->SetArmPIDPoint(CARRY);
						m_arm->EnableArmPID();
						m_drivetrain->EnableDistance();
						m_autonDuringDriveCase++;
						return false;

						break;

					case 1:
						if (m_drivetrain->DistanceAtSetPoint()) {
							m_drivetrain->DisableDistance();
							m_arm->DisableArmPID();
							return true;
						}

						break;
				}

				//driveX feet
				//arm to kickstand
				 *
				 */
				break;
			case kRockWall:
				/*
				switch (m_autonDuringDriveCase) {
					case 0:
						m_drivetrain->SetDistance(144);
						m_drivetrain->EnableDistance();
						m_autonDuringDriveCase++;
						return false;
						break;
					case 1:
						if (m_drivetrain->DistanceAtSetPoint()){
							m_drivetrain->DisableDistance();
							return true;
							break;
						}
				//driveX feet
				}
				*/
				break;
			case kRoughTerrain:

				/*switch (m_autonDuringDriveCase) {
					case 0:
						m_drivetrain->SetDistance(144);
						m_drivetrain->EnableDistance();
						m_autonDuringDriveCase++;
						return false;
						break;
					case 1:
						if (m_drivetrain->DistanceAtSetPoint()) {
							m_drivetrain->DisableDistance();
							return true;
							break;
						}
				}
				*/
				//driveX feet
				break;
			case kLowBar:
				/*
				 *
				 switch (m_autonDuringDriveCase) {
					case 0:
						m_drivetrain->SetDistance(144);
						m_drivetrain->EnableDistance();
						m_autonDuringDriveCase++;
						return false;
						break;
					case 1:
						if (m_drivetrain->DistanceAtSetPoint()) {
							m_drivetrain->DisableDistance();
							return true;
							break;
						}
				}
				//driveX feet
				*/
				break;
			case kChiliFries:
				/*switch (m_autonDuringDriveCase) {
					case 0:
						m_drivetrain->SetDistance(144);
						m_drivetrain->EnableDistance();
						m_autonDuringDriveCase++;
						return false;
						break;
					case 1:
						if (m_drivetrain->DistanceAtSetPoint()) {
							m_drivetrain->DisableDistance();
							return true;
							break;
						}
				} */

				//driveX feet
				break;
		}
	}

	bool AutonBeforeShoot() {
		switch (m_autonDefenseLocation) {

			 case k1:

			/*
				m_arm->SetArmPIDPoint(FAR_HIGH_GOAL);
				m_arm->EnableArmPID();

				m_intake->SetShooter(1.0);

				if (m_camera->SeeTarget() == false) {
					m_drivetrain->SetTurn(0.55);
					SmartDashboard::PutNumber("* Line Up Turn", 0.6);
					return false;
				} else {
					GyroAutoLineUp();
					return true;
				}

			*/
				 return false;
				break;
				//turn right
			case k2:
				/*m_arm->SetArmPIDPoint(FAR_HIGH_GOAL);
				m_arm->EnableArmPID();

				m_intake->SetShooter(1.0);

				if (m_camera->SeeTarget() == false) {
					m_drivetrain->SetTurn(0.55);
					SmartDashboard::PutNumber("* Line Up Turn", 0.6);
					return false;
				} else {
					GyroAutoLineUp();
					return true;
				} */
				m_light->Set(Relay::kForward);
				return false;
				break;
				//turn a lil right
			case k3:

				/*
				m_arm->SetArmPIDPoint(FAR_HIGH_GOAL);
				m_arm->EnableArmPID();

				m_intake->SetShooter(1.0);

				if (m_camera->SeeTarget() == false) {
					m_drivetrain->SetTurn(0.55);
					SmartDashboard::PutNumber("* Line Up Turn", 0.6);
					return false;
				} else {
					GyroAutoLineUp();
					return true;
				}

				*/
				return false;
				break;
				//dont turn just shoot man
			case k4:

				/*
				m_arm->SetArmPIDPoint(FAR_HIGH_GOAL);
				m_arm->EnableArmPID();

				m_intake->SetShooter(1.0);

				if (m_camera->SeeTarget() == false) {
					m_drivetrain->SetTurn(-0.55);
					SmartDashboard::PutNumber("* Line Up Turn", 0.6);
					return false;
				} else {
					GyroAutoLineUp();
					return true;
				}

				*/
				return false;
				break;
				//turn a lil left
			case k5:

				/*
				m_arm->SetArmPIDPoint(FAR_HIGH_GOAL);
				m_arm->EnableArmPID();

				m_intake->SetShooter(1.0);

				if (m_camera->SeeTarget() == false) {
					m_drivetrain->SetTurn(-0.55);
					SmartDashboard::PutNumber("* Line Up Turn", 0.6);
					return false;
				} else {
					GyroAutoLineUp();
					return true;
				}
				*/
				return false;
				break;
				//turn left
		}
	}

	bool AutonShooting() {
		//m_intake->SetRoller(-1.0);

	}

	bool AutonArmToKickstand() {
		//moves the arm to kickstand
		//zeroes the arm at the kickstand

		switch (m_autonArmToKickstandCase) {
			case 0:
				if (m_arm->IsLightSensorTriggered() == false) {
					m_arm->ZeroLightSensorArmEncoder();
					m_autonArmToKickstandCase++;
					return false;
				}
				else if (m_arm->IsLightSensorTriggered() == true) {
					m_arm->SetArm(0.0);
					return false;
				}
				break;
			case 1:
				m_arm->SetArmPIDPoint(CARRY);
				m_arm->EnableArmPID();

				if (m_arm->ArmAtPIDSetPoint()) {
					m_arm->DisableArmPID();
					m_autonArmToKickstandCase++;
					return false;
				}
				break;
		}
	}

	bool AutonArmToGround() {
		//moves arm to ground
		//zeores the arm at the kickstand

		switch (m_autonArmToGroundCase) {
		case 0:
			if (m_arm->IsLightSensorTriggered() == false) {
				m_arm->ZeroLightSensorArmEncoder();
				m_autonArmToGroundCase++;
				return false;
			}
			else if (m_arm->IsLightSensorTriggered() == true) {
				m_arm->SetArm(0.0);
				return false;
			}
			break;
		case 1:
			m_arm->SetArmPIDPoint(CARRY);
			m_arm->EnableArmPID();

			if (m_arm->ArmAtPIDSetPoint()) {
				m_arm->DisableArmPID();
				m_autonArmToGroundCase++;
				return false;
			}
			break;
		case 2:
			m_arm->ZeroArmEncoder();
			m_autonArmToGroundCase++;
			return false;
			break;
		case 3:
			m_arm->SetArmPIDPoint(PICKUP);
			m_arm->EnableArmPID();
			m_autonArmToGroundCase++;
			return false;
			break;
		case 4:
			if (m_arm->ArmAtPIDSetPoint()){
				m_arm->DisableArmPID();
				return true;
				break;
			}
		}
	}

	double GetManualTotalCurrent() {
		double totalCurrent = 0.0;
		for (int i = 0; i < 16; i++) {
			totalCurrent += m_pdp->GetCurrent(i);
		}

		return totalCurrent;
	}

	bool AutoRightLineUp() {
		if (m_camera->SeeTarget() == false) {
			m_drivetrain->SetTurn(0.55);
			SmartDashboard::PutNumber("* Line Up Turn", 0.6);
			return false;
		} else if (m_camera->SeeTargetRight()) {
			m_drivetrain->SetTurn(0.55);
			SmartDashboard::PutNumber("* Line Up Turn", 0.6);
			return false;
		} else if (m_camera->AtTarget()) {
			m_drivetrain->SetTurn(0.0);
			SmartDashboard::PutNumber("* Line Up Turn", 0.0);
			return true;
		} else if (m_camera->SeeTargetLeft()) {
			m_drivetrain->SetTurn(-0.55);
			SmartDashboard::PutNumber("* Line Up Turn", -0.6);
			return false;
		}

		return false;
	}

	bool AutoLeftLineUp() {
		if (m_camera->SeeTarget() == false) {
			m_drivetrain->SetTurn(-0.55);
			SmartDashboard::PutNumber("* Line Up Turn", 0.6);
			return false;
		} else if (m_camera->SeeTargetRight()) {
			m_drivetrain->SetTurn(0.55);
			SmartDashboard::PutNumber("* Line Up Turn", 0.6);
			return false;
		} else if (m_camera->AtTarget()) {
			m_drivetrain->SetTurn(0.0);
			SmartDashboard::PutNumber("* Line Up Turn", 0.0);
			return true;
		} else if (m_camera->SeeTargetLeft()) {
			m_drivetrain->SetTurn(-0.55);
			SmartDashboard::PutNumber("* Line Up Turn", -0.6);
			return false;
		}

		return false;
	}

	void GyroAutoLineUp () {
		SmartDashboard::PutNumber("* Camera X", m_camera->GetX());
		SmartDashboard::PutNumber("* Auto Line Up Count", m_gyroAutonLineUpCount);
		switch (m_gyroAutonLineUpStep) {
		case 0:
			//	Get Target and start PID
			if (m_camera->SeeTarget()) {
				m_gyroAutonLineUpCount++;
				SmartDashboard::PutNumber("* Auto Aim Target", m_drivetrain->GetAngle() + m_camera->GetX());
				m_drivetrain->SetAngle(m_drivetrain->GetAngle() + m_camera->GetX());
				m_drivetrain->EnableAngle();
				m_gyroAutonLineUpStep++;
			}
			break;
		case 1:
			//	If we moved, take another picture
			if (m_drivetrain->AngleAtSetPoint()) {
				m_gyroAutonLineUpStep = 0;
			}
			break;
		}
	}

	void DisableGyroAutoLineUp () {
		m_drivetrain->DisableAngle();
		m_gyroAutonLineUpStep = 0;
		m_gyroAutonLineUpCount = 0;
	}

	void TeleopInit()
	{
		/*
		 * Switches the auton case to 0 again...
		 */
		m_autonCase = 0;
	}

	void TeleopPeriodic()
	{
		/*
		 * Activates drive, arm and intake functions
		 */
		TeleopDrive();
		TeleopArm();
		TeleopIntake();

		/*if (m_driver->ButtonStart()) {
			AutoLineUp();
		}
		else {
			if (m_drivetrain->IsEnabledSpangle() == false){
				m_drivetrain->DisableSpangle();
			}
		} */

		m_light->Set(Relay::kForward);

		/*
		 * Publishes data to the SmartDashboard to be filtered through the LabView dashboard
		 */
		PrintData();
	}

	void TestPeriodic()
	{
	}

	void TeleopDrive ()
	{
		if (fabs(m_driver->AxisLY()) > 0.2 || fabs(m_driver->AxisRX()) > 0.2) {
			m_drivetrain->ArcadeDrive(-m_driver->AxisLY(), m_driver->AxisRX());
		} else if (m_driver->ButtonA()) {
			//m_drivetrain->ShiftLow();
			//AutoLineUp();
		} else {
			m_drivetrain->DisableDistance();
			m_drivetrain->ArcadeDrive(0.0, 0.0);
		}


		if (m_driver->ButtonBack()){
			m_drivetrain->ResetEncoder();
			m_arm->ZeroArmEncoder();
			//m_arm->ZeroScrewEncoder();
		}
		/**
		 * 	Hold Left Bumper to Shift low
		 */

		if (m_driver->ButtonLB()) {
			m_drivetrain->ShiftLow();

		} else {
			m_drivetrain->ShiftHigh();
		}


		if (m_driver->ButtonA()) {
			GyroAutoLineUp();
		} else {
			DisableGyroAutoLineUp();
		}
	}

	void TeleopArm ()
	{

		/*
		 * ARM MAPPING
		 *
		 * operator right bumper & y-button - arm to climb, screw extend
		 * operator right bumper & a-button - arm does nothing, screw retract
		 *
		 * operator left bumper & y-button - arm to medium low goal, prepares to shoot
		 * operator left bumper & x-button - arm to obstacle
		 * operator left bumper & a-button - arm to close low goal, prepares to shoot
		 * operator left bumper & b-button - arm to batter high goal
		 *
		 * operator y-button - arm to far high goal
		 * operator x-button - arm to carry
		 * operator a-button - arm to floor pickup
		 * operator b-button - arm to close high goal
		 *
		 * operator left joystick - manual screw
		 * operator right joystick - manual pivot
		 *
		 * operator button start - zero arm encoder
		 *
		 * operator button back
		 *
		 */

		m_arm->ArmPIDUpdate(); //to decrease PID coming down so it doesn't slam

		if (fabs(m_operator->AxisLY()) > 0.2) {
			//Manual Control
			m_arm->SetScrew(m_operator->AxisLY());
		} else {
			 if ((m_operator->ButtonRB() && m_operator->ButtonY()) == false) {
				 m_arm->SetScrew(0.0);
			 }
		}

		if (m_operator->ButtonRB() && m_operator->ButtonStart()) {
			m_arm->ApplyBrake();
		} else if (m_operator->ButtonStart()) {
			m_intake->SetShooter(1.0);
			m_arm->ReleaseBrake();
		} else {
			if ((m_operator->ButtonRB() && m_operator->ButtonB()) == false){
				m_arm->ReleaseBrake();
			}

		}

		if (fabs(m_operator->AxisRY()) > 0.2) {
			//Manual Control
			m_arm->SetArm(m_operator->AxisRY());

			if (m_operator->ButtonRB() && m_operator->ButtonStart()) {
				m_arm->ApplyBrake();
			}
			else if (m_operator->ButtonStart()) {
				m_intake->SetShooter(1.0);
				m_arm->ReleaseBrake();
			}
			else {
				m_intake->SetShooter(0.0);
				m_arm->ReleaseBrake();
			}
		}
		else if (m_operator->ButtonY() && m_operator->ButtonRB()) {
			/**
			 * 	For Climb Up
			 * 		Move the arm to climb position and extend the screw up
			 */
			m_arm->SetArmPIDPoint(CLIMB_ARM);
			m_arm->EnableArmPID();

			m_intake->SetShooter(0.0);

			if (m_arm->ArmAtPIDSetPoint()) {
				if (m_arm->GetScrewPos() < 75) {
					m_arm->SetScrew(-0.8);
				}
				else {
					m_arm->SetScrew(m_operator->AxisLY());
				}
			}
			else {
				m_arm->SetScrew(m_operator->AxisLY());
			}
		} else if (m_operator->ButtonB() && m_operator->ButtonRB()) {
			m_intake->SetShooter(0.0);

			m_arm->SetArmPIDPoint(CLIMBING_ARM);
			m_arm->EnableArmPID();

			if (m_arm->ArmAtPIDSetPoint() == true) {
				m_arm->ApplyBrake();
				m_arm->DisableArmPID();
			}
			else {
				m_arm->ReleaseBrake();
			}
		} else if (m_operator->ButtonY() && m_operator->ButtonLB()) {
			/**
			 * 	MEDIUM HIGH GOAL
			 * 		Arm: 50
			 * 		Shooter: 1.0
			 */
			m_arm->SetArmPIDPoint(MEDIUM_HIGH_GOAL);
			m_arm->EnableArmPID();
			/**
			 * Speed up Shooter
			 */
			m_intake->SetShooter(1.0);
		} else if (m_operator->ButtonY()) {
			/**
			 *
			 * 	High Goal
			 * 		Arm: 45
			 * 		Shooter: 1.0
			 */
			m_arm->SetArmPIDPoint(FAR_HIGH_GOAL);
			m_arm->EnableArmPID();

			/**
			 * Speed up Shooter
			 */
			m_intake->SetShooter(1.0);
		} else if (m_operator->ButtonB() && m_operator->ButtonLB()) {
			/*
			 * Batter Shot
			 * 		Arm: 35
			 * 		Shooter: 0.6
			 */
			m_arm->SetArmPIDPoint(BATTER_HIGH_GOAL);
			m_arm->EnableArmPID();

			/**
			 * 	Speed Up Shooter
			 */
			m_intake->SetShooter(1.0);
		} else if (m_operator->ButtonB()) {
			/**
			 * 	Close High Goal
			 * 		Arm: 60
			 * 		Shooter: 1.0
			 */
			m_arm->SetArmPIDPoint(CLOSE_HIGH_GOAL);
			m_arm->EnableArmPID();

			/**
			 * 	Speed Up Shooter
			 */
			m_intake->SetShooter(1.0);
		} else if (m_operator->ButtonX() && m_operator->ButtonLB()) {
			/**
			 * 	Low Goal
			 * 		Arm: 15
			 * 	This one does not start the shooter because for low goal, we don't need shooter to run
			 */
			m_arm->SetArmPIDPoint(CLOSE_LOW_GOAL);
			m_arm->EnableArmPID();

			m_intake->SetShooter(0.0);
		} else if (m_operator->ButtonX()) {
			/**
			 * 	Carry Position
			 * 		Arm: 20
			 * 	This one does not start the shooter because it is carrying
			 */
			m_arm->SetArmPIDPoint(CARRY);
			m_arm->EnableArmPID();

			m_intake->SetShooter(0.0);
		} else if (m_operator->ButtonA() && m_operator->ButtonLB()) {
			/**
			 * 	Over Obstacles
			 */
			m_arm->SetArmPIDPoint(OBSTACLE);
			m_arm->EnableArmPID();

			m_intake->SetShooter(0.0);
		} else if (m_operator->ButtonA()) {
			/**
			 * 	Floor Pick up
			 */
			m_arm->SetArmPIDPoint(5);
			m_arm->EnableArmPID();

			m_intake->SetShooter(0.0);
		} /*else if (m_operator->ButtonStart()) {
			m_intake->SetShooter(1.0); }*/
		 else {
			/**
			 * 	No command is given:
			 * 		Disable Arm PID
			 * 		Disable Screw PID
			 * 		Stop the arm
			 * 		Stop the shooter
			 */
			m_arm->DisableArmPID();
			m_arm->DisableScrewPID();
			m_arm->SetArm(0.0);
			m_intake->SetShooter(0.0);
		}



	}

	void TeleopIntake (){

		/*
		 * INTAKE MAPPING
		 *
		 * operator left trigger - intake roll out
		 * operator right trigger - intake roll in
		 *
		 * operator DPAD up - shoot speed increases by 1%
		 * operator DPAD down - shoot speed decreases by 1%
		 *
		 * driver right trigger - shoots (runs into the shooter)
		 * driver left trigger - roll out
		 *
		 * operator back button - runs up the shooter
		 *
		 */

		if (m_intake->GetShooterStatus() == Intake::kShooterStopped) {
			if (m_driver->AxisRT() > 0.2 && f_shooterOperatorHasControl == false) {
				m_intake->SetRoller(1.0);
				f_shooterDriverHasControl = true;
			} else if (m_driver->AxisLT() > 0.2 && f_shooterOperatorHasControl == false) {
				m_intake->SetRoller(-1.0);
				f_shooterDriverHasControl = true;
			} else if (m_operator->AxisRT() > 0.2 && f_shooterDriverHasControl == false) {
				m_intake->SetRoller(1.0);
				f_shooterOperatorHasControl = true;
			} else if (m_operator->AxisLT() > 0.2 && f_shooterDriverHasControl == false) {
				m_intake->SetRoller(-1.0);
				f_shooterOperatorHasControl = true;
			} else {
				m_intake->SetRoller(0.0);
				f_shooterDriverHasControl = f_shooterOperatorHasControl = false;
			}
		}
		else if (m_intake->GetShooterStatus() == Intake::kShooterSpeeding) {
			if (m_driver->AxisLT() > 0.2) {
				m_intake->SetRoller(-1);
			}
			else {
				m_intake->SetRoller(0);
			}
		}
		else if (m_intake->GetShooterStatus() == Intake::kShooterAtSpeed) {
			if (m_driver->AxisRT() > 0.2) {
				m_intake->SetRoller(1.0);
			}
			else if (m_driver->AxisLT() > 0.2) {
				m_intake->SetRoller(-1.0);
			}
			else {
				m_intake->SetRoller(0.0);
			}
		}


//WORKING INTAKE CODE
		/*if ((m_driver->AxisRT() > 0.2)) {
			//	Roll In
			m_intake->SetRoller(1.);

			m_rollLoop = true;

		} else if (m_operator->AxisLT() > 0.2) {
			//	Roll out
			m_intake->SetRoller(-1.);
		} else {

			if (m_rollLoop == true){
				m_rollForShootTime->Stop();
				m_rollForShootTime->Reset();
				m_rollForShootTime->Start();

				m_intake->SetRoller(-0.4);
				m_rollLoop = false;
			}

			if (m_rollForShootTime->Get() >= 0.1) {
				m_intake->SetRoller(0.0);
			}
		}

		if (m_operator->ButtonStart()){
			m_intake->SetShooter(1.);
		}
		else {
			m_intake->SetShooter(0.);
		}

		if ((m_operator->GetPOV()) == 0){
			m_intake->IncreaseShooterSpeed();
			//if operator presses up on DPAD, shooter speed increases by 1%
		}
		else if ((m_operator->GetPOV()) == 180){
			m_intake->DecreaseShooterSpeed();
			//if operator presses down on DPAD, shooter speed decreases by 1%
		} */
	}

	void PrintData(){
		SmartDashboard::PutNumber("Drive Angle", m_drivetrain->GetAngle());

		/*********************************
		 * Current Data to Dashboard
		 *********************************/
		/*
		 * Drive Current Data
		 */
		SmartDashboard::PutNumber("Left Drive Front Current", m_pdp->GetCurrent(1));
		SmartDashboard::PutNumber("Left Drive Rear Current", m_pdp->GetCurrent(0));
		SmartDashboard::PutNumber("Right Drive Front Current", m_pdp->GetCurrent(14));
		SmartDashboard::PutNumber("Right Drive Rear Current", m_pdp->GetCurrent(15));

		/*
		 * Arm Current Data
		 */
		SmartDashboard::PutNumber("Pivot Left Current", m_pdp->GetCurrent(4));
		SmartDashboard::PutNumber("Pivot Right Current", m_pdp->GetCurrent(5));

		/*
		 * Intake Current Data
		 */
		SmartDashboard::PutNumber("Roller Current", m_pdp->GetCurrent(6));

		/*
		 * Screw Current Data
		 */
		SmartDashboard::PutNumber("Lift Left Current", m_pdp->GetCurrent(11));
		SmartDashboard::PutNumber("Lift Right Current", m_pdp->GetCurrent(10));

		/*
		 * Shooter Current Data
		 */
		SmartDashboard::PutNumber("Shooter Current", m_pdp->GetCurrent(9));

		/*
		 * Gear Shift Current Data
		 */
		SmartDashboard::PutNumber("Gear Shift Current", m_pdp->GetCurrent(7));

		/*
		 * LED Ring Current Data
		 */
		SmartDashboard::PutNumber("LED Ring Current", m_pdp->GetCurrent(8));

		/*
		 * Total Current Data
		 */
		SmartDashboard::PutNumber("Total Current", GetManualTotalCurrent());

		/*
		 * Total Power Data
		 */
		SmartDashboard::PutNumber("Total Power", m_pdp->GetTotalPower());

		SmartDashboard::PutBoolean("Arm Light Sensor", m_arm->IsLightSensorTriggered());

		SmartDashboard::PutBoolean("Auton Finish", f_autonRan);

		/*
		 * Temperature
		 */
		SmartDashboard::PutNumber("PDP Temperature", m_pdp->GetTemperature());

		/*
		 * Auton Selection
		 */

		SmartDashboard::PutNumber("Auton Defense Location", m_autonDefenseLocation);
		SmartDashboard::PutNumber("Auton Defense Type", m_autonDefenseType);
		SmartDashboard::PutBoolean("Auton Shooting Or Not", f_shootingOrNot);
		/*********************************
		 * ENCODER DATA
		 *********************************/

		/***************
		 * Arm Encoder Information
		 ***************/
		/*
		 *  Arm Encode Position
		 */
		SmartDashboard::PutNumber("Arm Encoder Position", m_arm->GetArmPos());
		SmartDashboard::PutNumber("Screw Encoder Position", m_arm->GetScrewPos());
		//	Check if we are below carry and tell camera to switch
		SmartDashboard::PutBoolean("Below Carry", m_arm->GetArmPos() < CARRY + 5);

		/*
		 * Arm Encoder Rate
		 */
		SmartDashboard::PutNumber("Arm Encoder Speed", m_arm->GetArmSpeed());
		SmartDashboard::PutNumber("Screw Encoder Position", m_arm->GetScrewPos());

		SmartDashboard::PutNumber("Arm Right Encoder Position", m_arm->GetRightArmPos());
		SmartDashboard::PutNumber("Screw Right Encoder Position", m_arm->GetRightScrewPos());

		/*
		 * Arm PID SetPoint
		 */
		SmartDashboard::PutNumber("Arm PID SetPoint", m_arm->GetArmPIDSetPoint());
		SmartDashboard::PutNumber("Screw PID SetPoint", m_arm->GetScrewPIDSetPoint());

		/*
		 * Arm PID At Setpoint ?
		 */
		SmartDashboard::PutBoolean("Arm PID At SetPoint", m_arm->ArmAtPIDSetPoint());
		SmartDashboard::PutBoolean("Screw PID At SetPoint", m_arm->ScrewAtPIDSetPoint());

		/***************
		 * Intake Encoder Information
		 ***************/
		/*
		 * Shooter Speed
		 */
		SmartDashboard::PutNumber("Shooter RPM", m_intake->GetShooterSpeed());
		SmartDashboard::PutNumber("Shooter Period", m_intake->GetShooterPeriod());

		/*
		 *  Shooter Status
		 */
		SmartDashboard::PutNumber("Shooter Status", m_intake->GetShooterStatus());

		/***************
		 * Drivetrain Encoder Information
		 ***************/

		/*
		 * Drive Average Encoder
		 */
		SmartDashboard::PutNumber("Drive Encoder Distance", m_drivetrain->GetAverageDistance());

		SmartDashboard::PutBoolean("Drive Distance at Setpoint", m_drivetrain->DistanceAtSetPoint());
		SmartDashboard::PutNumber("Drive Distance PID Setpoint", m_drivetrain->GetDistancePIDSetPoint());
		/*
		 * Drive Left Encoder
		 */
		SmartDashboard::PutNumber("Drive Left Encoder Distance", m_drivetrain->GetLDistance());

		/*
		 * Auton Data
		 */
		SmartDashboard::PutNumber("Auton Case", m_autonCase);
		SmartDashboard::PutNumber("Auton Timer", m_autonTimer->Get());

		/*
		 * Drive Right Encoder
		 */
		SmartDashboard::PutNumber("Drive Right Encoder Distance", m_drivetrain->GetRDistance());

		/*
		 * Drive Average Speed
		 */
		SmartDashboard::PutNumber("Drive Average Speed", m_drivetrain->GetAverageSpeed());

		/*
		 * Drive Left Speed
		 */
		SmartDashboard::PutNumber("Drive Left Speed", m_drivetrain->GetLSpeed());

		/*
		 * Drive Right Speed
		 */
		SmartDashboard::PutNumber("Drive Right Speed", m_drivetrain->GetRSpeed());

		/*
		 * Drive train angle
		 */
	//	SmartDashboard::PutNumber("Drive Angle ahh", m_drivetrain->GetAngle());

		/***************
		 *  Camera
		 ***************/
		SmartDashboard::PutNumber("Camera X", m_camera->GetTargetNormalizedCenter());

		/*********************************
		 * CONTROL
		 *********************************/
		/***************
		 * Driver
		 ***************/
		/*
		 * Buttons
		 */
		SmartDashboard::PutBoolean("Driver A", m_driver->ButtonA());
		SmartDashboard::PutBoolean("Driver B", m_driver->ButtonB());
		SmartDashboard::PutBoolean("Driver X", m_driver->ButtonX());
		SmartDashboard::PutBoolean("Driver Y", m_driver->ButtonY());
		SmartDashboard::PutBoolean("Driver LB", m_driver->ButtonLB());
		SmartDashboard::PutBoolean("Driver RB", m_driver->ButtonRB());
		SmartDashboard::PutBoolean("Driver LT", m_driver->ButtonLT());
		SmartDashboard::PutBoolean("Driver RT", m_driver->ButtonRT());
		SmartDashboard::PutBoolean("Driver Start", m_driver->ButtonStart());
		SmartDashboard::PutBoolean("Driver Back", m_driver->ButtonBack());

		/*
		 * Axis
		 */

		SmartDashboard::PutNumber("Driver Left Y-Axis", SmartDashboard::GetNumber("ImageXCenter0", 1.2));//m_driver->AxisLY());
		SmartDashboard::PutNumber("Driver Right Y-Axis", m_driver->AxisRY());
		SmartDashboard::PutNumber("Driver Left X-Axis", m_driver->AxisLX());
		SmartDashboard::PutNumber("Driver Right X-Axis", m_driver->AxisRX());

		SmartDashboard::PutNumber("Driver Right Trigger Axis", m_driver->AxisRT());
		SmartDashboard::PutNumber("Driver Left Trigger Axis", m_driver->AxisLT());

		/***************
		 * Operator
		 ***************/
		/*
		 * Buttons
		 */
		SmartDashboard::PutBoolean("Operator A", m_operator->ButtonA());
		SmartDashboard::PutBoolean("Operator B", m_operator->ButtonB());
		SmartDashboard::PutBoolean("Operator X", m_operator->ButtonX());
		SmartDashboard::PutBoolean("Operator Y", m_operator->ButtonY());
		SmartDashboard::PutBoolean("Operator LB", m_operator->ButtonLB());
		SmartDashboard::PutBoolean("Operator RB", m_operator->ButtonRB());
		SmartDashboard::PutBoolean("Operator LT", m_operator->ButtonLT());
		SmartDashboard::PutBoolean("Operator RT", m_operator->ButtonRT());
		SmartDashboard::PutBoolean("Operator Start", m_operator->ButtonStart());
		SmartDashboard::PutBoolean("Operator Back", m_operator->ButtonBack());

		/*
		 * Axis
		 */

		SmartDashboard::PutNumber("Operator Left Y-Axis", m_operator->AxisLY());
		SmartDashboard::PutNumber("Operator Right Y-Axis", m_operator->AxisRY());
		SmartDashboard::PutNumber("Operator Left X-Axis", m_operator->AxisLX());
		SmartDashboard::PutNumber("Operator Right X-Axis", m_operator->AxisLX());

		SmartDashboard::PutNumber("Operator Right Trigger Axis", m_operator->AxisRT());
		SmartDashboard::PutNumber("Operator Left Trigger Axis", m_operator->AxisLT());
	}

};

START_ROBOT_CLASS(johncena);

