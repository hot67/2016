#include <memory>
#include "WPILib.h"
#include "RobotUtils/RobotUtils.h"
#include "Intake.h"
#include "Drivetrain.h"
#include "Arm.h"
//#include "CameraHandler.h"
#include <string>

 /*
  * ToDo:
  *
  * OK - RB, Y - bring arm away from tower, screw out, then move arm towards tower with fully etended screw
  * OK - RB, B - start retracting screw and once its like 1/4 way in, move arm up (bringing up the drivetrain) and lock break
  *
  * full speed on the screw
  *
  * drive straight code for ramparts in auton
  *
  * autonomous
  * 	low bar autonomous
  * 	moat autonomous
  * 		hold the arm up and drive a little further
  * 	ramparts autonomous
  * 		requiring gyro for driving straight
  * 	CDF autonomous
  * 		"dead reckoning" - Jim Meyer
  * 	shooting high
  *
  * shooter speed optical sensor working
  *
  * MUST TEST
  *		Right bumper b
  *		Right bumper y
  *		rodneys gyro for the arm
  *		shooter stuff
  *			two motors & the speed up
  *		OK - shooter encoder (tested on the board, but must be tested on the actual robot)
  *
  */

/*
 * DRIVE MAPPING
 *
 * driver left joystick - forward/backwards
 * driver right joystick - right/left
 *
 *
 * driver back - reset drivetrain encoders, reset arm encoder
 *
 *
 * driver a - gyro line up
 *
 *
 * driver left bumper - shift low when held
 * 		otherwise, drivetrain is shifted high
*/

/*
 * ARM MAPPING
 *
 * operator left joystick - manual screw
 * operator right joystick - manual arm
 *
 *
 * operator right bumper & start - manual apply brake
 * operator right bumper & y - about to climb arm
 * 		arm away from tower, screw out, arm towards tower once finished screwing
 * operator right bumper & b - climbing arm
 * 		screw in, once past a certain amount of screw, bring drivetrain up and lock the arm
 *
 *
 * operator left bumper & x - arm to 15
 * 		"close low goal" -> but i don't think they use it for this
 * operator left bumper & y - arm to 50
 * 		"medium high goal" -> but i don't think they use it for this
 * operator left bumper & a - arm to -10
 * 		"over obstacles" -> but i doubt we will ever use it for this purpose
 * operator left bumper & b - arm to 35
 * 		"batter high goal" -> but i don't think the use it for this
 *
 *
 * operator x - arm to 15
 * 		carry position
 * operator y - arm to 45
 * 		"far high goal" -> but i don't think they use it for this
 * operator a - arm to 5
 * 		pickup position
 * operator b - arm to 65
 * 		"close high goal"
 *
 *
 * operator start - shooter manual
 *
 */

/*
 * INTAKE MAPPING
	 *
	 * driver right trigger - roll in
	 * driver left trigger - roll out
	 *
	 * operator right trigger - roll in
	 * operator left trigger - roll out
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
	int m_autonBeforeShootCase = 0;

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

		//m_USBCamera = std::make_shared<USBCamera>("cam0", true);

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
		//m_USBCamera->SetExposureManual(0);
		//m_USBCamera->SetExposureHoldCurrent();

		/*
		 * Configure camera server
		 */
		//CameraServer::GetInstance()->SetQuality(50);
		//CameraServer::GetInstance()->StartAutomaticCapture("cam0");

		m_arm->ZeroAccelerometerArmEncoder();
		m_arm->ZeroScrewEncoder();
		m_drivetrain->ResetEncoder();
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
		m_autonBeforeShootCase = 0;

		f_autonRan = false;

		m_autonTimer->Stop();
		m_autonTimer->Reset();
		m_autonTimer->Start();

		m_arm->ZeroAccelerometerArmEncoder();
		m_drivetrain->ResetEncoder();
		m_drivetrain->ResetAngle();
	}

	void AutonomousPeriodic()
	{
		m_arm->ReleaseBrake();

		switch (m_autonCase) {
			case 0:
				if (AutonBeforeDrive() == true) {
					m_autonCase++;
				}
				break;
			case 1:
				//if the auton arm initial flag is going up
				if (AutonDuringDrive() == true && f_shootingOrNot == true) {
					m_autonCase++;
				}
				break;
			case 2:
				//if the auton drive initial flag is going up
				m_light->Set(Relay::kForward);
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
				m_autonCase++;
				f_autonRan = true;
				break;
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
				if (AutonArmToKickstand() == true) {
					return true;
				}
				//arm to kickstand
				break;
			case kMoat:
				if (AutonArmToKickstand() == true) {
					return true;
				}
				//arm kickstand
				break;
			case kRockWall:
				if (AutonArmToKickstand() == true){
					return true;
				}
				//arm kickstand
				break;
			case kRoughTerrain:
				if (AutonArmToKickstand() == true) {
					return true;
				}
				//arm kickstand
				break;
			case kLowBar:
				if (AutonArmToGround() == true) {
					return true;
				}
				//arm ground
				//you cannot use autonarmtokickstand function bc its to ground
				break;
			case kChiliFries:
				if (AutonArmToKickstand() == true) {
					return true;
				}


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

		switch (m_autonDefenseLocation) {
		case k1:
			switch (m_autonDuringDriveCase) {
			case 0:
				m_drivetrain->SetPIDSetpoint(193, 0); //234
				m_drivetrain->EnablePID();
				m_autonDuringDriveCase++;
				return false;
				break;
			case 1:
				if (m_drivetrain->DistanceAtSetpoint()) {
					m_drivetrain->DisablePID();
					m_autonDuringDriveCase++;
				}
				return false;
				break;
			case 2:
				if (f_shootingOrNot == true){
					m_autonDuringDriveCase = 5;
					return true;
				}
				else {
					m_arm->SetArmPIDPoint(CARRY + 5);
					m_arm->EnableArmPID();
					m_autonDuringDriveCase++;
					return false;
				}
				break;
			case 3:
				if (m_arm->ArmAtPIDSetPoint()) {
					m_arm->DisableArmPID();
					m_autonDuringDriveCase++;
					return false;
				}
				break;
			case 4:
				if (m_autonTimer->Get() > 13.0) {
					m_arm->ZeroArmEncoder();
					m_autonDuringDriveCase++;
				}
				return true;
				break;
			}
			break;
		case k2:
			switch (m_autonDuringDriveCase) {
			case 0:
				m_drivetrain->SetPIDSetpoint(190.0, m_drivetrain->GetAngle());
				m_drivetrain->EnablePID();
				m_autonDuringDriveCase++;
				return false;
				break;
			case 1:
				if (m_drivetrain->GetAverageSpeed() > -5.0) {
					m_arm->SetArmPIDPoint(CARRY);
					m_arm->EnableArmPID();
				}
				else {
					m_arm->DisableArmPID();
				}

				if (m_drivetrain->DistanceAtSetpoint()) {
					m_drivetrain->DisablePID();
					m_autonDuringDriveCase++;
					return false;
				}
				break;
			case 2:
				m_arm->SetArmPIDPoint(CARRY + 5);
				m_arm->EnableArmPID();
				m_autonDuringDriveCase++;
				return false;
				break;
			case 3:
				if (m_arm->ArmAtPIDSetPoint()) {
					m_arm->DisableArmPID();
					m_autonDuringDriveCase++;
				}
				return false;
				break;
			case 4:
				if (m_autonTimer->Get() > 13.0) {
					m_arm->ZeroArmEncoder();
					m_autonDuringDriveCase++;
				}
				return true;
				break;
			}
			break;
		case k3:
			switch (m_autonDuringDriveCase) {
			case 0:
				m_drivetrain->SetPIDSetpoint(190.0, m_drivetrain->GetAngle());
				m_drivetrain->EnablePID();
				m_autonDuringDriveCase++;
				return false;
				break;
			case 1:
				if (m_drivetrain->GetAverageSpeed() > - 5.0) {
					m_arm->SetArmPIDPoint(CARRY);
					m_arm->EnableArmPID();
				}
				else {
					m_arm->DisableArmPID();
				}

				if (m_drivetrain->DistanceAtSetpoint()) {
					m_drivetrain->DisablePID();
					m_autonDuringDriveCase++;
					return false;
				}
				break;
			case 2:
				m_arm->SetArmPIDPoint(CARRY + 5);
				m_arm->EnableArmPID();
				m_autonDuringDriveCase++;
				return false;
				break;
			case 3:
				if (m_arm->ArmAtPIDSetPoint()) {
					m_arm->DisableArmPID();
					m_autonDuringDriveCase++;
				}
				return false;
				break;
			case 4:
				if (m_autonTimer->Get() > 13.0) {
					m_arm->ZeroArmEncoder();
					m_autonDuringDriveCase++;
				}
				return true;
				break;
			}
			break;
		case k4:
			switch (m_autonDuringDriveCase) {
				case 0:
					m_drivetrain->SetPIDSetpoint(190.0, m_drivetrain->GetAngle());
					m_drivetrain->EnablePID();
					m_autonDuringDriveCase++;
					return false;
					break;
				case 1:
					if (m_drivetrain->GetAverageSpeed() > - 5.0) {
						m_arm->SetArmPIDPoint(CARRY);
						m_arm->EnableArmPID();
					}
					else {
						m_arm->DisableArmPID();
					}

					if (m_drivetrain->DistanceAtSetpoint()) {
						m_drivetrain->DisablePID();
						m_autonDuringDriveCase++;
						return false;
					}
					break;
				case 2:
					m_arm->SetArmPIDPoint(CARRY + 5);
					m_arm->EnableArmPID();
					m_autonDuringDriveCase++;
					return false;
					break;
				case 3:
					if (m_arm->ArmAtPIDSetPoint()) {
						m_arm->DisableArmPID();
						m_autonDuringDriveCase++;
					}
					return false;
					break;
				case 4:
					if (m_autonTimer->Get() > 13.0) {
						m_arm->ZeroArmEncoder();
						m_autonDuringDriveCase++;
					}
					return true;
					break;
				}
				break;
		case k5:
			switch (m_autonDuringDriveCase) {
				case 0:
					m_drivetrain->SetPIDSetpoint(190.0, m_drivetrain->GetAngle());
					m_drivetrain->EnablePID();
					m_autonDuringDriveCase++;
					return false;
					break;
				case 1:
					if (m_drivetrain->GetAverageSpeed() > - 5.0) {
						m_arm->SetArmPIDPoint(CARRY);
						m_arm->EnableArmPID();
					}
					else {
						m_arm->DisableArmPID();
					}

					if (m_drivetrain->DistanceAtSetpoint()) {
						m_drivetrain->DisablePID();
						m_autonDuringDriveCase++;
						return false;
					}
					break;
				case 2:
					m_arm->SetArmPIDPoint(CARRY + 5);
					m_arm->EnableArmPID();
					m_autonDuringDriveCase++;
					return false;
					break;
				case 3:
					if (m_arm->ArmAtPIDSetPoint()) {
						m_arm->DisableArmPID();
						m_autonDuringDriveCase++;
					}
					return false;
					break;
				case 4:
					if (m_autonTimer->Get() > 13.0) {
						m_arm->ZeroArmEncoder();
						m_autonDuringDriveCase++;
					}
					return true;
					break;
				}
				break;
		}

		return false;
		// Removed because we are just going to say 'hey these encoder values work' and ignore from there on
	}

	bool AutonBeforeShoot() {
		switch (m_autonDefenseLocation) {
		case k1:
			m_intake->SetShooter(1.0);

			switch (m_autonBeforeShootCase) {
				case 0:
					m_arm->SetArmPIDPoint(CLOSE_HIGH_GOAL);
					m_arm->EnableArmPID();

					if (m_arm->ArmAtPIDSetPoint()) {
						m_arm->ApplyBrake();
						m_arm->DisableArmPID();
						m_autonBeforeShootCase++;
					}
					return false;
					break;
				case 1:
					m_drivetrain->SetPIDSetpoint(m_drivetrain->GetAverageDistance(), 60);
					m_drivetrain->EnablePID();
					m_autonBeforeShootCase++;
					return false;
					break;
				case 2:
					if (m_drivetrain->AngleAtSetpoint()){
						m_drivetrain->ResetEncoder();
						m_drivetrain->SetPIDSetpoint(48.68, m_drivetrain->GetAngle());
						m_drivetrain->EnablePID();
						m_autonBeforeShootCase++;
					}
					return false;
					break;
				case 3:
					if (m_drivetrain->DistanceAtSetpoint()) {
						m_drivetrain->DisablePID();
						m_autonBeforeShootCase++;
					}
					return false;
					break;
				case 4:
					//m_drivetrain->ShiftLow();
					//GyroAutoLineUp();

					return true; //m_camera->AtTarget();
					break;
			}
		case k2:
			switch (m_autonBeforeShootCase) {
				case 0:
					m_drivetrain->SetPIDSetpoint(m_drivetrain->GetAverageDistance(), 60);
					m_drivetrain->EnablePID();
					m_autonBeforeShootCase++;
					return false;
					break;
				case 1:
					if (m_drivetrain->AngleAtSetpoint()){
						m_drivetrain->ResetEncoder();
						m_drivetrain->SetPIDSetpoint(5, m_drivetrain->GetAngle());
						m_drivetrain->EnablePID();
						m_autonBeforeShootCase++;
						return false;
					}
					return false;
					break;
				case 2:
					m_arm->SetArmPIDPoint(CLOSE_HIGH_GOAL);
					m_arm->EnableArmPID();

					//m_intake->SetShooter(1.0);
					m_autonBeforeShootCase++;
					return false;
					break;
				case 3:
					if (m_drivetrain->DistanceAtSetpoint()) {
						m_drivetrain->DisablePID();
						if (m_arm->ArmAtPIDSetPoint()){
							m_arm->ApplyBrake();
							m_arm->DisableArmPID();
							m_autonBeforeShootCase++;
						}
					}
					return false;
					break;
				case 4:
					//m_drivetrain->ShiftLow();
					//GyroAutoLineUp();
					return false; //m_camera->AtTarget();
					break;
			}
		case k3:
			switch (m_autonBeforeShootCase) {
				case 0:
					m_drivetrain->SetPIDSetpoint(40, m_drivetrain->GetAngle());
					m_drivetrain->EnablePID();
					m_autonCase++;
					return false;
					break;
				case 1:
					if (m_drivetrain->DistanceAtSetpoint() == true) {
						m_drivetrain->DisablePID();
						m_autonCase++;
					}
					return false;
					break;
				case 2:
					//GyroAutoLineUp();
					return true; //m_camera->AtTarget();
					break;
			}
		case k4:
			switch (m_autonBeforeShootCase) {
				case 0:
					m_drivetrain->SetPIDSetpoint(40, m_drivetrain->GetAngle());
					m_drivetrain->EnablePID();
					m_autonCase++;
					return false;
					break;
				case 1:
					if (m_drivetrain->DistanceAtSetpoint() == true) {
						m_drivetrain->DisablePID();
						m_autonCase++;
					}
					return false;
					break;
				case 2:
					//GyroAutoLineUp();
					return true; //m_camera->AtTarget();
					break;
			}
		case k5:
			switch (m_autonBeforeShootCase) {
				case 0:
					m_drivetrain->SetPIDSetpoint(m_drivetrain->GetAverageDistance(), 60);
					m_drivetrain->EnablePID();
					m_autonBeforeShootCase++;
					return false;
					break;
				case 1:
					if (m_drivetrain->AngleAtSetpoint()) {
						m_drivetrain->ResetEncoder();
						m_drivetrain->SetPIDSetpoint(19.4, m_drivetrain->GetAngle());
						m_drivetrain->EnablePID();
						m_autonBeforeShootCase++;
					}
					return false;
					break;
				case 2:
					m_arm->SetArmPIDPoint(FAR_HIGH_GOAL);
					m_arm->EnableArmPID();

					//m_intake->SetShooter(1.0);
					m_autonBeforeShootCase++;
					return false;
					break;
				case 3:
					if (m_drivetrain->DistanceAtSetpoint()) {
						m_drivetrain->DisablePID();
						if (m_arm->ArmAtPIDSetPoint()){
							m_arm->ApplyBrake();
							m_arm->DisableArmPID();
							m_autonBeforeShootCase++;
						}
					}
					return false;
					break;
				case 4:
					//m_arm->ApplyBrake();
					//GyroAutoLineUp();

					return true; //m_camera->AtTarget();
					break;
			}

		}
	}

	bool AutonShooting() {
		if (m_autonTimer->Get() > 14 && m_autonTimer->Get() < 14.99) {
			//m_intake->SetRoller(1.0);
		}
		else {
			//m_intake->SetRoller(0.0);
		}
		return false;
	}

	bool AutonArmToKickstand() {
		//moves arm to kickstand

		switch (m_autonArmToKickstandCase) {
			case 0:
				m_arm->SetArmPIDPoint(CARRY);
				m_arm->EnableArmPID();

				if (m_arm->ArmAtPIDSetPoint()) {
					m_arm->DisableArmPID();
					m_autonArmToKickstandCase++;
					return true;
				}
				break;
		}
		return false;
	}

	bool AutonArmToGround() {
		//moves arm to ground
		//zeores the arm at the kickstand

		switch (m_autonArmToGroundCase) {
		case 0:
			m_arm->SetArmPIDPoint(PICKUP - 1);
			m_arm->EnableArmPID();
			m_autonArmToGroundCase++;
			return false;
			break;
		case 1:
			if (m_arm->ArmAtPIDSetPoint()){
				m_arm->DisableArmPID();
				return true;
				break;
			}
		}

		return false;
	}

	double GetManualTotalCurrent() {
		double totalCurrent = 0.0;
		for (int i = 0; i < 16; i++) {
			totalCurrent += m_pdp->GetCurrent(i);
		}

		return totalCurrent;
	}

	/*
	void GyroAutoLineUp () {
		SmartDashboard::PutNumber("* Auto Line Up Count", m_gyroAutonLineUpCount);
		switch (m_gyroAutonLineUpStep) {
		case 0:
			//	Get Target and start PID
			if (m_camera->SeeTarget()) {
				m_gyroAutonLineUpCount++;
				m_drivetrain->SetPIDSetpoint(m_drivetrain->GetAverageDistance(), (m_drivetrain->GetAngle() + m_camera->GetX()));
				m_drivetrain->EnablePID();
				m_gyroAutonLineUpStep++;
			}
			break;
		case 1:
			//	If we moved, take another picture
			if (m_drivetrain->AngleAtSetpoint()) {
				m_drivetrain->DisablePID();
				m_gyroAutonLineUpStep = 0;
			}
			break;
		}
	}

	void DisableGyroAutoLineUp () {
		m_drivetrain->DisablePID();
		m_gyroAutonLineUpStep = 0;
		m_gyroAutonLineUpCount = 0;
	}
	*/

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
		/*
		 * DRIVE MAPPING
		 *
		 * driver left joystick - forward/backwards
		 * driver right joystick - right/left
		 *
		 *
		 * driver back - reset drivetrain encoders, reset arm encoder
		 *
		 *
		 * driver a - gyro line up
		 *
		 *
		 * driver left bumper - shift low when held
		 * 		otherwise, drivetrain is shifted high
		 */

		if (fabs(m_driver->AxisLY()) > 0.2 || fabs(m_driver->AxisRX()) > 0.2) {
			m_drivetrain->ArcadeDrive(-m_driver->AxisLY(), m_driver->AxisRX());
		} else if (m_driver->ButtonA()) {
			//m_drivetrain->ShiftLow();
			//AutoLineUp();
		} else {
			m_drivetrain->DisablePID();
			m_drivetrain->ArcadeDrive(0.0, 0.0);
		}

		if (m_driver->ButtonRB()) {
			m_arm->WedgeOut();
		}
		else {
			m_arm->WedgeIn();
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
			//GyroAutoLineUp();
			//m_drivetrain->SetPIDSetpoint(m_drivetrain->GetAverageDistance(), 0);
			//m_drivetrain->EnablePID();
			//if (m_drivetrain->AngleAtSetpoint()) {
				//m_drivetrain->DisablePID();
			//}
		} else {
			//DisableGyroAutoLineUp();
			//m_drivetrain->DisablePID();
		}
	}

	void TeleopArm ()
	{

		/*
		 * ARM MAPPING
		 *
		 * operator left joystick - manual screw
		 * operator right joystick - manual arm
		 *
		 *
		 * operator right bumper & start - manual apply brake
		 * operator right bumper & y - about to climb arm
		 * 		arm away from tower, screw out, arm towards tower once finished screwing
		 * operator right bumper & b - climbing arm
		 * 		screw in, once past a certain amount of screw, bring drivetrain up and lock the arm
		 *
		 *
		 * operator left bumper & x - arm to 15
		 * 		"close low goal" -> but i don't think they use it for this
		 * operator left bumper & y - arm to 50
		 * 		"medium high goal" -> but i don't think they use it for this
		 * operator left bumper & a - arm to -10
		 * 		"over obstacles" -> but i doubt we will ever use it for this purpose
		 * operator left bumper & b - arm to 35
		 * 		"batter high goal" -> but i don't think the use it for this
		 *
		 *
		 * operator x - arm to 15
		 * 		carry position
		 * operator y - arm to 42
		 * 		"far high goal" -> but i don't think they use it for this
		 * operator a - arm to 5
		 * 		pickup position
		 * operator b - arm to 30
		 * 		"close high goal"
		 *
		 *
		 * operator start - shooter manual
		 *
		 */

		m_arm->ArmPIDUpdate(); //to decrease PID coming down so it doesn't slam

		if (fabs(m_operator->AxisLY()) > 0.2) {
			//Manual Control
			m_arm->SetScrew(m_operator->AxisLY());
		} else {
			if (!(m_operator->ButtonRB())) {
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
			if (m_arm->GetScrewPos() < 75){
				m_arm->SetScrew(-0.8);
				m_arm->SetArmPIDPoint(CLIMB_ARM - 10);
				m_arm->EnableArmPID();
			}
			else {
				if (m_arm->ArmAtPIDSetPoint()) {
					m_arm->SetScrew(m_operator->AxisLY());
					m_arm->SetArmPIDPoint(CLIMB_ARM);
					m_arm->EnableArmPID();
				}
				else {
					m_arm->SetScrew(m_operator->AxisLY());
				}
			}

			m_intake->SetShooter(0.0);

		} else if (m_operator->ButtonB() && m_operator->ButtonRB()) {
			m_intake->SetShooter(0.0);

			if (m_arm->GetScrewPos() >= 3) {

				m_arm->SetScrew(0.8);

				if (m_arm->GetScrewPos() < 60) {
					m_arm->SetArmPIDPoint(CLIMBING_ARM);
					m_arm->EnableArmPID();
				}
				else {
					m_arm->SetArm(m_operator->AxisLY());
				}
			}
			else if (m_arm->GetScrewPos() < 3.0 && m_arm->GetScrewPos() > -5.0) {
				m_arm->SetScrew(0.5);

				if (m_arm->GetScrewSpeed() < 0.00002) {
					m_arm->SetScrew(m_operator->AxisLY());
				}
			}
			else {
				m_arm->SetScrew(m_operator->AxisLY());

				if (m_arm->ArmAtPIDSetPoint()) {
					m_arm->ApplyBrake();
					m_arm->DisableArmPID();
				}
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
			m_arm->SetArmPIDPoint(PICKUP);
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
			if (!m_operator->ButtonStart()) {
				m_intake->SetShooter(0.0);
			}
		}
	}

	void TeleopIntake (){

		/*
		 * INTAKE MAPPING
		 *
		 * driver right trigger - roll in
		 * driver left trigger - roll out
		 *
		 * operator right trigger - roll in
		 * operator left trigger - roll out
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
				m_intake->SetRoller(-1.);
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
		//SmartDashboard::PutBoolean("Ready to shoot", m_camera->AtTarget());
		//SmartDashboard::PutNumber("* Auto Aim Target", m_drivetrain->GetAngle() + m_camera->GetX());

		//SmartDashboard::PutNumber("* Camera X", m_camera->GetX());
		SmartDashboard::PutNumber("Auto Line UP Step", m_gyroAutonLineUpStep);

		SmartDashboard::PutNumber("ANGLE", m_drivetrain->GetAngle());

		/*********************************
		 * Current Data to Dashboard
		 *********************************/

		/*
		SmartDashboard::PutNumber("Left Drive Front Current", m_pdp->GetCurrent(1));
		SmartDashboard::PutNumber("Left Drive Rear Current", m_pdp->GetCurrent(0));
		SmartDashboard::PutNumber("Right Drive Front Current", m_pdp->GetCurrent(14));
		SmartDashboard::PutNumber("Right Drive Rear Current", m_pdp->GetCurrent(15));

		SmartDashboard::PutNumber("Pivot Left Current", m_pdp->GetCurrent(4));
		SmartDashboard::PutNumber("Pivot Right Current", m_pdp->GetCurrent(5));


		SmartDashboard::PutNumber("Roller Current", m_pdp->GetCurrent(6));


		SmartDashboard::PutNumber("Lift Left Current", m_pdp->GetCurrent(11));
		SmartDashboard::PutNumber("Lift Right Current", m_pdp->GetCurrent(10));

		SmartDashboard::PutNumber("Shooter Current", m_pdp->GetCurrent(9));

		SmartDashboard::PutNumber("Gear Shift Current", m_pdp->GetCurrent(7));

		SmartDashboard::PutNumber("LED Ring Current", m_pdp->GetCurrent(8));

		*/

		/*
		 * Total Current Data
		 */
		//SmartDashboard::PutNumber("Total Current", GetManualTotalCurrent());

		/*
		 * Total Power Data
		 */
		//SmartDashboard::PutNumber("Total Power", m_pdp->GetTotalPower());

		//SmartDashboard::PutBoolean("Arm Light Sensor", m_arm->IsLightSensorTriggered());

		SmartDashboard::PutBoolean("Auton Finish", f_autonRan);

		/*
		 * Temperature
		 */
		//SmartDashboard::PutNumber("PDP Temperature", m_pdp->GetTemperature());

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
		SmartDashboard::PutNumber("Arm Angle", m_arm->GetArmAngle());
		//	Check if we are below carry and tell camera to switch
		//SmartDashboard::PutBoolean("Below Carry", m_arm->GetArmPos() < CARRY + 5);

		/*
		 * Arm Encoder Rate
		 */
		//SmartDashboard::PutNumber("Arm Encoder Speed", m_arm->GetArmSpeed());
		SmartDashboard::PutNumber("Screw Encoder Position", m_arm->GetScrewPos());

		SmartDashboard::PutNumber("Arm Right Encoder Position", m_arm->GetRightArmPos());
		SmartDashboard::PutNumber("Screw Right Encoder Position", m_arm->GetRightScrewPos());

		/*
		 * Arm PID SetPoint
		 */
		SmartDashboard::PutNumber("Arm PID SetPoint", m_arm->GetArmPIDSetPoint());
		//SmartDashboard::PutNumber("Screw PID SetPoint", m_arm->GetScrewPIDSetPoint());

		/*
		 * Arm PID At Setpoint ?
		 */
		SmartDashboard::PutBoolean("Arm PID At SetPoint", m_arm->ArmAtPIDSetPoint());
		//SmartDashboard::PutBoolean("Screw PID At SetPoint", m_arm->ScrewAtPIDSetPoint());

		/***************
		 * Intake Encoder Information
		 ***************/

		/*
		 *  Shooter Status
		 */
		SmartDashboard::PutNumber("Shooter Status", m_intake->GetShooterStatus());

		SmartDashboard::PutNumber("Shooter Left Talon", m_intake->GetLeftShooter());
		SmartDashboard::PutNumber("Shooter Right Talon", m_intake->GetRightShooter());

		/***************
		 * Drivetrain Encoder Information
		 ***************/

		/*
		 * Drive Average Encoder
		 */
		SmartDashboard::PutNumber("Drive Encoder Distance", m_drivetrain->GetAverageDistance());

		SmartDashboard::PutBoolean("Drive Distance at Setpoint", m_drivetrain->DistanceAtSetpoint());
		SmartDashboard::PutNumber("Drive Distance PID Setpoint", m_drivetrain->GetDistancePIDSetpoint());
		/*
		 * Drive Left Encoder
		 */
		//SmartDashboard::PutNumber("Drive Left Encoder Distance", m_drivetrain->GetLDistance());

		/*
		 * Auton Data
		 */
		SmartDashboard::PutNumber("Auton Case", m_autonCase);
		SmartDashboard::PutNumber("Auton Timer", m_autonTimer->Get());

		/*
		 * Drive Right Encoder
		 */
		//SmartDashboard::PutNumber("Drive Right Encoder Distance", m_drivetrain->GetRDistance());

		/*
		 * Drive Average Speed
		 */
		SmartDashboard::PutNumber("Drive Average Speed", m_drivetrain->GetAverageSpeed());

		/*
		 * Drive Left Speed
		 */
		//SmartDashboard::PutNumber("Drive Left Speed", m_drivetrain->GetLSpeed());

		/*
		 * Drive Right Speed
		 */
		//SmartDashboard::PutNumber("Drive Right Speed", m_drivetrain->GetRSpeed());

		/*
		 * Drive train angle
		 */
	//	SmartDashboard::PutNumber("Drive Angle ahh", m_drivetrain->GetAngle());

		/***************
		 *  Camera
		 ***************/
		//SmartDashboard::PutNumber("Camera X", m_camera->GetTargetNormalizedCenter());

		/*********************************
		 * CONTROL
		 *********************************/
		/***************
		 * Driver
		 ***************/

		/*

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

		SmartDashboard::PutNumber("Driver Left Y-Axis", SmartDashboard::GetNumber("ImageXCenter0", 1.2));//m_driver->AxisLY());
		SmartDashboard::PutNumber("Driver Right Y-Axis", m_driver->AxisRY());
		SmartDashboard::PutNumber("Driver Left X-Axis", m_driver->AxisLX());
		SmartDashboard::PutNumber("Driver Right X-Axis", m_driver->AxisRX());

		SmartDashboard::PutNumber("Driver Right Trigger Axis", m_driver->AxisRT());
		SmartDashboard::PutNumber("Driver Left Trigger Axis", m_driver->AxisLT());

		*/

		/***************
		 * Operator
		 ***************/

		/*

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

		SmartDashboard::PutNumber("Operator Left Y-Axis", m_operator->AxisLY());
		SmartDashboard::PutNumber("Operator Right Y-Axis", m_operator->AxisRY());
		SmartDashboard::PutNumber("Operator Left X-Axis", m_operator->AxisLX());
		SmartDashboard::PutNumber("Operator Right X-Axis", m_operator->AxisLX());

		SmartDashboard::PutNumber("Operator Right Trigger Axis", m_operator->AxisRT());
		SmartDashboard::PutNumber("Operator Left Trigger Axis", m_operator->AxisLT());

		*/

	}

};

START_ROBOT_CLASS(johncena);

