#include <memory>
#include "WPILib.h"
#include "RobotUtils/RobotUtils.h"
#include "Intake.h"
#include "Drivetrain.h"
#include "Arm.h"
#include "CameraHandler.h"

/*
 * ARM MAPPING
 *
 * operator right bumper & y-button - arm to climb, screw extend
 * operator right bumper & a-button - arm does nothing, screw retract
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

enum auton_t {
	kNothing,
	kLowBar,
	kLowBarRaise,
	kLowBarBack,
	kLowBarShoot
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
	/*
	 * Auton choice/case selection initializations
	 */
	auton_t m_autonChoice;
	unsigned m_autonCase;
	unsigned m_autonLoop;

	unsigned m_lineUpCase;

	int m_gyroAutonLineUpStep = 0;
	int m_gyroAutonLineUpCount = 0;


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
		m_autonChoice = kNothing;
		//SmartDashboard::PutString("Auton Choice", "Do Nothing Auton");

		/*
		 * Sets auton case to 0
		 */
		m_autonCase = 0;
		m_autonLoop = 0;

		m_lineUpCase = 0;

		/**
		 *  First no one has control
		 */
		f_shooterDriverHasControl = f_shooterOperatorHasControl = false;

		f_autonRan = false;

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


		PrintData();

		/*****
		 * Select auton choice
		 * 	ToDo: Better auton UI
		 *****/
		if (m_operator->ButtonBack()){
			m_autonChoice = kNothing;
			//operator's BACK button sets auton to NOTHING
			//SmartDashboard::PutString("Auton Choice", "Do Nothing Auton");
		} else if (m_operator->ButtonA()) {
			m_autonChoice = kLowBar;
			//operator's A button sets auton to UNDER LOW BAR
			//SmartDashboard::PutString("Auton Choice", "Under Low Bar Auton");
		} else if (m_operator->ButtonB()) {
			m_autonChoice = kLowBarRaise;
		}
	}

	void AutonomousInit()
	{
		/*
		 * Sets default auton case to 0 just in case
		 */
		m_autonCase = 0;
		m_autonLoop = 0;

		f_autonRan = false;

		m_autonTimer->Stop();
		m_autonTimer->Reset();
		m_autonTimer->Start();
	}

	void AutonomousPeriodic()
	{
		/*
		 * Matches the enum to the auton function
		 */

		PrintData();

		switch (m_autonChoice)
		{
			case kNothing:
				AutonDoNothing();
				break;
			case kLowBar:
				AutonUnderLowBar();
				break;
			case kLowBarRaise:
				AutonUnderLowBarRaise();
				break;
		}


		m_arm->ArmPIDUpdate(); //to decrease PID coming down so it doesn't slam
	}

	void AutonDoNothing ()
	{
		//this auton does nothing
	}

	void AutonUnderLowBarRaise ()
	{
		//this auton will go under the lowbar and into the opponent's courtyard if robot is in front of lowbar

		m_arm->ReleaseBrake();

		m_drivetrain->ShiftLow();

		switch (m_autonCase) {
			case 0:
				//	Move the arm back to pick up
				if (m_arm->IsLightSensorTriggered() == false) {
					m_arm->ZeroLightSensorArmEncoder();
					m_autonCase++;
				}
				else if (m_arm->IsLightSensorTriggered() == true) {
					m_arm->SetArm(0.0);
				}
				break;
			case 1:
				m_arm->SetArmPIDPoint(CARRY);
				m_arm->EnableArmPID();

				if (m_arm->ArmAtPIDSetPoint()) {
					m_arm->DisableArmPID();
					m_autonCase++;
				}
				break;
			case 2:
				m_drivetrain->SetDistance(180.);
				m_drivetrain->EnableDistance();
				if (m_drivetrain->DistanceAtSetPoint() || m_autonTimer->Get() > 10.0) {
					m_drivetrain->DisableDistance();
					m_arm->SetArmPIDPoint(m_arm->GetArmPos() + 14.0);
					m_autonCase++;
				}
				break;
			case 3:
				// take current position and add 9 to clear the bar
				// pid to that point
				// disable pid when at setpoint so it rests on the bar
				// zero the arm encoder
				m_arm->EnableArmPID();

				if (m_arm->ArmAtPIDSetPoint() == true) {
					m_arm->DisableArmPID();
					m_autonCase++;
				}
				break;
			case 4:
				if (m_autonTimer->Get() >= 13.0) {
					m_arm->ZeroArmEncoder();
					m_autonCase++;
				}
				break;
			case 5:
				if (f_autonRan == false) {
					f_autonRan = true;
				}
				m_autonTimer->Stop();
				m_autonCase++;
		}

	}

	void AutonUnderLowBar() {
		//this auton will go under the lowbar and into the opponent's courtyard if robot is in front of lowbar

				m_arm->ReleaseBrake();

				m_drivetrain->ShiftLow();

				switch (m_autonCase) {
					case 0:
						//	Move the arm back to pick up
						if (m_arm->IsLightSensorTriggered() == false) {
							m_arm->ZeroLightSensorArmEncoder();
							m_autonCase++;
						}
						else if (m_arm->IsLightSensorTriggered() == true) {
							m_arm->SetArm(0.0);
						}
						break;
					case 1:
						m_arm->SetArmPIDPoint(CARRY);
						m_arm->EnableArmPID();

						if (m_arm->ArmAtPIDSetPoint()) {
							m_arm->DisableArmPID();
							m_autonCase++;
						}
						break;
					case 2:
						m_drivetrain->SetDistance(180.);
						m_drivetrain->EnableDistance();
						if (m_drivetrain->DistanceAtSetPoint()) {
							m_drivetrain->DisableDistance();
							m_autonCase++;
						}
						break;
				}
	}

	void AutonLowBarBack()
	{
		/*switch(m_autonCase)
			{
				case 0:
					if (f_armReset)
						m_autonCase++;
					break;
				case 1:
					//set arm to position zero to fit under low bar
					//if arm is not enabled
						//enable arm
					m_autonCase++;
					break;
				case 2:
					//m_drivetrain->SetDistance(); 6 ft to outerworks, outerworks are 4 ft, another 1 ft
					 *
					//m_drivetrain->SetLimit(); speed?? how fast can we go over ramp
					if (!m_drivetrain->IsEnabledDistance())
						m_drivetrain->EnableDistance();

					if (m_drivetrain->DistanceAtSetpoint())
						m_drivetrain->DisableDistance();

					m_autonCase++;
				case 3:
					//m_drivetrain->SetDistance(); -1 ft, -4 ft for outerworks, -1 ft into neutral zone
					//m_drivetrain->SetLimit(); speed?? how fast can we go over ramp
					if (!m_drivetrain->IsEnabledDistance())
						m_drivetrain->EnableDistance();

					if (m_drivetrain->DistanceAtSetpoint())
						m_drivetrain->DisableDistance();

					m_autonCase++;

			*/
	}

	void AutonLowBarShoot() {
		/*switch (m_autonCase) {
		case 0:
			/**
			 * 	Move the arm back to pick up
			 *
			m_arm->SetArmPIDPoint(PICKUP);
			m_arm->EnableArmPID();

			/**
			 * 	Move the robot back for 10 ft
			 * 		You may need to change this value
			 *
			m_drivetrain->SetDistance(-120);
			m_drivetrain->EnableDistance();

			/**
			 * 	If we arrive to the setpoints, move to next case
			 /
			if (m_arm->ArmAtPIDSetPoint() && m_drivetrain->DistanceAtSetPoint()) {
				m_arm->DisableArmPID();
				m_drivetrain->DisableDistance();

				m_autonCase++;
			}
			break;

		case 1:
			/**
			 * 	Move the arm ready to shoot
			 /
			m_arm->SetArmPIDPoint(MEDIUM_HIGH_GOAL);
			m_arm->EnableArmPID();

			/**
			 * 	Speed Up The shooter
			 /
			m_intake->SetShooter(1.0);

			/**
			 * 	Line Up
			 /
			AutoLineUp();

			/**
			 * 	Check if we are done
			 /
			if (m_camera->AtTarget() && m_arm->ArmAtPIDSetPoint()) {
				m_autonCase++;
			}
			break;

		case 2:
			/**
			 *	Shoot
			 *	ToDo: Do we stop this? Or keep it running until the auton ends
			 /
			m_intake->SetRoller(1.0);
		} */
	}

	double GetManualTotalCurrent() {
		double totalCurrent = 0.0;
		for (int i = 0; i < 16; i++) {
			totalCurrent += m_pdp->GetCurrent(i);
		}

		return totalCurrent;
	}

	bool AutoLineUp() {
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

	void GyroAutoLineUp () {
		SmartDashboard::PutNumber("* Camera X", m_camera->GetX());
		SmartDashboard::PutNumber("* Auto Line Up Count", m_gyroAutonLineUpCount);
		m_drivetrain->ShiftLow();
		switch (m_gyroAutonLineUpStep) {
		case 0:
			//	Get Target and start PID
			if (m_camera->SeeTarget()) {
				m_gyroAutonLineUpCount++;
				SmartDashboard::PutNumber("* Auto Aim Target", m_drivetrain->GetAngle() + m_camera->GetX());
				m_drivetrain->SetAngle(m_drivetrain->GetAngle() + m_camera->GetX());
				m_drivetrain->EnableAngle();
			}
			m_gyroAutonLineUpStep++;
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
		/*
		if (m_driver->ButtonLB()) {
			m_drivetrain->ShiftLow();
		} else {
			m_drivetrain->ShiftHigh();
		}
		*/

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

		SmartDashboard::PutNumber("Auton Choice Enums", m_autonChoice);

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

