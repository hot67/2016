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


/*Todo
 *
 * Driver Left bumper -> switching gears through pneumatics
 *
 */

using namespace std;

enum auton_t {
	kNothing,
	kLowBar,
	kLowBarBack
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
	//std::shared_ptr<USBCamera> m_camera;

	/*
	 * Power Distribution Panel
	 */
	PowerDistributionPanel* m_pdp;

	/*
	 * Timer for rolling out of the arm
	 */
	Timer* m_rollForShootTime;

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

	/*
	 * Auton choice/case selection initializations
	 */
	auton_t m_autonChoice;
	unsigned m_autonCase;
	unsigned m_autonLoop;

	unsigned m_lineUpCase;

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

		//m_camera = std::make_shared<USBCamera>("cam1", true);

		/*
		 * Power Distribution Panel initialization
		 */
		m_pdp = new PowerDistributionPanel;

		/*
		 * Timer for rolling out the arm
		 */
		m_rollForShootTime = new Timer;

		/*
		 * Default auton choice is nothing
		 */
		m_autonChoice = kNothing;

		/*
		 * Sets auton case to 0
		 */
		m_autonCase = 0;
		m_autonLoop = 0;

		m_lineUpCase = 0;

		/*
		 * Sets the flag for rolling out to false because we don't start the robot by rolling out
		 */
		f_rollingIn = false;

		/**
		 *  First no one has control
		 */
		f_shooterDriverHasControl = f_shooterOperatorHasControl = false;
	}

	void RobotInit()
	{
		//m_camera->SetExposureManual(0);
		//m_camera->SetExposureHoldCurrent();

		/*
		 * Configure camera server
		 */
		//CameraServer::GetInstance()->SetQuality(50);
		//CameraServer::GetInstance()->StartAutomaticCapture(m_camera);

		m_arm->ZeroArmEncoder();
		m_arm->ZeroScrewEncoder();
		m_drivetrain->ResetEncoder();
	}

	void DisabledInit()
		{
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
		if (m_operator->ButtonBack())
			m_autonChoice = kNothing;
			//operator's BACK button sets auton to NOTHING
		else if (m_operator->ButtonA())
			m_autonChoice = kLowBar;
			//operator's A button sets auton to UNDER LOW BAR
		else if (m_operator->ButtonB())
			m_autonChoice = kLowBarBack;
			//operator's B button sets auton to LOW BAR BACK
	}

	void AutonomousInit()
	{
		/*
		 * Sets default auton case to 0 just in case
		 */
		m_autonCase = 0;
		m_autonLoop = 0;
	}

	void AutonomousPeriodic()
	{
		/*
		 * Matches the enum to the auton function
		 */
		switch (m_autonChoice)
		{
			case kNothing:
				AutonDoNothing();
				break;
			case kLowBar:
				AutonUnderLowBar();
				break;
			case kLowBarBack:
				AutonLowBarBack();
				break;
		}

		m_arm->ArmPIDUpdate(); //to decrease PID coming down so it doesn't slam
	}

	void AutonDoNothing ()
	{
		//this auton does nothing
	}

	void AutonUnderLowBar ()
	{
		//this auton will go under the lowbar and into the opponent's courtyard if robot is in front of lowbar

		/*
		switch(m_autonCase)
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
				//m_drivetrain->SetDistance(); 6 ft to outerworks, outerworks are 4 ft, another 4 ft
				//m_drivetrain->SetLimit(); speed?? how fast can we go over ramp
				if (!m_drivetrain->IsEnabledDistance())
					m_drivetrain->EnableDistance();

				if (m_drivetrain->DistanceAtSetpoint())
					m_drivetrain->DisableDistance();

				m_autonCase++;

		} */

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

	double GetManualTotalCurrent() {
		double totalCurrent = 0.0;
		for (int i = 0; i < 16; i++) {
			totalCurrent += m_pdp->GetCurrent(i);
		}

		return totalCurrent;
	}

	bool AutoLineUp() {
		if (m_camera->SeeTarget() == false) {
			m_drivetrain->SetTurn(0.2);
			return false;
		} else if (m_camera->SeeTargetRight()) {
			m_drivetrain->SetTurn(0.2);
			return false;
		} else if (m_camera->AtTarget()) {
			m_drivetrain->SetTurn(0.0);
			return true;
		} else if (m_camera->SeeTargetLeft()) {
			m_drivetrain->SetTurn(-0.2);
			return false;
		}

		return false;
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
		/**
		 * ToDo:
		 *  Distance PID Stuff Should not be here
		 *  Erase after testing
		 */
		if (fabs(m_driver->AxisLY()) > 0.2 || fabs(m_driver->AxisRX()) > 0.2) {
			m_drivetrain->ArcadeDrive(m_driver->AxisLY(), m_driver->AxisRX());
		} else {
			m_drivetrain->DisableDistance();
			m_drivetrain->ArcadeDrive(0.0, 0.0);
		}
		{
			if (m_driver->ButtonA()) {
				AutoLineUp();
			}
		}


		if (m_driver->ButtonBack()){
			m_drivetrain->ResetEncoder();
			m_arm->ZeroArmEncoder();
			m_arm->ZeroScrewEncoder();
		}
		/**
		 * 	Hold Left Bumper to Shift low
		 */
		if (m_driver->ButtonLB()) {
			m_drivetrain->ShiftLow();
		} else {
			m_drivetrain->ShiftHigh();
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
		}
		else if (m_operator->ButtonBack() && m_operator->ButtonLB()) {
			// Retract the extension all way in
			m_arm->SetScrewPIDPoint(RETRACT_SCREW);
			m_arm->EnableScrewPID();
		}
		else if (m_operator->ButtonBack()) {
			m_arm->SetScrewPIDPoint(CLIMB_SCREW);
			m_arm->EnableScrewPID();
		}
		else {
			m_arm->SetScrew(0.0);

		}

		if (fabs(m_operator->AxisRY()) > 0.2) {
			//Manual Control
			m_arm->SetArm(m_operator->AxisRY());

			if (m_operator->ButtonStart()) {
				m_intake->SetShooter(1.0);
			}
			else {
				m_intake->SetShooter(0.0);
			}
		}
		else if (m_operator->ButtonY() && m_operator->ButtonRB()) {
			/**
			 * 	For Climb Up
			 */
			m_arm->SetArmPIDPoint(kClimbArm);
			m_arm->EnableArmPID();

			if (m_arm->ArmAtPIDSetPoint()) {
				m_arm->SetScrewPIDPoint(CLIMB_SCREW);
				m_arm->EnableScrewPID();
			} else {
				m_arm->DisableScrewPID();
			}

			m_intake->SetShooter(0.0);
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
		} else {
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
		if (m_operator->ButtonStart()){
			m_intake->SetShooter(1.0);
		}
		else{
			m_intake->SetShooter(0.);
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
	SmartDashboard::PutNumber("Roller/Gatherer Current", m_pdp->GetCurrent(6));

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

	/*********************************
	 * ENCODER DATA
	 *********************************/

	/***************
	 * Arm Encoder Information
	 ***************/

	/*
	 * Arm Encoder Rate
	 */
	SmartDashboard::PutNumber("Arm Encoder Speed", m_arm->GetArmSpeed());

	SmartDashboard::PutNumber("Screw Encoder Position", m_arm->GetScrewPos());
	/*
	 * Arm Encoder Position
	 */
	SmartDashboard::PutNumber("Arm Encoder Position", m_arm->GetArmPos());

	/*
	 * Arm PID SetPoint
	 */
	SmartDashboard::PutNumber("Arm PID SetPoint", m_arm->GetArmPIDSetPoint());

	/*
	 * Arm PID At Setpoint ?
	 */
	SmartDashboard::PutBoolean("Arm PID At SetPoint", m_arm->ArmAtPIDSetPoint());

	/***************
	 * Intake Encoder Information
	 ***************/
	/*
	 * Shooter Speed
	 */
	SmartDashboard::PutNumber("Shooter RPM", m_intake->GetShooterSpeed());
	SmartDashboard::PutNumber("Shooter Period", m_intake->GetShooterPeriod());

	SmartDashboard::PutNumber("Shooter Status", m_intake->GetShooterStatus());

	/***************
	 * Drivetrain Encoder Information
	 ***************/

	/*
	 * Drive Average Encoder
	 */
	SmartDashboard::PutNumber("Drive Encoder Distance", m_drivetrain->GetAverageDistance());

	/*
	 * Drive Left Encoder
	 */
	SmartDashboard::PutNumber("Drive Left Encoder Distance", m_drivetrain->GetLDistance());

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
