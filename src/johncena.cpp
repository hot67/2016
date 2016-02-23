
#include "WPILib.h"
#include "RobotUtils/RobotUtils.h"
#include "Intake.h"
#include "Drivetrain.h"
#include "Arm.h"

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

	/*
	 * Flag for checking if we have rolled out or not
	 */
	bool f_rollingIn;

	/*
	 * Auton choice/case selection initializations
	 */
	auton_t m_autonChoice;
	unsigned m_autonCase;
	unsigned m_autonLoop;

public:
	johncena()
	{
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

		/*
		 * Sets the flag for rolling out to false because we don't start the robot by rolling out
		 */
		f_rollingIn = false;
	}

	void RobotInit()
	{
		/*
		 * Configure camera server
		 */
		CameraServer::GetInstance()->SetQuality(50);
		CameraServer::GetInstance()->StartAutomaticCapture("cam1");
	}

	void DisabledInit()
		{
		}

	void DisabledPeriodic()
	{
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
		m_drivetrain->ArcadeDrive(m_driver->AxisLY(), m_driver->AxisRX());

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

		if (fabs(m_operator->AxisRY()) > 0.2) {
			/**
			 * 	Manual Control
			 */
			m_arm->SetArm(m_operator->AxisRY());
		} else if (m_operator->ButtonY() && m_operator->ButtonRB()) {
			/**
			 * 	For Climb Up
			 */
			m_arm->SetArmPIDPoint(kClimbArm);
			m_arm->EnableArmPID();
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
			m_intake->SetShooter(MEDIUM_HIGH_GOAL_SHOOTER);
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
			m_intake->SetShooter(FAR_HIGH_GOAL_SHOOTER);
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
			m_intake->SetShooter(BATTER_HIGH_GOAL_SHOOTER);
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
			m_intake->SetShooter(CLOSE_HIGH_GOAL_SHOOTER);
		} else if (m_operator->ButtonX() && m_operator->ButtonLB()) {
			/**
			 * 	Low Goal
			 * 		Arm: 15
			 * 	This one does not start the shooter because for low goal, we don't need shooter to run
			 */
			m_arm->SetArmPIDPoint(CLOSE_LOW_GOAL);
			m_arm->EnableArmPID();
		} else if (m_operator->ButtonX()) {
			/**
			 * 	Carry Position
			 * 		Arm: 20
			 * 	This one does not start the shooter because it is carrying
			 */
			m_arm->SetArmPIDPoint(CARRY);
			m_arm->EnableArmPID();
		} else if (m_operator->ButtonA() && m_operator->ButtonRB()) {
			/**
			 * Retract the extension all way in
			 */
			m_arm->SetScrewPIDPoint(RETRACT_SCREW);
			m_arm->EnableScrewPID();
		} else if (m_operator->ButtonA() && m_operator->ButtonLB()) {
			/**
			 * 	Over Obstacles
			 */
			m_arm->SetArmPIDPoint(OBSTACLE);
			m_arm->EnableArmPID();
		} else if (m_operator->ButtonA()) {
			/**
			 * 	Floor Pick up
			 */
			m_arm->SetArmPIDPoint(PICKUP);
			m_arm->EnableArmPID();
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

		if (m_driver->ButtonRT()) {
			m_intake->Shoot();
			//this shoot function doesn't have a thing to do with the shooter
			//it just runs the intake roller in by 0.3
		}
		else {
			m_intake->SetRoller(0.0);
			//we are setting intake roller to 0.0 in order to counteract the shoot function above
			//(which has nothing to do with the shooter)
		}

		if (m_operator->ButtonBack()){
			m_intake->SetDesiredShooterSpeed();
		}
		else {
			m_intake->SetShooter(0.);
		}

		if ((m_operator->AxisRT() > 0.2) || (m_driver->AxisLT() > 0.2)){
			m_intake->SetRoller(-1.0);
			//if operator presses left trigger, intake rollers roll out
		}
		else if ((m_operator->AxisLT()) > 0.2){
			m_intake->SetRoller(1.0);
			f_rollingIn = true;
			//if operator presses right trigger, intake rollers roll in
		} else if (f_rollingIn) {
			m_rollForShootTime->Stop();
			m_rollForShootTime->Reset();
			m_rollForShootTime->Start();

			f_rollingIn = false;

			m_intake->SetRoller(-0.4);
		}
		else {
			if (m_rollForShootTime->Get() < 0.1) {
				m_intake->SetRoller(-0.4);
			} else {
				m_intake->SetRoller(0.0);
			}
		}

		if ((m_operator->GetPOV()) == 0){
			m_intake->IncreaseShooterSpeed();
			//if operator presses up on DPAD, shooter speed increases by 1%
		}
		else if ((m_operator->GetPOV()) == 180){
			m_intake->DecreaseShooterSpeed();
			//if operator presses down on DPAD, shooter speed decreases by 1%
		}
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
	SmartDashboard::PutNumber("Total Current Data", m_pdp->GetTotalCurrent());

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
	SmartDashboard::PutNumber("Shooter Speed", m_intake->GetShooterSpeed());

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
	SmartDashboard::PutNumber("Drive Angle", m_drivetrain->GetAngle());

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

	SmartDashboard::PutNumber("Driver Left Y-Axis", m_driver->AxisLY());
	SmartDashboard::PutNumber("Driver Right Y-Axis", m_driver->AxisRY());
	SmartDashboard::PutNumber("Driver Left X-Axis", m_driver->AxisLX());
	SmartDashboard::PutNumber("Driver Right X-Axis", m_driver->AxisLX());

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
