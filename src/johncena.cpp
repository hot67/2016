#include <memory>
#include "WPILib.h"
#include "RobotUtils/RobotUtils.h"
#include "Intake.h"
#include "Drivetrain.h"
#include "Arm.h"
#include "CameraHandler.h"

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
	kRamparts,
	kMoat,
	kRockWall,
	kRoughTerrain,
	kLowBar,
	kChiliFries //Cheval de Frise
};

enum autonDefenseLocation {
	k1, //low bar
	k2,
	k3,
	k4,
	k5
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
	CameraHandler *m_camera;

	/*
	 * Power Distribution Panel
	 */
	PowerDistributionPanel* m_pdp;

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
		m_driver->SetDeadband(0.2);
		m_operator->SetDeadband(0.2);

		/*
		 * Subsystem initialization
		 */
		m_drivetrain = new Drivetrain(this);
		m_intake = new Intake(this);
		m_arm = new Arm(this);
		m_camera = new CameraHandler();

		/*
		 * Power Distribution Panel initialization
		 */
		m_pdp = new PowerDistributionPanel;
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
	}

	void AutonomousInit()
	{
	}

	void AutonomousPeriodic()
	{
	}

	void TeleopInit()
	{
	}

	void TeleopPeriodic()
	{
		/*
		 * Activates drive, arm and intake functions
		 */
		TeleopDrive();
		TeleopArm();
		TeleopIntake();

		m_drivetrain->Update();
		m_arm->Update();
		m_intake->Update();
		PrintData();
	}

	void TestPeriodic()
	{
	}

	void TeleopDrive ()
	{
		//	Manual Control
		m_drivetrain->ArcadeDrive(m_driver->AxisLY(), m_driver->AxisRX());

		/**
		 * 	Hold Left Bumper to Shift low
		 */
		if (m_driver->ButtonLB()) {
			m_drivetrain->ShiftLow();
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
		//	Manual Control of Arm
		m_arm->SetArm(m_operator->AxisRY());

		//	Manual Control of Screw
		m_arm->SetScrew(m_operator->AxisLY());

		//	Apply brake
		if (m_operator->ButtonStart(HotButton::kRB)) {
			m_arm->ApplyBrake();
		}

		//	PID Setpoints
		if (m_operator->ButtonY(HotButton::kRB)) {
			m_arm->SetArmPIDSetpoint(CLIMB_ARM);
			m_arm->EnableArmPID();

			if (m_arm->ArmAtPIDSetpoint()) {
				if (m_arm->GetScrewPosition() < 75) {
					m_arm->SetScrew(-0.8);
				}
			}
		}

		if (m_operator->ButtonB(HotButton::kRB)) {
			m_intake->SetShooter(0.0);

			m_arm->SetArmPIDSetpoint(CLIMBING_ARM);
			m_arm->EnableArmPID();

			if (m_arm->ArmAtPIDSetpoint()) {
				m_arm->ApplyBrake();
				m_arm->DisableArmPID();
			}
		}

		if (m_operator->ButtonY(HotButton::kLB)) {
			m_arm->SetArmPIDSetpoint(MEDIUM_HIGH_GOAL);
			m_arm->EnableArmPID();

			m_intake->SetShooter(1.0);
		}

		if (m_operator->ButtonY()) {
			m_arm->SetArmPIDSetpoint(FAR_HIGH_GOAL);
			m_arm->EnableArmPID();

			m_intake->SetShooter(1.0);
		}

		if (m_operator->ButtonB(HotButton::kLB)) {
			m_arm->SetArmPIDSetpoint(BATTER_HIGH_GOAL);
			m_arm->EnableArmPID();

			m_intake->SetShooter(1.0);
		}

		if (m_operator->ButtonB()) {
			m_arm->SetArmPIDSetpoint(CLOSE_HIGH_GOAL);
			m_arm->EnableArmPID();

			m_intake->SetShooter(1.0);
		}

		if (m_operator->ButtonX(HotButton::kLB)) {
			m_arm->SetArmPIDSetpoint(CLOSE_LOW_GOAL);
			m_arm->EnableArmPID();
		}

		if (m_operator->ButtonX()) {
			m_arm->SetArmPIDSetpoint(CARRY);
			m_arm->EnableArmPID();
		}

		if (m_operator->ButtonA(HotButton::kLB)) {
			m_arm->SetArmPIDSetpoint(OBSTACLE);
			m_arm->EnableArmPID();
		}

		if (m_operator->ButtonA()) {
			m_arm->SetArmPIDSetpoint(5);
			m_arm->EnableArmPID();
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
			m_intake->SetRoller(1.0);
		}
		if (m_driver->ButtonLT()) {
			m_intake->SetRoller(-1.0);
		}
		if (m_operator->ButtonRT()) {
			m_intake->SetRoller(1.0);
		}
		if (m_operator->ButtonLT()) {
			m_intake->SetRoller(-1.0);
		}

		if (m_operator->ButtonBack()) {
			m_intake->SetShooter(1.0);
		}
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
		 * Total Power Data
		 */
		SmartDashboard::PutNumber("Total Power", m_pdp->GetTotalPower());

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
		SmartDashboard::PutNumber("Arm Left Angle", m_arm->GetArmLAngle());
		SmartDashboard::PutNumber("Arm Right Angle", m_arm->GetArmRAngle());
		SmartDashboard::PutNumber("Arm Angle", m_arm->GetArmAngle());

		/*
		 * Arm Encoder Rate
		 */
		SmartDashboard::PutNumber("Arm Left Speed", m_arm->GetArmLSpeed());
		SmartDashboard::PutNumber("Arm Right Speed", m_arm->GetArmRSpeed());
		SmartDashboard::PutNumber("Arm Speed", m_arm->GetArmSpeed());

		/*
		 * 	Screw
		 */
		SmartDashboard::PutNumber("Screw Left Position", m_arm->GetScrewLPosition());
		SmartDashboard::PutNumber("Screw Right Position", m_arm->GetScrewRPosition());
		SmartDashboard::PutNumber("Screw Position", m_arm->GetScrewPosition());

		SmartDashboard::PutNumber("Screw Left Speed", m_arm->GetScrewLSpeed());
		SmartDashboard::PutNumber("Screw Right Speed", m_arm->GetScrewRSpeed());
		SmartDashboard::PutNumber("Screw Speed", m_arm->GetScrewSpeed());

		/*
		 * Arm PID SetPoint
		 */
		SmartDashboard::PutNumber("Arm PID SetPoint", m_arm->GetArmPIDSetpoint());

		/*
		 * Arm PID At Setpoint ?
		 */
		SmartDashboard::PutBoolean("Arm PID At SetPoint", m_arm->ArmAtPIDSetpoint());

		/***************
		 * Intake Encoder Information
		 ***************/
		/*
		 * Shooter Speed
		 */
		SmartDashboard::PutNumber("Shooter RPM", m_intake->GetShooterSpeed());

		/***************
		 * Drivetrain Encoder Information
		 ***************/
		/*
		 * Drive Average Encoder
		 */
		SmartDashboard::PutNumber("Drive Left Distance", m_drivetrain->GetLDistance());
		SmartDashboard::PutNumber("Drive Right Distance", m_drivetrain->GetRDistance());
		SmartDashboard::PutNumber("Drive Distance", m_drivetrain->GetDistance());

		SmartDashboard::PutNumber("Drive Left Speed", m_drivetrain->GetLSpeed());
		SmartDashboard::PutNumber("Drive Right Speed", m_drivetrain->GetRSpeed());
		SmartDashboard::PutNumber("Drive Speed", m_drivetrain->GetSpeed());

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

