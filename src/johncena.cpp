#include "WPILib.h"
#include "RobotUtils/RobotUtils.h"
#include "Intake.h"
#include "Drivetrain.h"

/*TO DO
 *
 * Driver Left bumper -> switching gears
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
	HotJoystick* m_driver;
	HotJoystick* m_operator;

	Timer* m_rollForShootTime;

	PowerDistributionPanel* m_pdp;

	Drivetrain* m_drivetrain;

	bool f_armReset;

	auton_t m_autonChoice;
	unsigned m_autonCase;
	unsigned m_autonLoop;

public:
	johncena()
	{
		m_driver = new HotJoystick(0);
		m_operator = new HotJoystick(1);

		m_drivetrain = new Drivetrain(this);

		m_driver->SetDeadband(HotJoystick::kAxisALL, 0.2);
		m_operator->SetDeadband(HotJoystick::kAxisALL, 0.2);

		m_pdp = new PowerDistributionPanel;

		m_rollForShootTime = new Timer;

		f_armReset = false;

		m_autonChoice = kNothing;
		//default auton choice is nothing

		m_autonCase= 0;
		m_autonLoop = 0;
	}

	void RobotInit()
	{
	}

	void DisabledInit()
		{
		}

	void DisabledPeriodic()
	{
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
		m_autonCase = 0;
		m_autonLoop = 0;
	}

	void AutonomousPeriodic()
	{

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
		m_autonCase = 0;
	}

	void TeleopPeriodic()
	{
		TeleopDrive();
	}

	void TestPeriodic()
	{
	}

	void TeleopDrive ()
	{
		m_drivetrain->ArcadeDrive(m_driver->AxisLX(), m_driver->AxisRX());
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

		if ((m_operator->ButtonRB()) && (m_operator->ButtonY())){
			//m_arm->SetArmPIDPoint(kClimbArm);
			//m_arm->SetScrewPIDPoint(kClimbScrew);
			//m_arm->EnableArmPID();
			//if (m_arm->ArmAtSetpoint()){
				//m_arm->EnableScrewPID();
				//}
			//if operator hits right bumper and the Y-button, arm goes to climbing position and screw extends fully
		}
		else if ((m_operator->ButtonRB()) && (m_operator->ButtonA())){
			//m_arm->SetScrewPIDPoint(kRetractScrew);
			//m_arm->EnableScrewPID();
			//if operator hits right bumper and the A-button, screw goes to retract position
		}
		else if ((m_operator->ButtonLB()) && (m_operator->ButtonY())){

			//m_arm->SetArmPIDPoint(kMediumLowGoal);
			//m_arm->EnableArmPID();

			//m_rollForShootTime->Start();
			//m_intake->SetRoller(0.1);

			//if ((m_rollForShootTime->Get()) > 0.4){
				//m_intake->SetRoller(0.0);
				//m_rollForShootTime->Stop();
				//m_rollForShootTime->Reset();
			//}

			//if (m_arm->ArmAtSetpoint()){
				//m_intake->SetShooter(m_desiredShooterSpeed);
				//}
			//if operator hits left bumper and the Y-button, arm goes to medium low goal position and prepares to shoot
		}
		else if ((m_operator->ButtonLB()) && (m_operator->ButtonA())){
			//m_arm->SetArmPIDPoint(kObstacle);
			//m_arm->EnableArmPID();
			//if operator hits left bumper and the X-button, arm goes to 'push-up for obstacles' position
		}
		else if ((m_operator->ButtonLB()) && (m_operator->ButtonX())){
			//m_arm->SetArmPIDPoint(kCloseLowGoal);
			//m_arm->EnableArmPID();

			//m_rollForShootTime->Start();
			//m_intake->SetRoller(0.1);

			//if ((m_rollForShootTime->Get()) > 0.4){
				//m_intake->SetRoller(0.0);
				//m_rollForShootTime->Stop();
				//m_rollForShootTime->Reset();
			//}

			//if (m_arm->ArmAtSetpoint()){
				//m_intake->SetShooter(m_desiredShooterSpeed);
			//}
			//if operator hits left bumper and the A-button, arm goes to close low goal position and prepares to shoot
		}
		else if (m_operator->ButtonY()){
			//m_arm->SetArmPIDPoint(kFarHighGoal);
			//m_arm->EnableArmPID();
			//if operator presses button Y, arm will set to High Goal angle
		}
		else if (m_operator->ButtonX()){
			//m_arm->SetArmPIDPoint(kCarry);
			//m_arm->EnableArmPID();
			//if operator presses button X, arm will set to Carry angle
		}
		else if (m_operator->ButtonA()){
			//m_arm->SetArmPIDPoint(kPickup);
			//m_arm->EnableArmPID();
			//if operator presses button A, arm will set to Pickup angle
		}
		else if (m_operator->ButtonB()){
			//m_arm->SetArmPIDPoint(kCloseHighGoal);
			//m_arm->EnableArmPID();
			//if operator presses button B, arm will set to Close High Goal angle
		}
		else if (m_operator->AxisLY() > 0.2){
			//m_arm->SetScrew(m_operator->AxisLY());
			//if operator uses left joystick up and down, will set manual screw
		}
		else if (m_operator->AxisRY() > 0.2){
			//m_arm->SetArm(m_operator->AxisLY());
			//if operator uses right joystick up and down, will set manual arm
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
		 * driver right trigger - shoots
		 * driver left trigger - roll out
		 *
		 */

		if (m_operator->AxisLT() > 0.2){
			//m_intake->SetRoller(1.0);
			//if operator presses left trigger, intake rollers roll out
		}
		else if ((m_operator->AxisRT()) > 0.2){
			//m_intake->Intake();
			//if operator presses right trigger, intake rollers roll in
		}
		else if ((m_operator->GetPOV()) == 0){
			//livespeed + 0.01
			//if operator presses up on DPAD, shooter speed increases by 1%
		}
		else if ((m_operator->GetPOV()) == 180){
			//livespeed - 0.01
			//if operator presses down on DPAD, shooter speed decreases by 1%
		}
		else if ((m_driver->AxisRT()) > 0.2){
			//if ((m_arm->OnTarget())
				//m_intake->Shoot();
			//if driver presses right trigger, shoots
		}
		else if ((m_driver->AxisLT()) > 0.2){
			//m_intake->Rollout();
			//if driver presses left trigger, intake rollers roll out
		}
	}

};

START_ROBOT_CLASS(johncena);
