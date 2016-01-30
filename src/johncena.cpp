#include <RobotUtils/RobotUtils.h>
#include "WPILib.h"
#include "RobotUtils/RobotUtils.h"

using namespace std;

enum auton_t {
	kNothing,
	kLowBar,
	kLowBarBack
};

class johncena: public IterativeRobot
{
private:
	HotJoystick* m_driver;
	HotJoystick* m_operator;

	PowerDistributionPanel* m_pdp;

	bool f_armReset;

	auton_t m_autonChoice;
	unsigned m_autonCase;
	unsigned m_autonLoop;

public:
	johncena()
	{
		m_driver = new HotJoystick(0);
		m_operator = new HotJoystick(1);

		m_driver->SetDeadband(HotJoystick::kAxisALL, 0.2);
		m_operator->SetDeadband(HotJoystick::kAxisALL, 0.2);

		m_pdp = new PowerDistributionPanel;

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
				//m_drivetrain->SetDistance(); 6 ft to outerworks, outerworks are 4 ft, another 4 ft
				//m_drivetrain->SetLimit(); speed?? how fast can we go over ramp
				if (!m_drivetrain->IsEnabledDistance())
					m_drivetrain->EnableDistance();

				if (m_drivetrain->DistanceAtSetpoint())
					m_drivetrain->DisableDistance();

				m_autonCase++;

		}*/

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
	}

	void TestPeriodic()
	{
	}

	void TeleopDrive ()
	{
		if ((m_driver->AxisLY() > 0.2) || (m_driver->AxisRX() > 0.2)){
			//arcade drive
			;
		}
		else
			//do not arcade drive
			;
	}

	void TeleopArm ()
	{
		/*
		 * OPERATOR BUTTON MAPPING
		 *
		 * right bumper & y-button - arm to climb, screw extend
		 * right bumper & a-button - arm does nothing, screw retract
		 *
		 * left bumper & y-button - arm to medium low goal, prepares to shoot
		 * left bumper & x-button - arm to obstacle
		 * left button & a-button - arm to close low goal, prepares to shoot
		 *
		 * y-button - arm to far high goal
		 * x-button - arm to carry
		 * a-button - arm to floor pickup
		 * b-button - arm to close high goal
		 *
		 */
		if ((m_operator->ButtonRB()) && (m_operator->ButtonY())){
			//m_arm->SetArmPIDPoint(kClimb);
			//m_arm->SetScrewPIDPoint(kClimb);
			//m_arm->EnableArmPID();
			//if (m_arm->ArmAtSetpoint()){
				//m_arm->EnableScrewPID();
			//if operator hits right bumper and the Y-button, arm goes to climbing position and screw extends fully
			;
		}
		else if ((m_operator->ButtonRB()) && (m_operator->ButtonA())){
			//m_arm->SetScrewPIDPoint(kRetractScrew);
			//m_arm->EnableScrewPID();
			//if operator hits right bumper and the A-button, screw goes to retract position
			;
		}
		else if ((m_operator->ButtonLB()) && (m_operator->ButtonY())){
			//m_arm->SetArmPIDPoint(kMediumLowGoal);
			//m_arm->EnableArmPID();
			//intake roller out for 0.1 seconds -> consider timer?
			//if (m_arm->ArmAtSetpoint())
				//shooter UP
			//if operator hits left bumper and the Y-button, arm goes to medium low goal position and prepares to shoot
			;
		}
		else if ((m_operator->ButtonLB()) && (m_operator->ButtonX())){
			//m_arm->SetArmPIDPoint(kObstacle);
			//m_arm->EnableArmPID();
			//if operator hits left bumper and the X-button, arm goes to 'push-up for obstacles' position
			;
		}
		else if ((m_operator->ButtonLB()) && (m_operator->ButtonA())){
			//m_arm->SetArmPIDPoint(kCloseLowGoal);
			//m_arm->EnableArmPID();
			//intake roller out for 0.1 seconds -> consider timer?
			//if (m_arm->ArmAtSetpoint())
				//shooter UP
			//if operator hits left bumper and the A-button, arm goes to close low goal position and prepares to shoot
			;
		}
		else if (m_operator->ButtonY()){
			//m_arm->SetArmPIDPoint(kFarHighGoal);
			//m_arm->EnableArmPID();
			//if operator presses button Y, arm will set to High Goal angle
			;
		}
		else if (m_operator->ButtonX()){
			//m_arm->SetArmPIDPoint(kCarry);
			//m_arm->EnableArmPID();
			//if operator presses button X, arm will set to Carry angle
			;
		}
		else if (m_operator->ButtonA()){
			//m_arm->SetArmPIDPoint(kPickup);
			//m_arm->EnableArmPID();
			//if operator presses button A, arm will set to Pickup angle
			;
		}
		else if (m_operator->ButtonB()){
			//m_arm->SetArmPIDPoint(kCloseHighGoal);
			//m_arm->EnableArmPID();
			//if operator presses button B, arm will set to Close High Goal angle
			;
		}
	}
	void TeleopIntake(){
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
			//m_intake->RollOut();
			//if operator presses left trigger, intake rollers roll out
			;
		}
		else if (m_operator->AxisRT() > 0.2){
			//m_intake->Intake();
			//if operator presses right trigger, intake rollers roll in
			;
		}
		else if (m_operator->GetPOV() == 0){
			//m_intake->IncreaseShooterSpeed();
			//if operator presses up on DPAD, shooter speed increases by 1%
			;
		}
		else if (m_operator->GetPOV() == 180){
			//m_intake->DecreaseShooterSpeed();
			//if operator presses down on DPAD, shooter speed decreases by 1%
			;
		}
		else if (m_driver->AxisRT() > 0.2){
			//m_intake->Shoot();
			//if driver presses right trigger, shoots
			;
		}
		else if (m_driver->AxisLT() > 0.2){
			//m_intake->Rollout();
			//if driver presses left trigger, intake rollers roll out
			;
		}
	}

};

START_ROBOT_CLASS(johncena);
