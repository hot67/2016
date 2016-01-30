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

};

START_ROBOT_CLASS(johncena);
