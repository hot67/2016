#include "WPILib.h"
#include "RobotUtils/RobotUtils.h"
#include "Drivetrain.h"

using namespace std;

enum auton_t {
	kNothing
};

class johncena: public HotBot
{
private:
	HotJoystick* m_driver;
	HotJoystick* m_operator;

	Drivetrain* m_drivetrain;

	PowerDistributionPanel* m_pdp;

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

		m_drivetrain = new Drivetrain(this);

		m_pdp = new PowerDistributionPanel;

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
		if (m_operator->GetRawButton(HotJoystick::kButtonBack))
			m_autonChoice = kNothing;
			//operator's BACK button sets auton to NOTHING
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
		}
	}

	void AutonDoNothing ()
	{
		//this auton does nothing
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

	void TeleopDrive(){
		m_drivetrain->ArcadeDrive(m_driver->AxisLX(), m_driver->AxisRX());

	}
};

START_ROBOT_CLASS(johncena);
