#include "WPILib.h"
#include "NetworkTables/NetworkTable.h"
using namespace std;

// This code will be test the applications of Network Tables for camera use
class Robot: public IterativeRobot
{
	Joystick* stick; // only joystick
	Relay* light;
public:
	shared_ptr<NetworkTable> cameraTable;
	Robot()
	{
		stick = new Joystick(1);
		light = new Relay(0);
		cameraTable = NetworkTable::GetTable("GRIP/ContoursTest"); // Get values from set location in network table
	}
void TeleopPeriodic(){

	double area = cameraTable->GetNumber("area"); // Reads value from table
	double height = cameraTable->GetNumber("height"); // Reads value from table
	double width = cameraTable->GetNumber("width"); // Reads value from table

	while (area == 4322.5){
	light->Set(Relay::kForward); // Tests correctness of readings
	}
}
};
