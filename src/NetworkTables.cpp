#include "WPILib.h"
#include "NetworkTables/NetworkTable.h"
using namespace std;

// This code will test the applications of Network Tables for camera use
class Camera: public IterativeRobot
{
	Joystick* driver;
	Relay* light;
public:
	shared_ptr<NetworkTable> cameraTable;
	shared_ptr<CameraServer> camera;
	Camera()
	{
		driver = new Joystick(1);
		light = new Relay(0);
		cameraTable = NetworkTable::GetTable("GRIP/ContoursResults"); // Get values from set location in network table
		camera = CameraServer::GetInstance(); // Camera captures video
		camera->SetQuality(50);
		camera->SetSize(100);
	}
void TeleopPeriodic(){

	if (driver->GetRawButton(1)){ // TODO find actual button value
		float valo; // Placeholder value on pending actual values
		float valf; // Second placeholder

		double area = camera->("area"); // Reads value from table
		double height = cameraTable->GetNumber("height"); // Reads value from table
		double width = cameraTable->GetNumber("width"); // Reads value from table
		double proportion = height/width;

		if ((valo <= area <= valf) && (valo <= proportion <= valf)){

		} // Gets values and lines up accordingly (gonna need drivetrain)
		else if ((valo <= area <= valf) && (valo <= proportion <= valf)){

		} // Gonna need a lot more of these to accommodate for all viable shooting positions
		else {

		}
	};
}
};

