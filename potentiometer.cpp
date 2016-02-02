#include "WPILib.h"
#include <AnalogPotentiometer.h>

int main() {
	AnalogPotentiometer IDEK(1);
	int x = IDEK.Get();

	SmartDashboard::PutNumber("AnalogPotentiometer", x);
}
