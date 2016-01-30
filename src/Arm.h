#ifndef SRC_ARM_H_
#define SRC_ARM_H_

#include <RobotUtils/HotSubsystem.h>
#include <string>

const enum arm_pid {
	far_high_goal = 1,
	climb = 2,
	medium_low_goal = 3, //I think. 50 degrees pulse roller for .1 seconds
	close_high_goal = 4,
	carry = 5,
	close_low_goal = 6,
	pickup = 7,
	retract_screw = 8,
	obstacles = 9
};


class Arm: public HotSubsystem {
public:

	Arm(HotBot* bot);
	virtual ~Arm();

	void Set(float speed); //Set the Speed of the Arm
	void SetExtend(float speed); //Set the Speed of the Screw Drive / Arm Extender

	void SetPIDPoint(arm_pid controller, double value); //Set the desired pidcontroller setpoint, such as close_low_goal

	void EnablePID(); //Enable the Pids

	void DisablePID(); //Disable the Pids

	void EnablePID(arm_pid controller); //Enable a specific pid controller

	void DisablePID(arm_pid controller); //Disable a specific pid controller
};

#endif /* SRC_ARM_H_ */
