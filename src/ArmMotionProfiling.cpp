#include <ArmMotionProfiling.h>



ArmMotionProfiling::ArmMotionProfiling(CANTalon* inputTalon) {

	talonStatus = 0; //Initialize all of our default variables.
	mpState = kStopped; //Used for handling the pausing and stopping of motion profiling
	MP = false; //probably going to be removed. redundant

}

void ArmMotionProfiling::PrepProfiling() {

	EndProfiling(); //Cleans out the buffer, just in case
	m_Talon->SetControlMode(CANTalon::kMotionProfile); //Make it start profiling
	MP = true; //We are using motion profiling
	mpState = kRunning;
}


void ArmMotionProfiling::EndProfiling() {

	mpState = kStopped; //Motion profiling has ended
	m_Talon->ClearMotionProfileTrajectories();
	m_Talon->SetControlMode(CANTalon::kVoltage);
	MP = false;

}

void ArmMotionProfiling::Pause() { //Unpause the profile movement

	mpState = kPaused;
	m_Talon->SetControlMode(CANTalon::kVoltage);
}

void ArmMotionProfiling::UnPause() { //Pause the profile movement

	mpState = kRunning;
	m_Talon->SetControlMode(CANTalon::kMotionProfile); //Turns off motion profiling for now. Note this will NOT clear the buffer.
	/*
	 * If you do want to clear the buffer, use MotionProfiling::EndProfiling().
	 *  Or you can simply run MotionProfiling::BeginProfiling() the next time you want to use motion profiling
	 *  this calls EndProfiling() also.
	 */

}



void ArmMotionProfiling::Iterate() {
	/*
	 * Call this about half of the delta time, needs to process data before the most recent trajectory point finishes
	 * Examples seem to show things wont work out if its called too much (teleop mode) so I think we will need to handle
	 * time.
	 */

	switch (mpState) { //Do various things based on whats going on with motion profiling
	case kRunning: //Data is buffered
		Process();
		break;
	case kStopped: //Data is dead
		if (MP) {EndProfiling();} //Clean things up if things are still trying to happen.
		break;
	case kPaused: //Data is paused
		break;
	}

}



void ArmMotionProfiling::Process() {
	m_Talon->ProcessMotionProfileBuffer(); //Push top to bottom. Not super clear on usage of this, hopefully this is correctly used here.
}


void ArmMotionProfiling::GeneratePoints() { //used with constructor where trajectory is a parameter.

	int time = 0; //will be in increments of ARM_DELTA_TIME
	bool first = true;

	CANTalon::TrajectoryPoint TalonTrajectory;
	do { //actually create our list. its a do while loop because we want a trajectory point at 0.


		/*
		 *Actual time. this will probably be in milliseconds, which should be considered with the units of m_trajectory's functions
		 */

		TalonTrajectory.velocity = m_trajectory->Velocity(time/1000);
		TalonTrajectory.position = m_trajectory->Position(time/1000);
		TalonTrajectory.timeDurMs = m_deltaTime;

		TalonTrajectory.velocityOnly = false;
		TalonTrajectory.zeroPos = false;


		if (first) {

			first=false;
			TalonTrajectory.zeroPos = true;

		}

		TalonTrajectory.isLastPoint = false;

		m_Talon->PushMotionProfileTrajectory(TalonTrajectory);

		time += m_deltaTime;

	} while (m_trajectory->Position(time)!=0);

	TalonTrajectory.velocity = m_trajectory->Velocity(time/1000);
	TalonTrajectory.position = m_trajectory->Velocity(time/1000);
	TalonTrajectory.timeDurMs = m_deltaTime;

	TalonTrajectory.velocityOnly = false;
	TalonTrajectory.zeroPos = false;
	TalonTrajectory.isLastPoint = true;
	m_Talon->PushMotionProfileTrajectory(TalonTrajectory);


}

void ArmMotionProfiling::BeginProfiling( //recreate our motion profile points.
		float current_position,
		float current_velocity,
		float target_position,
		float max_V,
		float max_A,
		float deltaTime) {

	m_deltaTime = deltaTime;
	m_trajectory = new Trajectory(current_velocity, current_position, target_position, max_A, max_V); //make a new set of trajectory points
	GeneratePoints();
	PrepProfiling();
}


bool ArmMotionProfiling::IsEnabled() {
	return MP;
}
