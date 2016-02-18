#include <ArmMotionProfiling.h>


/*
 * Motion Profiling
 */
ArmMotionProfiling::ArmMotionProfiling(CANTalon* inputTalon) {

	/*
	 * Default Variables
	 */
	mpState = kStopped;
	MP = false;

}

/*
 * Prepare Motion Profiling
 */
void ArmMotionProfiling::PrepProfiling() {

	EndProfiling();
	m_Talon->SetControlMode(CANTalon::kMotionProfile);
	MP = true;
	mpState = kRunning;
}

/*
 * End Profiling. Clean out the Talon Buffer. Set back to
 * voltage mode.
 */
void ArmMotionProfiling::EndProfiling() {

	mpState = kStopped; //Motion profiling has ended
	m_Talon->ClearMotionProfileTrajectories();
	m_Talon->SetControlMode(CANTalon::kVoltage);
	MP = false;

}

/*
 *	Temporarily pause Motion Profiling.
 *	Does this by using kVoltage mode.
 */
void ArmMotionProfiling::Pause() {

	mpState = kPaused;
	m_Talon->SetControlMode(CANTalon::kVoltage);
}

/*
 * Resume moving
 */
void ArmMotionProfiling::UnPause() {

	mpState = kRunning;
	m_Talon->SetControlMode(CANTalon::kMotionProfile);
	/*
	 * If you do want to clear the buffer, use MotionProfiling::EndProfiling(). Pause does not do this.
	 *  Or you can simply run MotionProfiling::BeginProfiling() the next time you want to use motion profiling
	 *  this calls EndProfiling() also.
	 */

}

/*
 * Processes stuff.
 * Call this half of delta time (10 ms)
 */
void ArmMotionProfiling::Iterate() {
	/*
	 * Call this about half of the delta time, needs to process data before the most recent trajectory point finishes
	 * Examples seem to show things wont work out if its called too much (teleop mode) so I think we will need to handle
	 * time.
	 */

	switch (mpState) {
	case kRunning:
		Process();
		break;
	case kStopped:
		if (MP) {EndProfiling();}
		break;
	case kPaused:
		break;
	}

}

/*
 * Push the top trajectory points to the bottom for the talon.
 */
void ArmMotionProfiling::Process() {
	m_Talon->ProcessMotionProfileBuffer();
}

/*
 * Generate the Motion profile points. Give them to the talon.
 */
void ArmMotionProfiling::GeneratePoints() {

	int time = 0;
	bool first = true;

	CANTalon::TrajectoryPoint TalonTrajectory;
	do {


		/*
		 *Actual time. this will probably be in milliseconds, which should be considered with the units of m_trajectory's functions
		 */

		TalonTrajectory.velocity = m_trajectory->Velocity(time/1000); //creates our trajectory points
		TalonTrajectory.position = m_trajectory->Position(time/1000);
		TalonTrajectory.timeDurMs = m_deltaTime; //always delta time

		TalonTrajectory.velocityOnly = false; //always false
		TalonTrajectory.zeroPos = false;


		if (first) { //this will only be true once. the first time this loop runs

			first=false;
			TalonTrajectory.zeroPos = true;

		}

		TalonTrajectory.isLastPoint = false;

		m_Talon->PushMotionProfileTrajectory(TalonTrajectory); //give them to the talon

		time += m_deltaTime;

	} while (m_trajectory->Position(time)!=0); //when we hit the position, there will be a trajectory point for this

	TalonTrajectory.velocity = m_trajectory->Velocity(time/1000); //create our last trajectory point
	TalonTrajectory.position = m_trajectory->Velocity(time/1000);
	TalonTrajectory.timeDurMs = m_deltaTime;

	TalonTrajectory.velocityOnly = false;
	TalonTrajectory.zeroPos = false;
	TalonTrajectory.isLastPoint = true; //actually the last one
	m_Talon->PushMotionProfileTrajectory(TalonTrajectory); //give it to the talon


}

/*
 * Begin moving the CANTalon through the motion profile points.
 */
void ArmMotionProfiling::BeginProfiling(
		float current_position,
		float current_velocity,
		float target_position,
		float max_V,
		float max_A,
		float deltaTime) {

	delete m_trajectory;
	m_deltaTime = deltaTime;
	m_trajectory = new Trajectory(current_velocity, current_position, target_position, max_A, max_V);
	GeneratePoints();
	PrepProfiling();
}

/*
 * Is the motion profiling enabled?
 */
bool ArmMotionProfiling::IsEnabled() {
	return MP;
}
