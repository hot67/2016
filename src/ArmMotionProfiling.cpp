#include <ArmMotionProfiling.h>



ArmMotionProfiling::ArmMotionProfiling(Trajectory* talonTrajectory,
		CANTalon* inputTalon,
		float deltaTime=ARM_DELTA_TIME) {
	talonStatus = 0; //Initialize all of our default variables.
	mpState = kStopped; //Used for handling the pausing and stopping of motion profiling
	MP = false; //probably going to be removed. redundant
	m_trajectory = talonTrajectory;
	m_deltaTime = deltaTime;
	m_Talon = inputTalon;
}

void ArmMotionProfiling::BeginProfiling() {

	GenerateMotionProfiles();

	EndProfiling(); //Cleans out the buffer, just in case
	m_Talon->SetControlMode(CANTalon::kMotionProfile); //Make it start profiling
	MP = true; //We are using motion profiling
	GiveBuffer(); //Add in our data!
}




void ArmMotionProfiling::GiveBuffer() {

	CANTalon::TrajectoryPoint talonTrajectory;
	for (int i = 0;i<pointsLen;++i) { //Iterate through the points


		/*
		 * transfer the data into the trajectory point
		 */
		talonTrajectory.position = points[i][0];

		talonTrajectory.velocity = points[i][1];

		talonTrajectory.timeDurMs = points[i][2];

		talonTrajectory.profileSlotSelect = 1;

		talonTrajectory.velocityOnly = false;

		talonTrajectory.zeroPos = false;

		if (i == 0) { //First point?
			talonTrajectory.zeroPos = true;
		}

		talonTrajectory.isLastPoint = false;

		if ((i+1)==pointsLen) { //Last point?
			talonTrajectory.isLastPoint = true;
		}

		m_Talon->PushMotionProfileTrajectory(talonTrajectory); //Put the trajectory point into the buffer
	}
		mpState = kBuffered; //Done adding data to the buffer
		delete points;
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

	mpState = kBuffered;
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
	case kBuffered: //Data is buffered
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


void ArmMotionProfiling::GenerateMotionProfiles() { //used with constructor where trajectory is a parameter.

	int time = 0; //will be in increments of ARM_DELTA_TIME
	int times_incremented = 0; //will be increments of 1
	bool reached = false; //are we done yet?

	do { //figure out how many points there will be. this will be equal to times_incremented.
		times_incremented++;
		time += m_deltaTime;
	} while (m_trajectory->Position(time)!=0);

	points = new float[times_incremented][3];

	times_incremented = 0;
	time = 0;

	do { //actually create our list. its a do while loop because we want a trajectory point at 0.


		/*
		 *Actual time. this will probably be in milliseconds, which should be considered with the units of m_trajectory's functions
		 */

		(*points)[times_incremented][0] = m_trajectory->Position(time*1000); //Put the data points into the motion profile array
		(*points)[times_incremented][1] = m_trajectory->Velocity(time*1000);
		(*points)[times_incremented][2] = m_deltaTime;

		times_incremented++; //Keeps track of the number of points. a bit simpler than a division problem.
		time +=	m_deltaTime;

	} while (m_trajectory->Position(time)!=0);

}
