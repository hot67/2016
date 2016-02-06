#include <ArmMotionProfiling.h>


void ArmMotionProfiling::BeginProfiling() {


	EndProfiling();
	m_Talon->SetControlMode(CANTalon::kMotionProfile);
	mpState = kUnBuffered;
	MP = true;
	GiveBuffer();
}




void ArmMotionProfiling::GiveBuffer() {

	if (mpState==kUnBuffered) {

		CANTalon::TrajectoryPoint ArmTrajectory;

		for (int i = 0;i<pointsLen;++i) { //Iterate through the points


			/*
			 * transfer the data into the trajectory point
			 */
			ArmTrajectory.position = points[i][0];

			ArmTrajectory.velocity = points[i][1];

			ArmTrajectory.timeDurMs = points[i][2];

			ArmTrajectory.profileSlotSelect = 1;

			ArmTrajectory.velocityOnly = false;

			ArmTrajectory.zeroPos = false;

			if (i == 0) { //First point?
				ArmTrajectory.zeroPos = true;
			}

			ArmTrajectory.isLastPoint = false;

			if ((i+1)==pointsLen) { //Last point?
				ArmTrajectory.isLastPoint = true;
			}

			m_Talon->PushMotionProfileTrajectory(ArmTrajectory); //Put the trajectory point into the buffer
		}
		mpState = kBuffered;
	}
}





void ArmMotionProfiling::EndProfiling() {

	mpState = kStopped; //Motion profiling has ended
	m_Talon->ClearMotionProfileTrajectories();
	m_Talon->SetControlMode(CANTalon::kVoltage);
	MP = false;

}

void ArmMotionProfiling::Pause() {

	mpState = kPaused;
	m_Talon->SetControlMode(CANTalon::kVoltage);
}

void ArmMotionProfiling::UnPause() {

	mpState = kBuffered;
	m_Talon->SetControlMode(CANTalon::kMotionProfile);
}



void ArmMotionProfiling::Iterate() {

	switch (mpState) { //Do various things based on whats going on with motion profiling
	case kUnBuffered:
		if (MP) {GiveBuffer();}
		Process();
		break;
	case kBuffered:
		break;
	case kStopped:
		if (MP) {EndProfiling();}
		break;
	case kPaused:
		break;
	}

}

void ArmMotionProfiling::Process() {
	m_Talon->ProcessMotionProfileBuffer();
}
