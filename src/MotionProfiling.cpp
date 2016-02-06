#include "MotionProfiling.h"


void MotionProfiling::BeginProfiling() {


	m_Talon->SetControlMode(CANTalon::kMotionProfile);
	EndProfiling();
	mpState = kUnBuffered;
	MP = true;
	GiveBuffer();
}




void MotionProfiling::GiveBuffer() {

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





void MotionProfiling::EndProfiling() {

	mpState = kStopped; //Motion profiling has ended
	m_Talon->ClearMotionProfileTrajectories();
	m_Talon->SetControlMode(CANTalon::kVoltage);
	MP = false;

}

void MotionProfiling::Pause() {

	mpState = kPaused;
	m_Talon->SetControlMode(CANTalon::kVoltage);
}

void MotionProfiling::UnPause() {

	mpState = kBuffered;
	m_Talon->SetControlMode(CANTalon::kMotionProfile);
}



void MotionProfiling::Iterate() {

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

void MotionProfiling::Process() {
	m_Talon->ProcessMotionProfileBuffer();
}
