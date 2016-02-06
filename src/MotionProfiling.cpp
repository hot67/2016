#include "MotionProfiling.h"


void MotionProfiling::BeginProfiling() {


	m_Talon->SetControlMode(CANTalon::kMotionProfile);
	startMP = true;
	GiveBuffer();
}


void MotionProfiling::GiveBuffer() {

	if (startMP) {

		CANTalon::TrajectoryPoint ArmTrajectory;

		for (int i = 0;i<pointsLen;++i) {

			ArmTrajectory.position = (points + i)[0];

			ArmTrajectory.velocity = (points + i)[1];

			ArmTrajectory.timeDurMs = (points + i)[2];

			ArmTrajectory.profileSlotSelect = 1;

			ArmTrajectory.velocityOnly = false;

			ArmTrajectory.zeroPos = false;

			if (i == 0) {
				ArmTrajectory.zeroPos = true;
			}

			ArmTrajectory.isLastPoint = false;

			if ((i+1)==pointsLen) {
				ArmTrajectory.isLastPoint = true;
			}

			m_Talon->PushMotionProfileTrajectory(ArmTrajectory);

		}
	}
}


void MotionProfiling::EndProfiling() {



}
