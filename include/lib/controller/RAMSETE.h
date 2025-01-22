#pragma once
#include "lib/geometry/Pose.h"
#include "lib/trajectory/Trajectory.h"

class RAMSETE {
private:
	double beta;
	double zeta;

public:
	struct WheelVelocities {
		// units are whatever the reference linear and angular vels are
		// which should be in/s
		double left;
		double right;
	};

	RAMSETE(double beta, double zeta);

	WheelVelocities calculate(const Pose& curPose, const Pose& targetPose, double linearVelRef, double angularVelRef) const;
	WheelVelocities calculate(const Pose& curPose, const Trajectory::State& state) const;
};