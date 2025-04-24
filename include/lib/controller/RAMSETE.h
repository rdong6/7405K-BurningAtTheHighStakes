#pragma once
#include "lib/geometry/Pose.h"
#include "lib/trajectory/Trajectory.h"

// The gain values for RAMSETE should be robot-agnostic as this controller doesn't do actual velocity tracking. It only provides
// corrective velocity control for Pose errors
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

	constexpr RAMSETE(double beta, double zeta) : beta(beta), zeta(zeta) {}

	WheelVelocities calculate(const Pose& curPose, const Pose& targetPose, double linearVelRef, double angularVelRef) const;
	WheelVelocities calculate(const Pose& curPose, const Trajectory::State& state) const;
};