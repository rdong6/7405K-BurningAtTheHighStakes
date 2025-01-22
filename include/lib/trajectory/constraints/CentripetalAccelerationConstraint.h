#pragma once
#include "TrajectoryConstraint.h"

// Constrant on absolute maximum centripetal acceleration. Limiting this will cause robot to slow down around tight turns,
// making it easier to track trajectories w/ sharp turns.
class CentripetalAccelerationConstraint : public ITrajectoryConstraint {
public:
	explicit CentripetalAccelerationConstraint(double maxCentripetalAcceleration)
	    : maxCentripetalAccel(maxCentripetalAcceleration) {}

	[[nodiscard]] constexpr double getMaxVel(const Pose& pose, double curvature, double vel) const override {
		// ac = v²/r
		// k = 1/r

		// therefore: ac = v²k
		// ac/k = v²
		// v = √(ac/k)
		return vsqrtd(maxCentripetalAccel / std::fabs(curvature));
	}

	[[nodiscard]] constexpr Acceleration getMinMaxAcceleration(const Pose& pose, double curvature, double vel) const override {
		return {};
	}

private:
	double maxCentripetalAccel;
};