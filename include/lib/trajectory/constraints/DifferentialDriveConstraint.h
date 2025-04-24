#pragma once
#include "TrajectoryConstraint.h"

// limits velocity of robot around turns so that no wheel of the robot violates requested max velocity
class DifferentialDriveConstraint : public ITrajectoryConstraint {
public:
	constexpr explicit DifferentialDriveConstraint(double maxVel, double trackwidth) : maxVel(maxVel), trackwidth(trackwidth) {}

	[[nodiscard]] constexpr double getMaxVel(const Pose& pose, double curvature, double vel) const override {
		double angularVel = vel * curvature;
		double leftWheelVel = vel - trackwidth / 2.0 * angularVel;
		double rightWheelVel = vel + trackwidth / 2.0 * angularVel;

		// normalize wheel vels to ensure they don't violate max vel
		double maxWheelVel = std::fmax(std::fabs(leftWheelVel), std::fabs(rightWheelVel));
		if (maxWheelVel > maxVel) {
			leftWheelVel = leftWheelVel / maxWheelVel * maxVel;
			rightWheelVel = rightWheelVel / maxWheelVel * maxVel;
		}

		// then convert wheel vels back into chassis speed to get max robot vel
		return (leftWheelVel + rightWheelVel) / 2.0;
	}

	[[nodiscard]] constexpr Acceleration getMinMaxAcceleration(const Pose& pose, double curvature, double vel) const override {
		return {};
	}

private:
	double maxVel;
	double trackwidth;
};