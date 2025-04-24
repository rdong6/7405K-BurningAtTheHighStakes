#include "lib/controller/RAMSETE.h"
#include "Constants.h"
#include "lib/utils/Math.h"
#include <cmath>
#include <numbers>

namespace {
	double sinc(double radians) {
		if (std::abs(radians) < 1e-9) { return 1.0 - 1.0 / 6.0 * radians * radians; }

		return std::sin(radians) / radians;
	}
}// namespace

RAMSETE::WheelVelocities RAMSETE::calculate(const Pose& curPose, const Pose& targetPose, double linearVelRef,
                                            double angularVelRef) const {
	Pose errorPose = targetPose.relativeTo(curPose);

	const double eX = errorPose.X();
	const double eY = errorPose.Y();
	const double eTheta = errorPose.theta();

	const double k = 2.0 * zeta * std::sqrt(angularVelRef * angularVelRef + beta * linearVelRef * linearVelRef);

	double vel = linearVelRef * std::cos(eTheta) + k * eX;
	double angularVel = angularVelRef + k * eTheta + beta * linearVelRef * sinc(eTheta) * eY;

	printf("[Ramsete] Vel: %.2f\tAngular Vel: %.2f\teX: %.2f\teY: %.2f\teThetha: %.2f\n", vel, angularVel, eX, eY,
	       util::toDeg(eTheta));
	// converts from chassis (linear vel & angular vel) to wheel speeds (left & right)
	return {vel - angularVel * odometers::trackWidth * 0.5, vel + angularVel * odometers::trackWidth * 0.5};
}

RAMSETE::WheelVelocities RAMSETE::calculate(const Pose& curPose, const Trajectory::State& state) const {
	return calculate(curPose, state.pose, state.vel, state.vel * state.curvature);
}