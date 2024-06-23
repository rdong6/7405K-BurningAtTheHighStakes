#include "lib/follower/Ramsete.h"
#include "Constants.h"
#include <cmath>
#include <numbers>

namespace {
	double sinc(double radians) {
		if (std::abs(radians) < 1e-9) { return 1.0 - 1.0 / 6.0 * radians * radians; }

		return std::sin(radians) / radians;
	}
}// namespace

RAMSETE::RAMSETE(double beta, double zeta) : beta(beta), zeta(zeta) {}

RAMSETE::WheelVelocities RAMSETE::calculate(const Pose& curPose, const Pose& targetPose, double linearVelRef,
                                            double angularVelRef) const {
	// convert from our normal robot heading to one suitable for trig
	double convertedRobotHeading = (std::numbers::pi / 2.0) - curPose.getTheta();
	double convertedTargetHeading = (std::numbers::pi / 2.0) - targetPose.getTheta();

	// get tracking error of robot to target point and transform it into a local coordinate frame of robot
	// Y = move forwards
	// X = move right

	// !-------------------!
	//      VERIFY THIS
	// !-------------------!
	Pose displacementVector = Pose::getTransformation(curPose, targetPose);
	double eX = std::sin(convertedRobotHeading) * displacementVector.getX() -
	            std::cos(convertedRobotHeading) * displacementVector.getY();

	double eY = std::cos(convertedRobotHeading) * displacementVector.getX() +
	            std::sin(convertedRobotHeading) * displacementVector.getY();

	// !-------------------!
	//      VERIFY THIS
	// !-------------------!
	double eTheta = convertedTargetHeading - convertedRobotHeading;

	double k = 2 * zeta * std::sqrt(beta * linearVelRef * linearVelRef + angularVelRef * angularVelRef);


	// I flipped eX and eY because in the paper, eX represents forwards/backwards
	// while eY represents forwards/backwards for us
	double linearVel = linearVelRef * std::cos(eTheta) + k * eY;
	double angularVel = angularVelRef + k * eTheta + beta * linearVelRef * ::sinc(eTheta) * eX;

	// apply a little differential drive kinematics to convert to wheel speeds
	return {linearVel - angularVel * odometers::trackWidth / 2, linearVel + angularVel * odometers::trackWidth / 2};
}