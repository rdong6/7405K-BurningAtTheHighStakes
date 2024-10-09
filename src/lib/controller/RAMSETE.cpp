#include "lib/controller/RAMSETE.h"
#include "Constants.h"
#include "lib/utils/Math.h"
#include <cmath>
#include <cstdio>
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
	// not the normal rotation <- i slightly scuffed it
	Pose displacementVector = Pose::getTransformation(curPose, targetPose);
	double eX = std::sin(convertedRobotHeading) * displacementVector.getX() -
	            std::cos(convertedRobotHeading) * displacementVector.getY();

	double eY = std::cos(convertedRobotHeading) * displacementVector.getX() +
	            std::sin(convertedRobotHeading) * displacementVector.getY();

	// !-------------------!
	//      VERIFY THIS
	// !-------------------!
	// double eTheta = util::toRad(
	//         -util::getShortestAngle(util::toDeg(convertedRobotHeading), util::toDeg(convertedTargetHeading)));

	double eTheta = (std::numbers::pi / 2) - util::toRad(util::getShortestAngle(util::toDeg(curPose.getTheta()),
	                                                                            util::toDeg(targetPose.getTheta())));

	double k = 2 * zeta * std::sqrt(beta * linearVelRef * linearVelRef + angularVelRef * angularVelRef);


	// I flipped eX and eY because in the paper, eX represents forwards/backwards
	// while eY represents forwards/backwards for us
	double linearVel = linearVelRef * std::cos(eTheta) + k * eY;
	double angularVel = angularVelRef + k * eTheta + beta * linearVelRef * ::sinc(eTheta) * eX;

	double leftWheelVel = linearVel + angularVel * odometers::trackWidth / 2;
	double rightWheelVel = linearVel - angularVel * odometers::trackWidth / 2;

	// logger->debug("Linear Vel: {:2f}  Angular Vel: {:2f}  eY: {:2f}  eX: {:2f}  eTheta: {:2f}  Left Vel: {:2f}  Right
	// "
	//               "Vel: {:2f}\n",
	//               linearVel, angularVel, eY, eX, eTheta, leftWheelVel, rightWheelVel);

	// apply a little differential drive kinematics to convert to wheel speeds
	return {linearVel - angularVel * odometers::trackWidth / 2, linearVel + angularVel * odometers::trackWidth / 2};
}