#include "lib/physics/PIDMotion.h"
#include "Robot.h"
#include "lib/utils/Math.h"
#include "subsystems/Odometry.h"
#include <cmath>

PIDMotion::PIDMotion(double dist, PID pwrPID, PID trnPID, bool headingCorrection, double threshold)
    : PIDMotion(Pose(), pwrPID, trnPID, headingCorrection, threshold) {
	auto odom = robotInstance->getSubsystem<Odometry>();
	if (odom) { this->targetPose = odom.value()->getCurrentState().position; }
	Pose originalPose = odom ? odom.value()->getCurrentState().position : Pose();

	double robotCurvHeading = M_PI_2 - originalPose.getTheta();
	double deltaX = std::cos(robotCurvHeading) * dist;
	double deltaY = std::sin(robotCurvHeading) * dist;

	this->targetPose = originalPose.transformBy(deltaX, deltaY, 0);
}

PIDMotion::PIDMotion(Pose pose, PID pwrPID, PID trnPID, bool headingCorrection, double threshold)
    : threshold(threshold), targetPose(pose), pwrPID(pwrPID), trnPID(trnPID), counter(0),
      headingCorrection(headingCorrection) {}

IMotion::MotorVoltages PIDMotion::calculate(const kinState state) {
	double errX = targetPose.getX() - state.position.getX();
	double errY = targetPose.getY() - state.position.getY();

	double err = -errX * std::sin(state.position.getTheta()) +
	             errY * std::cos(state.position.getTheta());// TODO: do the code to get only forwards dist

	// in degrees
	double headingToTarget = util::toDeg(state.position.headingToPoint(targetPose));
	double headingErr = util::getShortestAngle(util::toDeg(state.position.getTheta()), headingToTarget);

	if (err < threshold) {
		counter++;
	} else {
		counter = 0;
	}

	double pwr = pwrPID(err);
	double trn = trnPID(headingErr);

	// Current STATUS: this heading correction impl is wrong (and it was never correct), and may write a newer version
	// just ignore heading correction for now

	// stop heading correction 1 inch out
	if (err < 1) {
		trn = 0;
	} else if (headingCorrection) {
		double totalPwr = pwr + trn;

		double maxPwr = pwr * (12000 / totalPwr);
		double maxTrn = trn * (12000 / totalPwr);

		pwr = pwr < maxPwr ? pwr : maxPwr;
		trn = trn < maxTrn ? trn : maxTrn;
	}

	return {pwr + trn, pwr - trn};
}

bool PIDMotion::isSettled(const kinState state) {
	return counter > 5;
}