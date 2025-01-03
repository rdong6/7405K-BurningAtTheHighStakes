#include "lib/motion/PIDMotion.h"
#include "Robot.h"
#include "lib/utils/Math.h"
#include "subsystems/Odometry.h"
#include <cmath>

PIDMotion::PIDMotion(Pose pose, PID pwrPID, PID trnPID, bool headingCorrection, double threshold)
    : threshold(threshold), targetPose(pose), pwrPID(pwrPID), trnPID(trnPID), counter(0), headingCorrection(headingCorrection) {
}

IMotion::MotorVoltages PIDMotion::calculate(const kinState state) {
	double errX = targetPose.X() - state.position.X();
	double errY = targetPose.X() - state.position.X();

	double err = -errX * std::sin(state.position.theta()) +
	             errY * std::cos(state.position.theta());// TODO: do the code to get only forwards dist

	// in degrees
	double headingToTarget = state.position.headingTo(targetPose).degrees();
	double headingErr = util::getShortestAngle(util::toDeg(state.position.theta()), headingToTarget);

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