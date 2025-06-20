#include "lib/motion/ProfiledMotion.h"
#include "Constants.h"
#include "Logger.h"
#include "Robot.h"
#include "lib/utils/Math.h"
#include "subsystems/Odometry.h"

// Motion based on the trapezoidal profile

ProfiledMotion::ProfiledMotion(double dist, double maxVel, double accel, double decel, double threshold)
    : threshold(threshold), errorSum(0), startPose(), profile(dist, accel, decel, maxVel), prevSign(dist >= 0),
      overtime(false) {}

void ProfiledMotion::start() {
	if (startTime == 0) [[unlikely]] {
		auto odom = robotInstance->getSubsystem<Odometry>();
		if (odom) { startPose = odom.value()->getCurrentState().position; }
		IMotion::start();
	}
}

// for testing
bool ProfiledMotion::isVelocityControlled() const {
	return true;
}

IMotion::MotorVoltages ProfiledMotion::calculate(const kinState& state) {
	double time = (pros::millis() - startTime) / 1000.0;// because profile works in terms of seconds

	// testing to see if forcing it to the end of motion is better
	if (time > profile.getTotalTime()) { overtime = true; }

	// Returns position, velocity and acceleration at a given time during the trapezoid motion
	auto targetState = profile.getState(time);

	// error just to log
	double distTraveled = state.position.translation().distanceTo(startPose.translation()) * util::sign(targetState.pos);
	double error = -1 * (distTraveled - targetState.pos);

	// convert from in/s to in/min
	double wheelVel = targetState.vel * 60;

	// gear ratio is included in the deadwheel diamater
	double motorRPM = wheelVel / (odometers::driveGearRatio * std::numbers::pi);

	// printf("TIME: %.2f tpos: %.2f tvel: %.2f cvel: %.2f tacc: %.2f err: %.2f dist traveled: %.2f "
	//               "curHeading: %.2f x: %.2f  motor rpm: %.2f\n",
	//               time, targetState.pos, targetState.vel, state.velocity().y, targetState.acc, error, distTraveled,
	//               state.position.theta() / M_PI * 180, state.position.X(), motorRPM);

	// logger->debug("TIME: {} tpos: {:.2f} tvel: {:.2f} cvel: {:.2f} tacc: {:.2f} err: {:.2f} dist traveled: {:.2f} "
	//               "curHeading: {} x: {}  motor rpm: {}\n",
	//               time, targetState.pos, targetState.vel, state.velocity().y, targetState.acc, error, distTraveled,
	//               state.position.theta() / M_PI * 180, state.position.X(), motorRPM);

	return {motorRPM, motorRPM};
}

bool ProfiledMotion::isSettled(const kinState& state) {
	// Considers the bot settled if the total distance travelled during the motion of the bot is within a certain
	// threshold of the desired distance
	// auto odom = robotInstance->getSubsystem<Odometry>();
	// double distTraveled = odom ? odom.value()->getCurrentState().position.translation().distanceTo(startPose.translation()) :
	// 0;

	double distTraveled = state.position.translation().distanceTo(startPose.translation());
	return std::abs(profile.getTargetDist() - distTraveled) < threshold || overtime;
}