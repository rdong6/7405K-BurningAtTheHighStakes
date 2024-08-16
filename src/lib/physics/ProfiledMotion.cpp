#include "lib/physics/ProfiledMotion.h"
#include "Constants.h"
#include "Logger.h"
#include "Robot.h"
#include "lib/utils/Math.h"
#include "subsystems/Odometry.h"

// Motion based on the trapezoidal profile

LoggerPtr ProfiledMotion::logger = sLogger.createSource("ProfiledMotion", 0);

ProfiledMotion::ProfiledMotion(double dist, double maxVel, double accel, double decel, double threshold)
    : threshold(threshold), errorSum(0), startPose(), profile(dist, accel, decel, maxVel), prevSign(dist >= 0),
      overtime(false) {
	// toggles whether or not debug info from this motion is logged
	// logger->setLevel(static_cast<LogSource::LogLevel>(LogSource::INFO | LogSource::WARNING | LogSource::ERROR));
}

void ProfiledMotion::start() {
	if (startTime == 0) [[unlikely]] {
		logger->debug("Motion start. Total Time: {}\n", profile.getTotalTime());
		auto odom = robotInstance->getSubsystem<Odometry>();
		if (odom) { startPose = odom.value()->getCurrentState().position; }
		IMotion::start();
	}
}

// for testing
bool ProfiledMotion::isVelocityControlled() const {
	return true;
}

IMotion::MotorVoltages ProfiledMotion::calculate(const kinState state) {
	double time = (pros::millis() - startTime) / 1000.0;// because profile works in terms of seconds

	// testing to see if forcing it to the end of motion is better
	if (time >= profile.getTotalTime()) { overtime = true; }

	// Returns position, velocity and acceleration at a given time during the trapezoid motion
	auto targetState = profile.getState(time);

	// error just to log
	double distTraveled = state.position.distanceTo(startPose) * util::sign(targetState.pos);
	double error = -1 * (distTraveled - targetState.pos);

	// convert from in/s to in/min
	double wheelVel = targetState.vel * 60;

	// gear ratio is included in the deadwheel diamater
	double motorRPM = wheelVel / (odometers::leftDeadwheelDiameter * std::numbers::pi);

	logger->debug("TIME: {} tpos: {:.2f} tvel: {:.2f} cvel: {:.2f} tacc: {:.2f} err: {:.2f} dist traveled: {:.2f} "
	              "curHeading: {} x: {}  motor rpm: {}\n",
	              time, targetState.pos, targetState.vel, state.velocity().y, targetState.acc, error, distTraveled,
	              state.position.getTheta() / M_PI * 180, state.position.getX(), motorRPM);

	return {motorRPM, motorRPM};
}

bool ProfiledMotion::isSettled(const kinState state) {
	// Considers the bot settled if the total distance travelled during the motion of the bot is within a certain
	// threshold of the desired distance
	auto odom = robotInstance->getSubsystem<Odometry>();
	double distTraveled = odom ? odom.value()->getCurrentState().position.distanceTo(startPose) : 0;
	return std::abs(profile.getTargetDist() - distTraveled) < threshold && overtime;
}