#include "lib/physics/ProfiledMotion.h"
#include "Constants.h"
#include "Logger.h"
#include "Odometry.h"
#include "lib/utils/Math.h"

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
		startPose = sOdom.getCurrentState().position;
		IMotion::start();
	}
}

// for testing
bool ProfiledMotion::isVelocityControlled() const {
	return true;
}

IMotion::MotorVoltages ProfiledMotion::calculateVoltages(kinState state) {
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


	// TODO: shift this away to using velocity motion and have that manage the voltage calculation part
	// basically remove most of this code

	// testing
	/*constexpr double kSLeft = 1065.65839038;    // original
	constexpr double kVLeft = 152.4539999079319;// original
	// constexpr double kVLeft = 157.4539999079319;
	constexpr double kALeft = 35.450363464220203;
	constexpr double kDeLeft = 38.5;
	// constexpr double kDeLeft = kALeft;

	constexpr double kSRight = 835.517;
	// constexpr double kVRight = 157.135;
	constexpr double kVRight = 153.535;
	constexpr double kARight = 35.3875;
	// constexpr double kDeRight = 35;
	constexpr double kDeRight = kARight;

	int sign = util::sign(targetState.vel);
	if (util::fpEquality(targetState.vel, 0.0)) { sign = 0; }

	double FFLeft = kVLeft * targetState.vel + sign * kSLeft;
	FFLeft += targetState.acc > 0 ? kALeft * targetState.acc : kDeLeft * targetState.acc;

	double FFRight = kVRight * targetState.vel + sign * kSRight;
	FFRight += targetState.acc > 0 ? kARight * targetState.acc : kDeRight * targetState.acc;

	auto curState = sOdom.getCurrentState();
	double distTraveled = curState.position.distanceTo(startPose) * util::sign(targetState.pos);
	double error = -1 * (distTraveled - targetState.pos);

	// TODO: For wednesday - take out FB pwr first and just tune the FF values first - minimize drift and get as close
	// as possible
	// double fbPwr = 0;
	double fbPwr = error * 750;
	// double fbPwr = error > 0 ? error * 500 : error * 250;
	if (std::abs(profile.getTargetDist() - distTraveled) < 2) { errorSum += error; }
	if (util::sign(error) != prevSign) {
	    errorSum = 0;
	    prevSign = util::sign(error);
	}

	fbPwr += errorSum * 250;

	double leftPwr = FFLeft + fbPwr;
	double rightPwr = FFRight + fbPwr;

	logger->debug("TIME: {} tpos: {:.2f} tvel: {:.2f} cvel: {:.2f} tacc: {:.2f} err: {:.2f} dist traveled: {:.2f} "
	              "ff: {:.2f} fb: {:.2f} "
	              "total pwr: {:.2f} curHeading: {} x: {} err sum: {}"
	              "\n",
	              time, targetState.pos, targetState.vel, curState.velocity().y, targetState.acc, error, distTraveled,
	              FFLeft, fbPwr, leftPwr, curState.position.getTheta() / M_PI * 180, curState.position.getX(),
	              errorSum);

	return {leftPwr, rightPwr};*/
}

bool ProfiledMotion::isSettled(kinState state) {
	// Considers the bot settled if the total distance travelled during the motion of the bot is within a certain
	// threshold of the desired distance
	double distTraveled = sOdom.getCurrentState().position.distanceTo(startPose);
	return std::abs(profile.getTargetDist() - distTraveled) < threshold && overtime;
}