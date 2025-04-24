#include "lib/motion/PursuitMotion.h"
#include "Constants.h"
#include "Robot.h"
#include "lib/utils/Math.h"
#include "pros/rtos.hpp"
#include "subsystems/Drive.h"
#include <cmath>
#include <limits>
#include <memory>
#include <numbers>

namespace {
	// careful! sgn(0) = 0
	template<typename T>
	int sgn(T val) {
		return (T(0) < val) - (val < T(0));
	}
}// namespace


PursuitMotion::PursuitMotion(std::span<fttbtkjfk::PursuitPoint>&& path, double lookahead, double maxVel, double maxAccel,
                             bool reversed, double threshold, double k)
    : path(std::forward<std::span<fttbtkjfk::PursuitPoint>>(path)), k(k), threshold(threshold), lookahead(lookahead),
      prevTime(0), maxVel(maxVel), maxAccel(maxAccel), curVel(0), lastLookaheadIndex(0), lastLookaheadPose(),
      isReversed(reversed ? -1 : 1) {
	vels = std::make_shared<double[]>(this->path.size());

	// INITIAL VELOCITY - do max vel
	// veloicty is min of max vel or k/curv - k is 1-5
	// for (auto& point : path) {
	for (size_t i = 0; i < path.size(); i++) {
		auto& point = path[i];

		if (util::fpEquality(point.pose.x, 0.0)) { point.pose.x = 0; }

		if (util::fpEquality(point.pose.y, 0.0)) { point.pose.y = 0; }

		if (util::fpEquality(std::abs(point.curv), 0.0)) {
			vels[i] = maxVel;
		} else {
			// k = 1-5. change based on how slow you want robot to go around turns
			vels[i] = std::min(maxVel, k / std::abs(point.curv));
		}
	}


	// FINAL PASS - assumes infinite accel, but decel is based on the maxAccel passed in
	// we will use rate limiter to limit accel
	// but by using infinite accel, we sidestep issue of 0 vel at start of path

	// calulate vels from back of path to front - so that we rate limit decel
	// vf = √(vi^2 + 2 * a * d)
	vels[path.size() - 1] = 0;// ending will always be 0 vel
	for (size_t i = path.size() - 2; i >= 0; i--) {
		double dist =
		        std::sqrt(std::pow(path[i].pose.x - path[i + 1].pose.x, 2) + std::pow(path[i].pose.y - path[i + 1].pose.y, 2));

		vels[i] = std::min(vels[i], std::sqrt(std::pow(vels[i + 1], 2) + 2 * maxAccel * dist));
	}
}

int PursuitMotion::findClosestIndex(const Pose& pos) const {
	double minDist = std::numeric_limits<double>::max();
	int closestIndex = -1;

	for (size_t i = 0; i < path.size(); ++i) {
		double dist = std::sqrt(std::pow(path[i].pose.x - pos.X(), 2) + std::pow(path[i].pose.y - pos.X(), 2));

		if (dist < minDist) {
			minDist = dist;
			closestIndex = i;
		}
	}

	return closestIndex;
}

// https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm/1084899#1084899
[[deprecated("NEED TO FIX THIS FOR NEW COORDINATE SYSTEM")]] double
PursuitMotion::circleIntersect(const Pose& p1, const Pose& p2, const Pose& pos) const {
	Translation2D d = p2.translation() - p1.translation();
	Translation2D f = p1.translation() - pos.translation();
	double a = d.dot(d);
	double b = 2 * f.dot(d);
	double c = f.dot(f) - lookahead * lookahead;
	double discriminant = b * b - 4 * a * c;

	if (discriminant >= 0) {
		// possible intersection found
		discriminant = vsqrtd(discriminant);
		double t1 = (-b - discriminant) / (2 * a);
		double t2 = (-b + discriminant) / (2 * a);

		// prioritizes point further down the line - that is the one that moves down the path
		// currently preferring t2 over t1

		// if (t2 >= 0 && t2 <= 1) return t2;
		// else if (t1 >= 0 && t1 <= 1)
		// 	return t1;

		// testing out t1 over t2 - honestly this seemed to produce better results
		if (t1 >= 0 && t1 <= 1) return t1;
		else if (t2 >= 0 && t2 <= 1)
			return t2;
	}

	// no intersection found
	return -1;
}

Pose PursuitMotion::getLookaheadPoint(const Pose& pos) const {
	// search path in reverse as the first one found is farthest one down the path and the one we want to pursue
	for (size_t i = path.size() - 1; i > lastLookaheadIndex; i--) {
		// is this cast bad? yes. do i care? no

		// TODO: FIx this! Can't just recast a ptr anymore as Pose structure change
		const Pose& lineStart = *reinterpret_cast<const Pose*>(&path[i - 1].pose);
		const Pose& lineEnd = *reinterpret_cast<const Pose*>(&path[i].pose);

		double t = circleIntersect(lineStart, lineEnd, pos);
		if (t != -1) {
			lastLookaheadIndex = i;
			return lineStart.lerp(lineEnd, t);
		}
	}

	// we deviated too far from the path that there is no circle line intersection given the lookahead dist
	// just try and drive back to last lookahead pose
	return lastLookaheadPose;
}

[[deprecated("NEED TO FIX THIS FOR NEW COORDINATE SYSTEM")]] double PursuitMotion::calcCurv(const Pose& pos, double heading,
                                                                                            const Pose& lookahead) const {
	// side = left or right
	int side = sgn(std::sin(heading) * (lookahead.X() - pos.X()) - std::cos(heading) * (lookahead.X() - pos.X()));

	double a = -std::tan(heading);
	double c = std::tan(heading) * pos.X() - pos.X();
	double x = std::fabs(a * lookahead.X() + lookahead.X() + c) / vsqrtd((a * a) + 1);
	double d = std::hypot(lookahead.X() - pos.X(), lookahead.X() - pos.X());

	return side * ((2 * x) / (d * d));
}

double PursuitMotion::curDistFromEnd(kinState state) const {
	double totalDist = 0;
	int closest = findClosestIndex(state.position);

	for (int i = path.size() - 1; i > closest; --i) {
		totalDist +=
		        std::sqrt(std::pow(path[i].pose.x - path[i - 1].pose.x, 2) + std::pow(path[i].pose.y - path[i - 1].pose.y, 2));
	}

	totalDist += (1.0 - (closest - std::floor(closest))) *
	             std::sqrt(std::pow(path[std::floor(closest)].pose.x - path[std::ceil(closest)].pose.x, 2) +
	                       std::pow(path[std::floor(closest)].pose.y - path[std::ceil(closest)].pose.y, 2));

	return totalDist;
}

void PursuitMotion::start() {
	if (startTime == 0) [[unlikely]] {
		auto odom = robotInstance->getSubsystem<Odometry>();
		lastLookaheadIndex = findClosestIndex(odom ? odom.value()->getCurrentState().position : Pose());
		lastLookaheadPose = *reinterpret_cast<const Pose*>(&path[lastLookaheadIndex].pose);
		prevTime = pros::millis();
		IMotion::start();
	}
}

[[deprecated("NEED TO FIX THE DIFFERENTIAL DRIVE KINEMATICS CONVERTING CHASSIS SPEEDS TO WHEEL SPEEDS-> FLIP SIGNS")]] IMotion::
        MotorVoltages
        PursuitMotion::calculate(const kinState& state) {
	// dt is used for rate limiting of vel of robot
	double curTime = pros::millis();
	double dt = (curTime - prevTime) / 1000.0;
	prevTime = curTime;

	int closestIndex = findClosestIndex(state.position);

	Pose lookaheadPose = getLookaheadPoint(state.position);
	lastLookaheadPose = lookaheadPose;

	double curv = calcCurv(state.position, state.position.theta(), lookaheadPose);

	// target vel of the robot is based on the closest point to the robot after rate limiting acceleration
	// rate limit velocity - because we assumed infinite accel but that's not true earlier so that robot would move
	double targetVel = vels[closestIndex];

	// rate limiting of vel
	double maxVelChange = maxAccel * dt;
	curVel += util::clamp(-maxVelChange, maxVelChange, targetVel - curVel);

	double targetLeftVel = isReversed * curVel * (2 + curv * odometers::trackWidth) / 2.0;
	double targetRightVel = isReversed * curVel * (2 - curv * odometers::trackWidth) / 2.0;

	// sometimes the vels go above max vel
	// normalize so wheel vels don't violate constraints of robot
	double ratio = std::max(std::fabs(targetLeftVel), std::fabs(targetRightVel)) / maxVel;
	if (ratio > 1) {
		targetLeftVel /= ratio;
		targetRightVel /= ratio;
	}

	// logger->info(
	//         "TVel: {:2f} LVel: {:2f}  RVel: {:2f} Curv: {:2f} Look: ({:2f}, {:2f}) Cur: ({:2f}, {:2f}, {:2f}) Dist: "
	//         "{:2f} dLVel: {:2f} dRVel: {:2f}\n",
	//         curVel, targetLeftVel, targetRightVel, curv, lookaheadPose.X(), lookaheadPose.X(),
	//         state.position.X(), state.position.X(), util::toDeg(state.position.theta()),
	//         curDistFromEnd(state), sDrive.getLeftVelocity() * (odometers::leftDeadwheelDiameter * std::numbers::pi)
	//         / 60.0, sDrive.getRightVelocity() * (odometers::rightDeadwheelDiameter * std::numbers::pi) / 60.0);

	// convert wheel rpm into motor rpm
	// gear ratio is in wheel diamater

	// convert in/s into in/min
	targetLeftVel *= 60.0;
	targetRightVel *= 60.0;

	// convert from in/min into RPM
	targetLeftVel /= (odometers::leftDeadwheelDiameter * std::numbers::pi);
	targetRightVel /= (odometers::rightDeadwheelDiameter * std::numbers::pi);

	return {targetLeftVel, targetRightVel};
}

bool PursuitMotion::isVelocityControlled() const {
	return true;
}

bool PursuitMotion::isSettled(const kinState& state) {
	return curDistFromEnd(state) <= threshold;
}