#include "lib/physics/RAMSETEMotion.h"
#include "Constants.h"
#include "lib/controller/RAMSETE.h"
#include "lib/geometry/Pose.h"
#include "lib/geometry/kinState.h"
#include "lib/physics/Motion.h"
#include "lib/trajectory/GeneratedPoint.h"
#include "lib/utils/Math.h"
#include "pros/rtos.hpp"
#include <numbers>

RAMSETEMotion::RAMSETEMotion(std::span<fttbtkjfk::GeneratedPoint>&& path, RAMSETE ramsete)
    : path(std::forward<std::span<fttbtkjfk::GeneratedPoint>>(path)), ramsete(ramsete), counter(0) {}

IMotion::MotorVoltages RAMSETEMotion::calculate(const kinState state) {
	uint32_t curTime = pros::millis() - startTime;

	// simple bs testing of ramsete real quick
	const fttbtkjfk::GeneratedPoint& point = path[counter];
	Pose targetPose = Pose(point.pose.x, point.pose.y, util::toRad(point.pose.heading));
	counter++;

	RAMSETE::WheelVelocities wheelVels =
	        ramsete.calculate(state.position, targetPose, point.vel, point.vel * point.curv);

	// RAMSETE::WheelVelocities wheelVels{.left = point.wheelVels.left, .right = point.wheelVels.right};

	double leftMotorRPM = wheelVels.left * 60.0;
	// left deadwheel diameter is used only because we used motor encoders
	// HAVE TO CHANGE WHEN WE USE DEADWHEELS IN THE SEASON
	leftMotorRPM /= (odometers::leftDeadwheelDiameter * std::numbers::pi);

	double rightMotorRPM = wheelVels.right * 60.0;
	rightMotorRPM /= (odometers::rightDeadwheelDiameter * std::numbers::pi);

	// logger->info("Left RPM: {}  Right RPM: {}  Target: ({}, {}, {})  Cur: ({}, {}, {})  Vel: {}  Ang Vel: {}\n",
	//              leftMotorRPM, rightMotorRPM, targetPose.getX(), targetPose.getY(),
	//              util::toDeg(targetPose.getTheta()), state.position.getX(), state.position.getY(),
	//              util::toDeg(state.position.getTheta()), point.vel, point.vel * point.curv);

	return {leftMotorRPM, rightMotorRPM};
}

bool RAMSETEMotion::isVelocityControlled() const {
	return true;
}

bool RAMSETEMotion::isSettled(const kinState state) {
	return counter >= path.size();
}