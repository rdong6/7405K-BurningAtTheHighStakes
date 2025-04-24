#include "lib/motion/RAMSETEMotion.h"
#include "Constants.h"
#include "lib/controller/RAMSETE.h"
#include "lib/geometry/Pose.h"
#include "lib/geometry/kinState.h"
#include "lib/motion/Motion.h"
#include "lib/trajectory/Trajectory.h"
#include "pros/rtos.hpp"
#include <numbers>

RAMSETEMotion::RAMSETEMotion(const Trajectory& trajectory, RAMSETE ramsete)
    : ramsete(ramsete), trajectory(trajectory), elapsedTime(0) {}

IMotion::MotorVoltages RAMSETEMotion::calculate(const kinState& state) {
	elapsedTime = (pros::millis() - startTime) / 1000.0;

	Trajectory::State sampledState = trajectory.sample(elapsedTime);
	RAMSETE::WheelVelocities wheelVels = ramsete.calculate(state.position, sampledState);

	double leftMotorRPM = wheelVels.left * 60.0;// converts in/s to in/min
	leftMotorRPM /= (odometers::driveGearRatio * std::numbers::pi);

	double rightMotorRPM = wheelVels.right * 60.0;
	rightMotorRPM /= (odometers::driveGearRatio * std::numbers::pi);

	// TODO: Fix this as we cannot assume a drive will always use blue carts
	double maxAbsWheelRPM = std::fmax(std::fabs(leftMotorRPM), std::fabs(rightMotorRPM));
	if (maxAbsWheelRPM > 600) {
		leftMotorRPM = leftMotorRPM / maxAbsWheelRPM * 600;
		rightMotorRPM = rightMotorRPM / maxAbsWheelRPM * 600;
	}

	printf("[RamseteMotion] ElapsedTime: %.2f\tSampled State: (t: %.2f X: %.2f Y: %.2f H: %.2f v: %.2f)\tCur State: (%.2f, "
	       "%.2f, %.2f)\tLeft: %.2f\tRight: "
	       "%.2f\n",
	       elapsedTime, sampledState.t, sampledState.pose.X(), sampledState.pose.Y(), sampledState.pose.rotation().degrees(),
	       sampledState.vel, state.position.X(), state.position.Y(), state.position.rotation().degrees(), leftMotorRPM,
	       rightMotorRPM);
	return {leftMotorRPM, rightMotorRPM};
}

bool RAMSETEMotion::isVelocityControlled() const {
	return true;
}

bool RAMSETEMotion::isSettled(const kinState& state) {
	printf("[RamseteMotion] Elapsed Time: %f\tTotal Time: %f\n", elapsedTime, trajectory.getTotalTime());
	return elapsedTime > trajectory.getTotalTime();
}