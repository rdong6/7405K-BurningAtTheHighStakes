#include "lib/motion/BoomerangMotion.h"
#include "pros/rtos.hpp"

BoomerangMotion::BoomerangMotion(const Pose& start, const Pose& target, double dlead, double maxVel, double maxAcc,
                                 bool reversed)
    : boomerangPath(TrajectoryConfig{.startVel = 0, .endVel = 0, .maxVel = maxVel, .maxAcc = maxAcc, .reversed = reversed},
                    Boomerang(start, target, dlead)),
      generatedTrajectory(trajectoryGenerator.generate(boomerangPath)), elapsedTime(0) {}

void BoomerangMotion::start() {
	IMotion::start();
	prevTime = startTime;
}

IMotion::MotorVoltages BoomerangMotion::calculate(const kinState& state) {
	// TODO: Replace wrapper around ramsete w/ time dilation (kinda like a pure-pursuit, ramsete hybrid)
	// TODO: or just use a time-invariant path follower

	elapsedTime = (pros::millis() - startTime) / 1000.0;

	Trajectory::State sampledState = generatedTrajectory.sample(elapsedTime);
	RAMSETE::WheelVelocities wheelVels = trajectory::ramsete.calculate(state.position, sampledState);

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

	printf("[BoomerangMotion] ElapsedTime: %.2f\tSampled State: (t: %.2f X: %.2f Y: %.2f H: %.2f v: %.2f)\tCur State: (%.2f, "
	       "%.2f, %.2f)\tLeft: %.2f\tRight: "
	       "%.2f\n",
	       elapsedTime, sampledState.t, sampledState.pose.X(), sampledState.pose.Y(), sampledState.pose.rotation().degrees(),
	       sampledState.vel, state.position.X(), state.position.Y(), state.position.rotation().degrees(), leftMotorRPM,
	       rightMotorRPM);
	return {leftMotorRPM, rightMotorRPM};
}

bool BoomerangMotion::isVelocityControlled() const {
	return true;
}

bool BoomerangMotion::isSettled(const kinState& state) {
	return elapsedTime > generatedTrajectory.getTotalTime();
}