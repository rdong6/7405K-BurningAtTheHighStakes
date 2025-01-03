#pragma once
#include "Constants.h"
#include "Motion.h"
#include "lib/controller/PID.h"
#include "lib/geometry/Pose.h"
#include "lib/trajectory/TrapezoidProfile.h"

class ProfiledMotion : public IMotion {
private:
	double threshold;
	double errorSum;
	Pose startPose;
	TrapezoidProfile profile;
	int prevSign;

	bool overtime;

public:
	// units are in terms of inches
	ProfiledMotion(double dist, double maxVel, double accel, double decel, double threshold = 0.5);
	void start() override;
	bool isVelocityControlled() const override;// for testing
	MotorVoltages calculate(const kinState state) override;
	bool isSettled(const kinState state) override;
};