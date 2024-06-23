#pragma once
#include "Constants.h"
#include "Logger.h"
#include "Motion.h"
#include "lib/controllers/PID.h"
#include "lib/geometry/Pose.h"
#include "lib/geometry/TrapezoidProfile.h"

class ProfiledMotion : public IMotion {
private:
	double threshold;
	double errorSum;
	Pose startPose;
	TrapezoidProfile profile;
	int prevSign;

	bool overtime;

	static LoggerPtr logger;

public:
	// units are in terms of inches
	ProfiledMotion(double dist, double maxVel, double accel, double decel, double threshold = 0.5);
	void start() override;
	virtual bool isVelocityControlled() const override;// for testing
	MotorVoltages calculateVoltages(kinState state) override;
	bool isSettled(kinState state) override;
};