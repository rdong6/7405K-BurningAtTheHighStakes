#pragma once
#include "Motion.h"
#include "lib/controller/PID.h"
#include "lib/geometry/Pose.h"

// this entire motion just uses everything in degrees cause its easier
class PIDMotion : public IMotion {
private:
	double threshold;
	Pose targetPose;
	PID pwrPID;
	PID trnPID;
	int counter;// counter for how long we've been at a pos
	bool headingCorrection;

public:
	// turn pid is for heading correction
	PIDMotion(Pose pose, PID pwrPID, PID trnPID = PID(), bool headingCorrection = false, double threshold = 0.5);
	MotorVoltages calculate(const kinState state) override;
	bool isSettled(const kinState state) override;
};