#pragma once
#include "Motion.h"
#include "lib/controllers/FeedForward.h"
#include "lib/controllers/PID.h"

// WIP (Not really used at the moment)

// for when the motion deals with velocities
// and you don't want to use VEX's set_velocity
class VelocityMotion : public IMotion {
private:
	FeedForward ff;
	PID fbLeft;
	PID fbRight;

protected:
	struct ChassisSpeeds {
		// in/s
		double leftVel = 0;

		// in/s
		double rightVel = 0;
	};

	virtual ChassisSpeeds calculateVelocities() = 0;

public:
	VelocityMotion(FeedForward ff, PID fbLeft, PID fbRight);
	void start() override;
	MotorVoltages
	calculateVoltages(kinState state) override final;// calls calculateVelocities, and then outputs voltages
	virtual bool isSettled(kinState state) override = 0;
};