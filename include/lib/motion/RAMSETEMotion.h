#pragma once
#include "Motion.h"
#include "lib/controller/RAMSETE.h"
#include <span>

class RAMSETEMotion : public IMotion {
public:
	RAMSETEMotion(const Trajectory& trajectory, RAMSETE ramsete);
	bool isVelocityControlled() const override;
	MotorVoltages calculate(const kinState& state) override;
	bool isSettled(const kinState& state) override;

private:
	RAMSETE ramsete;
	const Trajectory& trajectory;
	double elapsedTime;
};