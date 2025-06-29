#pragma once
#include "Motion.h"

class OdomCharacterizationMotion : public IMotion {
private:
	double slewedPwr = 0;

public:
	OdomCharacterizationMotion() = default;
	MotorVoltages calculate(const kinState& state) override;
	bool isVelocityControlled() const override;
	bool isSettled(const kinState& state) override;
};