#pragma once
#include "Motion.h"

class OdomCharacterizationMotion : public IMotion {
public:
	OdomCharacterizationMotion() = default;
	MotorVoltages calculate(const kinState& state) override;
	bool isVelocityControlled() const override;
	bool isSettled(const kinState& state) override;
};