#pragma once
#include "Motion.h"

class OpControlMotion : public IMotion {
public:
	OpControlMotion() = default;
	MotorVoltages calculate(const kinState& state);
	bool isSettled(const kinState& state);
};