#pragma once
#include "Motion.h"
#include <cstdio>

class DriveCharacterizationMotion : public IMotion {
private:
	FILE* file = nullptr;
	bool exit;
	int startLeftPos;
	int startRightPos;

public:
	DriveCharacterizationMotion() = default;
	void start() override;
	MotorVoltages calculate(const kinState& state) override;
	bool isSettled(const kinState& state) override;
};