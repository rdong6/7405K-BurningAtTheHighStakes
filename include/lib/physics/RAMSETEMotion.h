#pragma once
#include "Logger.h"
#include "Motion.h"
#include "lib/controller/RAMSETE.h"
#include "lib/trajectory/GeneratedPoint.h"
#include <span>

class RAMSETEMotion : public IMotion {
private:
	RAMSETE ramsete;
	std::span<fttbtkjfk::GeneratedPoint> path;
	uint32_t counter;

public:
	RAMSETEMotion(std::span<fttbtkjfk::GeneratedPoint>&& path, RAMSETE ramsete);
	bool isVelocityControlled() const override;
	MotorVoltages calculate(const kinState state) override;
	bool isSettled(const kinState state) override;
};