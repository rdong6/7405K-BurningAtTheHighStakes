//
// Created by Joey Sorkin on 3/29/23.
//

#ifndef INC_7405SPINUP_TIMEDMOTION_H
#define INC_7405SPINUP_TIMEDMOTION_H

#include "Motion.h"
class TimedMotion : public IMotion {
private:
	double power;
	uint32_t delay;

public:
	// time in ms, power in millivolts for both sides of drive to get
	TimedMotion(uint32_t time, double power);
	MotorVoltages calculate(const kinState& state) override;
	bool isVelocityControlled() const override;
	bool isSettled(const kinState& state) override;
};

#endif// INC_7405SPINUP_TIMEDMOTION_H
