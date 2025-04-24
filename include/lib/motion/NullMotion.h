#ifndef INC_7405SPINUP_NULLMOTION_H
#define INC_7405SPINUP_NULLMOTION_H
#include "Motion.h"

class NullMotion : public IMotion {
private:
	bool velocityControlled;

public:
	// if true, acts as motor.brake() because motor.move_velocity(0) is what engages braking
	NullMotion(bool velControl = false);
	MotorVoltages calculate(const kinState& state) override;
	bool isVelocityControlled() const override;
	bool isSettled(const kinState& state) override;
};

#endif// INC_7405SPINUP_NULLMOTION_H
