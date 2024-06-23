//
// Created by Joey Sorkin on 3/24/23.
//

#ifndef INC_7405SPINUP_NULLMOTION_H
#define INC_7405SPINUP_NULLMOTION_H
#include "Motion.h"

class NullMotion : public IMotion {
private:
	bool velocityControlled;

public:
	NullMotion(bool velControl = false);
	MotorVoltages calculateVoltages(kinState state) override;
	bool isVelocityControlled() const override;
	bool isSettled(kinState state) override;
};

#endif// INC_7405SPINUP_NULLMOTION_H
