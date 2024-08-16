//
// Created by Joey Sorkin on 3/24/23.
//
#include "lib/physics/NullMotion.h"
#include "lib/geometry/kinState.h"

// No motion - does nothing

NullMotion::NullMotion(bool velControl) : velocityControlled(velControl) {}

IMotion::MotorVoltages NullMotion::calculate(const kinState state) {
	return {0.0, 0.0};
}

// returns true because a move_velocity of 0 acts as motor.brake()
bool NullMotion::isVelocityControlled() const {
	return velocityControlled;
}

bool NullMotion::isSettled(const kinState state) {
	return false;
}