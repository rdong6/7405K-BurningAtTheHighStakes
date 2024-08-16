#include "lib/geometry/kinState.h"

kinState::substate kinState::acceleration() const {
	return accel_state;
}

kinState::substate kinState::velocity() const {
	return velo_state;
}

void kinState::setVelocity(double _x, double _y, double _h) {
	velo_state.x = _x;
	velo_state.y = _y;
	velo_state.theta = _h;
}

void kinState::setAcceleration(double _x, double _y, double _h) {
	accel_state.x = _x;
	accel_state.y = _y;
	accel_state.theta = _h;
}

kinState kinState::predict(double dt) const {
	kinState future = *this;

	future.position.transformBy((velo_state.x * dt) + (accel_state.x * dt * dt * 0.5),
	                            (velo_state.y * dt) + (accel_state.y * dt * dt * 0.5),
	                            (velo_state.theta * dt) + (accel_state.theta * dt * dt * 0.5));

	future.velo_state = {velo_state.x + (velo_state.x * dt), velo_state.y + (velo_state.y * dt),
	                     velo_state.theta + (velo_state.theta * dt)};

	return future;
}