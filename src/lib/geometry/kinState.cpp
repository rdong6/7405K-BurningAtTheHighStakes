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