#include "lib/motion/OdomCharacterizationMotion.h"

IMotion::MotorVoltages OdomCharacterizationMotion::calculate(const kinState& state) {
 	printf("(%f, %f),", state.position.X(), state.position.Y());
	return {3000, -3000};
}

// testing for now
bool OdomCharacterizationMotion::isVelocityControlled() const {
	return false;
}

bool OdomCharacterizationMotion::isSettled(const kinState& state) {
	return std::abs(state.position.theta()) >= std::numbers::pi;
}