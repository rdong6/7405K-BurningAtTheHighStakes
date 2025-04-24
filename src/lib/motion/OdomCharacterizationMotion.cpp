#include "lib/motion/OdomCharacterizationMotion.h"

IMotion::MotorVoltages OdomCharacterizationMotion::calculate(const kinState& state) {
	if (slewedPwr < 2500) { slewedPwr += 500; }
	printf("(%f, %f),", state.position.X(), state.position.Y());
	return {slewedPwr, -1 * slewedPwr};
}

// testing for now
bool OdomCharacterizationMotion::isVelocityControlled() const {
	return false;
}

bool OdomCharacterizationMotion::isSettled(const kinState& state) {
	return std::abs(state.position.theta()) >= (2 * std::numbers::pi);
}