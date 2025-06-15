#include "lib/motion/OpControlMotion.h"
#include "lib/utils/Math.h"
#include "pros/misc.h"
#include <cmath>

namespace {
	int sign(double val) {
		return (val > 0) - (val < 0);
	}
}// namespace

// roman specific driver code from his years at N & R
static int dampen(int input) {
	double expo = 0.35;
	double output = 0;
    double in = input / 127.0;
    output = ((in *  in * in) * 118 * expo) + in * 118 * (1 - expo);
    return (int)std::round(output);
}

IMotion::MotorVoltages OpControlMotion::calculate(const kinState& state) {
	int power = pros::c::controller_get_analog(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_ANALOG_LEFT_Y);
	int turn = pros::c::controller_get_analog(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_ANALOG_RIGHT_X);

	double a = 0;
	double b = 1;

	// sign because when turn is 0, then it will multiply everything by 0
	// double curved_turn = ::sign(turn) * util::clamp(0.0, 127.0, (a + (b - a) * (std::pow(fabs(turn / 127.0), 2))) * 127.0);

	// THIS IS ROMAN SPECIFIC SCALING
	double curved_turn = dampen(turn);
	curved_turn += 9 * sign(curved_turn);


	double left = (power + curved_turn) / 127.0 * 12000;
	double right = (power - curved_turn) / 127.0 * 12000;

	return {left, right};
}

bool OpControlMotion::isSettled(const kinState& state) {
	return false;
}