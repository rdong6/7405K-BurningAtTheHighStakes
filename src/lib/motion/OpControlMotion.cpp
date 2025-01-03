#include "lib/motion/OpControlMotion.h"
#include "lib/utils/Math.h"
#include "pros/misc.h"
#include <cmath>

namespace {
	int sign(double val) {
		return (val > 0) - (val < 0);
	}
}// namespace

IMotion::MotorVoltages OpControlMotion::calculate(const kinState state) {
	// Gets analog inputs from controller (analog inputs means that the more the joystick if pushed, the larger the
	// input we get)
	int power = pros::c::controller_get_analog(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_ANALOG_LEFT_Y);
	int turn = pros::c::controller_get_analog(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_ANALOG_RIGHT_X);

	// Calculates the difference in power required in the motors to achieve the desired turn requested by the controller
	// This part makes it a curved line, instead of linear relationship between controller input and output
	// this makes it so at lower input values, driver has more control over motor power, but eventually reaches max
	// power - just gives more control
	double a = 0;
	double b = 1;

	// sign because when turn is 0, then it will multiply everything by 0
	double curved_turn = ::sign(turn) * util::clamp(0.0, 127.0, (a + (b - a) * (std::pow(fabs(turn / 127.0), 2))) * 127.0);

	// Combines the turn values with the motor powers
	// converts from +/-127 into mV as rest of the drive class works with mV

	// double left = (power + curved_turn) / 127.0 * 12000;
	// double right = (power - curved_turn) / 127.0 * 12000;

	double left = (power + turn) / 127.0 * 12000;
	double right = (power - turn) / 127.0 * 12000;

	return {left, right};
}

bool OpControlMotion::isSettled(const kinState state) {
	return false;
}