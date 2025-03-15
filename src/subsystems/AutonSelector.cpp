#include "subsystems/AutonSelector.h"
#include "autons.h"
#include "lib/utils/Math.h"
#include "pros/llemu.hpp"

AutonSelector::AutonSelector(RobotBase* robot) : Subsystem(robot) {}


void AutonSelector::registerTasks() {
	robot->registerTask([this]() { return this->thread(); }, TaskType::SENTINEL);
}

void AutonSelector::addAuton(const char* name, AutonFn_t autonFunc, bool isQual) {
	autons.emplace_back(name, autonFunc, isQual);
}

double AutonSelector::getPotValue() const {
	return pot.get_value() / 4100.0 * 100;
}

RobotThread AutonSelector::thread() {
	double prevAngle = getPotValue();
	double sum = 0;
	int counter = 0; // counter for how long it's been since we've last rotated it a substantial amount=

	while (true) {
	// this is some of the worst code ever written. but I hate auton selectors so it doesn't deserve anything better

		double curAngle = getPotValue();
		// check if we wrap around -> from 4100 to 0 or 0 to 4100
		// for now, we have a deadzone in place -> if we wrap around, ignore inputs. why? because potentiometer input when this happens is very wack and I haven't found a good way to avoid quickly cycling due to wacky inputs
		if ((curAngle < 20 && prevAngle > 70) || (curAngle > 70 && prevAngle < 20)) {
			counter = 0;
			sum = 0;
			prevAngle = curAngle;
			co_yield util::coroutine::nextCycle();
			continue;
		}

		double deltaAngle = curAngle - prevAngle;
		prevAngle = curAngle;

		// if we start turning in opposite direction, reset sum to 0 real quick
		if (util::sign(deltaAngle) != util::sign(sum)) {
			sum = 0;
		}

		sum += deltaAngle;

		// check if we stop rotating for a bit -> if we do, reset sum
		// this is so we won't rotate knob, then have it slightly move due to a vibration and cause a change in auton selected
		if (std::abs(deltaAngle) < 0.04/*TODO*/) {
			counter++;
		}

		if (counter > 4) {
			counter = 0;
			sum = 0;
		}

		if (std::abs(sum) >= 8) {
			// we've turned a certain amt, now cycle to next auton


			printf("Cycling to next one\n\n");
			if (autons.size() == 0) {
				pros::lcd::print(6, "Auton: NONE");
				continue;
			}

			autonIndex += 1 * util::sign(sum);
			// if we go negative, wrap index around
			if (autonIndex < 0) { autonIndex = autons.size() - 1; }
			if (autonIndex >= autons.size()) { autonIndex = autonIndex - autons.size(); }

			const Auton& selectedAuton = autons[autonIndex];
			robot->isElim = !selectedAuton.isQual;
			robot->autonFnPtr = selectedAuton.autonFunc;
			pros::lcd::print(6, "Auton: %s", selectedAuton.name);

			sum = 0;
		}

		co_yield util::coroutine::nextCycle();
	}
}