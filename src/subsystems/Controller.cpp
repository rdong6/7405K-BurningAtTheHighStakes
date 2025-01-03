#include "subsystems/Controller.h"
#include "RobotBase.h"
#include "lib/utils/CoroutineGenerator.h"
#include "pros/misc.h"
#include <tuple>

Controller::Controller(RobotBase* robot) : Subsystem(robot) {}

void Controller::registerTasks() {
	robot->registerTask([this]() { return this->update(); }, TaskType::OPCTRL);
}

RobotThread Controller::update() {
	// Initalize the button states to be false
	for (size_t i = Digital::l1; i <= Digital::a; i++) {
		buttonStates[{ID::master, (Digital) i}] = {false, false};
		buttonStates[{ID::partner, (Digital) i}] = {false, false};
	}

	while (true) {
		// poll for the new controller states
		// and call corresponding callbacks when the conditions for a callback is met
		for (const auto& [key, value] : buttonStates) {
			// true is the controller button (key.second) on the controller (key.first) is pressed
			bool state = pros::c::controller_get_digital(static_cast<pros::controller_id_e_t>(key.first),
			                                             static_cast<pros::controller_digital_e_t>(key.second));

			// updates state - now we store new current state, and now the previous state
			buttonStates[key] = {state, value.first};

			// fix it - so inside callCallback, it will call either one - so we don't have to redo it
			if (state) {
				// button currently pressed
				if (value.second) {
					// HOLD - button was previously pressed
					callCallback(key, hold);
				} else {
					// RISING - button wasn't previuosly pressed
					callCallback(key, rising);
					callCallback(key, hold);
				}
			} else {
				// button not currently pressed
				if (value.second) {
					// FALLING - button was previously pressed
					callCallback(key, falling);
				} else {
					// button wasn't pressed in last 2 updates
					// call all the default callbacks associated with this button

					// due to the current way we store the callbacks for the three different callback possibilities
					// we have to do a lookup 3 different times
					// which is kinda slow
					// kinda crappily written code
					callCallback(key, rising, true);
					callCallback(key, hold, true);
					callCallback(key, falling, true);
				}
			}
		}

		co_yield util::coroutine::nextCycle();
	}
}

void Controller::callCallback(const std::pair<ID, Digital>& key, ButtonMode mode, bool callDefault) {
	auto [id, digital] = key;
	std::tuple<ID, Digital, ButtonMode> lookup = std::make_tuple(id, digital, mode);

	auto iter = buttonBinds.find(lookup);
	if (iter != buttonBinds.end()) {
		if (callDefault) {
			std::get<1>(iter->second)();
		} else {
			std::get<0>(iter->second)();
		}
	}
}

void Controller::registerCallback(const DigitalFunc& callback, const DigitalFunc& defaultFunc, ID id, Digital digital,
                                  ButtonMode mode) {
	buttonBinds[std::make_tuple(id, digital, mode)] = {callback, defaultFunc};
}

bool Controller::removeCallback(ID id, Digital digital, ButtonMode mode) {
	return buttonBinds.erase(std::make_tuple(id, digital, mode));
}