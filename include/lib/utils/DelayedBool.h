#pragma once
#include "pros/rtos.h"
#include "pros/rtos.hpp"

namespace util {
	// boolean that is delayed until it returns true
	class DelayedBool {
	public:
		DelayedBool() : time(pros::millis()), timeout(std::numeric_limits<int>::max()) {}
		explicit DelayedBool(int timeout) : time(pros::millis()), timeout(timeout) {}

		bool operator()() { return (pros::millis() - time) >= timeout ? true : false; }

	private:
		int time;
		int timeout;
	};
}// namespace util