#pragma once
#include "main.h"

class Timeout {
private:
	uint32_t _time;
	uint32_t _timeout;

public:
	Timeout() = default;
	Timeout(uint32_t timeout) {
		_time = pros::c::millis();
		_timeout = timeout;
	}

	bool timedOut() const { return (pros::c::millis() - _time) > _timeout; }
};