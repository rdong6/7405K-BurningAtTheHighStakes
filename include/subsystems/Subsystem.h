#pragma once
#include "RobotBase.h"
#include <arm_neon.h>
#include <cxxabi.h>
#include <stdexcept>

class Subsystem {
protected:
	RobotBase* robot;

public:
	Subsystem() = delete;

	Subsystem(RobotBase* robot) : robot(robot) {
		if (!robot || !dynamic_cast<RobotBase*>(robot)) { throw std::runtime_error("Invalid Instantiation of Subsystem!"); }
	}

	virtual void registerTasks() {}
};