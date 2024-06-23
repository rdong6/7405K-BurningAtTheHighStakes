#pragma once

#include "Constants.h"
#include "Logger.h"
#include "Odometry.h"
#include "lib/physics/Motion.h"
#include "lib/physics/NullMotion.h"
#include "lib/utils/Mutex.h"
#include "lib/utils/Timeout.h"
#include "lib/utils/todo.h"
#include "main.h"
#include "pros/adi.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <atomic>
#include <cstddef>
#include <memory>

#define sDrive Drive::getInstance()

class Drive {
private:
	// Motors
	pros::MotorGroup leftDrive{0};
	pros::MotorGroup rightDrive{0};

	// General Stuff
	pros::task_t task = nullptr;
	LoggerPtr logger = nullptr;

	// thread safety
	std::atomic<bool> isSettled = false;
	std::atomic<bool> isTimedOut = false;
	// pros::Mutex currentMotionMutex = pros::Mutex();
	// Motion currentMotion = NullMotion();
	util::Mutex<Motion> currentMotion = util::Mutex<Motion>(NullMotion());

	Drive() = default;
	Drive(const Drive&) = delete;
	Drive& operator=(const Drive&) = delete;

	[[noreturn]] void runner();

	// Motor Control
	void setVoltageLeft(int voltage);
	void setVoltageRight(int voltage);

	void setVelocityLeft(int velocity);
	void setVelocityRight(int velocity);

public:
	inline static Drive& getInstance() {
		static Drive INSTANCE;

		return INSTANCE;
	}

	void initialize();
	void setCurrentMotion(Motion motion);

	double getLeftPosition() const;
	double getRightPosition() const;
	void resetPosition();

	void setBrakeMode(pros::motor_brake_mode_e_t mode);
	void setBrakeModeLeft(pros::motor_brake_mode_e_t mode);
	void setBrakeModeRight(pros::motor_brake_mode_e_t mode);
	pros::motor_brake_mode_e_t getBrakeMode() const;

	/**
	 * @return returns true if settled, false if timed out
	 */
	bool waitUntilSettled(uint32_t timeout = TIMEOUT_MAX);
	void waitUntilDist(double dist);
};