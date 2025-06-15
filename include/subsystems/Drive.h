#pragma once
#include "Constants.h"
#include "Logger.h"
#include "Odometry.h"
#include "RobotBase.h"
#include "Subsystem.h"
#include "lib/motion/Motion.h"
#include "lib/motion/NullMotion.h"
#include "lib/utils/Mutex.h"
#include "lib/utils/Timeout.h"
#include "lib/utils/todo.h"
#include "main.h"
#include "pros/adi.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <cstddef>
#include <memory>

class Drive : public Subsystem {
private:
	// Motors
	pros::MotorGroup leftDrive{ports::middleLeftMotor, ports::frontLeftMotor, ports::backLeftMotor};
	pros::Motor weakLeft{ports::weakLeftMotor};
	pros::MotorGroup rightDrive{ports::middleRightMotor, ports::frontRightMotor, ports::backRightMotor};
	pros::Motor weakRight{ports::weakRightMotor};

	// gear ratio between normal and weak motor -> used to scale velocities accordingly for 5.5w
	double gearRatioWeak2Normal;

	// motion stuff
	Motion curMotion = NullMotion();
	bool isSettled = false;


	RobotThread runner();

	void setVoltageLeft(int voltage);
	void setVoltageRight(int voltage);

	void setVelocityLeft(int velocity);
	void setVelocityRight(int velocity);

public:
	struct flags {};

	explicit Drive(RobotBase* robot);

	void registerTasks() override;

	void setCurrentMotion(Motion motion);

	double getLeftPosition() const;
	double getRightPosition() const;
	void resetPosition();

	void setBrakeMode(pros::motor_brake_mode_e_t mode);
	void setBrakeModeLeft(pros::motor_brake_mode_e_t mode);
	void setBrakeModeRight(pros::motor_brake_mode_e_t mode);
	pros::motor_brake_mode_e_t getBrakeMode() const;

	RobotFunc waitUntilSettled(uint32_t timeout = TIMEOUT_MAX);
	RobotFunc waitUntilDist(double dist) const;
};