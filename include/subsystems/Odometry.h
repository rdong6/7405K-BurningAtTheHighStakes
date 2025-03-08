#pragma once
#include "Constants.h"
#include "RobotBase.h"
#include "Subsystem.h"
#include "lib/geometry/kinState.h"
#include "main.h"
#include "pros/imu.hpp"

class Odometry : public Subsystem {
private:
	pros::Rotation leftWheel{ports::leftRotation}, rightWheel{ports::rightRotation}, backWheel{ports::backRotation}, verticalWheel{ports::verticalRotation};
	pros::IMU imu{ports::imu};

	double prev_l = 0, prev_r = 0, prev_b = 0, prev_v = 0;
	double curL = 0, curR = 0;
	double prevRotation = 0;// for IMU only

	kinState curState{};
	double leftVel = 0;
	double rightVel = 0;

	RobotThread updatePosition();

	void printOdom(kinState state);

public:
	struct flags {
	};

	explicit Odometry(RobotBase* robot);

	void registerTasks() override;

	void reset();
	void setPose(Pose pose);

	const kinState& getCurrentState() const;
	double getRotation() const;
	double getLeftVel() const;
	double getLeftPos() const;
	double getRightVel() const;
	double getRightPos() const;
};