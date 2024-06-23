#pragma once

#include "Logger.h"
#include "lib/geometry/kinState.h"
#include "lib/utils/Mutex.h"
#include "main.h"
#include "pros/imu.hpp"

#define sOdom Odometry::getInstance()

class Odometry {
private:
	LoggerPtr logger = nullptr;

	pros::Rotation leftWheel, rightWheel, backWheel;
	pros::IMU imu;
	pros::IMU backupIMU;
	pros::task_t odom_task;

	double prev_l, prev_r, prev_b;
	double curL, curR;
	double prevRotation;// for IMU only

	bool rollover;

	struct State {
		kinState curr_state{};
		double leftVel = 0, rightVel = 0;
	};

	util::Mutex<State> odomState;

	Odometry();
	Odometry(const Odometry&) = delete;
	Odometry& operator=(const Odometry&) = delete;

	[[noreturn]] void updatePosition();

	void printOdom(kinState state);

public:
	inline static Odometry& getInstance() {
		static Odometry INSTANCE;

		return INSTANCE;
	}

	kinState getCurrentState();
	double getRotation();
	double getLeftVel();
	double getLeftPos();
	double getRightVel();
	double getRightPos();
	void initialize();
	void reset();
	void setPose(Pose pose);
};