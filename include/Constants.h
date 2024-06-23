#pragma once
#include <cstdint>
#include <initializer_list>
#include <numbers>

// overunder bot
namespace ports {
	// K Ports
	constexpr int frontLeftMotor = 1;
	constexpr int middleLeftMotor = 11;
	constexpr int backLeftMotor = 13;
	constexpr int weakLeftMotor = 12;

	constexpr int frontRightMotor = 6;
	constexpr int middleRightMotor = 20;
	constexpr int backRightMotor = 18;
	constexpr int weakRightMotor = 19;

	// numbers indicate ports, negative numbers indicate reversed motors
	// this has become the puncher motors

	// K Bot
	constexpr std::initializer_list<std::int8_t> intake = {10};

	constexpr int leftRotation = 0;
	constexpr int rightRotation = 0;

	// K Bot
	constexpr int backRotation = 14;
	constexpr int imu = 7;

	constexpr int backupIMU = 0;
}// namespace ports

namespace odometers {
	// distance between left and right wheels of drivetrain/odometry wheels
	constexpr double trackWidth = 10.4714285483;

	// 11.2571429207

	// constexpr double leftDeadwheelDiameter = 3.25 * (1.0 / 1.375);// 1.375 is gear ratio
	// constexpr double rightDeadwheelDiameter = 3.25 * (1.0 / 1.375);

	constexpr double leftDeadwheelDiameter = 2.75; // 48:84 is gear ratio
	constexpr double rightDeadwheelDiameter = 2.75;// 48:84 is gear ratio
	constexpr double backDeadwheelDiameter = 2.75; // ignored if backRotation is 0
	constexpr double backOffset = 4.252888446;     // ignored if backRotation is 0
}// namespace odometers