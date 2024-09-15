#pragma once
#include <cstdint>
#include <initializer_list>
#include <numbers>


// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// numbers indicate ports, negative numbers indicate reversed devices
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


// overunder bot

namespace ports {
	constexpr int frontLeftMotor = -1;
	constexpr int middleLeftMotor = -18;
	constexpr int backLeftMotor = 29;
	constexpr int weakLeftMotor = 0;

	constexpr int frontRightMotor = 8;
	constexpr int middleRightMotor = 12;
	constexpr int backRightMotor = -13;
	constexpr int weakRightMotor = 0;

	constexpr std::initializer_list<std::int8_t> intake = {6};
	constexpr int intakeDistance = 0;

	constexpr int liftMotor = 14;
	constexpr int liftRotation = 10;

	constexpr int leftRotation = 0;
	constexpr int rightRotation = 0;

	constexpr int backRotation = 0;
	constexpr int imu = 0;
}// namespace ports

namespace odometers {
	// distance between left and right wheels of drivetrain/odometry wheels
	constexpr double trackWidth = 10.4714285483;


	// constexpr double leftDeadwheelDiameter = 3.25 * (1.0 / 1.375);// 1.375 is gear ratio
	// constexpr double rightDeadwheelDiameter = 3.25 * (1.0 / 1.375);

	constexpr double leftDeadwheelDiameter = 2.75; // 48:84 is gear ratio
	constexpr double rightDeadwheelDiameter = 2.75;// 48:84 is gear ratio
	constexpr double backDeadwheelDiameter = 2.75; // ignored if backRotation is 0
	constexpr double backOffset = 4.252888446;     // ignored if backRotation is 0
}// namespace odometers