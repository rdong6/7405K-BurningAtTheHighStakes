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
	constexpr int backLeftMotor = 20;
	constexpr int weakLeftMotor = 0;

	constexpr int frontRightMotor = 7;
	constexpr int middleRightMotor = 12;
	constexpr int backRightMotor = -13;
	constexpr int weakRightMotor = 0;

	constexpr std::initializer_list<std::int8_t> intake = {-6};
	constexpr int intakeDistance = 15;

	constexpr int liftMotor = 14;
	constexpr int liftRotation = 21;

	constexpr int leftRotation = 2;
	constexpr int rightRotation = -10;

	constexpr int backRotation = -5;
	constexpr int imu = 19;
}// namespace ports

namespace odometers {
	constexpr double driveGearRatio = 3.25 / 60.0 * 36;// diameter + gear ratio included

	// distance between left and right wheels of drivetrain/odometry wheels
	constexpr double trackWidth = 10.4714285483;

	constexpr double leftDeadwheelDiameter = 2.75;
	constexpr double rightDeadwheelDiameter = 2.75;
	constexpr double backDeadwheelDiameter = 2.75;// ignored if backRotation is 0
	constexpr double backOffset = 2.98188;// ignored if backRotation is 0 (offset of back deadwheel to center of rotation)
}// namespace odometers