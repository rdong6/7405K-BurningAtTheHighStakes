#pragma once
#include <cstdint>
#include <initializer_list>
#include <numbers>


// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// numbers indicate ports, negative numbers indicate reversed devices
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


// overunder bot

namespace ports {
	constexpr int frontLeftMotor = -9;
	constexpr int middleLeftMotor = -6;
	constexpr int backLeftMotor = 7;
	constexpr int weakLeftMotor = 0;

	constexpr int frontRightMotor = 3;
	constexpr int middleRightMotor = 1;
	constexpr int backRightMotor = -2;
	constexpr int weakRightMotor = 0;

	constexpr std::initializer_list<std::int8_t> intake = {11};
	constexpr int intakeDistance = 0;
	constexpr int intakeColor = 20;

	constexpr int clampDistance = 4;

	constexpr int liftMotor = -8;
	constexpr int liftRotation = 5;

	constexpr int leftRotation = 19;
	constexpr int rightRotation = -12;
	constexpr int backRotation = -10;
	constexpr int imu = 18;
}// namespace ports

namespace odometers {
	constexpr double driveGearRatio = 3.25 / 60.0 * 36;// diameter + gear ratio included

	// distance between left and right wheels of drivetrain/odometry wheels
	constexpr double trackWidth = 10.4714285483;

	constexpr double leftDeadwheelDiameter = 2.75;
	constexpr double rightDeadwheelDiameter = 2.75;
	constexpr double backDeadwheelDiameter = 2.75;// ignored if backRotation is 0
	constexpr double backOffset = 1.1353052607;// ignored if backRotation is 0 (offset of back deadwheel to center of rotation)
}// namespace odometers