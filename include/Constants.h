#pragma once
#include <cstdint>
#include <initializer_list>
#include <numbers>


// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// numbers indicate ports, negative numbers indicate reversed devices
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


namespace ports {
	constexpr int frontLeftMotor = -7;
	constexpr int middleLeftMotor = -3;
	constexpr int backLeftMotor = -12;
	constexpr int weakLeftMotor = 0;

	constexpr int frontRightMotor = 10;
	constexpr int middleRightMotor = 2;
	constexpr int backRightMotor = 11;
	constexpr int weakRightMotor = 0;

	constexpr std::initializer_list<std::int8_t> intake = {-17}; 
	constexpr int intakeDistance = 9;// for dist stop
	constexpr int intakeColor = 19;
	constexpr int blueismDistance = 20;

	constexpr int clampDistance = 0;

	constexpr int liftMotor = -8;
	constexpr int liftRotation = 18;

	constexpr int leftRotation = 0;
	constexpr int rightRotation = 0;
	constexpr int backRotation = 5;
	constexpr int verticalRotation = -6;// this will only ever be used if we have 1 vertical deadwheel on the bot
	constexpr int imu = 1;
}// namespace ports

namespace odometers {
	constexpr double driveGearRatio = 2.75;// wheel diameter + gear ratio included

	constexpr double trackWidth = 11.75;// distance between left and right wheels of drivetrain
	constexpr double deadwheelTrackWidth =
	        0.0;// dist between left & right deadwheels -> only used if deadwheels used to calc heading

	constexpr double leftDeadwheelDiameter = 2.75;
	constexpr double rightDeadwheelDiameter = 2.75;
	constexpr double backDeadwheelDiameter = 2.75;// ignored if backRotation is 0
	constexpr double verticalDeadwheelDiameter = 2; // used only if vertical deadwheel

	constexpr double backOffset = 2.35183757807;// ignored if backRotation is 0 (offset of back deadwheel to center of rotation)
	constexpr double verticalOffset = 0.107209776566;// only used if we have 1 vertical deadwheel (no left & right deadwheels)

	// constexpr double backOffset = 2.30037430088;// ignored if backRotation is 0 (offset of back deadwheel to center of
	// rotation) constexpr double verticalOffset = 0.0380916051888;// only used if we have 1 vertical deadwheel (no left & right
	// deadwheels)
}// namespace odometers