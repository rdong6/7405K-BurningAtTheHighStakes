#pragma once
#include "lib/controller/RAMSETE.h"
#include <cstdint>
#include <initializer_list>


// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// numbers indicate ports, negative numbers indicate reversed devices
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


namespace ports {
	constexpr int frontLeftMotor = -4;
	constexpr int middleLeftMotor = -8;
	constexpr int backLeftMotor = -6;
	constexpr int weakLeftMotor = 0;

	constexpr int frontRightMotor = 2;
	constexpr int middleRightMotor = 5;
	constexpr int backRightMotor = 21;
	constexpr int weakRightMotor = 0;

	constexpr std::initializer_list<std::int8_t> intake = {-1};
	constexpr int intakeDistance = 13;// for dist stop
	constexpr int intakeColor = 15;
	constexpr int blueismDistance = 14;

	constexpr int clampDistance = 0;

	constexpr int liftMotor = -11;
	constexpr int liftRotation = 3;

	constexpr int leftRotation = 0;
	constexpr int rightRotation = 0;
	constexpr int backRotation = -10;
	constexpr int verticalRotation = -12;// this will only ever be used if we have 1 vertical deadwheel on the bot
	constexpr int imu = 7;
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

	// constexpr double backOffset = 0;
	// constexpr double verticalOffset = 0;

	constexpr double backOffset = 0.364319505698;
	constexpr double verticalOffset = 2.09360573869;

	// constexpr double backOffset = 1.9812784336;// ignored if backRotation is 0 (offset of back deadwheel to center of
	// rotation)
	// constexpr double verticalOffset = 0.5168626979855;// only used if we have 1 vertical deadwheel (no left & right
	// deadwheels)
}// namespace odometers

namespace trajectory {
	constexpr double maxCentripetalAcceleration = 15;
	constexpr double maxDrivetrainVel = 50;
	constexpr RAMSETE ramsete = RAMSETE(0.02, 0.001);
}// namespace trajectory