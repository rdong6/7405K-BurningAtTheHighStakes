#include "subsystems/Odometry.h"
#include "Constants.h"
#include "RobotBase.h"
#include "lib/geometry/Pose.h"
#include "lib/geometry/kinState.h"
#include "lib/utils/Math.h"
#include "subsystems/Drive.h"
#include <cmath>
#include <string>

// Checking if IMU disconnects
#include "v5JumpTable.h"

// archaic code to debug something with IMU
namespace {
	int32_t getDeviceTimestamp(uint8_t port) {
		V5_DeviceT device = fnVexDeviceGetByIndex(port - 1);
		if (device == nullptr) { return std::numeric_limits<int32_t>::max(); }

		if (!port_mutex_take(port - 1)) { return std::numeric_limits<int32_t>::max(); }

		int32_t timestamp = fnVexDeviceGetTimestamp(device);
		return_port(port - 1);
		return timestamp;
	}
}// namespace

// Constructor sets up the ports of the devices, and initializes state of devices + members of the class
Odometry::Odometry(RobotBase* robot) : Subsystem(robot) {
	backWheel.reset_position();
	backWheel.set_data_rate(5);
	leftWheel.reset_position();
	leftWheel.set_data_rate(5);
	rightWheel.reset_position();
	rightWheel.set_data_rate(5);
	verticalWheel.reset_position();
	verticalWheel.set_data_rate(5);
	imu.set_data_rate(5);
	imu.reset(true);
}

void Odometry::registerTasks() {
	robot->registerTask([this]() { return this->updatePosition(); }, TaskType::SENTINEL);
}

void Odometry::reset() {
	curState = kinState({0, 0, 0}, {0, 0, 0}, {0, 0, 0});
}

void Odometry::setPose(Pose pose) {
	curState.position = pose;
}

RobotThread Odometry::updatePosition() {
	auto drive = robot->getSubsystem<Drive>();

	prevRotation = imu.get_rotation() * odometers::imuScalar;

	while (true) {
		double l_dist;
		double r_dist;
		double b_dist;
		double v_dist;
		double deltaY;
		double deltaX;
		double dh;

		// if rotation sensor port is defined, then we use that to get dist traveled
		// otherwise we use motor encoders

		if constexpr (ports::leftRotation != 0 && ports::rightRotation != 0) {
			// use right & left deadwheels to calculate deltaX
			double LE = leftWheel.get_position();
			l_dist = ((LE - prev_l) / 36000.0) * M_PI * odometers::leftDeadwheelDiameter;
			prev_l = LE;

			double RE = rightWheel.get_position();
			r_dist = ((RE - prev_r) / 36000.0) * M_PI * odometers::rightDeadwheelDiameter;
			prev_r = RE;

			deltaX = (l_dist + r_dist) / 2.0;
		} else if constexpr (ports::verticalRotation != 0) {
			// use 1 vertical deadwheel to calculate deltaX
			double VE = verticalWheel.get_position();
			pros::lcd::print(4, "VE: %f", VE / 36000.0 * M_PI * odometers::verticalDeadwheelDiameter);
			v_dist = ((VE - prev_v) / 36000.0) * M_PI * odometers::verticalDeadwheelDiameter;
			prev_v = VE;
		} else {
			// use motor encoders
			double LE = drive ? drive.value()->getLeftPosition() : 0;
			l_dist = ((LE - prev_l) / 360.0) * M_PI * odometers::driveGearRatio;
			prev_l = LE;

			double RE = drive ? drive.value()->getRightPosition() : 0;
			r_dist = ((RE - prev_r) / 360.0) * M_PI * odometers::driveGearRatio;
			prev_r = RE;

			deltaX = (l_dist + r_dist) / 2.0;
		}


		if constexpr (ports::imu > 0) {
			// negative as CCW is positive, while for VEX CCW is negative
			double curRotation = -imu.get_rotation() * odometers::imuScalar;
			pros::lcd::print(5, "Unscaled IMU: %f", imu.get_rotation());
			dh = util::toRad(curRotation - prevRotation);
			prevRotation = curRotation;
		} else {
			double trackWidth;// selects between track width of drivetrain or deadwheels
			if constexpr (ports::rightRotation != 0) {
				trackWidth = odometers::deadwheelTrackWidth;
			} else {
				trackWidth = odometers::trackWidth;// trackwidth of drivetrain
			}

			dh = (l_dist - r_dist) / (trackWidth);
		}

		if constexpr (ports::leftRotation == 0 && ports::rightRotation == 0 && ports::verticalRotation != 0) {
			deltaX = v_dist + (odometers::verticalOffset * dh);
		}

		if constexpr (ports::backRotation != 0) {
			double BE = backWheel.get_position();
			pros::lcd::print(3, "BE: %f", BE / 36000.0 * M_PI * odometers::backDeadwheelDiameter);
			b_dist = ((BE - prev_b) / 36000.0) * M_PI * odometers::backDeadwheelDiameter;
			prev_b = BE;

			deltaY = b_dist + (odometers::backOffset * dh);
		} else {
			deltaY = 0;
		}

		// calc new position
		Pose newPose = curState.position.exp(Twist2D{deltaX, deltaY, dh});
		curState.position = Pose(newPose.translation(), curState.position.rotation() + Rotation2D(dh));

		printOdom(curState);

		co_yield util::coroutine::nextCycle();
	}
}

double Odometry::getRotation() const {
	return imu.get_rotation();
}

double Odometry::getLeftVel() const {
	return leftVel;
}

double Odometry::getLeftPos() const {
	return curL;
}

double Odometry::getRightVel() const {
	return rightVel;
}

double Odometry::getRightPos() const {
	return curR;
}

void Odometry::printOdom(kinState state) {
	pros::lcd::set_text(0, "gH: " + std::to_string((180 / M_PI) * state.position.theta()));
	pros::lcd::set_text(1, "gX: " + std::to_string(state.position.X()));
	pros::lcd::set_text(2, "gY: " + std::to_string(state.position.Y()));
}

const kinState& Odometry::getCurrentState() const {
	return curState;
}