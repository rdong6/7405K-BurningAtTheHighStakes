#include "subsystems/Odometry.h"
#include "Constants.h"
#include "RobotBase.h"
#include "lib/geometry/Pose.h"
#include "lib/geometry/kinState.h"
#include "lib/utils/Math.h"
#include "subsystems/Drive.h"
#include <cmath>
#include <cstdio>
#include <math.h>
#include <string>
#include <type_traits>

// Constructor sets up the ports of the devices, and initializes state of devices + members of the class
Odometry::Odometry(RobotBase* robot) : Subsystem(robot) {
	backWheel.reset_position();
	backWheel.set_data_rate(5);
	leftWheel.reset_position();
	leftWheel.set_data_rate(5);
	rightWheel.reset_position();
	rightWheel.set_data_rate(5);
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

	prevRotation = imu.get_rotation();

	while (true) {
		double l_dist;
		double r_dist;
		double b_dist;
		double perp_offset;

		// if rotation sensor port is defined, then we use that to get dist traveled
		// otherwise we use motor encoders

		// TODO: clean up code - lots of repeat within the compile time selections
		if constexpr (ports::leftRotation != 0) {
			double LE = leftWheel.get_position();
			l_dist = ((LE - prev_l) / 36000.0) * M_PI * odometers::leftDeadwheelDiameter;
			prev_l = LE;
		} else {
			double LE = drive ? drive.value()->getLeftPosition() : 0;
			l_dist = ((LE - prev_l) / 360.0) * M_PI * odometers::leftDeadwheelDiameter;
			prev_l = LE;
		}

		if constexpr (ports::rightRotation != 0) {
			double RE = rightWheel.get_position();
			r_dist = ((RE - prev_r) / 36000.0) * M_PI * odometers::rightDeadwheelDiameter;
			prev_r = RE;
		} else {
			double RE = drive ? drive.value()->getRightPosition() : 0;
			r_dist = ((RE - prev_r) / 360.0) * M_PI * odometers::rightDeadwheelDiameter;
			prev_r = RE;
		}

		double dh = 0;
		if constexpr (ports::imu > 0) {
			double curRotation = -imu.get_rotation();
			dh = util::toRad(curRotation - prevRotation);
			prevRotation = curRotation;
		} else {
			dh = (l_dist - r_dist) / (odometers::trackWidth);
		}

		if constexpr (ports::backRotation != 0) {
			double BE = backWheel.get_position();
			pros::lcd::print(3, "BE: %f", BE / 36000.0 * M_PI * odometers::backDeadwheelDiameter);
			b_dist = ((BE - prev_b) / 36000.0) * M_PI * odometers::backDeadwheelDiameter;
			prev_b = BE;

			perp_offset = b_dist + (odometers::backOffset * dh);// need to test this
		} else {
			perp_offset = 0;
		}


		// testing pose exponential
		double deltaX = (l_dist + r_dist) / 2.0;// figure out what this should be
		double deltaY = b_dist;

		Rotation2D newHeading = curState.position.rotation() + Rotation2D(dh);
		Pose newPose = curState.position.exp(Twist2D{deltaX, perp_offset, dh});
		curState.position = Pose(newPose.translation(), newHeading);

		// original odom calculation (euler integration)
		// calculate our pos & heading
		// double distance =
		//         (dh == 0) ? (l_dist + r_dist) / 2.0 : ((l_dist + r_dist) / dh) * sin(dh / 2.0);// also need to test
		//         this

		// double theta = curState.position.theta();

		// double dx = distance * cos(theta - (M_PI / 2)) + perp_offset * sin(theta + (M_PI / 2));
		// double dy = distance * sin(theta + (M_PI / 2)) + perp_offset * cos(theta + (M_PI / 2));
		// double dt = 10.0 / 1000.0;


		// update odom's state w/ new calculations
		// curState.setAcceleration((curState.velocity().x - (dx / dt)) / dt, (curState.velocity().y - (dy / dt)) / dt,
		//                          (curState.velocity().theta - (dh / dt)) / dt);
		// curState.setVelocity(dx / dt, dy / dt, dh / dt);

		// curState.position = {curState.position.X() + dx, curState.position.X() + dy,
		//                      util::normalize(curState.position.theta() + dh, 2 * std::numbers::pi)};

		// leftVel = l_dist / dt;
		// rightVel = r_dist / dt;

		// maybe move this to separate thread
		printOdom(curState);

		// to tune the trackwidth - the data being printed to line 6 is not needed
		// pros::lcd::print(4, "LE: %f", sDrive.getLeftPosition() / 360.0 * M_PI * odometers::leftDeadwheelDiameter);
		// pros::lcd::print(5, "RE: %f", sDrive.getRightPosition() / 360.0 * M_PI * odometers::rightDeadwheelDiameter);
		// pros::lcd::print(6, "h: %f",
		//                  ((sDrive.getLeftPosition() / 360.0 * M_PI * odometers::leftDeadwheelDiameter) -
		//                   (sDrive.getRightPosition() / 360.0 * M_PI * odometers::rightDeadwheelDiameter)) /
		//                          10.4714285483 / M_PI * 180);

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
	// logger->debug("X: {:2f} Y: {:2f} H: {:2f}\n", state.position.X(), state.position.X(),
	//               util::toDeg(state.position.theta()));
}

kinState Odometry::getCurrentState() const {
	return curState;
}