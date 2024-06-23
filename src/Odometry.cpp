#include "Odometry.h"
#include "Constants.h"
#include "Drive.h"
#include "lib/geometry/Pose.h"
#include "lib/geometry/kinState.h"
#include "lib/utils/Math.h"
#include "pros/apix.h"
#include "pros/rtos.h"
#include <cmath>
#include <cstdio>
#include <math.h>
#include <string>
#include <type_traits>

// Constructor sets up the ports of the devices, and initializes state of devices + members of the class
Odometry::Odometry()
    : leftWheel(ports::leftRotation), rightWheel(ports::rightRotation), backWheel(-ports::backRotation),
      imu(ports::imu), backupIMU(ports::backupIMU), odom_task(nullptr), prev_b(0), prev_l(0), prev_r(0), curL(0),
      curR(0), prevRotation(0), odomState(), rollover(false) {
	backWheel.reset();
	backWheel.set_data_rate(5);
	leftWheel.reset_position();
	leftWheel.set_data_rate(5);
	rightWheel.reset_position();
	rightWheel.set_data_rate(5);
}

void Odometry::initialize() {
	prev_l = 0;
	prev_b = 0;
	prev_r = 0;
	prevRotation = 0;

	leftWheel.reset_position();
	rightWheel.reset_position();
	backWheel.reset_position();

	// initializes IMUs & calibrates IMUs
	imu.set_data_rate(5);
	backupIMU.set_data_rate(5);
	imu.reset(false);
	backupIMU.reset(true);

	// waits for first one to also calibrate
	// if first becomes unplugged then just run odom
	while (pros::c::registry_get_plugged_type(ports::imu - 1) == pros::c::E_DEVICE_IMU &&
	       static_cast<int>(imu.get_status()) & static_cast<int>(pros::ImuStatus::calibrating)) {
		pros::delay(10);
	}

	pros::delay(20);
	// this may be needed if odom decides to break itself again and this stupid fix is needed
	// curr_state = kinState({0, 0, 0}, {0, 0, 0}, {0, 0, 0});
	logger = sLogger.createSource("Odom");
	logger->toggleLevel(LogSource::DEBUG);

	odom_task = pros::c::task_create([](void* ign) { sOdom.updatePosition(); }, nullptr, TASK_PRIORITY_DEFAULT,
	                                 TASK_STACK_DEPTH_DEFAULT, "Odometry");
}

void Odometry::reset() {
	odomState.wlock([](auto& data) { data.curr_state = kinState({0, 0, 0}, {0, 0, 0}, {0, 0, 0}); });

	// leftWheel.reset();
	// rightWheel.reset();
}

void Odometry::setPose(Pose pose) {
	odomState.wlock([pose](auto& data) { data.curr_state.position = pose; });
}

void Odometry::updatePosition() {
	prevRotation = imu.get_rotation();
	if (std::isnan(prevRotation) || std::isinf(prevRotation)) { prevRotation = backupIMU.get_rotation(); }
	int rolloverIndicator = 0;
	int prevRolloverIndicator = 0;

	while (true) {
		uint32_t time = pros::millis();

		double l_dist;
		double r_dist;
		double b_dist;
		double perp_offset;

		// if rotation sensor port is defined, then we use that to get dist traveled
		// otherwise we use motor encoders

		// TODO: clean up code - lots of repeat within the compile time selections
		if constexpr (ports::leftRotation > 0) {
			double LE = leftWheel.get_position();
			l_dist = ((LE - prev_l) / 36000.0) * M_PI * odometers::leftDeadwheelDiameter;
			prev_l = LE;
		} else {
			double LE = sDrive.getLeftPosition();
			l_dist = ((LE - prev_l) / 360.0) * M_PI * odometers::leftDeadwheelDiameter;
			prev_l = LE;
		}

		if constexpr (ports::rightRotation > 0) {
			double RE = rightWheel.get_position();
			r_dist = ((RE - prev_r) / 36000.0) * M_PI * odometers::rightDeadwheelDiameter;
			prev_r = RE;
		} else {
			double RE = sDrive.getRightPosition();
			r_dist = ((RE - prev_r) / 360.0) * M_PI * odometers::rightDeadwheelDiameter;
			prev_r = RE;
		}

		// TODO: maybe clean this up and get rid of the compile time switch because we are doing a rollover to the motor
		// encoders for heading calc
		double dh = 0;
		if constexpr (ports::imu > 0) {
			if (rollover) {
				rolloverIndicator = 2;
				dh = (l_dist - r_dist) / (odometers::trackWidth);
			} else {
				rolloverIndicator = 0;
				double curRotation = imu.get_rotation();
				dh = util::toRad(curRotation - prevRotation);

				if (std::abs(dh) >= M_PI_4 || std::isnan(dh)) {
					// use backup imu
					rolloverIndicator = 1;
					curRotation = backupIMU.get_rotation();
					dh = util::toRad(curRotation - prevRotation);

					if (std::abs(dh) >= M_PI_4 || std::isnan(dh)) {
						rolloverIndicator = 2;
						dh = (l_dist - r_dist) / (odometers::trackWidth);
						rollover = true;
					}
				}
				prevRotation = curRotation;
			}
			if (prevRolloverIndicator != rolloverIndicator) {
				logger->error("Rolled over. Indicator: {}\n", rolloverIndicator);
			}
			prevRolloverIndicator = rolloverIndicator;
			pros::lcd::print(7, "Rollover Counter: %d", rolloverIndicator);
		} else {
			dh = (l_dist - r_dist) / (odometers::trackWidth);
		}

		if constexpr (ports::backRotation > 0) {
			double BE = backWheel.get_position();
			pros::lcd::print(4, "BE: %f", BE / 36000.0 * M_PI * odometers::backDeadwheelDiameter);
			b_dist = ((BE - prev_b) / 36000.0) * M_PI * odometers::backDeadwheelDiameter;
			prev_b = BE;

			perp_offset = b_dist + (odometers::backOffset * dh);// need to test this
		} else {
			b_dist = 0;
			perp_offset = 0;
		}

		// idk why Kylan put this here but I'm keeping it here for now - basically does not negate at all the movement
		// of the back deadwheel when rotating in place
		// so can delete - just double check that it is a + and not - in calculation of perp offset
		// perp_offset = 0;

		double distance =
		        (dh == 0) ? (l_dist + r_dist) / 2.0 : ((l_dist + r_dist) / dh) * sin(dh / 2.0);// also need to test this

		double theta = odomState.unsafeRead().curr_state.position.getTheta();

		double dx = distance * cos(theta - (M_PI / 2)) + perp_offset * sin(theta + (M_PI / 2));
		double dy = distance * sin(theta + (M_PI / 2)) + perp_offset * cos(theta + (M_PI / 2));
		double dt = 10.0 / 1000.0;


		kinState state;
		{
			auto guard = odomState.wlock();
			guard->curr_state.setAcceleration((guard->curr_state.velocity().x - (dx / dt)) / dt,
			                                  (guard->curr_state.velocity().y - (dy / dt)) / dt,
			                                  (guard->curr_state.velocity().theta - (dh / dt)) / dt);
			guard->curr_state.setVelocity(dx / dt, dy / dt, dh / dt);

			guard->curr_state.position = {
			        guard->curr_state.position.getX() + dx, guard->curr_state.position.getY() + dy,
			        util::normalize(guard->curr_state.position.getTheta() + dh, 2 * std::numbers::pi)};

			guard->leftVel = l_dist / dt;
			guard->rightVel = r_dist / dt;

			state = guard->curr_state;
		}

		// this don't need to be in mutex - or it wastes time that we could've released mutex for other
		// threads
		printOdom(state);

		// to tune the trackwidth - the data being printed to line 6 is not needed
		// pros::lcd::print(4, "LE: %f", sDrive.getLeftPosition() / 360.0 * M_PI * odometers::leftDeadwheelDiameter);
		// pros::lcd::print(5, "RE: %f", sDrive.getRightPosition() / 360.0 * M_PI * odometers::rightDeadwheelDiameter);
		// pros::lcd::print(6, "h: %f",
		//                  ((sDrive.getLeftPosition() / 360.0 * M_PI * odometers::leftDeadwheelDiameter) -
		//                   (sDrive.getRightPosition() / 360.0 * M_PI * odometers::rightDeadwheelDiameter)) /
		//                          10.4714285483 / M_PI * 180);

		pros::c::task_delay_until(&time, 10);
	}
}

double Odometry::getRotation() {
	return imu.get_rotation();
}

double Odometry::getLeftVel() {
	auto guard = odomState.rlock();
	return guard->leftVel;
}

double Odometry::getLeftPos() {
	return curL;
}

double Odometry::getRightVel() {
	auto guard = odomState.rlock();
	return guard->rightVel;
}

double Odometry::getRightPos() {
	return curR;
}

void Odometry::printOdom(kinState state) {
	pros::lcd::set_text(1, "gH: " + std::to_string((180 / M_PI) * state.position.getTheta()));
	pros::lcd::set_text(2, "gX: " + std::to_string(state.position.getX()));
	pros::lcd::set_text(3, "gY: " + std::to_string(state.position.getY()));
	logger->debug("X: {:2f} Y: {:2f} H: {:2f}\n", state.position.getX(), state.position.getY(),
	              util::toDeg(state.position.getTheta()));
}

kinState Odometry::getCurrentState() {
	auto guard = odomState.rlock();
	return guard->curr_state;
}