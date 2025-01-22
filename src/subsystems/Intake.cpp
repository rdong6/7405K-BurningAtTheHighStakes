#include "subsystems/Intake.h"
#include "Logger.h"
#include "RobotBase.h"
#include "lib/utils/CoroutineGenerator.h"
#include "lib/utils/Timeout.h"
#include "pros/motors.h"
#include "subsystems/Controller.h"
#include <cmath>

Intake::Intake(RobotBase* robot) : Subsystem(robot) {
	// sets to coast to avoid motor burnout
	motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

	// sets the proper cart of the motor incase you ever want to use moveVel
	motors.set_gearing_all(pros::E_MOTOR_GEAR_600);
	motors.set_encoder_units_all(pros::E_MOTOR_ENCODER_ROTATIONS);

	color.set_led_pwm(100);
}

void Intake::registerTasks() {
	robot->registerTask([this]() { return this->runner(); }, TaskType::SENTINEL);

	// TODO!!: DETERMINE ORDERING (IF BLUEISM CODE WOULD MESS UP ANTI-JAM CODE)
	robot->registerTask([this]() { return this->antiJamCoro(); }, TaskType::AUTON,
	                    [robot = this->robot]() { return robot->getFlag<Intake>().value()->antiJam; });
	robot->registerTask([this]() { return this->antiJamCoro(); }, TaskType::OPCTRL,
	                    [robot = this->robot]() { return robot->getFlag<Intake>().value()->antiJam; });
	robot->registerTask([this]() { return this->blueismCoro(); }, TaskType::AUTON);
	robot->registerTask([this]() { return this->ladyBrownClearanceCoro(); }, TaskType::OPCTRL,
	                    [robot = this->robot]() { return robot->getFlag<Intake>().value()->ladyBrownClearanceEnabled; });

	auto controller = robot->getSubsystem<Controller>();
	if (!controller) { return; }

	auto controllerRef = controller.value();

	// Maps holding r1 to intake
	controllerRef->registerCallback([this]() { this->moveVoltage(12000); }, []() {}, Controller::master, Controller::r2,
	                                Controller::hold);

	// Maps holding r2 to outtake - when both r1 and r2 are held, r2 takes precedent
	controllerRef->registerCallback([this]() { this->moveVoltage(-12000); }, []() {}, Controller::master, Controller::r1,
	                                Controller::hold);

	// stops the intake when neither button is pressed
	controllerRef->registerCallback([this]() { this->moveVoltage(0); }, []() {}, Controller::master, Controller::r1,
	                                Controller::falling);

	controllerRef->registerCallback([this]() { this->moveVoltage(0); }, []() {}, Controller::master, Controller::r2,
	                                Controller::falling);

	controllerRef->registerCallback([this]() { this->toggleExtender(); }, []() {}, Controller::master, Controller::up,
	                                Controller::falling);
}

/*
Coroutine which runs blueism code.

Thread Type: AUTON
*/
RobotThread Intake::blueismCoro() {
	// TODO: Determine if this should be done here or elsewhere in the program
	switch (robot->curAlliance) {
		case Alliance::BLUE:
			blueismDetector = &Intake::redRingDetector;
			break;
		case Alliance::RED:
			blueismDetector = &Intake::blueRingDetector;
			break;
		default:
			blueismDetector = nullptr;
			break;
	}

	// suspend coroutine until we want blueism enabled (blueismDetector points to either red or blue detector func)
	co_yield [this]() -> bool { return this->blueismDetector; };

	auto intakeFlags = robot->getFlag<Intake>().value();

	double motorStartPosition = 0.0;

	while (true) {
		// Checks if ring of opposite alliance is in intake. Check dist to elim false positives
		if ((this->*blueismDetector)() && color.get_proximity() >= 80) {
			// Determines how long after seeing the ring do we stop the intake*/

			motorStartPosition = motors.get_position();

			co_yield [&]() -> bool {
				return std::fabs(motors.get_position() - motorStartPosition) < 1250.0; /* TUNE THIS */
			};

			// co_yield util::coroutine::delay(40 /* TUNE THIS */);
		}

		// Now code takes over intake and stops it to eject ring
		codeOverride = true;
		motors.brake();

		// determines how long we stop intake for before resuming any normal operations
		co_yield util::coroutine::delay(200 /* TUNE THIS */);

		// returns control to intake and if requested, move intake to last commanded voltage before we blueism
		codeOverride = false;
		if (intakeFlags->colorSortResumes) {
			// determines how long after we ejected ring do we resume intake's last voltage
			co_yield util::coroutine::delay(100 /* TUNE THIS */);
			moveVoltage(lastCommandedVoltage);
		}


		co_yield [this]() -> bool { return this->blueismDetector; };
	}
}

RobotThread Intake::ladyBrownClearanceCoro() {
	while (true) {
		codeOverride = true;
		motors.move_voltage(4000);
		co_yield util::coroutine::delay(60);
		robot->getFlag<Intake>().value()->ladyBrownClearanceEnabled = false;
		motors.brake();
		codeOverride = false;
		co_yield [robot = this->robot]() { return robot->getFlag<Intake>().value()->ladyBrownClearanceEnabled; };
	}
}

/*
Coroutine that handles antijam logic for the intake.

Initial Predicate: Needs Intake::flags::antiJam == true before coro ever runs
Type: OPCTRL & AUTON (NOT SENTINEL!)
*/
RobotThread Intake::antiJamCoro() {
	while (true) {
		// do antijam code here
		co_yield [robot = this->robot]() { return robot->getFlag<Intake>().value()->antiJam; };
	}
}

// As of now, running constantly on robot no matter the competition state
RobotThread Intake::runner() {
	auto intakeFlags = robot->getFlag<Intake>().value();
	// anti-jam vars
	unsigned int timestamp = 0;// timestamp since last time intake was moving
	bool runAntiJam = false;
	Timeout antiJamTimeout;

	// lady-brown mech detector
	bool enableLadyBrownDetector = false;

	while (true) {
		// antijam code
		// keep updating timestamp since last time intake motor was moving
		/*if (std::abs(motors.get_actual_velocity()) >= 30) { timestamp = pros::millis(); }

		// temp disables antijam
		if (intakeFlags->antiJam && state == AntiJamState::IDLE && pros::millis() - timestamp > 250) {
		    // enable antijam code -> unwind the intake rq
		    state = AntiJamState::UNWIND;
		    antiJamTimeout = Timeout(333);
		    printf("\n\n\nANTI JAM ENABLED\n\n\n");
		}

		if (state == AntiJamState::UNWIND) {
		    motors.move_voltage(-12000);

		    if (antiJamTimeout.timedOut()) {
		        motors.move_voltage(0);
		        state = AntiJamState::IDLE;
		    }
		}*/

		// end of antijam code


		// lady brown loaded detector
		// detect ring exists
		// then for a bit, actively check if
		if (blueRingDetector() || redRingDetector()) {
			// enable loader detection
			enableLadyBrownDetector = true;
		}

		if (enableLadyBrownDetector) {
			// check the torque -> if motor doesn't move
			// torque counter
		}


		// dist & torque stop
		int32_t dist = distance.get();// in mm

		// printf("Torque: %f  Vel: %f\n", motors.get_torque(), motors.get_actual_velocity());
		if (intakeFlags->torqueStop &&
		    (motors.get_torque() > 0.97 && fabs(motors.get_actual_velocity()) >= 20) /*tune later*/) {
			printf("Stopping due to torque\n\n\n\n\n\n\n\n");
			this->brake();
			this->setTorqueStop(false);
		}

		if (intakeFlags->distStop) {
			if (dist <= 20) {
				intakeFlags->partiallyIn = false;
				intakeFlags->fullyIn = true;
			} else if (dist <= 60) {
				if (intakeFlags->storeSecond) { intakeFlags->storeSecond = false; }
				intakeFlags->fullyIn = false;
				intakeFlags->partiallyIn = true;
			} else {
				if (intakeFlags->storeSecond) { intakeFlags->storeSecond = false; }
				intakeFlags->partiallyIn = false;
				intakeFlags->fullyIn = false;
			}

			if (!intakeFlags->storeSecond && intakeFlags->fullyIn) {
				this->brake();
				this->setDistStop(false);
			}
		}


		// anti-jam code when intaking

		co_yield util::coroutine::nextCycle();
	}
}

bool Intake::redRingDetector() {
	return color.get_hue() < 10;
}

bool Intake::blueRingDetector() {
	return color.get_hue() >= 75;
}

void Intake::setTorqueStop(bool val) {
	auto intakeFlags = robot->getFlag<Intake>().value();
	intakeFlags->torqueStop = val;
}

void Intake::setDistStop(bool val) {
	auto intakeFlags = robot->getFlag<Intake>().value();
	intakeFlags->distStop = val;
	int32_t dist = distance.get();
	if (dist < 55) { intakeFlags->storeSecond = true; }
}

void Intake::moveVoltage(int mv) {
	if (codeOverride) { return; }
	if (state != AntiJamState::IDLE) { return; }
	robot->getFlag<Intake>().value()->isMoving = (mv != 0 ? true : false);
	lastCommandedVoltage = mv;
	motors.move_voltage(mv);
}

void Intake::moveVel(int vel) {
	if (codeOverride) { return; }
	if (state != AntiJamState::IDLE) { return; }
	robot->getFlag<Intake>().value()->isMoving = (vel != 0 ? true : false);
	motors.move_velocity(vel);
}

void Intake::brake() {
	robot->getFlag<Intake>().value()->isMoving = false;
	motors.brake();
}

void Intake::setExtender(bool val) {
	extenderEnabled = val;
	extender.set_value(val);
}

void Intake::toggleExtender() {
	extenderEnabled = !extenderEnabled;
	extender.set_value(extenderEnabled);
}