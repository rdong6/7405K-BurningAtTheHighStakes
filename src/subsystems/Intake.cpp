#include "subsystems/Intake.h"
#include "Logger.h"
#include "RobotBase.h"
#include "lib/utils/CoroutineGenerator.h"
#include "lib/utils/Timeout.h"
#include "pros/motors.h"
#include "subsystems/Controller.h"
#include "subsystems/Lift.h"

#include <cmath>

Intake::Intake(RobotBase* robot) : Subsystem(robot) {
	// sets to coast to avoid motor burnout
	motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

	// sets the proper cart of the motor incase you ever want to use moveVel
	motors.set_gearing_all(pros::E_MOTOR_GEAR_600);
	motors.set_encoder_units_all(pros::E_MOTOR_ENCODER_ROTATIONS);

	color.set_integration_time(20);
	color.set_led_pwm(100);
}

void Intake::registerTasks() {
	robot->registerTask([this]() { return this->runner(); }, TaskType::AUTON);

	// TODO: Stalled detector should run before everything

	// Stall detection coro -> should run before everything else
	robot->registerTask([this]() { return this->stalledDetectorCoro(); }, TaskType::AUTON);
	robot->registerTask([this]() { return this->stalledDetectorCoro(); }, TaskType::OPCTRL);

	// blueism coro -> only runs in auton
	robot->registerTask([this]() { return this->blueismCoro(); }, TaskType::AUTON);

	// antijam
	robot->registerTask([this]() { return this->antiJamCoro(); }, TaskType::AUTON,
	                    [robot = this->robot]() { return robot->getFlag<Intake>().value()->antiJam; });
	robot->registerTask([this]() { return this->antiJamCoro(); }, TaskType::OPCTRL,
	                    [robot = this->robot]() { return robot->getFlag<Intake>().value()->antiJam; });

	// prevent intake from stalling in auton when loading rings into lady brown
	robot->registerTask([this]() { return this->ladyBrownLoadedCoro(); }, TaskType::AUTON);

	// lady brown clearing coro -> so intake doesn't jam the lady brown as we go to score
	robot->registerTask([this]() { return this->ladyBrownClearanceCoro(); }, TaskType::AUTON,
	                    [robot = this->robot]() { return robot->getFlag<Intake>().value()->ladyBrownClearanceEnabled; });
	robot->registerTask([this]() { return this->ladyBrownClearanceCoro(); }, TaskType::OPCTRL,
	                    [robot = this->robot]() { return robot->getFlag<Intake>().value()->ladyBrownClearanceEnabled; });


	// robot->registerTask([this]() { return this->ladyBrownLoadedCoro(); }, TaskType::OPCTRL,
	//                     [robot = this->robot]() {
	// 	                    auto liftFlags = robot->getFlag<Lift>().value();
	// 	                    return liftFlags->state == Lift::LEVEL_1 /*&& liftFlags->isMoving == false &&
	// 	                           robot->getFlag<Intake>().value()->isMoving;*/
	//                     });

	auto controller = robot->getSubsystem<Controller>();
	if (!controller) { return; }

	auto controllerRef = controller.value();

	// Maps holding r2 to outtake - when both r1 and r2 are held, r2 takes precedent
	controllerRef->registerCallback([this]() { this->moveVoltage(-12000); }, []() {}, Controller::master, Controller::r2,
	                                Controller::hold);

	// Maps holding r1 to intake
	controllerRef->registerCallback([this]() { this->moveVoltage(12000); }, []() {}, Controller::master, Controller::r1,
	                                Controller::hold);

	// stops the intake when neither button is pressed
	controllerRef->registerCallback([this]() { this->moveVoltage(0); }, []() {}, Controller::master, Controller::r1,
	                                Controller::falling);

	controllerRef->registerCallback([this]() { this->moveVoltage(0); }, []() {}, Controller::master, Controller::r2,
	                                Controller::falling);
}

// coroutine that runes to detect if intake is stalled
RobotThread Intake::stalledDetectorCoro() {
	unsigned int intakeStalledCounter = 0;
	while (true) {
		// Can't use torque -> check if blueism triggers this
		// need something to check that we command the motor to actually move and it aint
		if (robot->getFlag<Intake>().value()->isMoving && motors.get_torque() >= 0.275 /* TODO: Determine this value*/ &&
		    std::fabs(motors.get_actual_velocity()) <= 10 /*TODO: Determine this threshold*/) {
			intakeStalledCounter++;
		} else {
			intakeStalledCounter = 0;
		}

		if (intakeStalledCounter >= 3) {
			intakeStalled = true;
		} else {
			intakeStalled = false;
		}

		co_yield util::coroutine::nextCycle();
	}
}

/*
Coroutine which runs blueism code.
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

	while (true) {
		if ((this->*blueismDetector)() && color.get_proximity() >= 80) {
			co_yield [&]() -> bool {
				return blueismDistance.get() <= 20; /* TUNE THIS */
			};

			co_yield util::coroutine::delay(50);

			// Now code takes over intake and stops it to eject ring
			motors.brake();
			codeOverride = true;

			// determines how long we stop intake for before resuming any normal operations
			co_yield util::coroutine::delay(250 /* TUNE THIS */);

			// returns control to intake and if requested, move intake to last commanded voltage before we blueism
			codeOverride = false;
			if (intakeFlags->colorSortResumes) { moveVoltage(lastCommandedVoltage); }
		}


		co_yield [this]() -> bool { return this->blueismDetector; };
	}
}

// coro to make sure that when lady brown goes to score from LEVEL_1 state, the ring in the lift doesn't get caught on the
// intake's hook
RobotThread Intake::ladyBrownClearanceCoro() {
	while (true) {
		codeOverride = true;
		motors.move_voltage(-4000);
		co_yield util::coroutine::delay(70);
		robot->getFlag<Intake>().value()->ladyBrownClearanceEnabled = false;
		motors.brake();
		codeOverride = false;
		co_yield [robot = this->robot]() { return robot->getFlag<Intake>().value()->ladyBrownClearanceEnabled; };
	}
}

// Code is written under the assumption that it only ran in autons -> might have some buggy behaviour in opcontrol (or not, my
// brain isn't working rn)
RobotThread Intake::ladyBrownLoadedCoro() {
	// this code should only run if ladybrown is at LEVEL_1, it's in position, and intake is intaking a ring
	// maybe get rid of the liftFlags->isMoving thing?? -> because it might do minor movements
	while (true) {
		co_yield [robot = this->robot]() {
			auto liftFlags = robot->getFlag<Lift>().value();
			return liftFlags->state == Lift::LEVEL_1 /*&& liftFlags->isMoving == false*/ &&
			       robot->getFlag<Intake>().value()->isMoving;
		};

		printf("Running stall detection for lady brown\n");
		if (blueismDistance.get() <= 20) {
			co_yield util::coroutine::delay(10);
			while (!isStalled()) {
				printf("Waiting for motor to stall\n");
				co_yield util::coroutine::delay(10);
			}

			// intake has stalled -> assuming that given lady brown is in position and we saw a ring @ top of intake, ladybrown
			// now has ring in it
			moveVoltage(0);
			robot->getFlag<Intake>().value()->ladyBrownClearanceEnabled = true;
		}
	}
}

/*
Coroutine that handles antijam logic for the intake.

Initial Predicate: Needs Intake::flags::antiJam == true before coro ever runs
Type: OPCTRL & AUTON (NOT SENTINEL!)
*/
RobotThread Intake::antiJamCoro() {
	while (true) {
		// do antijam code here -> stop intake, make it outtake for a bit, then reintake
		co_yield [robot = this->robot]() { return robot->getFlag<Intake>().value()->antiJam; };
	}
}

// TODO: change coro's name as this func is now only dist & torque stop
RobotThread Intake::runner() {
	auto intakeFlags = robot->getFlag<Intake>().value();

	while (true) {
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
			if (dist <= 24) {
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

		co_yield util::coroutine::nextCycle();
	}
}

bool Intake::redRingDetector() {
	return color.get_hue() <= 17.5;
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
	robot->getFlag<Intake>().value()->isMoving = mv != 0;
	lastCommandedVoltage = mv;
	motors.move_voltage(mv);
}

void Intake::moveVel(int vel) {
	if (codeOverride) { return; }
	if (state != AntiJamState::IDLE) { return; }
	robot->getFlag<Intake>().value()->isMoving = vel != 0;
	motors.move_velocity(vel);
}

void Intake::brake() {
	if (codeOverride) { return; }
	robot->getFlag<Intake>().value()->isMoving = false;
	lastCommandedVoltage = 0;
	motors.brake();
}

bool Intake::isStalled() const {
	return intakeStalled;
}