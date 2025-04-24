#include "subsystems/Intake.h"
#include "Logger.h"
#include "Robot.h"
#include "RobotBase.h"
#include "lib/utils/CoroutineGenerator.h"
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
	// Stall detection coro -> should run before everything else
	robot->registerTask([this]() { return this->stalledDetectorCoro(); }, TaskType::AUTON);
	robot->registerTask([this]() { return this->stalledDetectorCoro(); }, TaskType::OPCTRL);

	// blueism coro -> only runs in auton
	robot->registerTask([this]() { return this->ringDetectorCoro(); }, TaskType::AUTON);
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

	robot->registerTask([this]() { return this->runner(); }, TaskType::AUTON);

#ifdef SKILLS
	robot->registerTask([this]() { return this->runner(); }, TaskType::OPCTRL);
#endif


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
	controllerRef->registerCallback(
	        [this]() {
		        robot->getFlag<Intake>().value()->ladyBrownClearanceEnabled = true;
		        this->moveVoltage(0);
	        },
	        []() {}, Controller::master, Controller::r1, Controller::falling);

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

		if (intakeStalledCounter >= 25) {
			intakeStalled = true;
		} else {
			intakeStalled = false;
		}

		co_yield util::coroutine::nextCycle();
	}
}

// Detects color of ring passing through intake and above the color sensor
// Pushes ring into queue of rings that are currently in the intake but haven't passed the dist sensor yet
RobotThread Intake::ringDetectorCoro() {
	// TODO: THIS HAS A BUG IN IT! On initialization, as the MA's is populated w/ values from the sensor, it will rise from 0 to
	// whatever ambient value it is this will trigger the sensor to think it's seen a red ring when there is none.

	// whether or not we've currently already seen the current ring to decide if we append to the queue
	bool alreadySeenRing = true;// set to true to maybe prevent the bug mentioned in the TODO
	int counter = 0;

	while (true) {
		// round to int as the decimal don't really matter. math is quicker + no round off errors to cause drift from MA filter
		int hueMA = ringColorMA.update(std::lround(color.get_hue()));
		int proximity = color.get_proximity();
		// printf("Hue: %d\tProximity: %d\n", hueMA, proximity);

		if (!alreadySeenRing && proximity >= 200) {
			if (hueMA <= 15) {
				alreadySeenRing = true;
				ringsSeen.push(Alliance::RED);
				printf("Seen red ring. %d\n", ringsInIntake());
			} else if (hueMA >= 212) {
				alreadySeenRing = true;
				ringsSeen.push(Alliance::BLUE);
				printf("Seen blue ring. %d\n", ringsInIntake());
			}
		} else {
			// already seen ring or no ring is in front of color sensor
			if (25 <= hueMA <= 200 && proximity < 100) { counter++; }

			if (counter > 3) {
				counter = 0;
				alreadySeenRing = false;
			}
		}

		// delay of 15ms to ensure this thread runs every 2 cycles (20ms)
		// if it was 20ms, there's not 100% guarantee that this thread will run every 2 cycles. it might run every 3rd cycle if
		// other code runs quicker and 20ms hasn't been reached till this thread is getting scheduled
		co_yield util::coroutine::delay(15);
	}
}


RobotThread Intake::blueismCoro() {
	auto intakeFlags = robot->getFlag<Intake>().value();


	// TODO: implement a queue -> so we eject the right ring and not the wrong ring
	// Logic happening in parallel
	// 1) Detect color of ring passing through intake -> append it to a queue of rings seen
	// 2) Each time the dist sensor sees a ring, look at queue to see what color the ring is. If it's the color we want to
	// eject, initiate the ejection

	while (true) {
		// wait until we detect a ring w/ dist sensor
		co_yield [this]() -> bool { return !ringsSeen.empty() && blueismDistance.get() <= 50; };

		// if we see a ring, check what color it is and if it's the one we want, eject it
		//
		// we put the check here for if an alliance is set because regardless of if color sort is enabled, we want to continue
		// to remove rings from the queue as we see them
		if (robot->curAlliance != Alliance::INVALID && ringsSeen.front() != robot->curAlliance) {
			co_yield util::coroutine::delay(50);

			// Now code takes over intake and stops it to eject ring
			codeOverride = true;
			motors.brake();
			printf("Color sorting\n");

			// determines how long we stop intake for before resuming any normal operations
			co_yield util::coroutine::delay(250);

			// returns control to intake and if requested, resume intake to last commanded voltage before blueism commenced
			codeOverride = false;
			if (intakeFlags->colorSortResumes) { moveVoltage(lastCommandedVoltage); }
		}

		// remove ring from queue as we've now detected it by dist sensor
		ringsSeen.pop();
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

		motors.move_voltage(0);
		codeOverride = false;
		co_yield [robot = this->robot]() { return robot->getFlag<Intake>().value()->ladyBrownClearanceEnabled; };
	}
}


// Runs in auton. Kills the intake to prevent it stalling indefinitely after loading a ring into the lady brown.
// also clears intake's hook from the lady brown
RobotThread Intake::ladyBrownLoadedCoro() {
	// this code should only run if ladybrown is at LEVEL_1, it's in position, and we're intaking a ring
	// maybe get rid of the liftFlags->isMoving thing?? -> because it might do minor movements when ring is loaded
	while (true) {
		co_yield [robot = this->robot]() {
			auto liftFlags = robot->getFlag<Lift>().value();
			return liftFlags->state == Lift::LEVEL_1 /*&& liftFlags->isMoving == false*/ &&
			       robot->getFlag<Intake>().value()->isMoving;
		};

		// TODO: clean up this logic -> so we don't have duplicate detections or for example, we just detected lady brown was
		// loaded. and in next itteration, we're stalled while waiting for intake to stall
		// which could lead to false positives

		// we detected a ring in our intake that's about to be loaded into lady brown
		// wait until intake stalls -> that indicates that the ring has been loaded into lady brown
		if (blueismDistance.get() <= 20) {
			co_yield util::coroutine::nextCycle();
			while (!isStalled()) { co_yield util::coroutine::nextCycle(); }

			// intake has stalled -> assuming that given lady brown is in position and we saw a ring @ top of intake, lady brown
			// now has ring in it
			moveVoltage(0);
			printf("Lady brown has been loaded\n");
			// automatically retract intake hook slightly -> so we don't have to manually do it in autons
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
#ifdef SKILLS
				codeOverride = true;
#endif
			}
		}

		co_yield util::coroutine::nextCycle();
	}
}

void Intake::setTorqueStop(bool val) {
	auto intakeFlags = robot->getFlag<Intake>().value();
	intakeFlags->torqueStop = val;
}

void Intake::setDistStop(bool val) {
	auto intakeFlags = robot->getFlag<Intake>().value();

#ifdef SKILLS
	if (!val) { codeOverride = false; }
#endif

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

void Intake::setExtender(bool extended) {
	isExtended = extended;
	extender.set_value(isExtended);
}

void Intake::toggleExtender() {
	setExtender(!isExtended);
}

bool Intake::isStalled() const {
	return intakeStalled;
}
int Intake::ringsInIntake() const {
	return ringsSeen.size();
}