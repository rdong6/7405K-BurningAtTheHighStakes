#include "subsystems/Lift.h"
#include "Robot.h"
#include "RobotBase.h"
#include "lib/utils/CoroutineGenerator.h"
#include "lib/utils/Math.h"
#include "pros/motors.h"
#include "subsystems/Controller.h"
#include "subsystems/Subsystem.h"
#include <limits>

#define UPPER_BOUNDS 250
#define LOWER_BOUNDS 1.5

Lift::Lift(RobotBase* robot) : Subsystem(robot) {
	motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	motor.set_gearing(pros::E_MOTOR_GEAR_100);

	// init rotation sensor & starting position
	rotation.set_data_rate(5);
	pros::delay(20);// needed so get_angle() isn't 0

	// handles get_angle() being discontinuous. so when we wrap around, pos becomes like -1 instead of 359
	int32_t curPosition = rotation.get_angle();
	rotation.set_position(curPosition >= 35000 ? curPosition - 36000 : curPosition);
	pros::delay(20);// give time for VEXos to update stuff
}

void Lift::registerTasks() {
	robot->registerTask([this]() { return this->updateAngle(); }, TaskType::SENTINEL);
	robot->registerTask([this]() { return this->runner(); }, TaskType::AUTON);
	robot->registerTask([this]() { return this->runner(); }, TaskType::OPCTRL);

	auto controller = robot->getSubsystem<Controller>();
	if (!controller) { return; }
	auto controllerRef = controller.value();

#ifdef SKILLS
	// in skills, manage lift raising control differently than any other mode
	controllerRef->registerCallback(
	        [this]() {
		        //
	        },
	        []() {}, Controller::master, Controller::l1, Controller::rising);
#else
	// when lift controller inputs set, stop whatever code motion is happening
	controllerRef->registerCallback(
	        [this]() {
		        Lift::State& curState = robot->getFlag<Lift>().value()->state;
		        if (curState == State::LEVEL_1) {
			        // move intake slightly back so it doesn't get caught in ring
			        robot->getFlag<Intake>().value()->ladyBrownClearanceEnabled = true;
		        } else {
			        robot->getFlag<Intake>().value()->ladyBrownClearanceEnabled = false;
		        }

		        setState(State::IDLE);
		        move(12000);
	        },
	        []() {}, Controller::master, Controller::l1, Controller::hold);
#endif

	controllerRef->registerCallback(
	        [this]() {
		        // robot->getFlag<Lift>().value()->state = State::IDLE;
		        setState(State::IDLE);
		        move(-12000);
	        },
	        []() {}, Controller::master, Controller::l2, Controller::hold);

	controllerRef->registerCallback([this]() { move(0); }, []() {}, Controller::master, Controller::l1, Controller::falling);

	controllerRef->registerCallback([this]() { move(0); }, []() {}, Controller::master, Controller::l2, Controller::falling);

	// when roman hits the controller, register the coroutine once to run -> when he releases the button, kill the coroutine
	// maybe change this controller callback during skills
	controllerRef->registerCallback(
	        [this]() {
#ifdef SKILLS
		        robot->registerTask([this]() { return this->skillsWallstakeAutomationStage1(); }, TaskType::OPCTRL);
#else
		        toggleState();
#endif
	        },
	        []() {}, Controller::master, Controller::down, Controller::rising);
}

RobotThread Lift::updateAngle() {
	auto liftFlags = robot->getFlag<Lift>().value();
	int counter = 0;

	while (true) {
		int32_t pos = rotation.get_position();
		double rotationVel = rotation.get_velocity() / 100.0;// deg/s

		// 1:3
		double motorVel = motor.get_actual_velocity();
		// if (liftFlags->isMoving && (std::fabs(motorVel) <= 5 || std::fabs(rotationVel) <= 1 ||
		//                             rotation.get_velocity() == std::numeric_limits<int32_t>::max())) {
		// 	counter++;
		// } else {
		// 	counter = 0;
		// }

		// kill lift if rotation sensor becomes unplugged (data is max) or if rotation's data freezes
		if (pos == std::numeric_limits<int32_t>::max() || counter > 20) {
			liftFlags->kill = true;
			co_return;
		}

		liftFlags->curAngle = pos / 100.0;// converts centideg to deg
		co_yield util::coroutine::nextCycle();
	}
}

RobotThread Lift::runner() {
	auto liftFlags = robot->getFlag<Lift>().value();
	liftFlags->pid = PID(263, 10, 0, true, 10);
	liftFlags->isMoving = true;
	liftFlags->pid.reset();

	// code specific for slewing
	double slewOutput = 0;// slew output = pid's voltage when slew is disabled

	while (true) {
		if (liftFlags->kill) { break; }


		if (liftFlags->state == State::IDLE) {
			liftFlags->isMoving = false;
			motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			move(0);
			co_yield [&]() -> bool { return liftFlags->state != State::IDLE; };
			continue;
		}

		double error = liftFlags->targetAngle - liftFlags->curAngle;
		if (fabs(error) >= liftFlags->errorThresh) {
			liftFlags->isMoving = true;
			double pwr = liftFlags->pid(error);

			// slew or not
			if (liftFlags->slewEnabled) {
				double diff = pwr - slewOutput;

				if (std::abs(diff) <= liftFlags->slewRate) {
					slewOutput = pwr;
				} else {
					slewOutput += util::sign(diff) * liftFlags->slewRate;
				}

			} else {
				slewOutput = pwr;
			}


			move(slewOutput);
			// printf("Running. Error: %f  CurAngle: %f\n", error, liftFlags->curAngle);
		} else {
			liftFlags->isMoving = false;

			if (liftFlags->state == State::STOW) {
				motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
				liftFlags->state = State::IDLE;
			}

			move(0);
			liftFlags->pid.reset();
		}

		co_yield util::coroutine::nextCycle();
	}
}

void Lift::toggleState() {
	switch (robot->getFlag<Lift>().value()->state) {
		case State::LEVEL_1:
			setState(State::STOW);
			break;
		case State::LEVEL_2:
			setState(State::LEVEL_1);
			break;
		case State::STOW:
			setState(State::LEVEL_1);
			break;
		case State::IDLE:
			setState(State::LEVEL_1);
			break;
		case State::HOLD:
			setState(State::LEVEL_1);
			break;
	}
}

void Lift::setState(State state) {
	auto liftFlags = robot->getFlag<Lift>().value();
	liftFlags->state = state;
	liftFlags->errorThresh = 1.5;

	switch (state) {
		case State::LEVEL_1:
			liftFlags->targetAngle = 28;// mmeant to be 26 pre-smudge
			liftFlags->errorThresh = 1;
			motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			break;
		case State::LEVEL_2:
			liftFlags->targetAngle = 50;
			motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			break;
		case State::STOW:
			liftFlags->targetAngle = 2.5;// determine
			motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			break;
		case State::IDLE:
			motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			break;
		case State::HOLD:
			motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			break;
	}
}

void Lift::move(int mv) {
	double curAngle = robot->getFlag<Lift>().value()->curAngle;
	if (mv > 0 && curAngle > UPPER_BOUNDS) { mv = 0; }
	if (mv < 0 && curAngle < LOWER_BOUNDS) { mv = 0; }

	if (mv == 0) {
		motor.brake();
	} else {
		motor.move_voltage(mv);
	}
}