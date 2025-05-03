#include "subsystems/Lift.h"
#include "Robot.h"
#include "RobotBase.h"
#include "lib/utils/CoroutineGenerator.h"
#include "lib/utils/Math.h"
#include "pros/motors.h"
#include "subsystems/Controller.h"
#include "subsystems/Subsystem.h"
#include <limits>

#define UPPER_BOUNDS 300
#define LOWER_BOUNDS 15

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

#ifdef SKILLS
	robot->registerTask([this]() { return this->driverSkillsMacro(); },
	                    TaskType::OPCTRL);// for skills only -> at the start of the run
#endif

	auto controller = robot->getSubsystem<Controller>();
	if (!controller) { return; }
	auto controllerRef = controller.value();

#ifdef SKILLS
	// in skills, manage lift raising control differently than any other mode
	controllerRef->registerCallback(
	        [this]() {
		        auto liftFlags = robot->getFlag<Lift>().value();

		        if (liftFlags->curAngle < 40) {
			        liftIgnoreDriverInputTimeout = Timeout(100);
			        liftFlags->targetAngle = 60;
			        setState(Lift::HOLD);
			        robotInstance->getSubsystem<Intake>().value()->setDistStop(true);
		        } else {
			        robotInstance->getSubsystem<Intake>().value()->setDistStop(false);
		        }
	        },
	        []() {}, Controller::master, Controller::l1, Controller::rising);
#endif

	// when lift controller inputs set, stop whatever code motion is happening
	controllerRef->registerCallback(
	        [this]() {
		        if (!liftIgnoreDriverInputTimeout.timedOut()) { return; }

		        Lift::State& curState = robot->getFlag<Lift>().value()->state;
		        if (curState == State::LEVEL_1) {
			        // move intake slightly back so it doesn't get caught in ring
			        robot->getFlag<Intake>().value()->ladyBrownClearanceEnabled = true;
		        } else {
			        robot->getFlag<Intake>().value()->ladyBrownClearanceEnabled = false;
		        }

#ifdef SKILLS
		        robotInstance->getSubsystem<Intake>().value()->setDistStop(false);
#endif
		        setState(State::IDLE);
		        move(12000);
		        motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	        },
	        []() {}, Controller::master, Controller::l1, Controller::hold);

	controllerRef->registerCallback([this]() { liftDriverDownTimestamp = pros::millis(); }, []() {}, Controller::master,
	                                Controller::l2, Controller::hold);

	controllerRef->registerCallback(
	        [this]() {
#ifdef SKILLS
		        robotInstance->getSubsystem<Intake>().value()->setDistStop(false);
#endif
		        setState(State::IDLE);
		        move(-12000);

		        if (pros::millis() - liftDriverDownTimestamp < 333) { motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); }
	        },
	        []() {}, Controller::master, Controller::l2, Controller::hold);

	controllerRef->registerCallback([this]() { move(0); }, []() {}, Controller::master, Controller::l1, Controller::falling);

	controllerRef->registerCallback([this]() { move(0); }, []() {}, Controller::master, Controller::l2, Controller::falling);


	// when roman hits the controller, register the coroutine once to run -> when he releases the button, kill the coroutine
	// maybe change this controller callback during skills
	controllerRef->registerCallback([this]() { setState(Lift::State::DESCORE); }, []() {}, Controller::master, Controller::a,
	                                Controller::rising);

	// x -> alliance stake position & holds
	controllerRef->registerCallback([this]() { setState(State::ALLIANCE); }, []() {}, Controller::master, Controller::x,
	                                Controller::rising);

	controllerRef->registerCallback(
	        [this]() {
		        auto liftFlags = robot->getFlag<Lift>().value();
		        if (liftFlags->state == State::LEVEL_1) {
			        setState(State::STOW);
		        } else {
			        setState(State::LEVEL_1);
		        }
	        },
	        []() {}, Controller::master, Controller::down, Controller::rising);
}

RobotThread Lift::driverSkillsMacro() {
	auto liftFlags = robot->getFlag<Lift>().value();
	liftFlags->targetAngle = 210;
	setState(Lift::HOLD);
	co_yield util::coroutine::nextCycle();
	Timeout liftTimeout = Timeout(500);
	co_yield [=]() { return !liftFlags->isMoving || liftTimeout.timedOut(); };
	setState(Lift::STOW);
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
	liftFlags->pid = PID(200, 30, 40, true, 10);
	liftFlags->isMoving = true;
	liftFlags->pid.reset();

	// code specific for slewing
	double slewOutput = 0;// slew output = pid's voltage when slew is disabled

	while (true) {
		if (liftFlags->kill) { break; }

		// TODO: clean this up. but State::IDLE just stops subsystem thread from running. doesn't do anything to brake mode
		if (liftFlags->state == State::IDLE) {
			liftFlags->isMoving = false;
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
			// printf("Running. Error: %f  CurAngle: %f Target: %f\n", error, liftFlags->curAngle, liftFlags->targetAngle);
		} else {
			liftFlags->isMoving = false;

			if (liftFlags->state == State::STOW) {
				motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
				liftFlags->state = State::IDLE;
			}

			if (liftFlags->state == State::DESCORE) { liftFlags->state = State::IDLE; }

			move(0);
			liftFlags->pid.reset();
		}

		co_yield util::coroutine::nextCycle();
	}
}

void Lift::setState(State state) {
	auto liftFlags = robot->getFlag<Lift>().value();
	liftFlags->pid = PID(150, 0, 50, true, 10);
	liftFlags->state = state;
	liftFlags->errorThresh = 1.5;

	switch (state) {
		case State::LEVEL_1:
			liftFlags->pid = PID(240, 15, 50, true, 10);
			liftFlags->targetAngle = 42.25;// meant to be 26 pre-smudge
			liftFlags->errorThresh = 0.5;
			motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			break;
		case State::ALLIANCE:
			liftFlags->pid = PID(100, 0, 50, true, 10);
			liftFlags->targetAngle = 173;
			motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			break;
		case State::DESCORE:
			liftFlags->targetAngle = 163;// need to change for world
			motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			break;
		case State::STOW:
			liftFlags->targetAngle = 15;// determine
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