#include "subsystems/Intake.h"
#include "Constants.h"
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

	extender.set_value(false);
	color.set_led_pwm(100);
}

void Intake::registerTasks() {
	robot->registerTask([this]() { return this->runner(); }, TaskType::SENTINEL);

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
}

// As of now, running constantly on robot no matter the competition state
RobotThread Intake::runner() {
	auto intakeFlags = robot->getFlag<Intake>().value();

	unsigned int timestamp = 0;// timestamp since last time intake was moving
	bool runAntiJam = false;
	Timeout antiJamTimeout;

	bool firstOne = false;
	bool blueistPlanOne = false;
	bool blueistPlanTwo = false;
	Timeout blueistTimeout;
	Timeout blueistStartTimeout;

	while (true) {
		// antijam code
		// keep updating timestamp since last time intake motor was moving
		if (motors.get_actual_velocity() >= 10) { timestamp = pros::millis(); }

		// temp disables antijam
		// if (intakeFlags->antiJam && state == AntiJamState::IDLE && pros::millis() - timestamp > 250) {
		// 	// enable antijam code -> unwind the intake rq
		// 	state = AntiJamState::UNWIND;
		// 	antiJamTimeout = Timeout(333);
		// }

		if (state == AntiJamState::UNWIND) {
			motors.move_voltage(-12000);

			if (antiJamTimeout.timedOut()) {
				motors.move_voltage(0);
				state = AntiJamState::IDLE;
			}
		}

		// end of antijam code

		// ring ejector (Proof of concept)

		// double hue = color.get_hue();
		// if (hue >= 75) {
		// 	printf("BLUE ONE DETECTED\n");
		// 	firstOne = true;
		// 	blueistPlanOne = true;
		// 	blueistStartTimeout = Timeout(0);
		// }

		// if (0 < hue && hue < 25) {
		// 	printf("RED ONE DETECTED\n");
		// 	firstOne = true;
		// }

		// printf("%d  %f\n", color.get_proximity(), color.get_hue());

		// if (blueistPlanOne && blueistStartTimeout.timedOut()) {
		// 	blueistPlanOne = false;
		// 	blueistPlanTwo = true;
		// 	blueistTimeout = Timeout(200);
		// }

		// if (blueistPlanTwo) {
		// 	codeOverride = true;
		// 	// motors.move_voltage(12000);
		// 	motors.brake();
		// 	printf("BLUEISM SHALL COMMENCE\n\n\n\n");

		// 	if (blueistTimeout.timedOut()) {
		// 		codeOverride = false;
		// 		blueistPlanTwo = false;
		// 		firstOne = false;
		// 	}
		// }


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
			if (dist <= 55) {
				intakeFlags->partiallyIn = false;
				intakeFlags->fullyIn = true;
			} else if (dist <= 100) {
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

void Intake::setExtender(bool extended) {
	auto intakeFlags = robot->getFlag<Intake>().value();
	intakeFlags->isExtended = extended;
	extender.set_value(extended);
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

void Intake::toggleExtender() {
	setExtender(!robot->getFlag<Intake>().value()->isExtended);
}

void Intake::moveVoltage(int mv) {
	if (codeOverride) { return; }
	if (state != AntiJamState::IDLE) { return; }
	robot->getFlag<Intake>().value()->isMoving = (mv != 0 ? true : false);
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