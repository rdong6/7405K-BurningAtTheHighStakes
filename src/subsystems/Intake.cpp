#include "subsystems/Intake.h"
#include "Constants.h"
#include "Logger.h"
#include "RobotBase.h"
#include "lib/utils/CoroutineGenerator.h"
#include "lib/utils/DelayedBool.h"
#include "lib/utils/Timeout.h"
#include "pros/motors.h"
#include "subsystems/Controller.h"
#include <cmath>

Intake::Intake(RobotBase* robot) : Subsystem(robot) {
	// sets to coast to avoid motor burnout
	motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

	// sets the proper cart of the motor incase you ever want to use moveVel
	motors.set_gearing_all(pros::E_MOTOR_GEAR_600);

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

bool Intake::redRingDetector() {
	return color.get_hue() < 25;
}

bool Intake::blueRingDetector() {
	return color.get_hue() >= 75;
}

// As of now, running constantly on robot no matter the competition state
RobotThread Intake::runner() {
	auto intakeFlags = robot->getFlag<Intake>().value();

	// switch (robot->curAlliance) {
	// 	case Alliance::BLUE:
	// 		blueismDetector = &Intake::redRingDetector;
	// 	case Alliance::RED:
	// 		blueismDetector = &Intake::blueRingDetector;
	// }

	blueismDetector = &Intake::blueRingDetector;

	// anti-jam vars
	unsigned int timestamp = 0;// timestamp since last time intake was moving
	bool runAntiJam = false;
	Timeout antiJamTimeout;


	// bluism plan
	bool blueistPlanOne = false;
	util::DelayedBool enableBlueist;
	util::DelayedBool resetBlueist;
	Timeout blueistTimeout;

	// old blueist code
	bool firstOne = false;
	bool blueistPlanTwo = false;
	Timeout blueistStartTimeout;

	// lady-brown mech detector
	bool enableLadyBrownDetector = false;

	while (true) {
		// antijam code
		// keep updating timestamp since last time intake motor was moving
		if (motors.get_actual_velocity() >= 20) { timestamp = pros::millis(); }

		// temp disables antijam
		if (intakeFlags->antiJam && state == AntiJamState::IDLE && pros::millis() - timestamp > 250) {
			// enable antijam code -> unwind the intake rq
			state = AntiJamState::UNWIND;
			antiJamTimeout = Timeout(333);
		}

		if (state == AntiJamState::UNWIND) {
			motors.move_voltage(-12000);

			if (antiJamTimeout.timedOut()) {
				motors.move_voltage(0);
				state = AntiJamState::IDLE;
			}
		}

		// end of antijam code

		// ring ejector (Proof of concept)

		// short circuiting prevents calling a nullptr
		// check if we already engaged blueism plan -> don't wanna reset timeout
		/*if (blueismDetector && (this->*blueismDetector)() && !blueistPlanOne) {
		    blueistPlanOne = true;
		    // enableBlueist = util::DelayedBool(225);
		    enableBlueist = util::DelayedBool(300);
		    blueistTimeout = Timeout(1000);// TUNE
		    printf("BLUEISM SHALL COMMENCE\n");
		}

		if (enableBlueist()) {
		    printf("\nACTIVATING BLUEISM\n");
		    motors.brake();
		    codeOverride = true;

		    if (blueistTimeout.timedOut()) {
		        printf("\nSTOPPING BLUEISM\n");
		        codeOverride = false;
		        enableBlueist = util::DelayedBool();
		        resetBlueist = util::DelayedBool(250);
		    }
		}

		if (resetBlueist()) {
		    resetBlueist = util::DelayedBool();
		    blueistPlanOne = false;
		}*/

		double hue = color.get_hue();
		if (hue >= 75) {
			printf("BLUE ONE DETECTED\n");
			firstOne = true;
			blueistPlanOne = true;
			blueistStartTimeout = Timeout(0);
		}

		if (0 < hue && hue < 25) {
			printf("RED ONE DETECTED\n");
			firstOne = true;
		}

		printf("%d  %f\n", color.get_proximity(), color.get_hue());

		if (blueistPlanOne && blueistStartTimeout.timedOut()) {
			blueistPlanOne = false;
			blueistPlanTwo = true;
			blueistTimeout = Timeout(200);
		}

		if (blueistPlanTwo) {
			codeOverride = true;
			// motors.move_voltage(12000);
			motors.brake();
			printf("BLUEISM SHALL COMMENCE\n\n\n\n");

			if (blueistTimeout.timedOut()) {
				codeOverride = false;
				blueistPlanTwo = false;
				firstOne = false;
			}
		}


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