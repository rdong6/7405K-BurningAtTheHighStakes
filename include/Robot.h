#pragma once
#include <atomic>

#define sRobot Robot::getInstance()

enum class Auton : uint32_t { NONE = 0, LEFT, RIGHT };

class Robot {
public:
	enum OpMode { DRIVER, AUTONOMOUS };

private:
	// Singleton Stuff
	Robot() = default;
	Robot(const Robot&) = delete;
	Robot& operator=(const Robot&) = delete;

	// OpMode
	std::atomic<OpMode> opmode = AUTONOMOUS;
	std::atomic<Auton> auton = Auton::NONE;

public:
	// Robot Config stuff - yes it's public, idc <-- bad code that will be fixed at some point
	std::atomic<bool> isQual = true;// true if qual, false if elim

public:
	inline static Robot& getInstance() {
		static Robot INSTANCE;
		return INSTANCE;
	}

	void initialize();

	//  OpMode
	OpMode getOpMode();
	void setOpMode(OpMode op_mode);

	// Auton
	Auton getAuton();
	void setAuton(Auton auton);
};