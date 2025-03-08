#pragma once
#include "lib/utils/CoroutineGenerator.h"
#include "lib/utils/ReferenceWrapper.h"
#include <any>
#include <functional>
#include <list>
#include <map>
#include <optional>
#include <typeindex>
#include <utility>

using RobotFunc = std::function<bool(void)>;// lambda from each coro, indicating if it's ready to run or not
using TaskFunc = std::function<util::coroutine::Generator<RobotFunc>(void)>;// function which serves as the "thread"
typedef util::coroutine::Generator<RobotFunc> (*AutonFn_t)(void);// C func ptr -> specifically for auton selector only

using RobotThread = util::coroutine::Generator<RobotFunc>;

enum class Alliance : uint8_t { INVALID = 0, BLUE, RED };
enum class TaskType { AUTON, OPCTRL, DISABLED, SENTINEL };

class RobotBase {
private:
	// stores current coroutines that are being run (handle to the coroutine objs)
	// sentinel = coroutines that run no matter the competition state -> always runs before other coroutines
	std::list<std::pair<RobotFunc, util::coroutine::Generator<RobotFunc>::handle_type>> coroQueue, sentinel;

	std::list<std::pair<RobotFunc, TaskFunc>> autonomous, opcontrol, disabled;

protected:
	// function table with functions that return a pointer to each subsystem on the robot
	std::map<std::type_index, std::any> invokeTable;

private:
	// kills coros in coroQueue
	// and queues new coros to run
	void queueTasks(std::list<std::pair<RobotFunc, TaskFunc>>& tasks);

	// helper func to get access to value stored by invokeTable
	template<typename T>
	std::optional<util::reference_wrapper<T>> get() {
		if (auto f = invokeTable.find(typeid(T*)); f != invokeTable.end()) {
			return std::make_optional(util::reference_wrapper<T>(*(std::any_cast<std::function<T*()>>(f->second)())));
		}

		return std::nullopt;
	}


public:
	RobotBase() = default;

	// runs the scheduler (10ms loop)
	[[noreturn]] void run();

	// registers a task to be run by scheduler
	// tasks are ran in the order they are registered in (sentinel tasks get priority)
	void registerTask(TaskFunc func, TaskType type, const RobotFunc& pred = []() -> bool { return true; });

	// gets access to each subsystem
	template<typename T>
	std::optional<util::reference_wrapper<T>> getSubsystem() {
		return this->get<T>();
	}

	template<typename T>
	std::optional<util::reference_wrapper<typename T::flags>> getFlag() {
		return this->get<typename T::flags>();
	}

	AutonFn_t autonFnPtr = nullptr;
	Alliance curAlliance = Alliance::INVALID;

	bool isElim = false;// whether we're running elim or qual auton
	bool allianceStake = false;
	uint32_t delay = 0;// in ms
};

// Initializes all robotics code + starts the scheduler
void robot_init();