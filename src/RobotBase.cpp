#include "RobotBase.h"
#include "lib/utils/CoroutineGenerator.h"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <iterator>

void RobotBase::queueTasks(std::list<std::pair<RobotFunc, TaskFunc>>& tasks) {
	for (auto& coro : coroQueue) { coro.second.destroy(); }
	coroQueue.clear();
	for (auto [pred, func] : tasks) { coroQueue.emplace_back(pred, func().getHandle()); }
}

[[noreturn]] void RobotBase::run() {
	// flags that signal if we should queue the coros for the corresponding state to run
	//      ensures that we don't duplicate running threads
	bool disabledLastLoop = false;
	bool armAuton = false;
	bool armOpCtrl = false;

	uint32_t time = pros::millis();

	while (true) {
		// queue coroutines to run given robot state
		if (pros::competition::is_disabled()) {
			if (!disabledLastLoop) {
				queueTasks(disabled);
				disabledLastLoop = true;
				armAuton = true;
				armOpCtrl = true;
			}
		} else {
			disabledLastLoop = false;

			if (pros::c::competition_is_autonomous() && armAuton) {
				queueTasks(autonomous);
				armAuton = false;
			} else if (!pros::competition::is_disabled() && !pros::competition::is_autonomous() && armOpCtrl) {
				queueTasks(opcontrol);
				armOpCtrl = false;
			}
		}

		// firstly, run the coroutines that will always run
		// secondly, run any state specific coroutines

		auto sentinelIter = std::begin(sentinel);
		while (sentinelIter != std::end(sentinel)) {
			if (sentinelIter->second.done()) {
				// coroutine is done -> thread has terminated, cleanup resources
				sentinelIter->second.destroy();// cleanup coro obj from heap
				sentinelIter = sentinel.erase(sentinelIter);
			} else {
				// coroutine/thread hasn't terminated -> run it

				// temp access to coro based on coro handle
				auto coro = util::coroutine::Generator<RobotFunc>(sentinelIter->second, false);

				// run coro/"thread" if it indicates that it wants to run -> from lambda that it returns
				// this is what allows for delays in coros as lambda can return false until certain time passed
				if (sentinelIter->first && sentinelIter->first()) { sentinelIter->first = coro(); }

				++sentinelIter;
			}
		}

		// see above comments for how scheduling works
		auto coroIter = std::begin(coroQueue);
		while (coroIter != std::end(coroQueue)) {
			if (coroIter->second.done()) {
				// coroutine is done -> thread has terminated, cleanup resources
				coroIter->second.destroy();// cleanup coro obj from heap
				coroIter = coroQueue.erase(coroIter);
			} else {
				// coroutine/thread hasn't terminated -> run it

				// temp access to coro based on coro handle
				auto coro = util::coroutine::Generator<RobotFunc>(coroIter->second, false);

				// run coro/"thread" if it indicates that it wants to run -> from lambda that it returns
				// this is what allows for delays in coros as lambda can return false until certain time passed
				if (coroIter->first && coroIter->first()) { coroIter->first = coro(); }

				++coroIter;
			}
		}

		pros::c::task_delay_until(&time, 10);
	}
}

void RobotBase::registerTask(TaskFunc func, TaskType type, const RobotFunc& pred) {
	switch (type) {
		case TaskType::AUTON:
			autonomous.emplace_back(pred, std::move(func));
			if (pros::competition::is_autonomous()) {
				coroQueue.emplace_back(pred, autonomous.back().second().getHandle());
			}
			break;

		case TaskType::OPCTRL:
			opcontrol.emplace_back(pred, std::move(func));
			if (!pros::competition::is_disabled() && !pros::competition::is_autonomous()) {
				coroQueue.emplace_back(pred, opcontrol.back().second().getHandle());
			}
			break;

		case TaskType::DISABLED:
			disabled.emplace_back(pred, std::move(func));
			if (pros::competition::is_disabled()) {
				coroQueue.emplace_back(pred, disabled.back().second().getHandle());
			}
			break;

		case TaskType::SENTINEL:
			sentinel.emplace_back(pred, func().getHandle());
			break;

		default:
			break;
	}
}