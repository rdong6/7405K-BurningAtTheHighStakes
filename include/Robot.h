#pragma once
#include "RobotBase.h"
#include "lib/utils/ReferenceWrapper.h"
#include "lib/utils/TupleFuncs.h"
#include "pros/rtos.h"
#include "subsystems/AutonSelector.h"
#include "subsystems/Controller.h"
#include "subsystems/Drive.h"
#include "subsystems/Intake.h"
#include "subsystems/Lift.h"
#include "subsystems/Odometry.h"
#include "subsystems/Pnooomatics.h"
#include "subsystems/Subsystem.h"
#include <functional>
#include <optional>
#include <tuple>
#include <type_traits>

// archaic code that used to have a purpose
namespace detail {
	template<typename, typename>
	struct tuple_holds {};

	template<typename... A, typename B>
	struct tuple_holds<std::tuple<A...>, B> : std::bool_constant<(std::is_same_v<A, B> || ...)> {};
}// namespace detail

template<typename... Args>
class Robot : public RobotBase {
private:
	std::tuple<Args*...> subsystems{new Args(this)...};
	std::tuple<typename Args::flags*...> flags{new typename Args::flags()...};

public:
	Robot(const Robot&) = delete;
	Robot(Robot&&) = delete;
	Robot& operator=(const Robot&) = delete;
	Robot& operator=(Robot&&) = delete;

	Robot() : RobotBase() {
		// setup the function table to get access to each subsystem & register tasks

		// in order to ensure that the subsystems get scheduled in the order they are specified in the parameter pack
		// we need to apply this custom for_each func to the tuple
		// std::apply() doesn't work as expansion of the parameter pack is impl defined
		util::tuple::for_each(subsystems, [this](auto x) {
			this->invokeTable[typeid(x)] =
			        static_cast<std::function<decltype(x)()>>([this]() { return std::get<decltype(x)>(this->subsystems); });
		});

		util::tuple::for_each(flags, [this](auto x) {
			this->invokeTable[typeid(x)] =
			        static_cast<std::function<decltype(x)()>>([this]() { return std::get<decltype(x)>(this->flags); });
		});

		// have to do this after invoke table is initialized w/ subsystems
		// as controller callbacks are registered in registerTasks()
		// issue arises if a subsystem registers tasks before Controller subsystem is inside invoke table
		util::tuple::for_each(subsystems, [this](auto x) { x->registerTasks(); });
	}

	~Robot() {
		util::tuple::for_each(subsystems, [](auto x) { delete x; });
		util::tuple::for_each(flags, [](auto x) { delete x; });
	}
};

inline Robot<AutonSelector, Odometry, Drive, Lift, Intake, Pnooomatics, Controller>* robotInstance = nullptr;
inline pros::task_t robotTask = nullptr;