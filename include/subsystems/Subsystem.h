#pragma once
#include "Logger.h"
#include "RobotBase.h"
#include "lib/utils/is_derived.h"
#include <cxxabi.h>
#include <stdexcept>

namespace {
	inline auto typename_from(const std::type_info& info) -> std::string {
		struct MallocedString {
			const char* string;

			~MallocedString() {
				free(const_cast<char*>(string));
			}
		};

		const char* mangledName = info.name();
		int status = 0;
		const MallocedString demangled = {::abi::__cxa_demangle(mangledName,
		                                                        nullptr,// Output buffer
		                                                        nullptr,// length
		                                                        &status)};
		return (status == 0 ? demangled.string : mangledName);
	}
}// namespace

class Subsystem {
protected:
	RobotBase* robot;
	LoggerPtr logger;

public:
	Subsystem() = delete;

	template<util::Derived<Subsystem> Derived>
	Subsystem(RobotBase* robot, Derived* derived) : robot(robot) {
		if (!robot || !dynamic_cast<RobotBase*>(robot)) {
			throw std::runtime_error("Invalid Instantiation of Subsystem!");
		}

		logger = sLogger.createSource(::typename_from(typeid(*derived)));
	}

	virtual void registerTasks() {}
};