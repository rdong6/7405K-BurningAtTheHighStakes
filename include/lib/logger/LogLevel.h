#pragma once
#include <cstdint>

namespace lib {
	enum class LogLevel : uint8_t {
		TRACE,
		DEBUG,
		INFO,
		WARNING,
		ERROR,
		CRITICAL
	};
}
