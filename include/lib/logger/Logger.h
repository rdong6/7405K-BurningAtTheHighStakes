#pragma once
#include "LogLevel.h"
#include <string_view>

// TODO: finish this logger someday... ¯\_(ツ)_/¯

namespace lib {
	 struct Metadata {
		constexpr Metadata(const char* sourceLocation, const char* callerFunction, const char* fmt, LogLevel level) : sourceLocation(sourceLocation), callerFunction(callerFunction), fmt(fmt), level(level) {}

	 	const char* sourceLocation;
		const char* callerFunction;
		const char* fmt;
		const LogLevel level;
	};

	class Logger {
	public:
		[[nodiscard]] bool shouldLogStatement(LogLevel level) const noexcept {
			return level >= logLevel;
		}

		void setLogLevel(LogLevel level) noexcept;

		[[nodiscard]] LogLevel getLogLevel() const noexcept;

		template<typename... Args>
		void log(const Metadata* metadata, Args&&... args) {
			//
		}

	private:
		lib::LogLevel logLevel;
	};
}

extern Logger logger;


// -------------------------------------- BEGIN HELPER MACROS --------------------------------------

// CONVERT NUMBER TO STRING
#define LOGGER_AS_STR(x) #x
#define LOGGER_STRINGIFY(x) LOGGER_AS_STR(x)

#ifndef LOGGER_LIKELY
#if defined(__GNUC__)
#define LOGGER_LIKELY(x) (__builtin_expect((x), 1))
#else
#define LOGGER_LIKELY(x) (x)
#endif
#endif

#ifndef LOGGER_UNLIKELY
#if defined(__GNUC__)
#define LOGGER_UNLIKELY(x) (__builtin_expect((x), 0))
#else
#define LOGGER_UNLIKELY(x) (x)
#endif
#endif

// -------------------------------------- END HELPER MACROS --------------------------------------

#define LOGGER_DEFINE_MACRO_METADATA(caller_function, fmt, log_level) \
	static constexpr struct lib::Metadata macro_metadata{__FILE__ ":" LOGGER_STRINGIFY(__LINE__),		\
	caller_function,																				\
	fmt,																							\
	log_level																						\
	};

#define LOGGER_CALL(likelyhood, log_level, fmt, ...)	\
	do {\
		if (likelyhood(/* CHECK LOGGER IF THIS LOG_LEVEL IS CURRENTLY BEING LOGGED*/)) {\
			LOGGER_DEFINE_MACRO_METADATA(__FUNCTION__, fmt, log_level);\
			logger.log(&macro_metadata,  \
		}\
	}\
	while(0)