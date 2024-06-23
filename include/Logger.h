#pragma once
// clang-format off
#include "fmt/format.h"
// clang-format on
#include "pros/apix.h"
#include "pros/rtos.hpp"
#include <memory>
#include <string_view>
#include <unordered_map>

#define sLogger Logger::getInstance()

class LogSource;
using LoggerPtr = std::shared_ptr<LogSource>;

// To add extra 🌶️ to the logger
// Naming convention is a little off but whatever
namespace LoggerColor {
	// extern const char* RED;
	// extern const char* GREEN;
	// extern const char* YELLOW;
	// extern const char* BLUE;
	// extern const char* MAGENTA;
	// extern const char* CYAN;
	// extern const char* WHITE;
	// extern const char* BRIGHT_RED;
	// extern const char* BRIGHT_GREEN;
	// extern const char* BRIGHT_YELLOW;
	// extern const char* BRIGHT_BLUE;
	// extern const char* BRIGHT_MAGENTA;
	// extern const char* BRIGHT_CYAN;
	// extern const char* BRIGHT_WHITE;

	extern const char* RED_BG;
	extern const char* GREEN_BG;
	extern const char* YELLOW_BG;
	extern const char* BLUE_BG;
	extern const char* MAGENTA_BG;
	extern const char* CYAN_BG;
	extern const char* WHITE_BG;
	extern const char* BRIGHT_RED_BG;
	extern const char* BRIGHT_GREEN_BG;
	extern const char* BRIGHT_YELLOW_BG;
	extern const char* BRIGHT_BLUE_BG;
	extern const char* BRIGHT_MAGENTA_BG;
	extern const char* BRIGHT_CYAN_BG;
	extern const char* BRIGHT_WHITE_BG;
	extern const char* NONE;
}// namespace LoggerColor


// Overarching logging manager
// Sink for the logging sources
class Logger {
	friend class LogSource;

private:
	pros::task_t task = nullptr;
	FILE* logFile = nullptr;
	std::string filename = std::string("");
	std::unordered_map<std::string, std::shared_ptr<LogSource>> sources =
	        std::unordered_map<std::string, std::shared_ptr<LogSource>>();

	Logger() = default;
	Logger(const Logger&) = delete;
	Logger& operator=(const Logger&) = delete;

	// thread that runs in backend, logging to file/handler
	[[noreturn]] void backend();

public:
	inline static Logger& getInstance() {
		static Logger INSTANCE;

		return INSTANCE;
	}

	/**
	 * @brief Initializes the logger.
	 * Starts the internal worker thread
	 * and opens the file.
	 *
	 * For now: Everytime logger gets initialized, contents of the specified file
	 * get wiped. And the different modes that we are logging in are demarked by
	 * either AUTON or OPCONTROl within the file. Maybe split it into two files in
	 * the future
	 *
	 * @param filename - Name of file. Don't need to include "/usd/"
	 */
	void initialize(std::string filename);

	// literally just closes and reopens the file
	// because vex
	void flush();

	/**
	 * @brief Just closes the output file of the logger.
	 * Depending on what we decide to do with handling closing the file, when the
	 * robot switches to opctrl or auton, the file may be reopened again.
	 */
	void close();

	/**
	 * @brief Kills the logger from running and closes any opened file.
	 * Must call initialize to restart logger.
	 */
	void terminate();

	std::shared_ptr<LogSource> createSource(std::string name, uint32_t timeout = 0,
	                                        const char* color = LoggerColor::NONE);

	/**
	 * @brief Get the Source object
	 *
	 * @param name
	 * @return std::shared_ptr<LogSource>
	 */
	std::shared_ptr<LogSource> getSource(std::string name);

	/**
	 * @brief Removes a LogSource from the list of LogSources that the Logger
	 * logs. Thereby stopping any output from this source from being logged at
	 * all.
	 *
	 * @param name - Name of the LogSource to delete.
	 */
	void deleteSource(std::string name);
};

class LogSource {
	friend class Logger;

public:
	enum LogLevel : uint8_t { DEBUG = 1, INFO = 2, WARNING = 4, ERROR = 8 };
	enum Source : uint8_t { CONSOLE = 1, FILE = 2 };

private:
	struct Message {
		// in ms - might have to do μs to sort out contentions on synchronization of
		// messages from multiple sinks
		uint32_t timestamp;
		uint32_t len;// num of chars - excludes the \0

		// this must be a pointer due to  how the RTOS's queue implementation is
		// done because it's copied, the RTOS does a memcpy. memcpying the contents
		// of a string would result in interesting behavior especially if it is no
		// longer doing a short string optimization because you'd just be copying a
		// ptr which will be dangling when the dtor of the string gets called God i
		// love the risk of memory leaks
		char* msg;
	};

	pros::c::queue_t mailboxConsole;
	pros::c::queue_t mailboxFile;

	const char* loggerColor;
	std::string name;

	uint32_t timeout;

	// which log levels we should ignore - bitmask of which ones to exclude from logging
	uint8_t logLevels;

	// To which sinks are we outputting
	Source outputSources;

	/**
	 * @brief Construct a new Log Source object
	 * By default enables all log levels, and logs to all sources.
	 *
	 * We only want logger to be able to create these log sources as it manages
	 * these objects and we don't want people to willy nilly do these things.
	 */
	LogSource(std::string sourceName, uint32_t timeout = 0, const char* color = LoggerColor::NONE);

	std::string_view levelToString(LogLevel level);

	void log(LogLevel level, uint32_t timestamp, std::string_view fmt, fmt::format_args args);

public:
	/**
	 * @brief Sets which levels of logging will actually be logged and which will
	 * be ignroed.
	 *
	 * @param level - Set level via bit operations. ex: DEBUG | INFO | WARNING
	 */

	void toggleLevel(LogLevel level);

	/**
	 * @brief Sets which levels of logging will actually be logged and which will
	 * be ignored.
	 *
	 * @param sources - Which output sources this log source will log to.
	 */
	void setOutput(Source sources);

	void setTimeout(uint32_t timeout);

	// Can be treated sortof like a printf
	// but formatting must follow fmtlib's specs
	// https://fmt.dev/10.0.0/syntax.html

	template<class... Args>
	void debug(std::string fmt, Args&&... args) {
		if ((logLevels & DEBUG) != 0) { return; }
		log(DEBUG, pros::micros(), fmt, fmt::make_format_args(args...));
	}

	template<class... Args>
	void info(std::string fmt, Args&&... args) {
		if ((logLevels & INFO) != 0) { return; }
		log(INFO, pros::micros(), fmt, fmt::make_format_args(args...));
	}

	template<class... Args>
	void warn(std::string fmt, Args&&... args) {
		if ((logLevels & WARNING) != 0) { return; }
		log(WARNING, pros::micros(), fmt, fmt::make_format_args(args...));
	}

	template<class... Args>
	void error(std::string fmt, Args&&... args) {
		if ((logLevels & ERROR) != 0) { return; }
		log(ERROR, pros::micros(), fmt, fmt::make_format_args(args...));
	}
};