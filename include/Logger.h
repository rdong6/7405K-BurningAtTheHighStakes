#pragma once
#include "pros/rtos.hpp"
#include "readerwritercircularbuffer.h"
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string_view>
#include <type_traits>

namespace detail {
	template<typename... TTypes>
	struct PackSize {
		consteval std::size_t get_size_in_bytes() const { return ((sizeof(TTypes) + 1) + ... + 0); }
	};

	// Get index of element type T in Tuple
	template<typename T, typename Tuple>
	struct TupleIndex;

	template<typename T, typename... Types>
	struct TupleIndex<T, std::tuple<T, Types...>> {
		static constexpr const std::size_t value = 0;
	};

	template<typename T, typename U, typename... Types>
	struct TupleIndex<T, std::tuple<U, Types...>> {
		static constexpr const std::size_t value = 1 + TupleIndex<T, std::tuple<Types...>>::value;
	};

	// check if tuple contains a type
	template<typename, typename>
	struct TupleHolds {};

	template<typename... A, typename B>
	struct TupleHolds<std::tuple<A...>, B> : std::bool_constant<(std::is_same_v<A, B> || ...)> {};

	template<typename A, typename B>
	concept TupleContains = TupleHolds<A, B>::value;
}// namespace detail

#define GET_TYPE_ID(type) detail::TupleIndex<type, SupportedTypes>::value

enum class LogLevel : uint8_t { TRACE = 1, DEBUG = 2, INFO = 4, WARNING = 8, ERROR = 16 };

class Logger;// fwd to use for friend in LogEntry


// one entry which contains format, stack params, and info like timestamp
// entry by producer thread, where the main logger thread will then do heavy work of formatting later on
class LogEntry {
private:
	struct string_literal_t {
		explicit string_literal_t(const char* str) : m_str(str) {}

		const char* m_str;
	};

	typedef std::tuple<LogEntry::string_literal_t, char*, char, uint32_t, int32_t, uint64_t, int64_t, double> SupportedTypes;

	LogEntry();// only to use in Logger::writeEntries() when we need to default construct obj


public:
	LogEntry(LogLevel level, const char* const file, const char* const func, unsigned int line, const char* const name,
	         const char* const fmt)
	    : m_bytes_used(0), m_buffer_size(sizeof(m_stack_buffer)), m_heap_buffer() {
		uint64_t timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
		                             std::chrono::high_resolution_clock::now().time_since_epoch())
		                             .count();

		// TODO: Convert this into static struct for each log entry (filename, func name, opt name, log level, line
		// number, user fmt args)

		// 29 bytes used to store start of log
		encode_raw<uint64_t>(timestamp);
		encode_raw<string_literal_t>(string_literal_t(file));// encodes ptr to strng
		encode_raw<string_literal_t>(string_literal_t(func));// encodes ptr to string
		encode_raw<string_literal_t>(string_literal_t(name));
		encode_raw<uint32_t>(line);
		encode_raw<LogLevel>(level);

		encode_c_string(fmt, std::strlen(fmt));
	}

	template<typename... Args>
	LogEntry(LogLevel level, const char* const file, const char* const func, unsigned int line, const char* const name,
	         const char* const fmt, Args&&... args)
	    : LogEntry(level, file, func, line, name, fmt) {
		// encode the params into the stack

		// HAVE TO MORE EFFICIENTLY ENCODE PARAMS!
		resize_buffer_if_needed(detail::PackSize<Args...>().get_size_in_bytes());
		(void(encode(std::forward<Args>(args))), ...);
	}

	~LogEntry() = default;

	LogEntry(LogEntry&&) = default;
	LogEntry& operator=(LogEntry&&) = default;

	// decodes data back into string
	std::string stringify();

private:
	template<typename Arg>
	void encode(Arg arg)
	/*requires(TupleContains<SupportedTypes, Arg>)*/// TODO: Readd in constraint
	{
		static_assert(detail::TupleHolds<SupportedTypes, Arg>::value,
		              "Unsupported type attempting to be encoded in log entry!");

		encode_raw<uint8_t>(GET_TYPE_ID(Arg));// encode type id in order to recover type later
		encode_raw<Arg>(arg);
	}

	template<typename Arg>
	void encode_raw(Arg arg) {
		char* temp = buffer();
		*reinterpret_cast<Arg*>(temp) = arg;
		m_bytes_used += sizeof(Arg);
	}

	void encode(const char* str) {
		if (str) { encode_c_string(str, std::strlen(str)); }
	}

	void encode(char* str) {
		if (str) { encode_c_string(str, std::strlen(str)); }
	}

	void encode_c_string(const char* const str, std::size_t len);

	// returns access to underlying buffer of entry -> either heap or stack
	// pointer points to next free byte in buffer
	char* buffer();

	// assumes null terminated string -> maybe change so it isn't

	// resizes buffer -> moves to heap buffer if size exceeds heap buffer
	void resize_buffer_if_needed(std::size_t additional_bytes);

	// decode entry into formatted string to output

private:
	std::size_t m_bytes_used;
	std::size_t m_buffer_size;
	std::unique_ptr<char[]> m_heap_buffer;// by default stack buffer is used unless it overflows
	char m_stack_buffer[256 - 2 * sizeof(std::size_t) - sizeof(decltype(m_heap_buffer))];// makes class 256 bytes large

	friend Logger;
};

class Logger {
private:
	Logger(std::string_view log_filename, std::size_t buffer_size);

public:
	~Logger();

	void set_log_level(LogLevel level);

	LogLevel get_log_level() const;

	static bool log(LogEntry entry);


private:
	void writeEntries();

private:
	enum class State : uint8_t { INIT, RUN, SHUTDOWN };
	// moodycamel::ReaderWriterQueue<LogEntry> m_buffer;
	moodycamel::BlockingReaderWriterCircularBuffer<LogEntry> m_buffer;
	std::atomic<State> m_state;
	pros::Task m_thread;
	std::FILE* m_file;
	LogLevel m_level;// level at which to log -> anything below is ignored

	friend void logger_initialize(std::string_view log_filename, std::size_t buffer_size);
};

void logger_initialize(std::string_view log_filename, std::size_t buffer_size);

void logger_stop();

void set_log_level(LogLevel level);

bool is_logged(LogLevel level);

#undef GET_TYPE_ID

#define LOG_IMPL(LEVEL, fmt, ...)                                                                                              \
	Logger::log(std::move(LogEntry(LogLevel::LEVEL, __FILE__, __func__, __LINE__, nullptr, fmt, ##__VA_ARGS__)));

#define LOG_TRACE(fmt, ...) is_logged(LogLevel::TRACE) && LOG_IMPL(TRACE, fmt, ##__VA_ARGS__)
#define LOG_DEBUG(fmt, ...) is_logged(LogLevel::DEBUG) && LOG_IMPL(DEBUG, fmt, ##__VA_ARGS__)
#define LOG_INFO(fmt, ...) is_logged(LogLevel::INFO) && LOG_IMPL(INFO, fmt, ##__VA_ARGS__)
#define LOG_WARN(fmt, ...) is_logged(LogLevel::WARNING) && LOG_IMPL(WARNING, fmt, ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...) is_logged(LogLevel::ERROR) && LOG_IMPL(ERROR, fmt, ##__VA_ARGS__)