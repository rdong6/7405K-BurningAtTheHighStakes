#define FMT_HEADER_ONLY
#include "Logger.h"
#include "fmt/args.h"
#include "fmt/format.h"
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <string_view>
#include <tuple>
#include <type_traits>

// ASCII Escape Codes - should be compatible with most Unix terminals, and
// windows terminal should support it if using normal console, on windows, the
// application that opens console window must enable virtual processing bs
// thingimajig
#define RED_TERM "\x1B[31m"
#define BR_RED "\x1B[91m"
#define GRN "\x1B[32m"
#define BR_GREEN "\x1B[92m"
#define YEL "\x1B[33m"
#define BR_YEL "\x1B[93m"
#define BLU "\x1B[34m"
#define BR_BLU "\x1B[94m"
#define MAG "\x1B[35m"
#define BR_MAG "\x1B[95m"
#define CYN "\x1B[36m"
#define BR_CYN "\x1B[96m"
#define WHT "\x1B[37m"
#define BR_WHT "\x1B[97m"
#define BOLD "\x1B[1m"
#define RESET "\x1B[0m"

namespace {
	std::string_view level_to_string(quill::LogLevel level) {
		static constexpr std::array<std::string_view, 6> logLevelsMap = {"TRACE",   "DEBUG", "INFO",
		                                                                 "WARNING", "ERROR", "UNKNOWN"};

		// this will crash if there is no level set in the argument - it should never happen though - still bad code to
		// a certain extent?
		int logLevel = 31 - __builtin_clz(static_cast<int>(level));

		if (logLevel > logLevelsMap.size() - 1) { return logLevelsMap.back(); }

		return logLevelsMap[logLevel];
	}

	std::string_view level_to_color(quill::LogLevel level) {
		static constexpr std::array<std::string_view, 6> logLevelsMap = {BOLD BR_MAG, BOLD BR_BLU, BOLD BR_GREEN,
		                                                                 BOLD BR_YEL, BOLD BR_RED, BOLD BR_WHT};

		// this will crash if there is no level set in the argument - it should never happen though - still bad code to
		// a certain extent?
		int logLevel = 31 - __builtin_clz(static_cast<int>(level));

		if (logLevel > logLevelsMap.size() - 1) { return logLevelsMap.back(); }

		return logLevelsMap[logLevel];
	}
}// namespace

#define GET_TYPE_ID(type) detail::TupleIndex<type, SupportedTypes>::value

namespace quill {
	// LogEntry implementation

	LogEntry::LogEntry() : m_bytes_used(0), m_buffer_size(0), m_heap_buffer() {}

	void LogEntry::encode_c_string(const char* const str, std::size_t len) {
		if (len == 0) { return; }

		resize_buffer_if_needed(len + 2 /* null terminating & id */);
		char* buf = buffer();
		*reinterpret_cast<uint8_t*>(buf++) = GET_TYPE_ID(char*);
		memcpy(buf, str, len + 1);
		m_bytes_used += len + 2 /* null terminating & id */;
	}

	char* LogEntry::buffer() {
		return !m_heap_buffer ? &m_stack_buffer[m_bytes_used] : &(m_heap_buffer.get())[m_bytes_used];
	}

	void LogEntry::resize_buffer_if_needed(std::size_t additional_bytes) {
		const std::size_t requiredSize = m_bytes_used + additional_bytes;

		if (requiredSize <= m_buffer_size) { return; }

		m_buffer_size = std::max(static_cast<std::size_t>(512), requiredSize);

		if (!m_heap_buffer) {
			// move data from stack buffer onto newly allocated heap buffer
			m_heap_buffer.reset(new char[m_buffer_size]);
			memcpy(m_heap_buffer.get(), &m_stack_buffer, sizeof(m_stack_buffer));
			return;
		}

		// a heap buffer is already allocated -> resize heap buffer
		std::unique_ptr<char[]> new_heap_buffer(new char[m_buffer_size]);
		memcpy(new_heap_buffer.get(), m_heap_buffer.get(), m_bytes_used);
		m_heap_buffer.swap(new_heap_buffer);
	}

	std::string LogEntry::stringify() {
		// Debug/Info/Warn/Error:
		// 1) "[TIME LEVEL] MESSAGE"
		// 2) "[TIME NAME LEVEL] MESSAGE"
		// Trace:
		// 3) "[TIME NAME LEVEL] [__file__(__line__): __func__] MESSAGE"
		// 4) "[TIME LEVEL] [__file__(__line__): __func__] MESSAGE"
		static constexpr std::array<std::string_view, 4> headerFormats = {
		        "[{}{:2f}{} {}{}{}] {}\n", "[{}{:2f} {}{} {}{}{}] {}\n", "[{}{:2f}{} {}{}{}] [{}({}): {}] {}\n",
		        "[{}{:2f} {}{} {}{}{}] [{}({}): {}] {}\n"};

		char* start = !m_heap_buffer ? m_stack_buffer : m_heap_buffer.get();
		const char* const end = start + m_bytes_used;

		// decode header of entry
		// clang-format off
		uint64_t timestamp = *reinterpret_cast<uint64_t*>(start); start += sizeof(uint64_t);
		string_literal_t file = *reinterpret_cast<string_literal_t*>(start); start += sizeof(string_literal_t);
		string_literal_t func = *reinterpret_cast<string_literal_t*>(start); start += sizeof(string_literal_t);
		string_literal_t name = *reinterpret_cast<string_literal_t*>(start); start += sizeof(string_literal_t);
		uint32_t line = *reinterpret_cast<uint32_t*>(start); start += sizeof(uint32_t);
		LogLevel level = *reinterpret_cast<LogLevel*>(start); start += sizeof(LogLevel);
		// clang-format on

		// decode user params and format user str
		fmt::dynamic_format_arg_store<fmt::format_context> user_args;

		// gets the user fmt string
		std::string_view user_fmt(start);
		start += strlen(start) + 1 /* null terminating character */;

		// decodes user params
		while (start != end) {
			uint8_t type_id = *reinterpret_cast<uint8_t*>(start);
			start++;
			// there's better ways to do w/o using a switch statement (better flexibility when changing supported types)
			// -> but requires template metaprogramming because of std::tuple_element<>

#define GET_ARG(id)                                                                                                    \
	user_args.push_back(*reinterpret_cast<std::tuple_element<id, SupportedTypes>::type*>(start));                      \
	start += sizeof(std::tuple_element<id, SupportedTypes>::type);

			switch (type_id) {
				case 0:
					user_args.push_back(std::string_view(reinterpret_cast<string_literal_t*>(start)->m_str));
					start += sizeof(string_literal_t);
					break;
				case 1:
					// string implementation
					user_args.push_back(std::string_view(start));
					start += strlen(start);
					break;
				case 2:
					GET_ARG(2)
					break;
				case 3:
					GET_ARG(3)
					break;
				case 4:
					GET_ARG(4)
					break;
				case 5:
					GET_ARG(5)
					break;
				case 6:
					GET_ARG(6)
					break;
				case 7:
					GET_ARG(7)
					break;
			}
		}

		std::string user_output = fmt::vformat(user_fmt, user_args);


		// used to select which header format to use
		int header_index = 0;

		fmt::dynamic_format_arg_store<fmt::format_context> header_args;
		header_args.push_back(std::string_view(BOLD BR_WHT));
		header_args.push_back(timestamp * 0.000001);
		if (name.m_str) {
			header_args.push_back(std::string_view(name.m_str));
			header_index++;
		}
		header_args.push_back(std::string_view(RESET));
		header_args.push_back(level_to_color(level));
		header_args.push_back(level_to_string(level));
		header_args.push_back(std::string_view(RESET));
		if (file.m_str) {
			header_args.push_back(std::string_view(file.m_str));
			header_args.push_back(line);
			header_index++;
		}
		if (func.m_str) {
			header_args.push_back(std::string_view(func.m_str));
			header_index++;
		}

		header_args.push_back(std::string_view(user_output));

		return fmt::vformat(headerFormats[header_index], header_args);
	}


	// Logger implementation
	Logger::Logger(std::string_view log_filename, std::size_t buffer_size)
	    : m_buffer(buffer_size), m_state(State::INIT),
	      m_thread([](void* ptr) { static_cast<Logger*>(ptr)->writeEntries(); }, this, "Logger"),
	      m_level(LogLevel::TRACE /* by default all log levels enabled*/) {
		m_file = std::fopen("/usd/log.txt", "w");
		m_state.store(State::RUN, std::memory_order_release);
	}

	Logger::~Logger() {
		m_state.store(State::SHUTDOWN);
		m_thread.join();
		std::fclose(m_file);
	}

	void Logger::set_log_level(LogLevel level) {
		this->m_level = level;
	}

	LogLevel Logger::get_log_level() const {
		return this->m_level;
	}

	void Logger::writeEntries() {
		while (m_state.load(std::memory_order_acquire) == State::INIT) { pros::delay(10); }

		LogEntry entry;
		while (m_state.load() == State::RUN) {
			bool succeeded = m_buffer.try_dequeue(entry);
			if (!succeeded) {
				pros::delay(10);
				continue;
			}

			// decoded back into a formatted string
			std::string formatted_output = entry.stringify();

			// Write string to console & file
			std::fwrite(formatted_output.c_str(), sizeof(char), formatted_output.length(), stdout);

			if (m_file) { std::fwrite(formatted_output.c_str(), sizeof(char), formatted_output.length, m_file); }
		}

		while (m_buffer.try_dequeue(entry)) {
			std::string formatted_output = entry.stringify();
			std::fwrite(formatted_output.c_str(), sizeof(char), formatted_output.length(), stdout);
		}
	}

	// General Functions
	static std::unique_ptr<quill::Logger> logger_instance;

	bool Logger::log(LogEntry entry) {
		if (!logger_instance) { return true; }

		logger_instance->m_buffer.try_enqueue(std::move(entry));
		return true;
	}

	void initialize(std::string_view log_filename, std::size_t buffer_size) {
		logger_instance.reset(new Logger(log_filename, buffer_size));
	}

	void stop() {
		logger_instance.reset();
	}

	void set_log_level(LogLevel level) {
		if (logger_instance) { logger_instance->set_log_level(level); }
	}

	bool is_logged(LogLevel level) {
		if (!logger_instance) { return false; }

		using Type = std::underlying_type<LogLevel>::type;

		return static_cast<Type>(level) >= static_cast<Type>(logger_instance->get_log_level());
	}
}// namespace quill