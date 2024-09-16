#pragma once
#include "pros/rtos.h"
#include <coroutine>
#include <exception>
#include <functional>
#include <utility>

// Generic coroutine generator
namespace util::coroutine {
	template<typename T>
	struct Generator {
		struct promise_type;
		using handle_type = std::coroutine_handle<promise_type>;

		struct promise_type {
			T value_;
			std::exception_ptr exception_;

			Generator get_return_object() {
				return Generator(handle_type::from_promise(*this));
			}

			std::suspend_always initial_suspend() {
				return {};
			}

			std::suspend_always final_suspend() noexcept {
				return {};
			}

			void unhandled_exception() {
				exception_ = std::current_exception();
			}

			template<std::convertible_to<T> From>
			std::suspend_always yield_value(From&& from) {
				value_ = std::forward<From>(from);
				return {};
			}

			void return_void() {}
		};

		handle_type h_;
		bool destroyHandleOnExit = true;

		explicit Generator(handle_type h, bool destroyHandleOnExit = true)
		    : h_(h), destroyHandleOnExit(destroyHandleOnExit) {}

		Generator(Generator&& generator) noexcept
		    : h_(generator.h_), destroyHandleOnExit(generator.destroyHandleOnExit) {
			generator.destroyHandleOnExit = false;
		}

		handle_type getHandle() {
			destroyHandleOnExit = false;
			return h_;
		}

		~Generator() {
			if (destroyHandleOnExit) { h_.destroy(); }
		}

		explicit operator bool() {
			fill();
			return !h_.done();
		}

		T operator()() {
			fill();
			full_ = false;
			return std::move(h_.promise().value_);
		}

	private:
		bool full_ = false;

		void fill() {
			if (!full_) {
				h_();
				if (h_.promise().exception_) std::rethrow_exception(h_.promise().exception_);
				full_ = true;
			}
		}
	};

	inline std::function<bool(void)> nextCycle() {
		return []() -> bool { return true; };
	}

	inline std::function<bool(void)> delay(uint32_t ms) {
		uint32_t curTime = pros::c::millis();
		return [=]() -> bool { return pros::c::millis() - curTime > ms; };
	}
}// namespace util::coroutine