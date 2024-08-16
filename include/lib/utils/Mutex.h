#pragma once
#include "pros/rtos.hpp"
#include <functional>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <type_traits>

namespace detail {
	template<typename T>
	class is_shared_lock {
	private:
		typedef char Small;
		struct Big {
			Small x[2];
		};

		template<typename C>
		static Small test(decltype(&C::lock_shared));
		template<typename C>
		static Big test(...);

	public:
		enum { value = sizeof(test<T>(0)) == sizeof(Small) };
	};
}// namespace detail

namespace util {
	template<class T, class Lockable = pros::Mutex>
	class Mutex {
	private:
		typedef typename std::conditional<detail::is_shared_lock<Lockable>::value, std::shared_lock<Lockable>,
		                                  std::unique_lock<Lockable>>::type SharedLock;

		template<typename Var, typename Locker>
		class guard_common {
		private:
			friend class Mutex;

			Var* ptr;
			Locker lock;

			explicit guard_common(const Mutex& m) : ptr(std::addressof(m.data)), lock(m.mutex) {}

			guard_common(const guard_common&) = delete;
			guard_common& operator=(const guard_common&) = delete;

			// redundent - but to be more explicit
			guard_common(guard_common&&) = delete;
			guard_common& operator=(guard_common&&) = delete;

		public:
			Var& operator*() const noexcept {
				return *ptr;
			}

			Var* operator->() const noexcept {
				return ptr;
			}
		};

		mutable T data;
		mutable Lockable mutex;

	public:
		// exclusive lock acquired
		// read + write
		typedef guard_common<T, std::unique_lock<Lockable>> Guard;


		// either acquires exclusive lock or read lock if it is a shared_mutex
		// read only
		typedef guard_common<const T, SharedLock> ConstGuard;

		template<typename... Args, std::enable_if_t<std::is_constructible_v<T, Args...>, int> = 0>
		constexpr explicit Mutex(Args&&... args) : data(std::forward<Args>(args)...) {}

		/**
		 * @brief Holds a lock exclusively, and returns a RAII guard allowing access to the protected data.
		 * Allows read + write
		 *
		 * @return Guard
		 */
		[[nodiscard("Should not discard Mutex Guard immediately after construction")]] Guard wlock() const {
			return Guard(*this);
		}

		/**
		 * @brief Invoke a function while holding a lock in shared mode (if possible).
		 *
		 * A reference to the datum will be passed to the function as the only argument.
		 *
		 * This can be used with lambda argument to easily define small critical sections in code. For example:
		 *
		 *	auto value = obj.wlock([](auto& data) {
		 *		data.doStuff(); // modifies the data
		 *		return data;
		 *	});
		 *
		 * Or just simply:
		 *
		 *	obj.wlock([](auto& data) {
		 *		data.doStuff(); // modifies the data
		 *	})
		 */
		template<class Function>
		auto wlock(const Function& func) const {
			auto guard = Guard(*this);
			return func(*guard);
		}

		/**
		 * @brief Holds a lock (shared mode if possible), and returns a RAII guard allowing access to the protected
		 * data. Allows ONLY reading
		 *
		 * @return ConstGuard
		 */
		[[nodiscard("Should not discard Mutex Guard immediately after construction")]] ConstGuard rlock() const {
			return ConstGuard(*this);
		}

		/**
		 * @brief Invoke a function while holding a lock in shared mode (if possible).
		 *
		 * A const reference to the datum will be passed to the function as the only argument.
		 *
		 * This can be used with lambda argument to easily define small critical sections in code. For example:
		 *
		 *	auto value = obj.rlock([](auto& data) {
		 *		return data;
		 *	});
		 */
		template<class Function>
		auto rlock(const Function& func) const {
			auto guard = ConstGuard(*this);
			return func(*guard);
		}

		/*
		You SHOULD REALLY KNOW what your doing if you were to call this function.
		There is no synchronization in place when accessing the underlying datum.

		I included this as sometimes, there is no point in locking, but it really is a design flaw to have a mutex allow
		unguarded access to the data.
		*/
		__attribute__((
		        warning("You SHOULD REALLY KNOW what your doing. Unsafe read that does not have any synchronization.")))
		const T&
		unsafeRead() const {
			return data;
		}
	};
}// namespace util