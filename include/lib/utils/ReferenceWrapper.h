#pragma once

// same thing as std::reference_wrapper<T>
// except the accessor has overloaded the operators * and ->

namespace detail {
	template<class T>
	constexpr T& FUN(T& t) noexcept {
		return t;
	}
	template<class T>
	void FUN(T&&) = delete;
}// namespace detail

namespace util {
	template<class T>
	class reference_wrapper {
	public:
		// types
		using type = T;

		// construct/copy/destroy
		template<class U, class = decltype(detail::FUN<T>(std::declval<U>()),
		                                   std::enable_if_t<!std::is_same_v<reference_wrapper, std::remove_cvref_t<U>>>())>
		constexpr reference_wrapper(U&& u) noexcept(noexcept(detail::FUN<T>(std::forward<U>(u))))
		    : _ptr(std::addressof(detail::FUN<T>(std::forward<U>(u)))) {}

		reference_wrapper(const reference_wrapper&) noexcept = default;

		// assignment
		reference_wrapper& operator=(const reference_wrapper& x) noexcept = default;

		// access
		constexpr operator T&() const noexcept { return *_ptr; }

		constexpr T* operator->() const noexcept { return _ptr; }

		constexpr T& operator*() const noexcept { return *_ptr; }

	private:
		T* _ptr;
	};

	// deduction guides
	template<class T>
	reference_wrapper(T&) -> reference_wrapper<T>;
}// namespace util