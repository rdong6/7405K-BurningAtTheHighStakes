#pragma once
#include <type_traits>

namespace util {
	template<class D, class B>
	concept Derived = std::is_base_of<B, D>::value;
}