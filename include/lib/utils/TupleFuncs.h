#pragma once
#include <forward_list>
#include <initializer_list>

namespace util::tuple {
	namespace {
		template<class Tuple, class F, std::size_t... I>
		constexpr F for_each_impl(Tuple&& t, F&& f, std::index_sequence<I...>) {
			return (void) std::initializer_list<int>{(std::forward<F>(f)(std::get<I>(std::forward<Tuple>(t))), 0)...},
			       f;
		}
	}// namespace

	template<class Tuple, class F>
	constexpr F for_each(Tuple&& t, F&& f) {
		return for_each_impl(std::forward<Tuple>(t), std::forward<F>(f),
		                     std::make_index_sequence<std::tuple_size<std::remove_reference_t<Tuple>>::value>{});
	}

	template<typename Tuple, typename Action>
	void perform(Tuple&& tuple, size_t index, Action action) {
		size_t currentIndex = 0;
		for_each(tuple, [action = std::move(action), index, &currentIndex](auto&& value) {
			if (currentIndex == index) { action(std::forward<decltype(value)>(value)); }
			++currentIndex;
		});
	}
}// namespace util::tuple