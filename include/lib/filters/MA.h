#pragma once
#include <type_traits>
#include "lib/utils/CircularBuffer.h"

// Moving average filter
// Be careful using floating point types! Round-off error can cause the moving average to drift over time
template<typename T> requires (std::is_arithmetic<T>::value)
class MA {
public:
	explicit MA(unsigned int taps) : circularBuffer(taps), total(0) {
		// initialize the ring buffer w/ all 0's to start out
		for (unsigned int i = 0; i < taps; i++) {
			circularBuffer.emplace_back(0);
		}
	}

	// calculates new moving average val w/ new val
	T update(T val) {
		total -= circularBuffer.back();
		total += val;
		circularBuffer.push_front(val);

		return total / circularBuffer.size();
	}

	// gets current calculated moving average value
	T getMA() const noexcept {
		return total / circularBuffer.size();
	}

	void reset() {
		total = 0;
		for (unsigned int i = 0; i < circularBuffer.max_size(); i++) {
			circularBuffer.push_back(0);
		}
	}

private:
	// newest values are at front of buffer
	util::CircularBuffer<T> circularBuffer;
	T total;
};