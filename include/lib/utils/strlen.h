#pragma once
#include "arm_neon.h"
#include <cstdint>

#define VMIN8(x, index, value)                                                                                         \
	do {                                                                                                               \
		uint8x8_t m = vpmin_u8(vget_low_u8(x), vget_high_u8(x));                                                       \
		m = vpmin_u8(m, m);                                                                                            \
		m = vpmin_u8(m, m);                                                                                            \
		uint8x16_t r = vceqq_u8(x, m);                                                                                 \
                                                                                                                       \
		uint8x16_t z = vandq_u8(vmask, r);                                                                             \
                                                                                                                       \
		z = vpadd_u8(z, z);                                                                                            \
		z = vpadd_u8(z, z);                                                                                            \
		z = vpadd_u8(z, z);                                                                                            \
                                                                                                                       \
		unsigned u32 = vget_lane_u32(vreinterpret_u32_u8(z), 0);                                                       \
		index = __lzcnt(u32);                                                                                          \
		value = vget_lane_u8(m, 0);                                                                                    \
	} while (0)


// THIS IS A VECTORIZED VERSION OF STRLEN
std::size_t _strlen(const char* str) {
	std::size_t len = 0;

	uint8x16_t mask = vdupq_n_u8(255);

	while (true) {
		// load 16 bytes into vector
		uint8x16_t input = vld1q_u8((const uint8_t*) str + len);

		uint8x16_t matches = vtstq_u8(input, mask);

		// if the 16 bytes are all non-zero, than matches will have all bits set to 1
		// if there is a \0 as a character, than matches will have 0 somewhere in the 16 bytes
		uint8x8_t m = vpmin_u8(vget_low_u8(matches), vget_high_u8(matches));
		m = vpmin_u8(m, m);
		m = vpmin_u8(m, m);
		m = vpmin_u8(m, m);

		uint8x8_t posMask = vceq_u8(vget_low_u8(matches), m);

		uint8_t min = vget_lane_u8(m, 0);

		// to figure out position within vector
		// and it and zero out all elements which aren't it

		// hard part is figuring out first zero -> otherwise can just do a compare, mask aynthing that isn't equal to
		// min value and then sum up the things

		if (!min) {
			return len;// need to figure out offset within the vector
		}

		len += 16;
	}

	return len;
}