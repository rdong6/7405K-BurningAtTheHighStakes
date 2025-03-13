#pragma once
#include "Math.h"
#include <cstddef>
#include <vector>

namespace util {
	template<typename T>
	class CircularBuffer {
	public:
		explicit CircularBuffer(std::size_t size) : data(size, T{}), length(0), m_front(0) {}

		class iterator {
		public:
			iterator(CircularBuffer* buf, size_t index) : buf(buf), index(index) {}

			iterator& operator++() {
				++index;
				return *this;
			}

			iterator operator++(int) {
				iterator ret = *this;
				++(*this);
				return ret;
			}

			bool operator==(const iterator& rhs) const {
				return this->buf == rhs.buf && this->index == rhs.index;
			}

			bool operator!=(const iterator& rhs) const {
				return !(*this == rhs);
			}

			T& operator*() {
				return (*buf)[index];
			}

		private:
			CircularBuffer* buf;
			size_t index;
		};

		class const_iterator {
		public:
			const_iterator(const CircularBuffer* buf, size_t index) : buf(buf), index(index) {}

			const_iterator& operator++() {
				++index;
				return *this;
			}

			const_iterator operator++(int) {
				const_iterator ret = *this;
				++(*this);
				return ret;
			}

			bool operator==(const const_iterator& rhs) const {
				return this->buf == rhs.buf && this->index == rhs.index;
			}

			bool operator!=(const const_iterator& rhs) const {
				return !(*this == rhs);
			}

			const T& operator*() const {
				return (*buf)[index];
			}

		private:
			const CircularBuffer* buf;
			size_t index;
		};

		iterator begin() {
			return iterator(this, 0);
		}

		iterator end() {
			return iterator(this, ::util::CircularBuffer<T>::size());
		}

		const_iterator begin() const {
			return const_iterator(this, 0);
		}
		const_iterator end() const {
			return const_iterator(this, ::util::CircularBuffer<T>::size());
		}

		const_iterator cbegin() const {
			return const_iterator(this, 0);
		}
		const_iterator cend() const {
			return const_iterator(this, ::util::CircularBuffer<T>::size());
		}

		size_t size() const {
			return length;
		}

		size_t max_size() const {
			return data.max_size();
		}

		T& front() {
			return (*this)[0];
		}

		const T& front() const {
			return (*this)[0];
		}

		T& back() {
			return data[(m_front + length - 1) % data.size()];
		}

		const T& back() const {
			return data[(m_front + length - 1) % data.size()];
		}

		void push_front(T val) {
			if (data.empty()) { return; }

			m_front = moduloDec(m_front);
			data[m_front] = val;

			if (length < data.size()) { length++; }
		}

		void push_back(T val) {
			if (data.empty()) { return; }

			data[(m_front + length) % data.size()] = val;
			if (length < data.size()) {
				length++;
			} else {
				m_front = moduloInc(m_front);
			}
		}

		template<typename... Args>
		void emplace_front(Args&&... args) {
			if (data.empty()) { return; }

			m_front = moduloDec(m_front);
			data[m_front] = T{args...};
			if (length < data.size()) { length++; }
		}

		template<typename... Args>
		void emplace_back(Args&&... args) {
			if (data.size() == 0) { return; }

			data[(m_front + length) % data.size()] = T{args...};

			if (length < data.size()) {
				length++;
			} else {
				// Increment m_front if buffer is full to maintain size
				m_front = moduloInc(m_front);
			}
		}

		// UB when nothing at m_front of buffer
		T pop_front() {
			T& temp = data[m_front];
			m_front = moduloInc(m_front);
			length--;
			return temp;
		}

		// UB when nothing in back of buffer
		T pop_back() {
			length--;
			return data[(m_front + length) % data.size()];
		}

		void resize(size_t size) {
			//
		}

		void reset() {
			m_front = 0;
			length = 0;
		}

		T& operator[](size_t index) {
			return data[(m_front + index) % data.size()];
		}

		const T& operator[](size_t index) const {
			return data[(m_front + index) % data.size()];
		}

	private:
		// keep this defined as the first variable
		std::vector<T> data;
		size_t length; // num of elements in buf
		size_t m_front;// index for m_front element

		size_t indexMath(size_t index, size_t amt) {
			return util::normalize(index + amt, data.max_size());
		}

		size_t moduloInc(size_t index) { return (index + 1) % data.size(); }

	size_t moduloDec(size_t index) {
		if (index == 0) {
			return data.size() - 1;
			} else {
				return index - 1;
			}
		}
	};
}// namespace util