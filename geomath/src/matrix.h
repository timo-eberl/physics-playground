#pragma once

namespace gm {

template <unsigned int rows, unsigned int cols, typename T = double>
struct Matrix {
	static_assert(rows > 0 && cols > 0, "rows and cols must be greater than 0");
	union {
		T data[rows][cols];
		T contiguous_data[rows * cols];
	};
};

} // gm
