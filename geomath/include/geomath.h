#pragma once

#include <cstddef>
#include <cassert>
#include <initializer_list>
#include <algorithm>
#include <unordered_set>
#include <ostream>
#include <cmath>

namespace gm {

// todo matrices constructors
// todo matrix comparison
// todo matrix <<

struct Quaternion {
	double i;
	double j;
	double k;
	double w;
};

template <unsigned int n, typename T = float>
struct Vector {
	static_assert(n > 0, "n must be greater than 0");

	T data[n];

	explicit Vector() = default;

	explicit constexpr Vector(T value) { for (size_t i = 0; i < n; i++) { data[i] = value; } }

	template <typename ... Ts>
	explicit constexpr Vector(Ts ... vals) {
		static_assert(std::conjunction<std::is_convertible<T,Ts>...>::value, "incompatible type");
		static_assert(sizeof...(vals) == n, "incorrect number of arguments");

		unsigned int i = 0;
		for (const auto element : {vals...}) {
			data[i] = element;
			i++;
		}
	}

	explicit constexpr Vector(std::initializer_list<T> l) {
		assert(l.size() == n);
		unsigned int i = 0;
		for (const auto element : l) {
			data[i] = element;
			i++;
		}
	}

	T &operator[](std::size_t idx) { assert(idx < n); return data[idx]; }
	const T &operator[](std::size_t idx) const { assert(idx < n); return data[idx]; }
};

template <unsigned int rows, unsigned int cols, typename T = float>
struct Matrix {
	static_assert(rows > 0 && cols > 0, "rows and cols must be greater than 0");
	union {
		T data[rows][cols];
		T contiguous_data[rows * cols];
	};
};

typedef Vector<2, float> Vector2;
typedef Vector<3, float> Vector3;
typedef Vector<4, float> Vector4;
typedef Matrix<2,2, float> Matrix2;
typedef Matrix<3,3, float> Matrix3;
typedef Matrix<4,4, float> Matrix4;

template <> struct Vector<2, float> {
	union {
		float data[2];
		struct { float x, y; };
	};

	// template specializations don't inherit member functions - we have to re-implement them
	explicit Vector() = default;
	explicit constexpr Vector(float value) { data[0] = value; data[1] = value; }
	explicit constexpr Vector(float t0, float t1) { data[0] = t0; data[1] = t1; }
	explicit constexpr Vector(std::initializer_list<float> l) {
		assert(l.size() == 2);
		unsigned int i = 0;
		for (const auto element : l) {
			data[i] = element;
			i++;
		}
	}
	float &operator[](std::size_t idx) { assert(idx < 2); return data[idx]; }
	const float &operator[](std::size_t idx) const { assert(idx < 2); return data[idx]; }
};

template <> struct Vector<3, float> {
	union {
		float data[3];
		struct { float x, y, z; };
		Vector<2> xy;
	};
	// template specializations don't inherit mumber functions - we have to re-implement them
	explicit Vector() = default;
	explicit constexpr Vector(float value) { data[0] = value; data[1] = value; data[2] = value; }
	explicit constexpr Vector(float t0, float t1, float t2) { data[0] = t0; data[1] = t1; data[2] = t2; }
	explicit constexpr Vector(std::initializer_list<float> l) {
		assert(l.size() == 3);
		unsigned int i = 0;
		for (const auto element : l) {
			data[i] = element;
			i++;
		}
	}
	float &operator[](std::size_t idx) { assert(idx < 3); return data[idx]; }
	const float &operator[](std::size_t idx) const { assert(idx < 3); return data[idx]; }
};

template <> struct Vector<4, float> {
	union {
		float data[4];
		struct { float x, y, z, w; };
		Vector<2> xy;
		Vector<3> xyz;
	};
	// template specializations don't inherit mumber functions - we have to re-implement them
	explicit Vector() = default;
	explicit constexpr Vector(float value) { data[0] = value; data[1] = value; data[2] = value; data[3] = value; }
	explicit constexpr Vector(float t0, float t1, float t2, float t3) {
		data[0] = t0; data[1] = t1; data[2] = t2; data[3] = t3;
	}
	explicit constexpr Vector(std::initializer_list<float> l) {
		assert(l.size() == 4);
		unsigned int i = 0;
		for (const auto element : l) {
			data[i] = element;
			i++;
		}
	}
	float &operator[](std::size_t idx) { assert(idx < 4); return data[idx]; }
	const float &operator[](std::size_t idx) const { assert(idx < 4); return data[idx]; }
};

// free functions

template <unsigned int n, typename T>
bool operator==(const Vector<n, T> &lhs, const Vector<n, T> &rhs) {
	for (size_t i = 0; i < n; i++) { if (lhs[i] != rhs[i]) { return false; } }
	return true;
}

template <unsigned int n, typename T>
std::ostream &operator<<(std::ostream &os, const Vector<n, T> &value) {
	os << "{ ";
	for (size_t i = 0; i < n; i++) {
		os << value[i];
		if (i < n-1) { os << ", "; }
	}
	os << " }";
	return os;
}

// mathematical operations

template <unsigned int n, typename Floating,
	std::enable_if_t<std::is_floating_point<Floating>::value, bool> = true>
Floating length_squared(const Vector<n, Floating> &value) {
	Floating ls = 0.0;
	for (size_t i = 0; i < n; i++) { ls += value[i] * value[i]; }
	return ls;
}

template <unsigned int n, typename Floating,
	std::enable_if_t<std::is_floating_point<Floating>::value, bool> = true>
Floating length(const Vector<n, Floating> &value) { return sqrt(length_squared(value)); }

template <unsigned int n, typename Floating,
	std::enable_if_t<std::is_floating_point<Floating>::value, bool> = true>
Vector<n, Floating> normalize(const Vector<n, Floating> &value) {
	return value / length(value);
}

template <typename Floating,
	std::enable_if_t<std::is_floating_point<Floating>::value, bool> = true>
Floating dot(const Vector<3, Floating> &lhs, const Vector<3, Floating> &rhs) {
	return (lhs[0]*rhs[0]) + (lhs[1]*rhs[1]) + (lhs[2]*rhs[2]);
}

template <unsigned int n, typename Floating,
	std::enable_if_t<std::is_floating_point<Floating>::value, bool> = true>
bool is_zero_approx(const Vector<n, Floating> &value, const Floating epsilon) {
	for (size_t i = 0; i < n; i++) { if (abs(value[i]) >= epsilon) return false; }
	return true;
}

// add

template <unsigned int n, typename T>
Vector<n, T> operator+(const Vector<n, T> &lhs, const T &rhs) {
	Vector<n, T> v;
	for (size_t i = 0; i < n; i++) { v[i] = lhs[i] + rhs; }
	return v;
}

template <unsigned int n, typename T>
Vector<n, T> operator+(const T &lhs, const Vector<n, T> &rhs) { return rhs + lhs; }

template <unsigned int n, typename T>
Vector<n, T> operator+(const Vector<n, T> &lhs, const Vector<n, T> &rhs) {
	Vector<n, T> v;
	for (size_t i = 0; i < n; i++) { v[i] = lhs[i] + rhs[i]; }
	return v;
}

template <unsigned int n, typename T>
Vector<n, T> &operator+=(Vector<n, T> &lhs, const Vector<n, T> &rhs) { lhs = lhs + rhs; return lhs; }

template <unsigned int n, typename T>
Vector<n, T> &operator+=(Vector<n, T> &lhs, const T &rhs) { lhs = lhs + rhs; return lhs; }

// unary negation

template <unsigned int n, typename T>
Vector<n, T> operator-(const Vector<n, T> &value) {
	Vector<n, T> v;
	for (size_t i = 0; i < n; i++) { v[i] = -value[i]; }
	return v;
}

// subtract

template <unsigned int n, typename T>
Vector<n, T> operator-(const Vector<n, T> &lhs, const T &rhs) {
	Vector<n, T> v;
	for (size_t i = 0; i < n; i++) { v[i] = lhs[i] - rhs; }
	return v;
}

template <unsigned int n, typename T>
Vector<n, T> operator-(const T &lhs, const Vector<n, T> &rhs) { return -rhs + lhs; }

template <unsigned int n, typename T>
Vector<n, T> operator-(const Vector<n, T> &lhs, const Vector<n, T> &rhs) {
	Vector<n, T> v;
	for (size_t i = 0; i < n; i++) { v[i] = lhs[i] - rhs[i]; }
	return v;
}

template <unsigned int n, typename T>
Vector<n, T> &operator-=(Vector<n, T> &lhs, const Vector<n, T> &rhs) { lhs = lhs - rhs; return lhs; }

template <unsigned int n, typename T>
Vector<n, T> &operator-=(Vector<n, T> &lhs, const T &rhs) { lhs = lhs - rhs; return lhs; }

// multiply

template <unsigned int n, typename T>
Vector<n, T> operator*(const Vector<n, T> &lhs, const T &rhs) {
	Vector<n, T> v;
	for (size_t i = 0; i < n; i++) { v[i] = lhs[i] * rhs; }
	return v;
}

template <unsigned int n, typename T>
Vector<n, T> operator*(const T &lhs, const Vector<n, T> &rhs) { return rhs * lhs; }

template <unsigned int n, typename T>
Vector<n, T> operator*(const Vector<n, T> &lhs, const Vector<n, T> &rhs) {
	Vector<n, T> v;
	for (size_t i = 0; i < n; i++) { v[i] = lhs[i] * rhs[i]; }
	return v;
}

template <unsigned int n, typename T>
Vector<n, T> &operator*=(Vector<n, T> &lhs, const Vector<n, T> &rhs) { lhs = lhs * rhs; return lhs; }

template <unsigned int n, typename T>
Vector<n, T> &operator*=(Vector<n, T> &lhs, const T &rhs) { lhs = lhs * rhs; return lhs; }

// division

template <unsigned int n, typename T>
Vector<n, T> operator/(const Vector<n, T> &lhs, const T &rhs) {
	Vector<n, T> v;
	for (size_t i = 0; i < n; i++) { v[i] = lhs[i] / rhs; }
	return v;
}

template <unsigned int n, typename T>
Vector<n, T> operator/(const T &lhs, const Vector<n, T> &rhs) {
	Vector<n, T> v;
	for (size_t i = 0; i < n; i++) { v[i] = lhs / rhs[i]; }
	return v;
}

template <unsigned int n, typename T>
Vector<n, T> operator/(const Vector<n, T> &lhs, const Vector<n, T> &rhs) {
	Vector<n, T> v;
	for (size_t i = 0; i < n; i++) { v[i] = lhs[i] / rhs[i]; }
	return v;
}

template <unsigned int n, typename T>
Vector<n, T> &operator/=(Vector<n, T> &lhs, const Vector<n, T> &rhs) { lhs = lhs / rhs; return lhs; }

template <unsigned int n, typename T>
Vector<n, T> &operator/=(Vector<n, T> &lhs, T &rhs) { lhs = lhs / rhs; return lhs; }

} // gm
