#pragma once

#include <cassert>
#include <initializer_list>
#include <algorithm>
#include <unordered_set>
#include <sstream>
#include <ostream>
#include <cmath>

namespace gm {

template <unsigned int n, typename T = double, std::enable_if_t<(n > 0), bool> = true>
struct Vector {
	T data[n];

	explicit Vector() = default;

	explicit constexpr Vector(T value) { for (size_t i = 0; i < n; i++) { data[i] = value; } }

	template <typename ... Ts,
		std::enable_if_t<std::conjunction<std::is_convertible<T,Ts>...>::value, bool> = true>
	explicit constexpr Vector(Ts ... vals) {
		static_assert(sizeof...(vals) == n || sizeof...(vals) == 1, "incorrect number of arguments");

		if (sizeof...(vals) == 1) { // fill whole array with a single value
			for (const auto element : {vals...}) {
				for (size_t i = 0; i < n; i++) { data[i] = element; }
			}
		}
		else {
			unsigned int i = 0;
			for (const auto element : {vals...}) {
				data[i] = element;
				i++;
			}
		}
	}

	template <typename T2,
		std::enable_if_t<std::is_convertible<T,T2>::value, bool> = true>
	explicit constexpr Vector(const Vector<n,T2> &vec) { for (size_t i = 0; i < n; i++) { data[i] = vec[i]; } }

	explicit constexpr Vector(std::initializer_list<T> l) {
		assert(l.size() == n);
		unsigned int i = 0;
		for (const auto element : l) {
			data[i] = element;
			i++;
		}
	}

	template <unsigned int nv, typename ... Ts,
		std::enable_if_t<std::conjunction<std::is_convertible<T,Ts>...>::value, bool> = true> // is type compatible?
	explicit constexpr Vector(const Vector<nv, T> &vector, Ts ... vals) {
		static_assert(sizeof...(vals) + nv == n, "incorrect number of arguments");

		unsigned int i = 0;
		for (; i < nv; i++) {
			data[i] = vector[i];
		}
		for (const auto element : {vals...}) {
			data[i] = element;
			i++;
		}
	}

	T &operator[](std::size_t idx) { assert(idx < n); return data[idx]; }
	const T &operator[](std::size_t idx) const { assert(idx < n); return data[idx]; }
};

// vector specializations for 2D, 3D, 4D to enable member access like this: vec.x vec.y vec.z vec.xy

template <typename T> struct Vector<2, T> {
	union {
		T data[2];
		struct { T x, y; };
	};

	// template specializations don't inherit member functions - we have to re-implement them
	explicit Vector() = default;
	explicit constexpr Vector(T value) { data[0] = value; data[1] = value; }
	explicit constexpr Vector(T t0, T t1) { data[0] = t0; data[1] = t1; }
	template <typename T2, std::enable_if_t<std::is_convertible<T,T2>::value, bool> = true>
	explicit constexpr Vector(const Vector<2,T2> &vec) { for (size_t i = 0; i < 2; i++) { data[i] = vec[i]; } }
	explicit constexpr Vector(std::initializer_list<T> l) {
		assert(l.size() == 2);
		unsigned int i = 0;
		for (const auto element : l) {
			data[i] = element;
			i++;
		}
	}
	explicit constexpr Vector(Vector<1,T> t0, T t1) { data[0] = t0[0]; data[1] = t1; }
	T &operator[](std::size_t idx) { assert(idx < 2); return data[idx]; }
	const T &operator[](std::size_t idx) const { assert(idx < 2); return data[idx]; }
};

template <typename T> struct Vector<3, T> {
	union {
		T data[3];
		struct { T x, y, z; };
		Vector<2, T> xy;
	};
	// template specializations don't inherit mumber functions - we have to re-implement them
	explicit Vector() = default;
	explicit constexpr Vector(T value) { data[0] = value; data[1] = value; data[2] = value; }
	explicit constexpr Vector(T t0, T t1, T t2) { data[0] = t0; data[1] = t1; data[2] = t2; }
	template <typename T2, std::enable_if_t<std::is_convertible<T,T2>::value, bool> = true>
	explicit constexpr Vector(const Vector<3,T2> &vec) { for (size_t i = 0; i < 3; i++) { data[i] = vec[i]; } }
	explicit constexpr Vector(std::initializer_list<T> l) {
		assert(l.size() == 3);
		unsigned int i = 0;
		for (const auto element : l) {
			data[i] = element;
			i++;
		}
	}
	explicit constexpr Vector(Vector<1,T> t0, T t1, T t2) { data[0] = t0[0]; data[1] = t1; data[2] = t2; }
	explicit constexpr Vector(Vector<2,T> t01, T t2) { data[0] = t01[0]; data[1] = t01[1]; data[2] = t2; }
	T &operator[](std::size_t idx) { assert(idx < 3); return data[idx]; }
	const T &operator[](std::size_t idx) const { assert(idx < 3); return data[idx]; }
};

template <typename T> struct Vector<4, T> {
	union {
		T data[4];
		struct { T x, y, z, w; };
		Vector<2, T> xy;
		Vector<3, T> xyz;
	};
	// template specializations don't inherit mumber functions - we have to re-implement them
	explicit Vector() = default;
	explicit constexpr Vector(T value) { data[0] = value; data[1] = value; data[2] = value; data[3] = value; }
	explicit constexpr Vector(T t0, T t1, T t2, T t3) { data[0] = t0; data[1] = t1; data[2] = t2; data[3] = t3; }
	template <typename T2, std::enable_if_t<std::is_convertible<T,T2>::value, bool> = true>
	explicit constexpr Vector(const Vector<4,T2> &vec) { for (size_t i = 0; i < 4; i++) { data[i] = vec[i]; } }
	explicit constexpr Vector(std::initializer_list<T> l) {
		assert(l.size() == 4);
		unsigned int i = 0;
		for (const auto element : l) {
			data[i] = element;
			i++;
		}
	}
	explicit constexpr Vector(Vector<1,T> t0, T t1, T t2, T t3) { data[0]=t0[0]; data[1]=t1; data[2]=t2; data[3]=t3; }
	explicit constexpr Vector(Vector<2,T> t01, T t2, T t3) { data[0]=t01[0]; data[1]=t01[1]; data[2]=t2; data[3]=t3; }
	explicit constexpr Vector(Vector<3,T> t012, T t3) { data[0]=t012[0]; data[1]=t012[1]; data[2]=t012[2]; data[3]=t3; }
	T &operator[](std::size_t idx) { assert(idx < 4); return data[idx]; }
	const T &operator[](std::size_t idx) const { assert(idx < 4); return data[idx]; }
};

// free functions

template <unsigned int n, typename T>
bool operator==(const Vector<n, T> &lhs, const Vector<n, T> &rhs) {
	for (size_t i = 0; i < n; i++) { if (lhs[i] != rhs[i]) { return false; } }
	return true;
}

template <unsigned int n, typename T>
std::string to_string(const Vector<n, T> &value) {
	std::stringstream stream;
	stream << "{ ";
	for (size_t i = 0; i < n; i++) {
		stream << value[i];
		if (i < n-1) { stream << ", "; }
	}
	stream << " }";
	return stream.str();
}

template <unsigned int n, typename T>
std::ostream &operator<<(std::ostream &os, const Vector<n, T> &value) { os << to_string(value); return os; }

// mathematical operations

template <unsigned int n, typename F,
	std::enable_if_t<std::is_floating_point<F>::value, bool> = true>
F length_squared(const Vector<n, F> &value) {
	F ls = 0.0;
	for (size_t i = 0; i < n; i++) { ls += value[i] * value[i]; }
	return ls;
}

template <unsigned int n, typename F,
	std::enable_if_t<std::is_floating_point<F>::value, bool> = true>
F length(const Vector<n, F> &value) { return sqrt(length_squared(value)); }

template <unsigned int n, typename F,
	std::enable_if_t<std::is_floating_point<F>::value, bool> = true>
F distance(const Vector<n, F> &lhs, const Vector<n, F> &rhs) { return length(lhs - rhs); }

template <unsigned int n, typename F,
	std::enable_if_t<std::is_floating_point<F>::value, bool> = true>
F distance_squared(const Vector<n, F> &lhs, const Vector<n, F> &rhs) { return length_squared(lhs - rhs); }

template <unsigned int n, typename F,
	std::enable_if_t<std::is_floating_point<F>::value, bool> = true>
Vector<n, F> normalize(const Vector<n, F> &value) {
	return value / length(value);
}

template <typename F,
	std::enable_if_t<std::is_floating_point<F>::value, bool> = true>
F dot(const Vector<3, F> &lhs, const Vector<3, F> &rhs) {
	return (lhs[0]*rhs[0]) + (lhs[1]*rhs[1]) + (lhs[2]*rhs[2]);
}

template <typename F>
Vector<3, F> cross(const Vector<3, F> &a, const Vector<3, F> &b) {
	return Vector<3, F>(
		a.y*b.z - a.z*b.y,
		a.z*b.x - a.x*b.z,
		a.x*b.y - a.y*b.x 
	);
}

template <unsigned int n, typename F, typename FT,
	std::enable_if_t<std::is_floating_point<FT>::value, bool> = true,
	std::enable_if_t<std::is_floating_point<F>::value, bool> = true>
Vector<n, F> lerp(const Vector<n, F> &a, const Vector<n, F> &b, const FT t) { return a + t*(b-a); }

template <unsigned int n, typename F, typename FT,
	std::enable_if_t<std::is_floating_point<FT>::value, bool> = true,
	std::enable_if_t<std::is_floating_point<F>::value, bool> = true>
Vector<n, F> lerp(const Vector<n, F> &a, const Vector<n, F> &b, const Vector<n, FT> &t) { return a + t*(b-a); }

template <unsigned int n, typename F, typename FT,
	std::enable_if_t<std::is_floating_point<FT>::value, bool> = true,
	std::enable_if_t<std::is_floating_point<F>::value, bool> = true>
Vector<n, F> inverse_lerp(const Vector<n, F> &a, const Vector<n, F> &b, const FT v) {
	return (v-a) / (b-a);
}

template <unsigned int n, typename F, typename FT,
	std::enable_if_t<std::is_floating_point<FT>::value, bool> = true,
	std::enable_if_t<std::is_floating_point<F>::value, bool> = true>
Vector<n, F> inverse_lerp(const Vector<n, F> &a, const Vector<n, F> &b, const Vector<n, FT> &v) {
	return (v-a) / (b-a);
}

template <unsigned int n, typename F, typename FC,
	std::enable_if_t<std::is_floating_point<FC>::value || std::is_integral<FC>::value, bool> = true,
	std::enable_if_t<std::is_floating_point<F>::value, bool> = true>
Vector<n, F> max(const Vector<n, F> &a, const FC &b) {
	Vector<n, F> v;
	for (size_t i = 0; i < n; i++) { v[i] = a[i] > b ? a[i] : b; }
	return v;
}

template <unsigned int n, typename F, typename FC,
	std::enable_if_t<std::is_floating_point<FC>::value || std::is_integral<FC>::value, bool> = true,
	std::enable_if_t<std::is_floating_point<F>::value, bool> = true>
Vector<n, F> max(const FC &a, const Vector<n, F> &b) { return max(b, a); }

template <unsigned int n, typename F,
	std::enable_if_t<std::is_floating_point<F>::value, bool> = true>
Vector<n, F> max(const Vector<n, F> &a, const Vector<n, F> &b) {
	Vector<n, F> v;
	for (size_t i = 0; i < n; i++) { v[i] = a[i] > b[i] ? a[i] : b[i]; }
	return v;
}

template <unsigned int n, typename F, typename FC,
	std::enable_if_t<std::is_floating_point<FC>::value || std::is_integral<FC>::value, bool> = true,
	std::enable_if_t<std::is_floating_point<F>::value, bool> = true>
Vector<n, F> min(const Vector<n, F> &a, const FC &b) {
	Vector<n, F> v;
	for (size_t i = 0; i < n; i++) { v[i] = a[i] < b ? a[i] : b; }
	return v;
}

template <unsigned int n, typename F, typename FC,
	std::enable_if_t<std::is_floating_point<FC>::value || std::is_integral<FC>::value, bool> = true,
	std::enable_if_t<std::is_floating_point<F>::value, bool> = true>
Vector<n, F> min(const FC &a, const Vector<n, F> &b) { return min(b, a); }

template <unsigned int n, typename F,
	std::enable_if_t<std::is_floating_point<F>::value, bool> = true>
Vector<n, F> min(const Vector<n, F> &a, const Vector<n, F> &b) {
	Vector<n, F> v;
	for (size_t i = 0; i < n; i++) { v[i] = a[i] < b[i] ? a[i] : b[i]; }
	return v;
}

template <unsigned int n, typename F,
	std::enable_if_t<std::is_floating_point<F>::value, bool> = true>
Vector<n, F> clamp01(const Vector<n, F> &v) { return clamp(v, 0, 1); }

template <unsigned int n, typename F, typename FC,
	std::enable_if_t<std::is_floating_point<FC>::value || std::is_integral<FC>::value, bool> = true,
	std::enable_if_t<std::is_floating_point<F>::value, bool> = true>
Vector<n, F> clamp(const Vector<n, F> &v, const FC &min, const FC &max) {
	return gm::max(min, gm::min(max, v));
}

template <unsigned int n, typename F,
	std::enable_if_t<std::is_floating_point<F>::value, bool> = true>
Vector<n, F> clamp(const Vector<n, F> &v, const Vector<n, F> &min, const Vector<n, F> &max) {
	return gm::max(min, gm::min(max, v));
}

template <unsigned int n, typename F, typename FE = F,
	std::enable_if_t<std::is_floating_point<F>::value, bool> = true>
bool is_zero_approx(const Vector<n, F> &value, const FE epsilon = 0.001) {
	for (size_t i = 0; i < n; i++) { if (std::abs(value[i]) >= epsilon) return false; }
	return true;
}

template <typename F, typename FE = F,
	std::enable_if_t<std::is_floating_point<F>::value, bool> = true>
bool is_zero_approx(const F &value, const FE epsilon = 0.001) { return std::abs(value) < epsilon; }

template <unsigned int n, typename F, typename FE = F,
	std::enable_if_t<std::is_floating_point<F>::value, bool> = true>
bool equals_approx(const Vector<n, F> &lhs, const Vector<n, F> &rhs, const FE epsilon = 0.001) {
	for (size_t i = 0; i < n; i++) { if (std::abs(lhs[i] - rhs[i]) >= epsilon) return false; }
	return true;
}

template <typename F, typename FE = F,
	std::enable_if_t<std::is_floating_point<F>::value, bool> = true>
bool equals_approx(const F &lhs, const F &rhs, const FE epsilon = 0.001) {
	return std::abs(rhs - lhs) < epsilon;
}

// add

template <unsigned int n, typename TVec, typename TVal>
Vector<n, TVec> operator+(const Vector<n, TVec> &lhs, const TVal &rhs) {
	Vector<n, TVec> v;
	for (size_t i = 0; i < n; i++) { v[i] = lhs[i] + rhs; }
	return v;
}

template <unsigned int n, typename TVec, typename TVal>
Vector<n, TVec> operator+(const TVal &lhs, const Vector<n, TVec> &rhs) { return rhs + lhs; }

template <unsigned int n, typename T>
Vector<n, T> operator+(const Vector<n, T> &lhs, const Vector<n, T> &rhs) {
	Vector<n, T> v;
	for (size_t i = 0; i < n; i++) { v[i] = lhs[i] + rhs[i]; }
	return v;
}

template <unsigned int n, typename T>
Vector<n, T> &operator+=(Vector<n, T> &lhs, const Vector<n, T> &rhs) { lhs = lhs + rhs; return lhs; }

template <unsigned int n, typename TVec, typename TVal>
Vector<n, TVec> &operator+=(Vector<n, TVec> &lhs, const TVal &rhs) { lhs = lhs + rhs; return lhs; }

// unary negation

template <unsigned int n, typename T>
Vector<n, T> operator-(const Vector<n, T> &value) {
	Vector<n, T> v;
	for (size_t i = 0; i < n; i++) { v[i] = -value[i]; }
	return v;
}

// subtract

template <unsigned int n, typename TVec, typename TVal>
Vector<n, TVec> operator-(const Vector<n, TVec> &lhs, const TVal &rhs) {
	Vector<n, TVec> v;
	for (size_t i = 0; i < n; i++) { v[i] = lhs[i] - rhs; }
	return v;
}

template <unsigned int n, typename TVec, typename TVal>
Vector<n, TVec> operator-(const TVal &lhs, const Vector<n, TVec> &rhs) { return -rhs + lhs; }

template <unsigned int n, typename T>
Vector<n, T> operator-(const Vector<n, T> &lhs, const Vector<n, T> &rhs) {
	Vector<n, T> v;
	for (size_t i = 0; i < n; i++) { v[i] = lhs[i] - rhs[i]; }
	return v;
}

template <unsigned int n, typename T>
Vector<n, T> &operator-=(Vector<n, T> &lhs, const Vector<n, T> &rhs) { lhs = lhs - rhs; return lhs; }

template <unsigned int n, typename TVec, typename TVal>
Vector<n, TVec> &operator-=(Vector<n, TVec> &lhs, const TVal &rhs) { lhs = lhs - rhs; return lhs; }

// multiply

template <unsigned int n, typename TVec, typename TVal>
Vector<n, TVec> operator*(const Vector<n, TVec> &lhs, const TVal &rhs) {
	Vector<n, TVec> v;
	for (size_t i = 0; i < n; i++) { v[i] = lhs[i] * rhs; }
	return v;
}

template <unsigned int n, typename TVec, typename TVal>
Vector<n, TVec> operator*(const TVal &lhs, const Vector<n, TVec> &rhs) { return rhs * lhs; }

template <unsigned int n, typename T>
Vector<n, T> operator*(const Vector<n, T> &lhs, const Vector<n, T> &rhs) {
	Vector<n, T> v;
	for (size_t i = 0; i < n; i++) { v[i] = lhs[i] * rhs[i]; }
	return v;
}

template <unsigned int n, typename T>
Vector<n, T> &operator*=(Vector<n, T> &lhs, const Vector<n, T> &rhs) { lhs = lhs * rhs; return lhs; }

template <unsigned int n, typename TVec, typename TVal>
Vector<n, TVec> &operator*=(Vector<n, TVec> &lhs, const TVal &rhs) { lhs = lhs * rhs; return lhs; }

// division

template <unsigned int n, typename TVec, typename TVal>
Vector<n, TVec> operator/(const Vector<n, TVec> &lhs, const TVal &rhs) {
	Vector<n, TVec> v;
	for (size_t i = 0; i < n; i++) { v[i] = lhs[i] / rhs; }
	return v;
}

template <unsigned int n, typename TVec, typename TVal>
Vector<n, TVec> operator/(const TVal &lhs, const Vector<n, TVec> &rhs) {
	Vector<n, TVec> v;
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

template <unsigned int n, typename TVec, typename TVal>
Vector<n, TVec> &operator/=(Vector<n, TVec> &lhs, TVal &rhs) { lhs = lhs / rhs; return lhs; }

} // gm
