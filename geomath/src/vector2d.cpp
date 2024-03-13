#include "geomath.h"

#include <cmath>

using geomath::Vector2D;

double Vector2D::length() const {
	return sqrt(length_squared());
}

double Vector2D::length_squared() const {
	return x*x + y*y;
}

Vector2D Vector2D::normalized() const {
	auto len = length();
	return { x/len, y/len };
}

Vector2D Vector2D::operator+(const Vector2D & rhs) const {
	return { x + rhs.x, y + rhs.y };
}

Vector2D Vector2D::operator+(const double & rhs) const {
	return { x + rhs, y + rhs };
}

Vector2D & Vector2D::operator+=(const Vector2D & rhs) {
	x += rhs.x;
	y += rhs.y;
	return *this;
}

Vector2D & Vector2D::operator+=(const double & rhs) {
	x += rhs;
	y += rhs;
	return *this;
}

Vector2D Vector2D::operator-(const Vector2D & rhs) const {
	return { x - rhs.x, y - rhs.y };
}

Vector2D Vector2D::operator-(const double & rhs) const {
	return { x - rhs, y - rhs };
}

Vector2D & Vector2D::operator-=(const Vector2D & rhs) {
	x -= rhs.x;
	y -= rhs.y;
	return *this;
}

Vector2D & Vector2D::operator-=(const double & rhs) {
	x -= rhs;
	y -= rhs;
	return *this;
}

Vector2D Vector2D::operator*(const Vector2D & rhs) const {
	return { x * rhs.x, y * rhs.y };
}

Vector2D Vector2D::operator*(const double & rhs) const {
	return { x * rhs, y * rhs };
}

Vector2D & Vector2D::operator*=(const Vector2D & rhs) {
	x *= rhs.x;
	y *= rhs.y;
	return *this;
}

Vector2D & Vector2D::operator*=(const double & rhs) {
	x *= rhs;
	y *= rhs;
	return *this;
}

Vector2D Vector2D::operator/(const Vector2D & rhs) const {
	return { x / rhs.x, y / rhs.y };
}

Vector2D Vector2D::operator/(const double & rhs) const {
	return { x / rhs, y / rhs };
}

Vector2D & Vector2D::operator/=(const Vector2D & rhs) {
	x /= rhs.x;
	y /= rhs.y;
	return *this;
}

Vector2D & Vector2D::operator/=(const double & rhs) {
	x /= rhs;
	y /= rhs;
	return *this;
}

Vector2D geomath::operator*(const double & lhs, const Vector2D & rhs) {
	return { lhs * rhs.x, lhs * rhs.y };
}
