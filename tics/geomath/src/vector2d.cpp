#include "geomath.h"

using geomath::Vector2D;

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

Vector2D geomath::operator*(const double & lhs, const Vector2D & rhs) {
	return { lhs * rhs.x, lhs * rhs.y };
}
