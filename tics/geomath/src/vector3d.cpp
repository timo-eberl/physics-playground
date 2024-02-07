#include "geomath.h"

#include <cmath>

using geomath::Vector3D;

double Vector3D::length() const {
	return sqrt(length_squared());
}

double Vector3D::length_squared() const {
	return x*x + y*y + z*z;
}

Vector3D Vector3D::normalized() const {
	return *this / length();
}

double Vector3D::dot(const Vector3D & rhs) const {
	return (x*rhs.x) + (y*rhs.y) + (z*rhs.z);
}

Vector3D Vector3D::operator+(const Vector3D & rhs) const {
	return { x + rhs.x, y + rhs.y, z + rhs.z };
}

Vector3D Vector3D::operator+(const double & rhs) const {
	return { x + rhs, y + rhs, z + rhs };
}

Vector3D & Vector3D::operator+=(const Vector3D & rhs) {
	x += rhs.x;
	y += rhs.y;
	z += rhs.z;
	return *this;
}

Vector3D & Vector3D::operator+=(const double & rhs) {
	x += rhs;
	y += rhs;
	z += rhs;
	return *this;
}

Vector3D Vector3D::operator-(const Vector3D & rhs) const {
	return { x - rhs.x, y - rhs.y, z - rhs.z };
}

Vector3D Vector3D::operator-(const double & rhs) const {
	return { x - rhs, y - rhs, z - rhs };
}

Vector3D & Vector3D::operator-=(const Vector3D & rhs) {
	x -= rhs.x;
	y -= rhs.y;
	z -= rhs.z;
	return *this;
}

Vector3D & Vector3D::operator-=(const double & rhs) {
	x -= rhs;
	y -= rhs;
	z -= rhs;
	return *this;
}

Vector3D Vector3D::operator*(const Vector3D & rhs) const {
	return { x * rhs.x, y * rhs.y, z * rhs.z };
}

Vector3D Vector3D::operator*(const double & rhs) const {
	return { x * rhs, y * rhs, z * rhs };
}

Vector3D & Vector3D::operator*=(const Vector3D & rhs) {
	x *= rhs.x;
	y *= rhs.y;
	z *= rhs.z;
	return *this;
}

Vector3D & Vector3D::operator*=(const double & rhs) {
	x *= rhs;
	y *= rhs;
	z *= rhs;
	return *this;
}

Vector3D Vector3D::operator/(const Vector3D & rhs) const {
	return { x / rhs.x, y / rhs.y, z / rhs.z };
}

Vector3D Vector3D::operator/(const double & rhs) const {
	return { x / rhs, y / rhs, z / rhs };
}

Vector3D & Vector3D::operator/=(const Vector3D & rhs) {
	x /= rhs.x;
	y /= rhs.y;
	z /= rhs.z;
	return *this;
}

Vector3D & Vector3D::operator/=(const double & rhs) {
	x /= rhs;
	y /= rhs;
	z /= rhs;
	return *this;
}

Vector3D geomath::operator*(const double & lhs, const Vector3D & rhs) {
	return { lhs * rhs.x, lhs * rhs.y, lhs * rhs.z };
}
