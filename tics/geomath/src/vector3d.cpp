#include "geomath.h"

using geomath::Vector3D;

Vector3D Vector3D::operator+(const Vector3D & rhs) const {
	return { x + rhs.x, y + rhs.y, z + rhs.z };
}

Vector3D Vector3D::operator+(const float & rhs) const {
	return { x + rhs, y + rhs, z + rhs };
}

Vector3D & Vector3D::operator+=(const Vector3D & rhs) {
	x += rhs.x;
	y += rhs.y;
	z += rhs.z;
	return *this;
}

Vector3D Vector3D::operator-(const Vector3D & rhs) const {
	return { x - rhs.x, y - rhs.y, z - rhs.z };
}

Vector3D Vector3D::operator-(const float & rhs) const {
	return { x - rhs, y - rhs, z - rhs };
}

Vector3D & Vector3D::operator-=(const Vector3D & rhs) {
	x -= rhs.x;
	y -= rhs.y;
	z -= rhs.z;
	return *this;
}

Vector3D Vector3D::operator*(const Vector3D & rhs) const {
	return { x * rhs.x, y * rhs.y, z * rhs.z };
}

Vector3D Vector3D::operator*(const float & rhs) const {
	return { x * rhs, y * rhs, z * rhs };
}

Vector3D & Vector3D::operator*=(const Vector3D & rhs) {
	x *= rhs.x;
	y *= rhs.y;
	z *= rhs.z;
	return *this;
}

Vector3D Vector3D::operator/(const Vector3D & rhs) const {
	return { x / rhs.x, y / rhs.y, z / rhs.z };
}

Vector3D Vector3D::operator/(const float & rhs) const {
	return { x / rhs, y / rhs, z / rhs };
}

Vector3D & Vector3D::operator/=(const Vector3D & rhs) {
	x /= rhs.x;
	y /= rhs.y;
	z /= rhs.z;
	return *this;
}

Vector3D geomath::operator*(const float & lhs, const Vector3D & rhs) {
	return { lhs * rhs.x, lhs * rhs.y, lhs * rhs.z };
}
