#pragma once

namespace geomath {

struct Vector2D {
	float x;
	float y;

	Vector2D operator+(const Vector2D & rhs) const;
	Vector2D operator+(const float & rhs) const;
	Vector2D & operator+=(const Vector2D & rhs);
	Vector2D operator-(const Vector2D & rhs) const;
	Vector2D operator-(const float & rhs) const;
	Vector2D & operator-=(const Vector2D & rhs);
	Vector2D operator*(const Vector2D & rhs) const; // Hadamard product (like in shaders)
	Vector2D operator*(const float & rhs) const;
	Vector2D & operator*=(const Vector2D & rhs);    // Hadamard product (like in shaders)
	Vector2D operator/(const Vector2D & rhs) const;
	Vector2D operator/(const float & rhs) const;
	Vector2D & operator/=(const Vector2D & rhs);
};
Vector2D operator*(const float & lhs, const Vector2D & rhs);

struct Vector3D {
	float x;
	float y;
	float z;

	Vector3D operator+(const Vector3D & rhs) const;
	Vector3D operator+(const float & rhs) const;
	Vector3D & operator+=(const Vector3D & rhs);
	Vector3D operator-(const Vector3D & rhs) const;
	Vector3D operator-(const float & rhs) const;
	Vector3D & operator-=(const Vector3D & rhs);
	Vector3D operator*(const Vector3D & rhs) const; // Hadamard product (like in shaders)
	Vector3D operator*(const float & rhs) const;
	Vector3D & operator*=(const Vector3D & rhs);    // Hadamard product (like in shaders)
	Vector3D operator/(const Vector3D & rhs) const;
	Vector3D operator/(const float & rhs) const;
	Vector3D & operator/=(const Vector3D & rhs);
};
Vector3D operator*(const float & lhs, const Vector3D & rhs);

} // geomath
