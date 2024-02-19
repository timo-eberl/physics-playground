#pragma once

namespace geomath {

class Vector2D {
public:
	double x;
	double y;

	double length() const;
	double length_squared() const;
	Vector2D normalized() const;

	Vector2D operator+(const Vector2D & rhs) const;
	Vector2D operator+(const double & rhs) const;
	Vector2D & operator+=(const Vector2D & rhs);
	Vector2D & operator+=(const double & rhs);
	Vector2D operator-(const Vector2D & rhs) const;
	Vector2D operator-(const double & rhs) const;
	Vector2D & operator-=(const Vector2D & rhs);
	Vector2D & operator-=(const double & rhs);
	Vector2D operator*(const Vector2D & rhs) const; // Hadamard product (like in shaders)
	Vector2D operator*(const double & rhs) const;
	Vector2D & operator*=(const Vector2D & rhs);    // Hadamard product (like in shaders)
	Vector2D & operator*=(const double & rhs);
	Vector2D operator/(const Vector2D & rhs) const;
	Vector2D operator/(const double & rhs) const;
	Vector2D & operator/=(const Vector2D & rhs);
	Vector2D & operator/=(const double & rhs);
};
Vector2D operator*(const double & lhs, const Vector2D & rhs);

class Vector3D {
public:
	double x;
	double y;
	double z;

	double length() const;
	double length_squared() const;
	Vector3D normalized() const;
	double dot(const Vector3D & rhs) const;
	bool is_zero() const;
	bool is_zero_approx(const double epsilon) const;

	Vector3D operator+(const Vector3D & rhs) const;
	Vector3D operator+(const double & rhs) const;
	Vector3D & operator+=(const Vector3D & rhs);
	Vector3D & operator+=(const double & rhs);
	Vector3D operator-(const Vector3D & rhs) const;
	Vector3D operator-(const double & rhs) const;
	Vector3D & operator-=(const Vector3D & rhs);
	Vector3D & operator-=(const double & rhs);
	Vector3D operator*(const Vector3D & rhs) const; // Hadamard product (like in shaders)
	Vector3D operator*(const double & rhs) const;
	Vector3D & operator*=(const Vector3D & rhs);    // Hadamard product (like in shaders)
	Vector3D & operator*=(const double & rhs);
	Vector3D operator/(const Vector3D & rhs) const;
	Vector3D operator/(const double & rhs) const;
	Vector3D & operator/=(const Vector3D & rhs);
	Vector3D & operator/=(const double & rhs);
};
Vector3D operator*(const double & lhs, const Vector3D & rhs);

struct Quaternion {
	double i;
	double j;
	double k;
	double w;
};

} // geomath
