#pragma once

#include "../src/vector.h"
#include "../src/matrix.h"

namespace gm {

// todo matrix constructors
// todo matrix operators: == , << , mat * mat , mat * vec , vec * mat (?)
// todo matrix functions: transpose, inverse, equals_approx
// todo matrix 3d stuff: mat_frustum(left, right,...), mat_look_at, mat_ortho, mat_perspective
// todo matrix transformations: scale, translate, rotate_[x,y,z], rotate_axis
// todo matrix optimized transform constructors (?): mat_from_scaling, mat_from_translation, mat_from_rotation_[x,y,z]

typedef Vector<2, double> Vector2;
typedef Vector<3, double> Vector3;
typedef Vector<4, double> Vector4;
typedef Vector<2, int> IntVector2;
typedef Vector<3, int> IntVector3;
typedef Vector<4, int> IntVector4;

typedef Matrix<2,2, double> Matrix2;
typedef Matrix<3,3, double> Matrix3;
typedef Matrix<4,4, double> Matrix4;

struct Quaternion {
	double i;
	double j;
	double k;
	double w;
};

} // gm
