#include "geomath.h"

gm::Vector3 generate_random_vec3() {
	return gm::Vector3(
		static_cast<double>(std::rand()) / RAND_MAX,
		static_cast<double>(std::rand()) / RAND_MAX,
		static_cast<double>(std::rand()) / RAND_MAX
	);
}

struct CustomType { char c; };
bool operator==(const CustomType &lhs, const CustomType &rhs) { return lhs.c == rhs.c; }

int main() {
	const auto v0_0 = gm::Vector<5>(1,2,3,4,5);
	assert(v0_0[0] == 1.0f && v0_0[1] == 2.0f && v0_0[2] == 3.0f && v0_0[3] == 4.0f && v0_0[4] == 5.0f);
	const auto v0_1 = gm::Vector<5>(1.0,2.0,3.0,4.0,5.0);
	assert(v0_0 == v0_1);
	const auto v0_2 = gm::Vector<5>(1.0f,2.0f,3.0f,4.0f,5.0f);
	assert(v0_0 == v0_2);
	// initializer list supports initializing with different types that are convertible
	const auto v0_3 = gm::Vector<5>( { 1, 2.0, 3.0f, 4u, 5l } );
	assert(v0_0 == v0_3);
	const auto v0_4 = gm::Vector<5>(gm::Vector3(1,2,3) ,4,5);
	assert(v0_0 == v0_4);

	assert( gm::Vector2(1,2) == gm::Vector2(gm::Vector<1>(1),2) );
	assert( gm::Vector3(1,2,3) == gm::Vector3(gm::Vector<1>(1),2,3) );
	assert( gm::Vector3(1,2,3) == gm::Vector3(gm::Vector2(1,2),3) );
	assert( gm::Vector4(1,2,3,4) == gm::Vector4(gm::Vector<1>(1),2,3,4) );
	assert( gm::Vector4(1,2,3,4) == gm::Vector4(gm::Vector<2>(1,2),3,4) );
	assert( gm::Vector4(1,2,3,4) == gm::Vector4(gm::Vector<3>(1,2,3),4) );

	assert((
		gm::Vector<5,double>(1,2,3,4,5)
		== gm::Vector<5,double>(gm::Vector<5,int>(1,2,3,4,5))
	));

	// constructor that takes same size vector with different (convertible) data type
	assert(( gm::Vector2(1,2) == gm::Vector2(gm::IntVector3(1,2,3).xy) ));
	assert(( gm::Vector3(1,2,3) == gm::Vector3(gm::IntVector3(1,2,3)) ));
	assert(( gm::Vector4(1,2,3,4) == gm::Vector4(gm::IntVector4(1,2,3,4)) ));

	const auto v1_0 = gm::Vector<5>(3,4,5,6,7);
	assert(v0_0 != v1_0);
	auto v1_1 = gm::Vector<5>();
	assert(v1_0 != v1_1);
	v1_1[0] = 3; v1_1[1] = 4; v1_1[2] = 5; v1_1[3] = 6; v1_1[4] = 7;
	assert(v1_0 == v1_1);

	const auto v2_0 = gm::Vector<5>();
	const auto v2_1 = gm::Vector<5>(0);
	assert(v2_0 == v2_1);
	const auto v2_2 = gm::Vector<5>(0.0f);
	assert(v2_0 == v2_2);

	const auto v3_0 = gm::Vector<5>(-3,-3,-3,-3,-3);
	const auto v3_1 = gm::Vector<5>(-3);
	assert(v3_0 == v3_1);

	const auto v4_0 = gm::Vector<5, int>(-3,-3,-3,-3,-3);
	const auto v4_1 = gm::Vector<5, int>(-3.7); // - 3.7 gets converted to -3
	assert(v4_0 == v4_1);

	// v5 - v8: specialized implementations vor Vector2, Vector3, Vector4

	const auto v5_0 = gm::Vector3();
	const auto v5_1 = gm::Vector3(0);
	assert(v5_0 == v5_1);
	const auto v5_2 = gm::Vector3(0,0,0);
	assert(v5_0 == v5_2);
	const auto v5_3 = gm::Vector3( { 0.0, 0, 0u } );
	assert(v5_0 == v5_3);

	const auto v6_0 = gm::Vector4(0,1,2,3);
	assert(v6_0[0] == 0 && v6_0[1] == 1 && v6_0[2] == 2 && v6_0[3] == 3);
	assert(v6_0.xyz != v5_0);

	const auto v7_0 = gm::Vector3(0,1,2);
	const auto v7_1 = v6_0.xyz;
	assert(v7_0 == v7_1);

	const auto v8_0 = gm::Vector2(0,1);
	const auto v8_1 = v6_0.xy;
	assert(v8_0 == v8_1);
	const auto v8_2 = v6_0.xyz.xy;
	assert(v8_0 == v8_2);
	const auto v8_3 = v7_0.xy;
	assert(v8_0 == v8_3);
	auto v8_4 = gm::Vector2();
	assert(v8_0 != v8_4);
	v8_4 = v8_0;
	assert(v8_0 == v8_4);
	auto v8_5 = gm::Vector2();
	assert(v8_0 != v8_5);
	v8_5 = gm::Vector2(0,1);
	assert(v8_0 == v8_5);

	// v9 - v10: approx comparison

	assert(gm::equals_approx(3.0, 3.0001));
	assert(gm::equals_approx(3.0f, 3.0001f));
	assert(!gm::equals_approx(3.0f, 3.1f));

	const auto v9_0 = gm::Vector2(1.0, -2.0);
	const auto v9_1 = gm::Vector2(1.0001, -2.0001);
	assert(gm::equals_approx(v9_0, v9_1));
	assert(!gm::equals_approx(v9_0, v9_1, 0.00005)); // custom accuracy
	const auto v9_2 = gm::Vector2(1.0001, -2.1);
	assert(!gm::equals_approx(v9_0, v9_2));

	assert(gm::is_zero_approx(0.0001));
	assert(!gm::is_zero_approx(0.0001, 0.00005)); // custom accuracy
	assert(!gm::is_zero_approx(0.0001f, 0.00005)); // custom accuracy
	assert(!gm::is_zero_approx(0.0001, 0.00005f)); // custom accuracy
	assert(!gm::is_zero_approx(0.0001, 0));
	assert(!gm::is_zero_approx(0.001));

	const auto v10_0 = gm::Vector<10>(0.0001);
	assert(gm::is_zero_approx(v10_0));
	assert(!gm::is_zero_approx(v10_0, 0)); // custom accuracy
	assert(!gm::is_zero_approx(v10_0, 0.00005f)); // custom accuracy
	assert(!gm::is_zero_approx(v10_0, 0.00005)); // custom accuracy
	const auto v10_1 = gm::Vector<10>(0.1);
	assert(!gm::is_zero_approx(v10_1));
	const auto v10_2 = gm::Vector<10>(-0.1f);
	assert(!gm::is_zero_approx(v10_2));
	const auto v10_3 = gm::Vector<10, double>(0.0001);
	assert(gm::is_zero_approx(v10_3));

	// v11 - v15: component-wise math operations

	const auto v11_0 = gm::Vector2(2.234, 4.234);
	const auto v11_1 = gm::Vector2(2, 4) + gm::Vector2(0.234);
	assert(gm::equals_approx(v11_0, v11_1));
	const auto v11_2 = gm::Vector<2, double>(2, 4) + 0.234f; // add float to double vector
	assert(gm::equals_approx(v11_0, v11_2));
	const auto v11_3 = 0.234f + gm::Vector2(2, 4);
	assert(gm::equals_approx(v11_0, v11_3));
	auto v11_4 = gm::Vector2(2, 4);
	assert(!gm::equals_approx(v11_0, v11_4));
	v11_4 += gm::Vector2(0.234);
	assert(gm::equals_approx(v11_0, v11_4));
	auto v11_5 = gm::Vector2(2, 4);
	assert(!gm::equals_approx(v11_0, v11_5));
	v11_5 += 0.234f;
	assert(gm::equals_approx(v11_0, v11_5));

	const auto v12_0 = gm::Vector2(2.234, 4.234);
	const auto v12_1 = gm::Vector2(3, 5) - 0.766f;
	assert(gm::equals_approx(v12_0, v12_1));
	auto v12_2 = gm::Vector2(-2.234, -4.234);
	assert(!gm::equals_approx(v12_0, v12_2));
	v12_2 = -v12_2; // unary negation
	assert(gm::equals_approx(v12_0, v12_2));

	const auto v13_0 = gm::Vector2(9, 12);
	const auto v13_1 = gm::Vector2(3, 4) * 3.0f;
	assert(gm::equals_approx(v13_0, v13_1));

	const auto v14_0 = gm::Vector2(12, 20);
	const auto v14_1 = gm::Vector2(3, 4) * gm::Vector2(4, 5);
	assert(gm::equals_approx(v14_0, v14_1));

	const auto v15_0 = gm::Vector2(3, 4);
	const auto v15_1 = gm::Vector2(12, 20) / gm::Vector2(4, 5);
	assert(gm::equals_approx(v15_0, v15_1));
	auto v15_2 = gm::Vector2(12, 20);
	assert(!gm::equals_approx(v15_0, v15_2));
	v15_2 /= gm::Vector2(4, 5);
	assert(gm::equals_approx(v15_0, v15_2));

	// default initialized vector should be all zeros
	const auto v16_0 = gm::Vector<10000>();
	for (size_t i = 0; i < 10000; i++) {
		assert(v16_0[i] == 0);
	}

	assert(gm::equals_approx(gm::length(gm::Vector3(1, 2, 3)), 3.7416573867739413));
	assert(gm::equals_approx(gm::length(gm::Vector<5,float>(3,6,3,5,3)), 9.38083151964686f));

	for (size_t i = 0; i < 10000; i++) {
		const auto random_vector_0 = generate_random_vec3();
		const auto random_vector_1 = generate_random_vec3();
		// if length_squared of one vector is greater than of the other vector
		// the same is true for length
		assert(
			gm::length(random_vector_0) > gm::length(random_vector_1)
			== gm::length_squared(random_vector_0) > gm::length_squared(random_vector_1)
		);
	}

	const auto v18_0 = gm::Vector3(-5.2,3.8,10);
	assert(gm::equals_approx(gm::distance(v18_0, v18_0), 0.0));
	const auto v18_1 = gm::Vector3(8.7,-4.1,3);
	assert(gm::equals_approx(gm::distance(v18_0, v18_1), 17.4534));

	const auto dist_squared_0 = gm::distance_squared(
		gm::Vector3(4), gm::Vector3(4,4,3.8)
	);
	const auto dist_squared_1 = gm::distance_squared(
		gm::Vector3(4), gm::Vector3(4,4,3.9)
	);
	assert(dist_squared_0 > dist_squared_1);

	{ // lerp and inverse lerp
		const auto a = gm::Vector2(-2, 10);
		const auto b = gm::Vector2(2, 20);
		const auto v = gm::lerp(a, b, 0.35);
		assert(gm::equals_approx(v, gm::Vector2(-0.6, 13.5)));
		const auto t = gm::inverse_lerp(a, b, 13.5);
		assert(gm::equals_approx(t, gm::Vector2(3.875, 0.35)));
		const auto v2 = gm::lerp(a, b, gm::Vector2(0.5, 0.35));
		assert(gm::equals_approx(v2, gm::Vector2(0.0,13.5)));
		const auto t2 = gm::inverse_lerp(a, b, gm::Vector2(-0.6, 13.5));
		assert(gm::equals_approx(t2, gm::Vector2(0.35)));
	}

	assert( gm::max(gm::Vector2(-1,3), gm::Vector2(2,1)) == gm::Vector2(2,3) );
	assert( gm::min(gm::Vector2(-1,3), gm::Vector2(2,1)) == gm::Vector2(-1,1) );
	assert( gm::clamp(gm::Vector2(-2, 3), -1.0, 1.0) == gm::Vector2(-1, 1) );
	assert( gm::clamp(gm::Vector2(-2, 3), -1, 1) == gm::Vector2(-1, 1) );
	assert(
		gm::clamp(gm::Vector2(-2, 3), gm::Vector2(-1, 1), gm::Vector2(1, 2))
		== gm::Vector2(-1, 2)
	);
	assert( gm::clamp01(gm::Vector2(-2.0, 0.5)) == gm::Vector2(0.0, 0.5) );

	assert( gm::cross(gm::Vector3(1,0,0), gm::Vector3(0,1,0)) == gm::Vector3(0,0,1) );
	assert( gm::cross(gm::Vector3(2,0,0), gm::Vector3(0,3,0)) == gm::Vector3(0,0,6) );
	assert( gm::cross(gm::Vector3(1,2,3), gm::Vector3(3,4,5)) == gm::Vector3(-2,4,-2) );

	// .xy access for all data types
	assert( ( gm::Vector<3, int>(3,4,5).xy == gm::Vector<2, int>(3,4) ) );
	assert( ( gm::Vector<4, CustomType>().xyz == gm::Vector<3, CustomType>() ) );
}
