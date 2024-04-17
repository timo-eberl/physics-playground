#include "geomath.h"

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
	assert(!gm::equals_approx(v9_0, v9_1, 0.00005f)); // custom accuracy
	const auto v9_2 = gm::Vector2(1.0001, -2.1);
	assert(!gm::equals_approx(v9_0, v9_2));

	assert(gm::is_zero_approx(0.0001));
	assert(!gm::is_zero_approx(0.0001, 0.00005));
	assert(!gm::is_zero_approx(0.001));

	const auto v10_0 = gm::Vector<10>(0.0001);
	assert(gm::is_zero_approx(v10_0));
	assert(!gm::is_zero_approx(v10_0, 0.00005f)); // custom accuracy
	const auto v10_1 = gm::Vector<10>(0.1);
	assert(!gm::is_zero_approx(v10_1));
	const auto v10_2 = gm::Vector<10>(-0.1);
	assert(!gm::is_zero_approx(v10_2));
	const auto v10_3 = gm::Vector<10, double>(0.0001);
	assert(gm::is_zero_approx(v10_3));

	// v11 - v15: component-wise math operations

	const auto v11_0 = gm::Vector2(2.234, 4.234);
	const auto v11_1 = gm::Vector2(2, 4) + gm::Vector2(0.234);
	assert(gm::equals_approx(v11_0, v11_1));
	const auto v11_2 = gm::Vector2(2, 4) + 0.234f;
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
}
