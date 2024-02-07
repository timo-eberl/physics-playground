#include "tics.h"

#include <cassert>
#include <iostream>

using tics::ObjectTransform;
using tics::CollisionPoints;
using tics::ColliderType;
using tics::Collider;
using tics::SphereCollider;
using tics::PlaneCollider;

CollisionPoints collision_test_sphere_sphere(
	const Collider& a, const ObjectTransform& ta,
	const Collider& b, const ObjectTransform& tb
) {
	assert(a.type == ColliderType::SPHERE);
	assert(b.type == ColliderType::SPHERE);
	// spheres support only uniform scales
	assert(ta.scale.x == ta.scale.y && ta.scale.y == ta.scale.z);
	assert(tb.scale.x == tb.scale.y && tb.scale.y == tb.scale.z);

	auto& a_collider = static_cast<const SphereCollider&>(a);
	auto& b_collider = static_cast<const SphereCollider&>(b);

	auto a_center = a_collider.center + ta.position;
	auto b_center = b_collider.center + tb.position;

	auto a_radius = ta.scale.x * a_collider.radius;
	auto b_radius = tb.scale.x * b_collider.radius;

	auto ab_vector = b_center - a_center;
	auto ab_distance = ab_vector.length();

	if (ab_distance > a_radius + b_radius) {
		// no collision
		return CollisionPoints();
	}

	auto ab_normal = ab_vector.normalized();

	auto collision_points = CollisionPoints();

	collision_points.has_collision = true;

	collision_points.a = a_center + ab_normal * a_radius;
	collision_points.b = b_center - ab_normal * b_radius;

	collision_points.normal = ab_normal * -1.0;
	collision_points.depth = (collision_points.b - collision_points.a).length();

	return collision_points;
}

CollisionPoints collision_test_sphere_plane(
	const Collider& a, const ObjectTransform& ta,
	const Collider& b, const ObjectTransform& tb
) {
	assert(a.type == ColliderType::SPHERE);
	assert(b.type == ColliderType::PLANE);
	// spheres support only uniform scales
	assert(ta.scale.x == ta.scale.y && ta.scale.y == ta.scale.z);

	auto& sphere_collider = static_cast<const SphereCollider&>(a);
	auto& plane_collider = static_cast<const PlaneCollider&>(b);

	auto sphere_center = sphere_collider.center + ta.position;
	auto sphere_radius = ta.scale.x * sphere_collider.radius;

	auto plane_normal = plane_collider.normal; // TODO: rotate with tb
	auto point_on_plane = plane_normal * plane_collider.distance + tb.position;

	auto distance = plane_normal.dot(sphere_center - point_on_plane);

	if (distance > sphere_radius) {
		// no collision
		return CollisionPoints();
	}

	auto collision_points = CollisionPoints();

	collision_points.has_collision = true;

	collision_points.a = sphere_center - plane_normal * sphere_radius;
	collision_points.b = sphere_center + plane_normal * distance;

	collision_points.normal = plane_normal;
	collision_points.depth = (collision_points.b - collision_points.a).length();

	return collision_points;
}

// define a function type
using FindCollisionTestFunc = CollisionPoints(*)(
	const Collider&, const ObjectTransform&,
	const Collider&, const ObjectTransform&
);

CollisionPoints tics::collision_test(
	const Collider& a, const ObjectTransform& at,
	const Collider& b, const ObjectTransform& bt
) {
	static const FindCollisionTestFunc function_table[2][2] = {
		  // Sphere                     Plane
		{ collision_test_sphere_sphere, collision_test_sphere_plane }, // Sphere
		{ nullptr,                      nullptr                     }  // Plane
	};

	// make sure the colliders are in the correct orders
	bool swap = a.type > b.type;

	auto& sorted_a = swap ? b : a;
	auto& sorted_b = swap ? a : b;
	auto& sorted_at = swap ? bt : at;
	auto& sorted_bt = swap ? at : bt;

	auto collision_test_function = function_table[sorted_a.type][sorted_b.type];
	// check if collision test function is defined for the given colliders
	assert(collision_test_function != nullptr);

	CollisionPoints points = collision_test_function(sorted_a, sorted_at, sorted_b, sorted_bt);
	// if we swapped the input colliders, we also need to swap the result
	if (swap) {
		std::swap(points.a, points.b);
		points.normal *= -1.0f;
	}

	return points;
};
