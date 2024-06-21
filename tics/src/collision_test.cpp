#include "tics.h"

#include <cassert>
#include <iostream>

using tics::Transform;
using tics::CollisionPoints;
using tics::ColliderType;
using tics::Collider;
using tics::SphereCollider;
using tics::PlaneCollider;
using tics::MeshCollider;

// A support function takes a direction d and returns a point on the boundary of a shape "furthest" in direction d

gm::Vector3 support_function_sphere(
	const Collider &c, const gm::Vector3 &d
) {
	assert(c.type == ColliderType::SPHERE);

	const auto &collider = static_cast<const SphereCollider&>(c);

	return collider.center + d * collider.radius;
}

gm::Vector3 support_function_mesh(
	const Collider &c, const Transform &t, const gm::Vector3 &d
) {
	assert(c.type == ColliderType::MESH);

	const auto &collider = static_cast<const MeshCollider&>(c);

	// translate d into the local space of the shape
	const auto d_local_space = gm::normalize( d / t.scale );

	// find the support point in local space
	auto support_point_dot = 0.0;
	auto support_point = gm::Vector3(0);
	for (const auto &p : collider.positions) {
		const auto p_dot_d = gm::dot(p, d_local_space);
		if (p_dot_d > support_point_dot) {
			support_point_dot = p_dot_d;
			support_point = p;
		}
	}

	// translate back to world space
	support_point = support_point * t.scale + t.position;

	return support_point;
}

CollisionPoints collision_test_sphere_sphere(
	const Collider& a, const Transform& ta,
	const Collider& b, const Transform& tb
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
	auto ab_distance = gm::length(ab_vector);

	if (ab_distance > a_radius + b_radius) {
		// no collision
		return CollisionPoints();
	}

	auto ab_normal = gm::normalize(ab_vector);

	auto collision_points = CollisionPoints();

	collision_points.has_collision = true;

	collision_points.a = a_center + ab_normal * a_radius;
	collision_points.b = b_center - ab_normal * b_radius;

	collision_points.normal = -ab_normal;
	collision_points.depth = gm::length(collision_points.b - collision_points.a);

	return collision_points;
}

CollisionPoints collision_test_sphere_plane(
	const Collider& a, const Transform& ta,
	const Collider& b, const Transform& tb
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

	auto distance = gm::dot(plane_normal, sphere_center - point_on_plane);

	if (distance > sphere_radius) {
		// no collision
		return CollisionPoints();
	}

	auto collision_points = CollisionPoints();

	collision_points.has_collision = true;

	// furthest point of sphere a into plane b
	collision_points.a = sphere_center - plane_normal * sphere_radius;
	// furthest point of plane b into sphere a
	collision_points.b = sphere_center - plane_normal * distance;

	collision_points.normal = plane_normal;
	collision_points.depth = gm::length(collision_points.b - collision_points.a);

	return collision_points;
}

CollisionPoints collision_test_sphere_mesh(
	const Collider& a, const Transform& ta,
	const Collider& b, const Transform& tb
) {
	// assert(false); // not implemented
	return CollisionPoints();
}

CollisionPoints collision_test_plane_mesh(
	const Collider& a, const Transform& ta,
	const Collider& b, const Transform& tb
) {
	// assert(false); // not implemented
	return CollisionPoints();
}

CollisionPoints collision_test_mesh_mesh(
	const Collider& a, const Transform& ta,
	const Collider& b, const Transform& tb
) {
	assert(a.type == ColliderType::MESH);
	assert(b.type == ColliderType::MESH);

	auto& a_collider = static_cast<const SphereCollider&>(a);
	auto& b_collider = static_cast<const PlaneCollider&>(b);

	// GJK Algorithm https://youtu.be/ajv46BSqcK4

	// the first direction is arbitrary. we choose the direction from the origin of one shape to the other
	auto d = gm::normalize( tb.position - ta.position );

	gm::Vector3 simplex [4] = { gm::Vector3(0), gm::Vector3(0), gm::Vector3(0), gm::Vector3(0) };
	// find the first support point on the minkowski difference in direction d
	simplex[0] = support_function_mesh(a_collider, ta, d) - support_function_mesh(b_collider, tb, - d);

	// the next direction is towards the origin
	d = gm::normalize( - simplex[0] );

	// find the second support point
	simplex[1] = support_function_mesh(a_collider, ta, d) - support_function_mesh(b_collider, tb, - d);
	// if the next support point did not "pass" the origin, the shapes do not intersect
	if (gm::dot(simplex[1], d) < 0) {
		return CollisionPoints();
	}

	// A = most recently added vertex, O = Origin
	const auto AB = simplex[0] - simplex[1];
	const auto AO = - simplex[1];

	// triple product: vector perpendicular to AB pointing toward the origin
	d = gm::normalize( gm::cross( gm::cross(AB, AO), AB ) );

	// find the third support point
	while (true) {
		simplex[2] = support_function_mesh(a_collider, ta, d) - support_function_mesh(b_collider, tb, - d);

		// if the new support point did not "pass" the origin, the shapes do not intersect
		if (gm::dot(simplex[2], d) < 0) {
			return CollisionPoints();
		}

		// A = most recently added vertex, O = Origin
		const auto AB = simplex[1] - simplex[2];
		const auto AC = simplex[0] - simplex[2];
		const auto AO = - simplex[2];

		// triple products to define regions R_AB and R_AC
		const auto ABC_normal = gm::cross(AB, AC);
		const auto AB_normal = gm::normalize( gm::cross( gm::cross(AC, AB), AB ) );
		const auto AC_normal = gm::normalize( gm::cross( ABC_normal, AC ) );

		// TODO: Add check if the origin lies on the line AB or AC

		if (gm::dot( AB_normal, AO ) > 0) {
			// We are in region AB
			// Remove current C, move the array so that the most recently added vertex is always at simplex[2]
			simplex[0] = simplex[1]; simplex[1] = simplex[2];
			d = AB_normal;
		}
		else if (gm::dot( AC_normal, AO ) > 0) {
			// We are in region AC
			// Remove current B, move the array so that the most recently added vertex is always at simplex[2]
			simplex[1] = simplex[2];
			d = AC_normal;
		}
		else {
			// We are in region ABC. Check if the origin is above or below ABC and move on.

			if (gm::dot( ABC_normal, AO ) > 0) {
				// above ABC
				d = gm::normalize(ABC_normal);
			}
			else {
				// below ABC
				// swap current C and B (change winding order), so we are above ABC again
				const auto B = simplex[1]; simplex[1] = simplex[0]; simplex[0] = B;
				d = - gm::normalize(ABC_normal);
			}

			break;
		}
	}

	while (true) {
		simplex[3] = support_function_mesh(a_collider, ta, d) - support_function_mesh(b_collider, tb, - d);

		// if the new support point did not "pass" the origin, the shapes do not intersect
		if (gm::dot(simplex[3], d) < 0) {
			return CollisionPoints();
		}

		const auto A = simplex[3];
		const auto B = simplex[2];
		const auto C = simplex[1];
		const auto D = simplex[0];

		const auto AB = B - A;
		const auto AC = C - A;
		const auto AD = D - A;
		const auto AO =   - A;

		const auto ABC_normal = gm::cross(AB, AC);
		const auto ACD_normal = gm::cross(AC, AD);
		const auto ADB_normal = gm::cross(AD, AB);

		// Check in which region we are. Remove the vertex that is not part of that region
		if (gm::dot( ABC_normal, AO ) > 0) {
			simplex[2] = A; simplex[1] = B; simplex[0] = C;
			d = ABC_normal;
		}
		else if (gm::dot( ACD_normal, AO ) > 0) {
			simplex[2] = A; simplex[1] = C; simplex[0] = D;
			d = ACD_normal;
		}
		else if (gm::dot( ADB_normal, AO ) > 0) {
			simplex[2] = A; simplex[1] = D; simplex[0] = B;
			d = ADB_normal;
		}
		else {
			// Collision detected!
			auto collision_points = CollisionPoints();

			collision_points.has_collision = true;

			return collision_points;
		}
	}
}

// define the function type for a collision test function
using CollisionTestFunc = CollisionPoints(*)(
	const Collider&, const Transform&,
	const Collider&, const Transform&
);

CollisionPoints tics::collision_test(
	const Collider& a, const Transform& at,
	const Collider& b, const Transform& bt
) {
	static const CollisionTestFunc function_table[3][3] = {
		  // Sphere                     Plane                        // Mesh
		{ collision_test_sphere_sphere, collision_test_sphere_plane, collision_test_sphere_mesh }, // Sphere
		{ nullptr,                      nullptr                    , collision_test_plane_mesh  },  // Plane
		{ nullptr,                      nullptr                    , collision_test_mesh_mesh   },  // Mesh
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
		points.normal = -points.normal;
	}

	return points;
};
