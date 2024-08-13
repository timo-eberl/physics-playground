#include "tics.h"

#include <cassert>
#include <iostream>
#include <limits>
#include <algorithm>

using tics::Transform;
using tics::CollisionPoints;
using tics::ColliderType;
using tics::Collider;
using tics::SphereCollider;
using tics::PlaneCollider;
using tics::MeshCollider;

// A support function takes a direction d and returns a point on the boundary of a shape "furthest" in direction d

Terathon::Vector3D support_point_sphere(
	const Collider &c, const Terathon::Vector3D &d
) {
	assert(c.type == ColliderType::SPHERE);

	const auto &collider = static_cast<const SphereCollider&>(c);

	return collider.center + d * collider.radius;
}

Terathon::Vector3D support_point_mesh(
	const Collider &c, const Transform &t, const Terathon::Vector3D &d
) {
	assert(c.type == ColliderType::MESH);

	const auto &collider = static_cast<const MeshCollider&>(c);

	const auto local_d = Terathon::Transform(d, t.rotation);

	// find the support point in local space
	auto support_point_dot = -1.0;
	auto support_point = Terathon::Vector3D(0,0,0);
	for (const auto &p : collider.positions) {
		const auto p_dot_d = Terathon::Dot(p, local_d);
		if (p_dot_d > support_point_dot) {
			support_point_dot = p_dot_d;
			support_point = p;
		}
	}

	if (support_point_dot < 0.0) {
		std::cout << "alarm\n";
	}

	// this fails if the center position of a mesh is not inside the mesh
	assert(support_point_dot >= 0.0);

	support_point = Terathon::Transform(support_point, Terathon::Inverse(t.rotation));
	support_point = support_point + t.position;

	return support_point;
}

Terathon::Vector3D support_point_on_minkowski_diff_mesh_mesh(
	const Collider &ca, const Transform &ta,
	const Collider &cb, const Transform &tb,
	const Terathon::Vector3D &d
) {
	assert(ca.type == ColliderType::MESH);
	assert(cb.type == ColliderType::MESH);

	return support_point_mesh(ca, ta, d) - support_point_mesh(cb, tb, - d);
}

CollisionPoints collision_test_sphere_sphere(
	const Collider& a, const Transform& ta,
	const Collider& b, const Transform& tb
) {
	assert(a.type == ColliderType::SPHERE);
	assert(b.type == ColliderType::SPHERE);

	auto& a_collider = static_cast<const SphereCollider&>(a);
	auto& b_collider = static_cast<const SphereCollider&>(b);

	auto a_center = a_collider.center + ta.position;
	auto b_center = b_collider.center + tb.position;

	auto ab_vector = b_center - a_center;
	auto ab_distance = Terathon::Magnitude(ab_vector);

	if (ab_distance > a_collider.radius + b_collider.radius) {
		// no collision
		return CollisionPoints();
	}

	auto ab_normal = Terathon::Normalize(ab_vector);

	auto collision_points = CollisionPoints();

	collision_points.has_collision = true;

	const auto collision_points_a = a_center + ab_normal * a_collider.radius;
	const auto collision_points_b = b_center - ab_normal * b_collider.radius;

	collision_points.normal = -ab_normal;
	collision_points.depth = Terathon::Magnitude(collision_points_b - collision_points_a);

	return collision_points;
}

CollisionPoints collision_test_sphere_plane(
	const Collider& a, const Transform& ta,
	const Collider& b, const Transform& tb
) {
	assert(a.type == ColliderType::SPHERE);
	assert(b.type == ColliderType::PLANE);

	auto& sphere_collider = static_cast<const SphereCollider&>(a);
	auto& plane_collider = static_cast<const PlaneCollider&>(b);

	auto sphere_center = sphere_collider.center + ta.position;

	auto plane_normal = plane_collider.normal; // TODO: rotate with tb
	auto point_on_plane = plane_normal * plane_collider.distance + tb.position;

	auto distance = Terathon::Dot(plane_normal, sphere_center - point_on_plane);

	if (distance > sphere_collider.radius) {
		// no collision
		return CollisionPoints();
	}

	auto collision_points = CollisionPoints();

	collision_points.has_collision = true;

	// furthest point of sphere a into plane b
	const auto collision_points_a = sphere_center - plane_normal * sphere_collider.radius;
	// furthest point of plane b into sphere a
	const auto collision_points_b = sphere_center - plane_normal * distance;

	collision_points.normal = plane_normal;
	collision_points.depth = Terathon::Magnitude(collision_points_b - collision_points_a);

	return collision_points;
}

void add_if_unique_edge(
	std::vector<std::pair<uint32_t, uint32_t>>& edges,
	const std::vector<uint32_t>& faces, uint32_t edge_a, uint32_t edge_b
) {
	const auto reverse = std::find(
		edges.begin(), edges.end(),
		std::make_pair(edge_b, edge_a)
	);

	// edge was already present -> remove it
	if (reverse != edges.end()) {
		edges.erase(reverse);
	}
	else {
		edges.emplace_back(edge_a, edge_b);
	}
}

// Mesh vs Mesh collisions use the GJK and EPA Algorithm
CollisionPoints collision_test_mesh_mesh(
	const Collider& a, const Transform& ta,
	const Collider& b, const Transform& tb
) {
	assert(a.type == ColliderType::MESH);
	assert(b.type == ColliderType::MESH);

	auto& a_collider = static_cast<const MeshCollider&>(a);
	auto& b_collider = static_cast<const MeshCollider&>(b);

	// GJK Algorithm https://youtu.be/ajv46BSqcK4

	// the first direction is arbitrary. we choose the direction from the origin of one shape to the other
	auto d = Terathon::Normalize( tb.position - ta.position );

	Terathon::Vector3D simplex [4] = {
		Terathon::Vector3D(0,0,0), Terathon::Vector3D(0,0,0), Terathon::Vector3D(0,0,0), Terathon::Vector3D(0,0,0)
	};
	// find the first support point on the minkowski difference in direction d
	simplex[0] = support_point_on_minkowski_diff_mesh_mesh(a_collider, ta, b_collider, tb, d);

	// the next direction is towards the origin
	// FIXME: probably the normalize can be removed?
	d = Terathon::Normalize( - simplex[0] );

	// find the second support point
	simplex[1] = support_point_on_minkowski_diff_mesh_mesh(a_collider, ta, b_collider, tb, d);
	// if the next support point did not "pass" the origin, the shapes do not intersect
	if (Terathon::Dot(simplex[1], d) < 0.001) {
		return CollisionPoints();
	}

	// A = most recently added vertex, O = Origin
	const auto AB = simplex[0] - simplex[1];
	const auto AO =            - simplex[1];

	// triple product: vector perpendicular to AB pointing toward the origin
	// FIXME: probably the normalize can be removed?
	d = Terathon::Normalize ( Terathon::Cross ( Terathon::Cross (AB, AO), AB ) );

	
	// find the third support point
	while (true) {
		simplex[2] = support_point_on_minkowski_diff_mesh_mesh(a_collider, ta, b_collider, tb, d);

		// if the new support point did not "pass" the origin, the shapes do not intersect
		if (Terathon::Dot(simplex[2], d) < 0.001) {
			return CollisionPoints();
		}

		// A = most recently added vertex, O = Origin
		const auto AB = simplex[1] - simplex[2];
		const auto AC = simplex[0] - simplex[2];
		const auto AO =            - simplex[2];

		// triple products to define regions R_AB and R_AC
		const auto ABC_normal = Terathon::Cross(AB, AC);
		// FIXME: probably the normalize can be removed?
		const auto AB_normal = Terathon::Normalize( Terathon::Cross( Terathon::Cross(AC, AB), AB ) );
		// FIXME: probably the normalize can be removed?
		const auto AC_normal = Terathon::Normalize( Terathon::Cross( ABC_normal, AC ) );

		// TODO: Add check if the origin lies on the line AB or AC

		if (Terathon::Dot( AB_normal, AO ) > 0) {
			// We are in region AB
			// Remove current C, move the array so that the most recently added vertex is always at simplex[2]
			simplex[0] = simplex[1]; simplex[1] = simplex[2];
			d = AB_normal;
		}
		else if (Terathon::Dot( AC_normal, AO ) > 0) {
			// We are in region AC
			// Remove current B, move the array so that the most recently added vertex is always at simplex[2]
			simplex[1] = simplex[2];
			d = AC_normal;
		}
		else {
			// We are in region ABC. Check if the origin is above or below ABC and move on.

			if (Terathon::Dot( ABC_normal, AO ) > 0) {
				// above ABC
				// FIXME: probably the normalize can be removed?
				d = Terathon::Normalize(ABC_normal);
			}
			else {
				// below ABC
				// swap current C and B (change winding order), so we are above ABC again
				const auto B = simplex[1]; simplex[1] = simplex[0]; simplex[0] = B;
				// FIXME: probably the normalize can be removed?
				d = - Terathon::Normalize(ABC_normal);
			}

			break;
		}
	}

	// find the fourth (last) support point
	// only iterate a limited number of times to work around being stuck in a loop
	for (size_t i = 0; i < 100; i++) {
	// while (true) {
		simplex[3] = support_point_on_minkowski_diff_mesh_mesh(a_collider, ta, b_collider, tb, d);

		const auto fkdasjl = Terathon::Dot(simplex[3], d);
		// if the new support point did not "pass" the origin, the shapes do not intersect
		if (Terathon::Dot(simplex[3], d) < 0.001) {
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

		const auto ABC_normal = Terathon::Normalize( Terathon::Cross(AB, AC) );
		const auto ACD_normal = Terathon::Normalize( Terathon::Cross(AC, AD) );
		const auto ADB_normal = Terathon::Normalize( Terathon::Cross(AD, AB) );

		const auto fasdkjfl1 = Terathon::Dot( ABC_normal, AO );
		const auto fasdkjfl2 = Terathon::Dot( ACD_normal, AO );
		const auto fasdkjfl3 = Terathon::Dot( ADB_normal, AO );

		// Check in which region we are. Remove the vertex that is not part of that region
		if (Terathon::Dot( ABC_normal, AO ) > 0.001) {
			simplex[2] = A; simplex[1] = B; simplex[0] = C;
			d = ABC_normal;
		}
		else if (Terathon::Dot( ACD_normal, AO ) > 0.001) {
			simplex[2] = A; simplex[1] = C; simplex[0] = D;
			d = ACD_normal;
		}
		else if (Terathon::Dot( ADB_normal, AO ) > 0.001) {
			simplex[2] = A; simplex[1] = D; simplex[0] = B;
			d = ADB_normal;
		}
		else {
			// Collision detected!
			auto collision_points = CollisionPoints();

			collision_points.has_collision = true;

			// EPA (Expanding Polytope Algorithm): GJK Extension for collision information
			// We want to find the normal of the collision
			// normal of collision = (b - a if a a nd b are each the furthest points of the one shape into the other)
			// This normal is the normal of the face of the minkowski difference that is closest to the origin
			// Problem: The simplex we found in which the origin lies is a subspace of the minkowski difference.
			//          It does not necessarily contain the required face.
			// Solution: We are adding vertices to the simplex (making it a polytope) until we find the shortest normal
			//           from a face that is on the original mesh

			// we find at the face that is closest
			// then we try to expand the polytope in the direction of the faces normal
			// if we were able to expand - repeat
			// if not, we found the closest face

			// initialize the polytope with the data from the simplex
			std::vector<Terathon::Vector3D> polytope_positions = {};
			polytope_positions.insert(polytope_positions.end(), { simplex[0], simplex[1], simplex[2], simplex[3] });
			// order the vertices of the triangles so that the normals are always pointing outwards
			std::vector<uint32_t> polytope_indices = {};
			polytope_indices.insert(polytope_indices.end(), {
				0, 1, 2,
				0, 3, 1,
				0, 2, 3,
				1, 3, 2
			});

			// calculate face normals vec4(vec3(normal), distance)
			// and find the face closest to the origin
			std::vector<Terathon::Vector4D> polytope_normals = {};
			auto closest_distance = std::numeric_limits<float>::max();
			size_t closest_index = 0;
			for (size_t i = 0; i < polytope_indices.size() / 3; i++) {
				Terathon::Vector3D a = polytope_positions[polytope_indices[i * 3    ]];
				Terathon::Vector3D b = polytope_positions[polytope_indices[i * 3 + 1]];
				Terathon::Vector3D c = polytope_positions[polytope_indices[i * 3 + 2]];

				const auto normal = Terathon::Normalize( Terathon::Cross(b - a, c - a) );
				const double distance = Terathon::Dot(normal, a); // works with any vertex of the plane

				polytope_normals.emplace_back(normal, distance);

				if (distance < closest_distance) {
					closest_distance = distance;
					closest_index = i;
				}
			}

			while (true) {
				// search for a new support point in the direction of the normal of the closest face
				d = polytope_normals[closest_index].xyz;
				const auto new_support_point = support_point_on_minkowski_diff_mesh_mesh(a_collider, ta, b_collider, tb, d);
				const auto support_distance = Terathon::Dot(d, new_support_point);

				// check if the support point lies on the same plane as the closest face
				// if it does, the polytype cannot be further expanded
				if (abs(support_distance - closest_distance) <= 0.001) {
					break; // cannot be expanded - found the closest face!
				}

				// expand the polytope by adding the support point
				// to make sure the polytope stays convex, we remove all faces that point towards the support point
				// and create new faces afterwards

				std::vector<std::pair<uint32_t, uint32_t>> unique_edges;

				for (size_t i = 0; i < polytope_indices.size() / 3; i++) {
					// check if the support point is in front of the triangle
					if (Terathon::Dot(polytope_normals[i].xyz, new_support_point - polytope_positions[polytope_indices[i * 3]]) > 0) {
						// if it is, collect all unique edges
						add_if_unique_edge(unique_edges, polytope_indices, polytope_indices[i*3    ], polytope_indices[i*3 + 1]);
						add_if_unique_edge(unique_edges, polytope_indices, polytope_indices[i*3 + 1], polytope_indices[i*3 + 2]);
						add_if_unique_edge(unique_edges, polytope_indices, polytope_indices[i*3 + 2], polytope_indices[i*3    ]);

						polytope_indices.erase(polytope_indices.begin() + i*3, polytope_indices.begin() + i*3 + 3);
						polytope_normals.erase(polytope_normals.begin() + i);

						i--;
					}
				}

				// create new vertex and faces
				const auto new_vertex_index = polytope_positions.size();

				polytope_positions.push_back(new_support_point);

				for (auto [edge_index_a, edge_index_b] : unique_edges) {
					polytope_indices.push_back(edge_index_a);
					polytope_indices.push_back(edge_index_b);
					polytope_indices.push_back(new_vertex_index);

					Terathon::Vector3D a = polytope_positions[edge_index_a];
					Terathon::Vector3D b = polytope_positions[edge_index_b];
					Terathon::Vector3D c = polytope_positions[new_vertex_index];

					auto normal = Terathon::Normalize( Terathon::Cross(b - a, c - a) );
					double distance = Terathon::Dot(normal, a);

					if (distance < 0) {
						normal = -normal;
						distance = -distance;
					}

					polytope_normals.emplace_back(normal, distance);
				}

				// (re)iterate over all faces and find the closest
				closest_distance = std::numeric_limits<double>::max();
				closest_index = 0;
				for (size_t i = 0; i < polytope_indices.size() / 3; i++) {
					const double distance = polytope_normals[i].w;
					if (distance < closest_distance) {
						closest_distance = distance;
						closest_index = i;
					}
				}
			}

			collision_points.normal = -polytope_normals[closest_index].xyz;
			collision_points.depth = closest_distance;

			// find orthogonal vector to normal vector
			auto not_parallel_to_normal = Terathon::Vector3D(0,0,1);
			if (Terathon::Dot(collision_points.normal, not_parallel_to_normal) > 0.9) {
				not_parallel_to_normal = Terathon::Vector3D(0,1,0);
			}
			const auto orthogonal_to_normal = Terathon::Cross(not_parallel_to_normal, collision_points.normal);

			// hacky way to find penetration points, works badly if two edges collide
			// we want to find out which one of the objects is "poking" into the other
			// choose similar vectors to the normal and look for the support point, for both a and b
			// collision_points.normal points from b to a
			const auto b0 = support_point_mesh(b_collider, tb,  collision_points.normal - orthogonal_to_normal*0.01);
			const auto b1 = support_point_mesh(b_collider, tb,  collision_points.normal + orthogonal_to_normal*0.01);
			const auto b_dist = Terathon::Magnitude(b0-b1);
			const auto a0 = support_point_mesh(a_collider, ta, -collision_points.normal - orthogonal_to_normal*0.01);
			const auto a1 = support_point_mesh(a_collider, ta, -collision_points.normal + orthogonal_to_normal*0.01);
			const auto a_dist = Terathon::Magnitude(a0-a1);
			if (a_dist < b_dist) {
				// a is poking into b
				const auto a = support_point_mesh(a_collider, ta, -collision_points.normal);
				collision_points.a = a;
				collision_points.b = a - (collision_points.normal * collision_points.depth);
			}
			else {
				// b is poking into a
				const auto b = support_point_mesh(b_collider, tb, collision_points.normal);
				collision_points.b = b;
				collision_points.a = b + (collision_points.normal * collision_points.depth);
			}

			return collision_points;
		}
	}
	std::cout << "Terminating GJK (stuck in a loop)\n";
	return CollisionPoints(); // workaround
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
	// a collision table as described by valve in this pdf on page 33
	// https://media.steampowered.com/apps/valve/2015/DirkGregorius_Contacts.pdf
	static const CollisionTestFunc function_table[3][3] = {
		  // Sphere                     Plane                        // Mesh
		{ collision_test_sphere_sphere, collision_test_sphere_plane, nullptr                    },  // Sphere
		{ nullptr,                      nullptr                    , nullptr                    },  // Plane
		{ nullptr,                      nullptr                    , collision_test_mesh_mesh   },  // Mesh
	};

	// make sure the colliders are in the correct order
	// example: (mesh, sphere) gets swapped to (sphere, mesh)
	bool swap = a.type > b.type;

	auto& sorted_a = swap ? b : a;
	auto& sorted_b = swap ? a : b;
	auto& sorted_at = swap ? bt : at;
	auto& sorted_bt = swap ? at : bt;

	// pick the function that matches the collider types from the table
	auto collision_test_function = function_table[sorted_a.type][sorted_b.type];
	// check if collision test function is defined for the given colliders
	assert(collision_test_function != nullptr);

	CollisionPoints points = collision_test_function(sorted_a, sorted_at, sorted_b, sorted_bt);
	// if we swapped the input colliders, we need to invert the collision data
	if (swap) {
		points.normal = -points.normal;
	}

	return points;
};
