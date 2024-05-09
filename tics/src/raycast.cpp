#include "tics.h"

#include <iostream>

bool tics::raycast(const MeshCollider &mesh_collider, const gm::Vector3 ray_start, const gm::Vector3 direction) {
	for (size_t triangle_index = 0; triangle_index < mesh_collider.indices.size() / 3; triangle_index++) {
		const uint32_t indices [3] = {
			mesh_collider.indices.at(triangle_index * 3 + 0),
			mesh_collider.indices.at(triangle_index * 3 + 1),
			mesh_collider.indices.at(triangle_index * 3 + 2),
		};
		gm::Vector3 vertices [3] = {
			mesh_collider.positions.at(indices[0]),
			mesh_collider.positions.at(indices[1]),
			mesh_collider.positions.at(indices[2]),
		};

		// Let l be a line containing the point ray_start and running parallel to the direction vector direction

		// Translate the line so that ray_start coincides with the origin.
		// Translate the triangle by subtracting ray_start from the vertices

		for (size_t i = 0; i < 3; i++) {
			vertices[i] -= ray_start;
		}

		// Calculate the scalar triple products (a x b) dot direction, (b x c) dot direction, (c x a) dot direction
		// If any of these products is positive, then the line does not intersect the triangle.

		const auto scalar_triple_product_ab = gm::dot( gm::cross(vertices[0], vertices[1]), direction );
		const auto scalar_triple_product_bc = gm::dot( gm::cross(vertices[1], vertices[2]), direction );
		const auto scalar_triple_product_ca = gm::dot( gm::cross(vertices[2], vertices[0]), direction );

		const bool intersecting = !
			(  scalar_triple_product_ab > 0
			|| scalar_triple_product_bc > 0
			|| scalar_triple_product_ca > 0 );

		if (intersecting) {
			// std::cout << "Intersection at triangle " << triangle_index << " : "
			// 	<< vertices[0] << ", " << vertices[1] << ", " << vertices[2] << "\n";
			return true;
		}
	}

	return false;
}
