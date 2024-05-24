#include "tics.h"

#include <iostream>

#include <TSRigid3D.h>
#include <TSVector4D.h>
#include <TSVector3D.h>
#include <TSBivector3D.h>

bool tics::pga_raycast(const MeshCollider &mesh_collider, const gm::Vector3 ray_start, const gm::Vector3 direction) {
	for (size_t triangle_index = 0; triangle_index < mesh_collider.indices.size() / 3; triangle_index++) {
		const uint32_t indices [3] = {
			mesh_collider.indices.at(triangle_index * 3 + 0),
			mesh_collider.indices.at(triangle_index * 3 + 1),
			mesh_collider.indices.at(triangle_index * 3 + 2),
		};
		const gm::Vector3 vertices [3] = {
			mesh_collider.positions.at(indices[0]),
			mesh_collider.positions.at(indices[1]),
			mesh_collider.positions.at(indices[2]),
		};

		auto a = Terathon::Point3D(vertices[0].x, vertices[0].y, vertices[0].z);
		auto b = Terathon::Point3D(vertices[1].x, vertices[1].y, vertices[1].z);
		auto c = Terathon::Point3D(vertices[2].x, vertices[2].y, vertices[2].z);

		// two points on the line
		const auto p = Terathon::Point3D(ray_start.x, ray_start.y, ray_start.z);
		const auto q = Terathon::Point3D(ray_start.x + direction.x, ray_start.y + direction.y, ray_start.z + direction.z);

		auto l = Terathon::Wedge(p, q);

		// Translate the line by subtracting (l.v cross a) from its moment l.m
		// NOTE: The PGA Illuminated book uses a cross l.v but that gives incorrect results
		const auto a_cross_lv = Terathon::Cross(a, l.v);
		const auto lv_cross_a = Terathon::Cross(l.v, a);
		l.m -= Terathon::Bivector3D(lv_cross_a.x, lv_cross_a.y, lv_cross_a.z);

		// Translate a to the origin and subtract a from b and c
		b -= a;
		c -= a;
		a = Terathon::Point3D(0.0f, 0.0f, 0.0f);

		// Lines representing the edges of the triangle
		const auto k_1 = Terathon::Wedge(a, b);
		const auto k_2 = Terathon::Wedge(b, c);
		const auto k_3 = Terathon::Wedge(c, a);

		// If any of the Antiwedge products is negative, then the line does not intersect the triangle.
		const auto any_negative =
			Terathon::Antiwedge(l, k_1) < 0 ||
			Terathon::Antiwedge(l, k_2) < 0 ||
			Terathon::Antiwedge(l, k_3) < 0;
		
		// Otherwise, we have a hit.
		if (!any_negative) {
			return true;
		}
	}

	return false;
}

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
