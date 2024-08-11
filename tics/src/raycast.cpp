#include "tics.h"

#include <iostream>

#include <TSRigid3D.h>
#include <TSVector4D.h>
#include <TSVector3D.h>
#include <TSBivector3D.h>
#include <TSMatrix3D.h>
#include <TSMatrix4D.h>
#include <TSMotor3D.h>

static std::string vec_to_str(Terathon::Vector3D v) {
	std::stringstream stream;
	stream << "{ " << v.x << ", " << v.y << ", " << v.z << " }";
	return stream.str();
}

bool tics::pga_raycast(const Terathon::Motor3D &model_motor, const MeshCollider &mesh_collider, const Terathon::Point3D p, const Terathon::Point3D q) {
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

		auto a = Terathon::Transform( Terathon::Point3D(vertices[0].x, vertices[0].y, vertices[0].z), model_motor );
		auto b = Terathon::Transform( Terathon::Point3D(vertices[1].x, vertices[1].y, vertices[1].z), model_motor );
		auto c = Terathon::Transform( Terathon::Point3D(vertices[2].x, vertices[2].y, vertices[2].z), model_motor );

		std::cout << "pga a: " << vec_to_str(a) << std::endl;
		std::cout << "pga b: " << vec_to_str(b) << std::endl;
		std::cout << "pga c: " << vec_to_str(c) << std::endl;

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

		std::cout << "pga a_translated: " << vec_to_str(a) << std::endl;
		std::cout << "pga b_translated: " << vec_to_str(b) << std::endl;
		std::cout << "pga c_translated: " << vec_to_str(c) << std::endl;

		// Lines representing the edges of the triangle
		const auto k_1 = Terathon::Line3D(b, Terathon::Bivector3D(0.0f, 0.0f, 0.0f));
		const auto k_2 = Terathon::Wedge(b, c);
		const auto k_3 = Terathon::Line3D(-c, Terathon::Bivector3D(0.0f, 0.0f, 0.0f));

		std::cout << "Antiwedge(l, k_1): " << Antiwedge(l, k_1) << std::endl;
		std::cout << "Antiwedge(l, k_2): " << Antiwedge(l, k_2) << std::endl;
		std::cout << "Antiwedge(l, k_3): " << Antiwedge(l, k_3) << std::endl;
		// If any of the Antiwedge products is negative, then the line does not intersect the triangle.

		return !(
			Terathon::Antiwedge(l, k_1) < 0 ||
			Terathon::Antiwedge(l, k_2) < 0 ||
			Terathon::Antiwedge(l, k_3) < 0
		);
	}

	return false;
}

bool tics::raycast(const Terathon::Transform3D &model_mat, const MeshCollider &mesh_collider, const Terathon::Vector3D ray_start, const Terathon::Vector3D direction) {
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

		auto a = model_mat * Terathon::Point3D(vertices[0].x, vertices[0].y, vertices[0].z);
		auto b = model_mat * Terathon::Point3D(vertices[1].x, vertices[1].y, vertices[1].z);
		auto c = model_mat * Terathon::Point3D(vertices[2].x, vertices[2].y, vertices[2].z);

		// Let l be a line containing the point ray_start and running parallel to the direction vector direction

		Terathon::Vector4D plane;
		plane.xyz = Terathon::Cross( (b-a), (c-a) );
		plane.w = Terathon::Dot( -plane.xyz, a - ray_start );

		// find the point of intersection q
		Terathon::Vector3D q =
			ray_start +
			( -plane.w / Terathon::Dot(plane.xyz, direction) ) * direction;

		a -= q;
		b -= q;
		c -= q;
		q = Terathon::Vector3D(0.0f, 0.0f, 0.0f);

		// edge 0
		const Terathon::Vector3D edge0 = b - a;
		const Terathon::Vector3D aq = q - a;
		const auto scalar_triple_product_edge0_aq = Terathon::Dot( plane.xyz * plane.w, Terathon::Cross(edge0, aq) );

		// edge 1
		const Terathon::Vector3D edge1 = c - b;
		const Terathon::Vector3D bq = q - b;
		const auto scalar_triple_product_edge1_bq = Terathon::Dot( plane.xyz * plane.w, Terathon::Cross(edge1, bq) );

		// edge 2
		const Terathon::Vector3D edge2 = a - c;
		const Terathon::Vector3D cq = q - c;
		const auto scalar_triple_product_edge2_cq = Terathon::Dot( plane.xyz * plane.w, Terathon::Cross(edge2, cq) );

		std::cout << "scalar_triple_product_edge0_aq: " << scalar_triple_product_edge0_aq << std::endl;
		std::cout << "scalar_triple_product_edge1_bq: " << scalar_triple_product_edge1_bq << std::endl;
		std::cout << "scalar_triple_product_edge2_cq: " << scalar_triple_product_edge2_cq << std::endl;

		return !(
			scalar_triple_product_edge0_aq < 0 ||
			scalar_triple_product_edge1_bq < 0 ||
			scalar_triple_product_edge2_cq < 0
		);
	}

	return false;
}
