#pragma once

#include <tics.h>
#include <ron.h>
#include <glm/gtx/string_cast.hpp>
#include <glm/mat4x4.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <TSVector3D.h>
#include <TSMatrix3D.h>
#include <TSMatrix4D.h>
#include <TSMotor3D.h>

struct Sphere {
	const std::shared_ptr<tics::RigidBody> rigid_body;
	const std::shared_ptr<tics::Transform> transform;
	const std::shared_ptr<tics::MeshCollider> collider;
	std::shared_ptr<ron::MeshNode> mesh_node;
	glm::vec3 color;
};

struct StaticObject {
	const std::shared_ptr<tics::StaticBody> static_body;
	const std::shared_ptr<tics::Transform> transform;
	const std::shared_ptr<tics::MeshCollider> collider;
	std::shared_ptr<ron::MeshNode> mesh_node;
};

glm::mat4 transform_to_model_matrix(const tics::Transform &transform) {
	auto model_matrix = glm::identity<glm::mat4>();

	model_matrix = glm::translate(model_matrix, glm::vec3(
		transform.position.x, transform.position.y, transform.position.z
	));

	return model_matrix;
}

ron::DirectionalLight create_generic_light() {
	ron::DirectionalLight light = {};

	light.use_custom_shadow_target_world_position = true;
	light.custom_shadow_target_world_position = glm::vec3(0.0f, 0.0f, 0.0f);
	light.world_direction = glm::normalize(glm::vec3(0.1f, 1.0f, 0.0f));
	light.shadow.enabled = true;
	light.shadow.map_size = glm::uvec2(2048);
	light.shadow.bias = 0.05f;
	light.shadow.far = 30.0f;
	light.shadow.frustum_size = 50.0f;

	return light;
}

Sphere create_sphere(
	Terathon::Vector3D position, Terathon::Vector3D velocity, const ron::Scene &scene,
	glm::vec3 color = glm::vec3(1.0), float scale = 1.0f, float mass = 1.0f
) {
	Sphere sphere = Sphere({
		std::make_shared<tics::RigidBody>(),
		std::make_shared<tics::Transform>(),
		std::make_shared<tics::MeshCollider>(),
		ron::gltf::import("models/icosphere_lowres.glb").get_mesh_nodes().front(),
	});

	sphere.rigid_body->set_collider(sphere.collider);
	sphere.rigid_body->set_transform(sphere.transform);
	sphere.rigid_body->mass = mass;
	sphere.transform->position = position;
	sphere.rigid_body->velocity = velocity;
	sphere.color = color;

	// apply scale to mesh
	for (auto &section : sphere.mesh_node->get_mesh()->sections) {
		for (auto &position : section.geometry->positions) {
			position *= scale;
		}
	}

	// copy the default material and modify it
	const auto material = std::make_shared<ron::Material>(*scene.default_material);
	material->uniforms["albedo_color"] = ron::make_uniform(glm::vec4(color, 1.0));

	// copy the mesh node to set its color
	const auto cloned_mesh_node = std::make_shared<ron::MeshNode>(
		std::make_shared<ron::Mesh>(*sphere.mesh_node->get_mesh()),
		transform_to_model_matrix(*sphere.transform)
	);
	cloned_mesh_node->get_mesh()->sections.front().material = material;

	sphere.mesh_node = cloned_mesh_node;

	const auto geometry = sphere.mesh_node->get_mesh()->sections.front().geometry;
	// copy positions and inidices to MeshCollider
	sphere.collider->indices = geometry->indices;
	for (const auto &vertex_pos : geometry->positions) {
		sphere.collider->positions.push_back(Terathon::Vector3D(vertex_pos.x, vertex_pos.y, vertex_pos.z));
	}

	return sphere;
}

std::shared_ptr<std::vector<StaticObject>> create_static_objects(const std::string& gltf_path) {
	auto objects = std::make_shared<std::vector<StaticObject>>();
	const auto mesh_nodes = ron::gltf::import(gltf_path).get_mesh_nodes();

	for (auto &mesh_node : mesh_nodes) {
		StaticObject static_object {
			std::make_shared<tics::StaticBody>(),
			std::make_shared<tics::Transform>(),
			std::make_shared<tics::MeshCollider>(),
			mesh_node,
		};

		const auto center = mesh_node->get_model_matrix() * glm::vec4(0,0,0,1);
		static_object.transform->position = Terathon::Vector3D(center.x,center.y,center.z);

		const auto ground_geometry = static_object.mesh_node->get_mesh()->sections.front().geometry;
		static_object.collider->indices = ground_geometry->indices;
		for (const auto &vertex_pos : ground_geometry->positions) {
			static_object.collider->positions.push_back(Terathon::Vector3D(vertex_pos.x, vertex_pos.y, vertex_pos.z));
		}
		static_object.static_body->set_collider(static_object.collider);
		static_object.static_body->set_transform(static_object.transform);

		objects->emplace_back(static_object);
	}

	return objects;
}
