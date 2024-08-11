#include "tics.h"

#include <algorithm>
#include <cassert>

using tics::World;

void World::add_object(const std::weak_ptr<tics::ICollisionObject> object) {
	m_objects.emplace_back(object);
}

void World::remove_object(const std::weak_ptr<tics::ICollisionObject> object) {
	auto is_equals = [object](std::weak_ptr<tics::ICollisionObject> obj) {
		return !obj.expired() && !object.expired() && object.lock() == obj.lock();
	};
	// find the object, move it to the end of the list and erase it
	m_objects.erase(std::remove_if(m_objects.begin(), m_objects.end(), is_equals), m_objects.end());
}

void World::add_solver(const std::weak_ptr<ISolver> solver) {
	m_solvers.emplace_back(solver);
}

void World::remove_solver(const std::weak_ptr<ISolver> solver) {
	auto is_equals = [solver](std::weak_ptr<tics::ISolver> s) {
		return !s.expired() && !solver.expired() && solver.lock() == s.lock();
	};
	// find the solver, move it to the end of the list and erase it
	m_solvers.erase(std::remove_if(m_solvers.begin(), m_solvers.end(), is_equals), m_solvers.end());
}

void World::update(const float delta) {
	resolve_collisions(delta);

	// dynamics
	for (auto wp_object : m_objects) {
		if (auto sp_object = wp_object.lock()) {
			const auto rigid_body = dynamic_cast<RigidBody *>(sp_object.get());
			if (!rigid_body) { continue; }

			rigid_body->force += rigid_body->mass * rigid_body->gravity_scale * m_gravity;

			assert(rigid_body->mass != 0.0f);
			auto acceleration = rigid_body->force / rigid_body->mass;
			rigid_body->velocity += acceleration * delta;
			rigid_body->get_transform().lock()->position += rigid_body->velocity * delta;

			rigid_body->force = Terathon::Vector3D(0,0,0);
		}
	}
}

void World::resolve_collisions(const float delta) {
	std::vector<Collision> collisions;

	for (auto wp_a : m_objects) {
		for (auto wp_b : m_objects) {
			auto sp_a = wp_a.lock();
			auto sp_b = wp_b.lock();
			if (!sp_a || !sp_b) { continue; }

			// break if both pointers point to the same object -> we will only check unique pairs
			if (sp_a == sp_b) { break; }

			if (sp_a->get_collider().expired() || sp_b->get_collider().expired() ||
				sp_a->get_transform().expired() || sp_b->get_transform().expired()
			) {
				continue;
			}

			auto collision_points = collision_test(
				*(sp_a->get_collider().lock()), *(sp_a->get_transform().lock()),
				*(sp_b->get_collider().lock()), *(sp_b->get_transform().lock())
			);

			if (collision_points.has_collision) {
				collisions.emplace_back(sp_a, sp_b, collision_points);
			}
		}
	}

	for (const auto& collision : collisions) {
		if (m_collision_event) { m_collision_event(collision); }
	}

	for (auto wp_solver : m_solvers) {
		if (auto sp_solver = wp_solver.lock()) {
			sp_solver->solve(collisions, delta);
		}
	}
}

void World::set_gravity(const Terathon::Vector3D gravity) {
	m_gravity = gravity;
}

void World::set_collision_event(const std::function<void(const Collision&)> collision_event) {
	m_collision_event = collision_event;
}
