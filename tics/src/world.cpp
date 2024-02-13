#include "tics.h"

#include <algorithm>
#include <cassert>

using tics::World;

void World::add_object(const std::weak_ptr<tics::Object> object) {
	m_objects.emplace_back(object);
}

void World::remove_object(const std::weak_ptr<tics::Object> object) {
	auto is_equals = [object](std::weak_ptr<tics::Object> obj) {
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
			if (!sp_object->is_dynamic) { continue; }

			sp_object->force += sp_object->mass * sp_object->gravity_scale * m_gravity;

			assert(sp_object->mass != 0.0f);
			auto acceleration = sp_object->force / sp_object->mass;
			sp_object->velocity += acceleration * delta;
			auto fjasdlkj = sp_object->transform.lock()->position;
			sp_object->transform.lock()->position += sp_object->velocity * delta;

			sp_object->force = { 0.0f, 0.0f, 0.0f };
		}
	}
}

void World::resolve_collisions(const float delta) {
	std::vector<Collision> collisions;

	for (auto wp_object : m_objects) {
		if (auto sp_object = wp_object.lock()) {
			sp_object->is_colliding = false;
		}
	}

	for (auto wp_a : m_objects) {
		for (auto wp_b : m_objects) {
			auto sp_a = wp_a.lock();
			auto sp_b = wp_b.lock();
			if (!sp_a || !sp_b) { continue; }

			// break if both pointers point to the same object -> we will only check unique pairs
			if (sp_a == sp_b) { break; }

			if (sp_a->collider.expired() || sp_b->collider.expired() ||
				sp_a->transform.expired() || sp_b->transform.expired()
			) {
				continue;
			}

			auto collision_points = collision_test(
				*(sp_a->collider.lock()), *(sp_a->transform.lock()),
				*(sp_b->collider.lock()), *(sp_b->transform.lock())
			);

			if (collision_points.has_collision) {
				collisions.emplace_back(sp_a, sp_b, collision_points);
			}

			if (collision_points.has_collision) {
				sp_a->is_colliding = true;
				sp_b->is_colliding = true;
			}
		}
	}

	for (auto wp_solver : m_solvers) {
		if (auto sp_solver = wp_solver.lock()) {
			sp_solver->solve(collisions, delta);
		}
	}
}

void World::set_gravity(const geomath::Vector3D gravity) {
	m_gravity = gravity;
}
