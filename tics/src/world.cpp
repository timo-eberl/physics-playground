#include "tics.h"

#include <algorithm>
#include <cassert>
#include <iostream>

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

			rigid_body->impulse += rigid_body->mass * delta * rigid_body->gravity_scale * m_gravity;

			assert(rigid_body->mass != 0.0f);
			rigid_body->velocity += rigid_body->impulse / rigid_body->mass;

			const auto &transform = rigid_body->get_transform().lock();
			transform->position += rigid_body->velocity * delta;

			// angular velocity is stored in meter / 0.01s, because a quaternion
			// using rad/s would only be able to store a maximum of 0.5 rotations per second

			if (rigid_body->angular_impulse.x != rigid_body->angular_impulse.x) {
				rigid_body->angular_impulse = Terathon::Quaternion::identity;
			}

			// "scale" angular impulse
			auto interpolator = 1.0f/rigid_body->moment_of_inertia;
			rigid_body->angular_impulse = (
				Terathon::Quaternion::identity * (1.0 - interpolator)
				+ rigid_body->angular_impulse *         interpolator
			);
			rigid_body->angular_impulse.Normalize();
			assert(rigid_body->angular_impulse.x == rigid_body->angular_impulse.x);

			// accelerate angular velocity by angular impulse
			rigid_body->angular_velocity *= rigid_body->angular_impulse;
			assert(rigid_body->angular_velocity.x == rigid_body->angular_velocity.x);

			// "scale" velocity by delta to get angular impulse of current frame
			auto angular_impulse = (
				Terathon::Quaternion::identity * (1.0 - delta*100.0)
				+ rigid_body->angular_velocity *        delta*100.0
			);
			angular_impulse.Normalize();

			// "add" impulse
			transform->rotation *= angular_impulse;

			rigid_body->impulse = Terathon::Vector3D(0,0,0);
			rigid_body->angular_impulse = Terathon::Quaternion::identity;

			// friction
			interpolator = 0.3f * delta;
			rigid_body->velocity -= rigid_body->velocity * interpolator;
			interpolator = 1.5f * delta;
			rigid_body->angular_velocity = (
				  Terathon::Quaternion::identity *      interpolator
				+ rigid_body->angular_velocity * (1.0 - interpolator)
			);
			rigid_body->angular_velocity.Normalize();
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

			// don't test static against static
			const auto sb_a = dynamic_cast<StaticBody *>(sp_a.get());
			const auto sb_b = dynamic_cast<StaticBody *>(sp_b.get());
			if (sb_a && sb_b) { continue; }

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
