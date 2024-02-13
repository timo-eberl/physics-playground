#include "tics.h"

using tics::ImpulseSolver;

void ImpulseSolver::solve(std::vector<Collision>& collisions, float delta) {
	for (auto collision : collisions) {
		auto sp_a = collision.a.lock();
		auto sp_b = collision.b.lock();
		if (!sp_a || !sp_b) { continue; }
		if (!sp_a->is_dynamic && !sp_b->is_dynamic) { continue; }
		
		const auto a_velocity = sp_a->is_dynamic ? sp_a->velocity : geomath::Vector3D { 0.0, 0.0, 0.0 };
		const auto b_velocity = sp_b->is_dynamic ? sp_b->velocity : geomath::Vector3D { 0.0, 0.0, 0.0 };

		const auto r_velocity = a_velocity - b_velocity;
		const auto n_speed = r_velocity.dot(collision.points.normal);

		// a negative impulse would drive the objects closer together
		if (n_speed >= 0.0) { continue; }

		const auto a_inv_mass = sp_a->is_dynamic ? 1.0f / sp_a->mass : 1.0f;
		const auto b_inv_mass = sp_b->is_dynamic ? 1.0f / sp_b->mass : 1.0f;

		const auto a_bounciness = 1.0f;
		const auto b_bounciness = 1.0f;

		const auto impulse_strength =
			(1.0f + (a_bounciness * b_bounciness)) * n_speed
			/ (a_inv_mass + b_inv_mass);
		
		const auto impulse = collision.points.normal * impulse_strength;

		if (sp_a->is_dynamic) {
			sp_a->velocity -= impulse * a_inv_mass;
		}
		if (sp_b->is_dynamic) {
			sp_b->velocity += impulse * b_inv_mass;
		}
	}
}
