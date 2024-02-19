#include "tics.h"

#include <iostream>

using tics::ImpulseSolver;

void ImpulseSolver::solve(std::vector<Collision>& collisions, float delta) {
	for (auto collision : collisions) {
		auto sp_a = collision.a.lock();
		auto sp_b = collision.b.lock();
		if (!sp_a || !sp_b) { continue; }
		if (!sp_a->is_dynamic && !sp_b->is_dynamic) { continue; }
		
		const auto a_velocity = sp_a->is_dynamic ? sp_a->velocity : geomath::Vector3D { 0.0, 0.0, 0.0 };
		const auto b_velocity = sp_b->is_dynamic ? sp_b->velocity : geomath::Vector3D { 0.0, 0.0, 0.0 };

		const auto relative_velocity = a_velocity - b_velocity;
		// relative velocity in the collision normal direction
		const auto n_dot_v = relative_velocity.dot(collision.points.normal);

		// the objects are separating which means the collision response has already happened
		if (n_dot_v >= 0.0) { continue; }

		// coefficient of restitution
		const auto cor = 0.75f;
		
		const auto velocity_change = collision.points.normal * (1.0f + cor) * n_dot_v;

		if (!sp_b->is_dynamic) {
			// b is static -> accelerate only a
			sp_a->velocity -= velocity_change;
		}
		else if (!sp_a->is_dynamic) {
			// a is static -> accelerate only b
			sp_b->velocity += velocity_change;
		}
		else {
			// a and b are dynamic -> accelerate both taking into account their weights
			const auto b_percentage_of_total_mass = (sp_b->mass) / (sp_a->mass + sp_b->mass);
			sp_a->velocity -= velocity_change * b_percentage_of_total_mass;
			sp_b->velocity += velocity_change * (1.0f - b_percentage_of_total_mass);
		}
	}
}
