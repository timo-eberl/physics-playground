#include "tics.h"

#include <cmath>

using tics::PositionSolver;

void PositionSolver::solve(std::vector<Collision>& collisions, float delta) {
	for (auto collision : collisions) {
		auto sp_a = collision.a.lock();
		auto sp_b = collision.b.lock();
		if (!sp_a || !sp_b) { continue; }
		if (!sp_a->is_dynamic && !sp_b->is_dynamic) { continue; }

		const auto percent = 0.8;
		const auto depth_tolerance = 0.01; // how much they are allowed to glitch into another

		const auto depth_with_tolerance = fmax(collision.points.depth - depth_tolerance, 0.0);
		// distance that the objects are moved away from each other
		const auto correction = collision.points.normal * ( percent *  depth_with_tolerance);

		if (sp_a->is_dynamic && sp_b->is_dynamic) {
			const auto b_percentage_of_total_mass = (sp_b->mass) / (sp_a->mass + sp_b->mass);
			// if b is heavier, move a more
			sp_a->transform.lock()->position += correction * b_percentage_of_total_mass;
			sp_b->transform.lock()->position -= correction * (1.0f - b_percentage_of_total_mass);
		}
		else if (sp_a->is_dynamic) {
			// b is static -> move only a
			sp_a->transform.lock()->position += correction;
		}
		else {
			// a is static -> move only b
			sp_b->transform.lock()->position -= correction;
		}
	}
}
