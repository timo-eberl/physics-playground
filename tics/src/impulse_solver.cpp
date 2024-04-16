#include "tics.h"

#include <cassert>

using tics::ImpulseSolver;

enum ObjectCombination { Invalid, RigidBodyRigidBody, RigidBodyStaticBody, StaticBodyRigidBody };

void ImpulseSolver::solve(std::vector<Collision>& collisions, float delta) {
	for (auto collision : collisions) {
		auto sp_a = collision.a.lock();
		auto sp_b = collision.b.lock();
		if (!sp_a || !sp_b) { continue; }

		// check the objects are a valid combination
		const auto rb_a = dynamic_cast<RigidBody *>(sp_a.get());
		const auto rb_b = dynamic_cast<RigidBody *>(sp_b.get());
		const auto sb_a = dynamic_cast<StaticBody *>(sp_a.get());
		const auto sb_b = dynamic_cast<StaticBody *>(sp_b.get());
		auto object_combination = ObjectCombination();
		if (rb_a && rb_b) { object_combination = RigidBodyRigidBody; }
		else if (rb_a && sb_b) { object_combination = RigidBodyStaticBody; }
		else if (sb_a && rb_b) { object_combination = StaticBodyRigidBody; }
		else { continue; } // no valid object combination combination

		const auto velocity_a = rb_a ? rb_a->velocity : gm::Vector3(0.0, 0.0, 0.0);
		const auto velocity_b = rb_b ? rb_b->velocity : gm::Vector3(0.0, 0.0, 0.0);

		const auto relative_velocity = velocity_a - velocity_b;
		// relative velocity in the collision normal direction
		const auto n_dot_v = gm::dot(relative_velocity, collision.points.normal);

		// the objects are separating which means the collision response has already happened
		if (n_dot_v >= 0.0) { continue; }

		// coefficient of restitution (bounciness)
		const auto cor = 0.75f;
		
		const auto velocity_change = collision.points.normal * (1.0f + cor) * n_dot_v;

		switch (object_combination) {
			case RigidBodyRigidBody: {
				// a and b are dynamic -> accelerate both taking into account their weights
				const auto b_percentage_of_total_mass = (rb_b->mass) / (rb_a->mass + rb_b->mass);
				rb_a->velocity -= velocity_change * b_percentage_of_total_mass;
				rb_b->velocity += velocity_change * (1.0f - b_percentage_of_total_mass);
			} break;
			case RigidBodyStaticBody: // b is static -> accelerate only a
				rb_a->velocity -= velocity_change;
				break;
			case StaticBodyRigidBody: // a is static -> accelerate only b
				rb_b->velocity += velocity_change;
			default: break;
		}
	}
}
