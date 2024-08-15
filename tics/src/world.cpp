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

// test that helped me find a bug in the Terathon library
static void test_terathon_geometric_anti_product() {
	{
		const auto R = Terathon::Motor3D::MakeRotation(
			0.2 * 3.141, Terathon::Normalize(Terathon::Bivector3D(1,0.3,-0.05))
		);
		const auto T = Terathon::Motor3D::MakeTranslation(
			Terathon::Vector3D(0.2, -0.5, 0.1)
		);

		// rotate, then translate

		auto p_rt = Terathon::Transform(Terathon::Point3D(-1.0, 0.5, 0.5), R);
		p_rt = Terathon::Transform(p_rt, T);

		const auto Q_0 = T * R;
		const auto p_q_0 = Terathon::Transform(Terathon::Point3D(-1.0, 0.5, 0.5), Q_0);
		assert(Terathon::Magnitude(p_rt - p_q_0) < 0.001);

		auto Q_1 = T * R.v;
		const auto p_q_1 = Terathon::Transform(Terathon::Point3D(-1.0, 0.5, 0.5), Q_1);
		assert(Terathon::Magnitude(p_rt - p_q_1) < 0.001);

		// translate, then rotate

		auto p_tr = Terathon::Transform(Terathon::Point3D(-1.0, 0.5, 0.5), T);
		p_tr = Terathon::Transform(p_tr, R);

		auto Q_2 = R * T;
		const auto p_q_2 = Terathon::Transform(Terathon::Point3D(-1.0, 0.5, 0.5), Q_2);
		assert(Terathon::Magnitude(p_tr - p_q_2) < 0.001);

		auto Q_3 = R.v * T;
		const auto p_q_3 = Terathon::Transform(Terathon::Point3D(-1.0, 0.5, 0.5), Q_3);
		assert(Terathon::Magnitude(p_tr - p_q_3) < 0.001);
	}

	{
		// first frame
		const auto R_1 = Terathon::Motor3D::MakeRotation(
			0.2 * 3.141, Terathon::Normalize(Terathon::Bivector3D(1,0.3,-0.05))
		);
		const auto T_1 = Terathon::Motor3D::MakeTranslation(
			Terathon::Vector3D(0.2, -0.5, 0.01)
		);
		const auto Q_1 = T_1 * R_1;
		// second frame
		const auto R_2 = Terathon::Motor3D::MakeRotation(
			0.2 * 3.141, Terathon::Normalize(Terathon::Bivector3D(1,0.3,-0.05))
		);
		const auto T_2 = Terathon::Motor3D::MakeTranslation(
			Terathon::Vector3D(0.2, -0.5, 0.01)
		);
		const auto Q_2 = T_2 * R_2;

		// decompase previous frame to translation and rotation
		// const auto T_1_rec = Terathon::Motor3D::MakeTranslation( Q_1.GetPosition() );
		const auto T_1_rec = Q_1 * Terathon::Inverse(Q_1.v);
		const auto R_1_rec = Q_1.v;
		const auto combined_motor = T_1_rec * Q_2 * R_1_rec;
		const auto control_motor = (T_1 * T_2) * (R_1 * R_2);

		const auto p_comb = Terathon::Transform(Terathon::Point3D(-1.0, 0.5, 0.5), combined_motor);
		const auto p_c_0 = Terathon::Transform(Terathon::Point3D(-1.0, 0.5, 0.5), control_motor);
		auto p_c_1 = Terathon::Transform(Terathon::Point3D(-1.0, 0.5, 0.5), R_1);
		p_c_1 = Terathon::Transform(p_c_1, R_2);
		p_c_1 = Terathon::Transform(p_c_1, T_1);
		p_c_1 = Terathon::Transform(p_c_1, T_2);
		assert(Terathon::Magnitude(p_c_1 - p_c_0) < 0.001);
		assert(Terathon::Magnitude(p_comb - p_c_1) < 0.001);
		assert(Terathon::Magnitude(p_comb - p_c_0) < 0.001);
	}
}

static Terathon::Motor3D scale_motor(const Terathon::Motor3D &motor, const float scale) {
	// "scale" operator by delta
	// - implemented as a lerp(identity, motor) + unitize
	// - maybe it could be implemented more optimized
	// - used += because no + operator exists ...
	auto m = Terathon::Motor3D::identity * (1.0 - scale);
	m +=     motor                       *        scale;
	// m.Unitize();
	m.v.Normalize();
	return m;
}

static Terathon::Quaternion scale_quaternion(const Terathon::Quaternion &quaternion, const float scale) {
	// "scale" operator by delta
	// - implemented as a lerp(identity, quaternion) + normalize
	// - maybe it could be implemented more optimized
	auto q = Terathon::Quaternion::identity * (1.0 - scale);
	q +=     quaternion                     *        scale;
	q.Normalize();
	return q;
}

void World::update(const float delta) {
	resolve_collisions(delta);
	test_terathon_geometric_anti_product();

	// dynamics
	for (auto wp_object : m_objects) {
		if (auto sp_object = wp_object.lock()) {
			const auto rigid_body = dynamic_cast<RigidBody *>(sp_object.get());
			if (!rigid_body) { continue; }

			const auto &transform = rigid_body->get_transform().lock();

			// start with linear impulse (will be supplied)
			auto motor_impulse = Terathon::Motor3D::MakeTranslation(rigid_body->impulse * 0.1f);
			// add gravity
			const auto gravity_impulse = Terathon::Motor3D::MakeTranslation(
				(rigid_body->mass * delta * rigid_body->gravity_scale) * m_gravity
			);
			motor_impulse = gravity_impulse * motor_impulse;
			assert(rigid_body->mass != 0.0f);
			// convert from kg*m/0.1s to kg*m/s
			motor_impulse = scale_motor(motor_impulse, 10.0f);
			// apply impulse -> accelerate
			rigid_body->motor_velocity = rigid_body->motor_velocity * motor_impulse;
			const auto transform_change = scale_motor(rigid_body->motor_velocity, delta);
			transform->motor = transform_change * transform->motor;

			// linear movement
			// {
				rigid_body->impulse += rigid_body->mass * delta * rigid_body->gravity_scale * m_gravity;

				assert(rigid_body->mass != 0.0f);
				rigid_body->velocity += rigid_body->impulse / rigid_body->mass;

				transform->position += rigid_body->velocity * delta;
			// }

			// angular movement
			// {
				// angular velocity is stored in meter / 0.1s, because a quaternion
				// using rad/s would only be able to store a maximum of 1 rotation per second

				const auto angular_impulse = scale_quaternion(
					rigid_body->an_imp_div_sq_dst, 1.0f/rigid_body->mass
				);
				assert(angular_impulse.x == angular_impulse.x);

				// accelerate angular velocity by angular impulse
				rigid_body->angular_velocity = angular_impulse * rigid_body->angular_velocity;
				assert(rigid_body->angular_velocity.x == rigid_body->angular_velocity.x);

				// "scale" velocity by delta to get angular position change of current frame
				const auto pos_change = scale_quaternion(rigid_body->angular_velocity, delta*10.0);
				transform->rotation = transform->rotation * pos_change;
			// }

			// translation motor test
			auto t_motor = Terathon::Motor3D::MakeTranslation(rigid_body->velocity*0.1);
			t_motor = scale_motor(t_motor, delta*10.0);
			const auto t_pos_motor = Terathon::Transform(Terathon::Point3D(-1.0, 0.5, 0.5), t_motor);
			const auto t_pos_conv  = Terathon::Point3D(-1.0, 0.5, 0.5) + (rigid_body->velocity * delta);
			assert(Terathon::Magnitude(t_pos_motor - t_pos_conv) < 0.001);

			// rotation motor test
			auto r_motor = Terathon::Motor3D(rigid_body->angular_velocity);
			const auto rot_pos_motor = Terathon::Transform(Terathon::Point3D(-1.0, 0.5, 0.5), r_motor);
			const auto rot_pos_quat  = Terathon::Transform(Terathon::Point3D(-1.0, 0.5, 0.5), rigid_body->angular_velocity);
			assert(Terathon::Magnitude(rot_pos_motor - rot_pos_quat) < 0.001);

			// motor test
			auto motor = t_motor * r_motor;
			const auto pos_motor = Terathon::Transform(Terathon::Point3D(-1.0, 0.5, 0.5), motor);
			const auto pos_conv  = (
				Terathon::Transform(Terathon::Point3D(-1.0, 0.5, 0.5), rigid_body->angular_velocity)
				+ (rigid_body->velocity * delta)
			);
			assert(Terathon::Magnitude(pos_motor - pos_conv) < 0.001);

			// linear friction
			const auto lin_fric = 0.1f;
			rigid_body->velocity -= rigid_body->velocity * (lin_fric * delta);
			// angular friction
			const auto ang_fric = 0.8f;
			rigid_body->angular_velocity = scale_quaternion(rigid_body->angular_velocity, 1.0 - (ang_fric*delta));

			// reset impulses
			rigid_body->impulse = Terathon::Vector3D(0,0,0);
			rigid_body->an_imp_div_sq_dst = Terathon::Quaternion::identity;
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
