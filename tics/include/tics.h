#pragma once

#include <vector>
#include <memory>
#include <functional>

#include "geomath.h"

namespace tics {

struct ObjectTransform {
	geomath::Vector3D position;
	geomath::Vector3D scale = { 1.0, 1.0, 1.0 };
};

enum ColliderType {
	SPHERE,
	PLANE
};

struct Collider {
	ColliderType type;
};

struct SphereCollider : Collider {
	SphereCollider() { type = SPHERE; };
	geomath::Vector3D center = { 0.0, 0.0, 0.0 };
	float radius = 1.0f;
};

struct PlaneCollider : Collider {
	PlaneCollider() { type = PLANE; };
	geomath::Vector3D normal = { 0.0, 1.0, 0.0 };
	float distance = 0.0f;
};

struct CollisionPoints {
	geomath::Vector3D a; // furthest point of a into b
	geomath::Vector3D b; // furthest point of b into a
	geomath::Vector3D normal; // b - a normalized
	float depth; // length of b - a
	bool has_collision = false;
};

CollisionPoints collision_test(
	const Collider& a, const ObjectTransform& at,
	const Collider& b, const ObjectTransform& bt
);

struct Collision;

struct Object {
	geomath::Vector3D velocity = { 0.0, 0.0, 0.0 };
	geomath::Vector3D force = { 0.0, 0.0, 0.0 };
	float mass = 1.0f;
	float gravity_scale = 1.0f;

	std::function<void(const Collision&)> collision_event;

	bool is_dynamic = true;

	std::weak_ptr<Collider> collider;
	std::weak_ptr<ObjectTransform> transform;
};

struct Collision {
	const std::weak_ptr<Object> a;
	const std::weak_ptr<Object> b;
	const CollisionPoints points;
};

class ISolver {
public:
	virtual ~ISolver() {};

	virtual void solve(std::vector<Collision>& collisions, float delta) = 0;
};

class World {
public:
	void add_object(const std::weak_ptr<Object> object);
	void remove_object(const std::weak_ptr<Object> object);

	void add_solver(const std::weak_ptr<ISolver> solver);
	void remove_solver(const std::weak_ptr<ISolver> solver);

	void update(const float delta);

	void resolve_collisions(const float delta);

	void set_gravity(const geomath::Vector3D gravity);
	void set_collision_event(const std::function<void(const Collision&)> collision_event);
private:
	std::vector<std::weak_ptr<Object>> m_objects;
	std::vector<std::weak_ptr<ISolver>> m_solvers;
	geomath::Vector3D m_gravity = { 0.0f, -9.81f, 0.0f };
	std::function<void(const Collision&)> m_collision_event;
};

class ImpulseSolver : public ISolver {
public:
	~ImpulseSolver() {};

	virtual void solve(std::vector<Collision>& collisions, float delta) override;
};

class PositionSolver : public ISolver {
public:
	~PositionSolver() {};

	virtual void solve(std::vector<Collision>& collisions, float delta) override;
};

} // tics
