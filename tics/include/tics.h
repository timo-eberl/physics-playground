#pragma once

#include <vector>
#include <memory>

#include "geomath.h"

namespace tics {

struct ObjectTransform {
	geomath::Vector3D position;
	geomath::Quaternion rotation;
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
	geomath::Vector3D center;
	float radius;
};

struct PlaneCollider : Collider {
	PlaneCollider() { type = PLANE; };
	geomath::Vector3D normal;
	float distance;
};

struct CollisionPoints {
	geomath::Vector3D a; // furthest point of a into b
	geomath::Vector3D b; // furthest point of b into a
	geomath::Vector3D normal; // b - a normalized
	float depth; // length of b - a
	bool has_collision;
};

CollisionPoints collision_test(
	const Collider& a, const ObjectTransform& at,
	const Collider& b, const ObjectTransform& bt
);

struct Object {
	geomath::Vector3D velocity;
	geomath::Vector3D force;
	float mass = 1.0f;
	float gravity_scale = 1.0f;
	bool is_colliding; // for debugging

	std::weak_ptr<Collider> collider;
	std::weak_ptr<ObjectTransform> transform;
};

struct Collision {
	const std::weak_ptr<Object> a;
	const std::weak_ptr<Object> b;
	const CollisionPoints points;
};

class World {
public:
	void add_object(const std::weak_ptr<Object> object);
	void remove_object(const std::weak_ptr<Object> object);
	void update(const float delta);

	void resolve_collisions(const float delta);

	void set_gravity(const geomath::Vector3D gravity);
private:
	std::vector<std::weak_ptr<Object>> m_objects;
	geomath::Vector3D m_gravity = { 0.0f, -9.81f, 0.0f };
};

} // tics
