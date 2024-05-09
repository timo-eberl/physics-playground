#pragma once

#include <vector>
#include <unordered_map>
#include <map>
#include <memory>
#include <functional>

#include <geomath.h>

namespace tics {

struct Transform {
	gm::Vector3 position;
	gm::Vector3 scale = gm::Vector3(1);
};

enum ColliderType {
	SPHERE,
	PLANE,
	MESH,
};

struct Collider {
	ColliderType type;
};

struct SphereCollider : Collider {
	SphereCollider() { type = SPHERE; };
	gm::Vector3 center = gm::Vector3(0);
	float radius = 1.0f;
};

struct PlaneCollider : Collider {
	PlaneCollider() { type = PLANE; };
	gm::Vector3 normal = gm::Vector3(0, 1, 0);
	float distance = 0.0f;
};

struct MeshCollider : Collider {
	MeshCollider() { type = MESH; };
	std::vector<gm::Vector3> positions = {};
	std::vector<uint32_t> indices = {};
};

bool raycast(const MeshCollider &mesh_collider, const gm::Vector3 ray_start, const gm::Vector3 ray_end);

struct CollisionPoints {
	gm::Vector3 a; // furthest point of a into b
	gm::Vector3 b; // furthest point of b into a
	gm::Vector3 normal; // b - a normalized
	float depth; // length of b - a
	bool has_collision = false;
};

CollisionPoints collision_test(
	const Collider& a, const Transform& at,
	const Collider& b, const Transform& bt
);

struct Collision;

class ICollisionObject {
public:
	virtual ~ICollisionObject() = default;

	virtual void set_collider(const std::weak_ptr<Collider> collider) = 0;
	virtual std::weak_ptr<Collider> get_collider() const = 0;

	virtual void set_transform(const std::weak_ptr<Transform> transform) = 0;
	virtual std::weak_ptr<Transform> get_transform() const = 0;
};

// A physics body that is not moved by physics simulation. RigidBodies can collide with it.
// When moved manually, it doesn't affect objects in its path.
class StaticBody : public ICollisionObject {
public:
	virtual ~StaticBody() = default;
	virtual void set_collider(const std::weak_ptr<Collider> collider) override;
	virtual std::weak_ptr<Collider> get_collider() const override;
	virtual void set_transform(const std::weak_ptr<Transform> transform) override;
	virtual std::weak_ptr<Transform> get_transform() const override;
private:
	std::weak_ptr<Collider> m_collider;
	std::weak_ptr<Transform> m_transform;
};

// A physics body that is moved by physics simulation.
class RigidBody : public ICollisionObject {
public:
	virtual ~RigidBody() = default;
	virtual void set_collider(const std::weak_ptr<Collider> collider) override;
	virtual std::weak_ptr<Collider> get_collider() const override;
	virtual void set_transform(const std::weak_ptr<Transform> transform) override;
	virtual std::weak_ptr<Transform> get_transform() const override;

	gm::Vector3 velocity = gm::Vector3(0);
	gm::Vector3 force = gm::Vector3(0);

	float mass = 1.0f;
	float gravity_scale = 1.0f;
private:
	std::weak_ptr<Collider> m_collider;
	std::weak_ptr<Transform> m_transform;
};

// A region that detects other CollisionAreas, RigidBodies and StaticBodies entering or exiting it
class CollisionArea : public ICollisionObject {
public:
	virtual ~CollisionArea() = default;
	virtual void set_collider(const std::weak_ptr<Collider> collider) override;
	virtual std::weak_ptr<Collider> get_collider() const override;
	virtual void set_transform(const std::weak_ptr<Transform> transform) override;
	virtual std::weak_ptr<Transform> get_transform() const override;

	std::function<void(const std::weak_ptr<ICollisionObject> other)> on_collision_enter;
	std::function<void(const std::weak_ptr<ICollisionObject> other)> on_collision_exit;
private:
	std::weak_ptr<Collider> m_collider;
	std::weak_ptr<Transform> m_transform;
};

struct Collision {
	const std::weak_ptr<ICollisionObject> a;
	const std::weak_ptr<ICollisionObject> b;
	const CollisionPoints points;
};

class ISolver {
public:
	virtual ~ISolver() {};

	virtual void solve(std::vector<Collision>& collisions, float delta) = 0;
};

class World {
public:
	void add_object(const std::weak_ptr<ICollisionObject> object);
	void remove_object(const std::weak_ptr<ICollisionObject> object);

	void add_solver(const std::weak_ptr<ISolver> solver);
	void remove_solver(const std::weak_ptr<ISolver> solver);

	void update(const float delta);

	void resolve_collisions(const float delta);

	void set_gravity(const gm::Vector3 gravity);
	void set_collision_event(const std::function<void(const Collision&)> collision_event);
private:
	std::vector<std::weak_ptr<ICollisionObject>> m_objects;
	std::vector<std::weak_ptr<ISolver>> m_solvers;
	gm::Vector3 m_gravity = gm::Vector3(0.0f, -9.81f, 0.0f);
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

class CollisionAreaSolver : public ISolver {
public:
	~CollisionAreaSolver() {};

	virtual void solve(std::vector<Collision>& collisions, float delta) override;
private:
	typedef std::map<CollisionArea *, std::vector<std::weak_ptr<ICollisionObject>>>
		AreasCollisionRecord;

	AreasCollisionRecord m_areas_collision_record = {};
};

} // tics
