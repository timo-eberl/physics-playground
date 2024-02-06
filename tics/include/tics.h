#pragma once

#include <vector>
#include <memory>

#include "geomath.h"

namespace tics {

struct Object {
	geomath::Vector3D position;
	geomath::Vector3D velocity;
	geomath::Vector3D force;
	float mass = 1.0f;
};

class World {
public:
	void add_object(const std::weak_ptr<Object> object);
	void remove_object(const std::weak_ptr<Object> object);
	void update(const float delta);

	void set_gravity(const geomath::Vector3D gravity);
private:
	std::vector<std::weak_ptr<Object>> m_objects;
	geomath::Vector3D m_gravity = { 0.0f, -9.81f, 0.0f };
};

} // tics
