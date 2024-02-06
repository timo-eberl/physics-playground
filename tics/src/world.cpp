#include "tics.h"

#include <algorithm>
#include <cassert>

using tics::World;

void World::add_object(const std::weak_ptr<tics::Object> object) {
	m_objects.emplace_back(object);
}

void World::remove_object(const std::weak_ptr<tics::Object> object) {
	auto is_equals = [object](std::weak_ptr<tics::Object> obj) {
		return !obj.expired() && !object.expired() && object.lock() == obj.lock();
	};
	// find the object, move it to the end of the list and erase it
	m_objects.erase(std::remove_if(m_objects.begin(), m_objects.end(), is_equals), m_objects.end());
}

void World::update(const float delta) {
	for (auto wp_object : m_objects) {
		if (auto sp_object = wp_object.lock()) {
			sp_object->force += sp_object->mass * m_gravity;

			assert(sp_object->mass != 0.0f);
			auto acceleration = sp_object->force / sp_object->mass;
			sp_object->velocity += acceleration * delta;
			sp_object->position += sp_object->velocity * delta;

			sp_object->force = { 0.0f, 0.0f, 0.0f };
		}
	}
}

void World::set_gravity(const geomath::Vector3D gravity) {
	m_gravity = gravity;
}
