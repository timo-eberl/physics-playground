#include "raylib.h"

#include "tics.h"

#include <cmath>

struct Sphere {
	const std::shared_ptr<tics::Object> object;
	const std::shared_ptr<tics::ObjectTransform> transform;
	const std::shared_ptr<tics::SphereCollider> collider;
	Color color;
};

void create_sphere(
	Sphere &sphere,
	float radius, geomath::Vector3D position, geomath::Vector3D velocity, Color color = BLUE,
	float scale = 1.0f, float mass = 1.0f
) {
	sphere.color = color;
	sphere.collider->radius = radius;
	sphere.object->collider = sphere.collider;
	sphere.object->transform = sphere.transform;
	sphere.object->mass = mass;
	sphere.transform->position = position;
	sphere.transform->scale = { scale, scale, scale };
	sphere.object->velocity = velocity;
}

int main () {
	int frame_rate = 60;
	InitWindow(1280, 720, "Physics Playground");

	// Define the camera to look into our 3d world
	Camera3D camera = { 0 };
	camera.position = { 3.0f, 8.0f, 10.0f };
	camera.target = { 0.0f, 0.0f, 0.0f };
	camera.up = { 0.0f, 1.0f, 0.0f };
	camera.fovy = 60.0f;
	camera.projection = CAMERA_PERSPECTIVE;

	SetTargetFPS(frame_rate);

	tics::World physics_world;
	physics_world.set_gravity({ 0.0, -20.0, 0.0 });

	std::vector<Sphere> spheres {};

	auto sphere_1 = Sphere({
		std::make_shared<tics::Object>(),
		std::make_shared<tics::ObjectTransform>(),
		std::make_shared<tics::SphereCollider>()
	});
	create_sphere(
		sphere_1, 0.75f, { 0.0, 1.5, 0.25 }, { 0.0, 8.0, 0.0 }, BLUE
	);
	spheres.emplace_back(sphere_1);
	physics_world.add_object(sphere_1.object);

	auto sphere_2 = Sphere({
		std::make_shared<tics::Object>(),
		std::make_shared<tics::ObjectTransform>(),
		std::make_shared<tics::SphereCollider>()
	});
	create_sphere(
		sphere_2, 1.25f, { -3.0, 1.0, 0.0 }, { 2.0, 12.0, 0.0 }, BEIGE, 1.0f, 2.0f
	);
	spheres.emplace_back(sphere_2);
	physics_world.add_object(sphere_2.object);

	auto plane = std::make_shared<tics::Object>();
	auto plane_collider = std::make_shared<tics::PlaneCollider>();
	plane->collider = plane_collider;
	plane_collider->normal = { 0.0, 1.0, 0.0 };
	auto plane_transform = std::make_shared<tics::ObjectTransform>();
	plane->transform = plane_transform;
	plane_transform->position = { -0.0, -0.05, 0.0 };
	plane->is_dynamic = false;
	physics_world.add_object(plane);

	auto impulse_solver = std::make_shared<tics::ImpulseSolver>();
	auto position_solver = std::make_shared<tics::PositionSolver>();
	physics_world.add_solver(impulse_solver);
	physics_world.add_solver(position_solver);

	while (!WindowShouldClose()) {
		if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
			auto sphere = Sphere({
				std::make_shared<tics::Object>(),
				std::make_shared<tics::ObjectTransform>(),
				std::make_shared<tics::SphereCollider>()
			});
			create_sphere(
				sphere, 0.75f,
				{ // position
					(0.5-static_cast<double>(std::rand()) / RAND_MAX)*1.0,
					(0.5-static_cast<double>(std::rand()) / RAND_MAX) + 3.0,
					(0.5-static_cast<double>(std::rand())/ RAND_MAX)*1.0
				},
				{ // velocity
					(0.5-static_cast<double>(std::rand()) / RAND_MAX)*5,
					(static_cast<double>(std::rand()) / RAND_MAX)*5,
					(0.5-static_cast<double>(std::rand()) / RAND_MAX)*5
				},
				Color {
					static_cast<unsigned char>(std::rand() % 256),
					static_cast<unsigned char>(std::rand() % 256),
					static_cast<unsigned char>(std::rand() % 256),
					255
				}
			);
			spheres.emplace_back(sphere);
			physics_world.add_object(sphere.object);
		}

		float time_scale = 1.0f;
		physics_world.update(time_scale/frame_rate);

		// UpdateCamera(&camera, CAMERA_ORBITAL);

		// "zoom" with mouse wheel
		float mouse_wheel = GetMouseWheelMove();
		camera.position.x -= camera.position.x * 0.1f * mouse_wheel;
		camera.position.y -= camera.position.y * 0.1f * mouse_wheel;
		camera.position.z -= camera.position.z * 0.1f * mouse_wheel;

		BeginDrawing();

			ClearBackground(Color { 58, 58, 58, 255 });

			BeginMode3D(camera);

				for (auto sphere : spheres) {
					Vector3 sphere_pos = {
						static_cast<float>(sphere.transform->position.x),
						static_cast<float>(sphere.transform->position.y),
						static_cast<float>(sphere.transform->position.z),
					};
					const auto radius = sphere.collider->radius;
					DrawSphereEx(
						sphere_pos, radius, 10, 10, sphere.color
					);
				}

				Vector3 plane_pos = {
					static_cast<float>(plane_transform->position.x),
					static_cast<float>(plane_transform->position.y),
					static_cast<float>(plane_transform->position.z),
				};
				DrawPlane(plane_pos, { 1000.0f, 1000.0f }, Color { 30, 30, 30, 255 });

				DrawGrid(1000, 1.0f);

				DrawLine3D({0.0f, 0.01f, 0.0f}, {1000.0f,   0.01f,    0.0f}, RED);
				DrawLine3D({0.0f,  0.0f, 0.0f}, {   0.0f, 1000.0f,    0.0f}, GREEN);
				DrawLine3D({0.0f, 0.01f, 0.0f}, {   0.0f,   0.01f, 1000.0f}, BLUE);

			EndMode3D();

			DrawFPS(10, 10);

		EndDrawing();
	}

	CloseWindow();

	return 0;
}
