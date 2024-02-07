#include "raylib.h"

#include "tics.h"

int main () {
	int frame_rate = 60;
	InitWindow(1280, 720, "Physics Playground");

	// Define the camera to look into our 3d world
	Camera3D camera = { 0 };
	camera.position = { 7.0f, 4.0f, 7.0f };
	camera.target = { 0.0f, 0.0f, 0.0f };
	camera.up = { 0.0f, 1.0f, 0.0f };
	camera.fovy = 60.0f;
	camera.projection = CAMERA_PERSPECTIVE;

	float sphereRadius = 1.5f;

	SetTargetFPS(frame_rate);

	tics::World physics_world;
	physics_world.set_gravity({ 0.0, -5.0, 0.0 });

	auto sphere1 = std::make_shared<tics::Object>();
	auto sphere1_collider = std::make_shared<tics::SphereCollider>();
	sphere1_collider->radius = sphereRadius;
	sphere1->collider = sphere1_collider;
	auto sphere1_transform = std::make_shared<tics::ObjectTransform>();
	sphere1->transform = sphere1_transform;
	sphere1_transform->position = { 0.0, -2.0, 0.0 };
	sphere1->velocity = { 0.0, 8.0, 5.0 };
	physics_world.add_object(sphere1);

	auto sphere2 = std::make_shared<tics::Object>();
	auto sphere2_collider = std::make_shared<tics::SphereCollider>();
	sphere2_collider->radius = sphereRadius;
	sphere2->collider = sphere2_collider;
	auto sphere2_transform = std::make_shared<tics::ObjectTransform>();
	sphere2->transform = sphere2_transform;
	sphere2_transform->position = { 5.0, -3.0, 0.0 };
	sphere2->velocity = { -5.0, 10.0, 5.0 };
	physics_world.add_object(sphere2);

	auto plane = std::make_shared<tics::Object>();
	auto plane_collider = std::make_shared<tics::PlaneCollider>();
	plane->collider = plane_collider;
	plane_collider->normal = { 0.0, 1.0, 0.0 };
	auto plane_transform = std::make_shared<tics::ObjectTransform>();
	plane->transform = plane_transform;
	plane_transform->position = { 0.0, -0.05, 0.0 };
	plane->velocity = { 0.0, 0.0, 0.0 };
	plane->gravity_scale = 0.0f;
	physics_world.add_object(plane);

	while (!WindowShouldClose()) {
		physics_world.update(1.0f/frame_rate);

		UpdateCamera(&camera, CAMERA_ORBITAL);

		// "zoom" with mouse wheel
		float mouse_wheel = GetMouseWheelMove();
		camera.position.x -= camera.position.x * 0.1f * mouse_wheel;
		camera.position.y -= camera.position.y * 0.1f * mouse_wheel;
		camera.position.z -= camera.position.z * 0.1f * mouse_wheel;

		BeginDrawing();

			ClearBackground(Color { 58, 58, 58, 255 });

			BeginMode3D(camera);

				Vector3 sphere1_pos = {
					static_cast<float>(sphere1_transform->position.x),
					static_cast<float>(sphere1_transform->position.y),
					static_cast<float>(sphere1_transform->position.z),
				};
				DrawSphereEx(
					sphere1_pos, sphereRadius, 50, 50, sphere1->is_colliding ? RED : BLUE
				);
				DrawSphereWires(sphere1_pos, sphereRadius, 50, 4, WHITE);

				Vector3 sphere2_pos = {
					static_cast<float>(sphere2_transform->position.x),
					static_cast<float>(sphere2_transform->position.y),
					static_cast<float>(sphere2_transform->position.z),
				};
				DrawSphereEx(
					sphere2_pos, sphereRadius, 50, 50, sphere2->is_colliding ? RED : BEIGE
				);
				DrawSphereWires(sphere2_pos, sphereRadius, 50, 4, WHITE);

				Vector3 plane_pos = {
					static_cast<float>(plane_transform->position.x),
					static_cast<float>(plane_transform->position.y),
					static_cast<float>(plane_transform->position.z),
				};
				DrawPlane(plane_pos, { 1000.0f, 1000.0f }, Color { 30, 30, 30, 255 });

				DrawGrid(1000, 1.0f);

			EndMode3D();

			DrawFPS(10, 10);

		EndDrawing();
	}

	CloseWindow();

	return 0;
}
