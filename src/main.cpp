#include "raylib.h"

#include "tics.h"

int main () {
	int frame_rate = 60;
	InitWindow(1280, 720, "Physics Playground");

	// Define the camera to look into our 3d world
	Camera3D camera = { 0 };
	camera.position = { 10.0f, 5.0f, 10.0f };
	camera.target = { 0.0f, 0.0f, 0.0f };
	camera.up = { 0.0f, 1.0f, 0.0f };
	camera.fovy = 60.0f;
	camera.projection = CAMERA_PERSPECTIVE;

	float sphereRadius = 0.5f;

	SetTargetFPS(frame_rate);

	tics::World physics_world;

	auto sphere = std::make_shared<tics::Object>();
	sphere->position = geomath::Vector3D(0.0f, 0.0f, 0.0f);
	sphere->velocity = geomath::Vector3D(-7.0f, 10.0f, 7.0f);
	sphere->mass = 1.0f;
	physics_world.add_object(sphere);

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

				Vector3 spherePos = { sphere->position.x, sphere->position.y, sphere->position.z };
				DrawSphereEx(spherePos, sphereRadius, 25, 25, MAROON);
				DrawSphereWires(spherePos, sphereRadius, 25, 25, BEIGE);

				DrawGrid(10, 1.0f);

			EndMode3D();

			DrawFPS(10, 10);

		EndDrawing();
	}

	CloseWindow();

	return 0;
}
