#include "raylib.h"

#include "tics.h"

int main () {
    InitWindow(1280, 720, "Physics Playground");

    // Define the camera to look into our 3d world
    Camera3D camera = { 0 };
    camera.position = (Vector3){ 2.5f, 2.5f, 2.5f };
    camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    Vector3 spherePosition = { 0.0f, 0.0f, 0.0f };
    float sphereRadius = 1.0f;

    SetTargetFPS(60);

    tics::World physics_world;

    while (!WindowShouldClose()) {
        // "zoom" with mouse wheel
        float mouse_wheel = GetMouseWheelMove();
        camera.position.x -= mouse_wheel;
        camera.position.y -= mouse_wheel;
        camera.position.z -= mouse_wheel;

        UpdateCamera(&camera, CAMERA_ORBITAL);

        BeginDrawing();

            ClearBackground(Color { 58, 58, 58, 255 });

            BeginMode3D(camera);

                DrawSphereEx(spherePosition, sphereRadius, 25, 25, MAROON);
                DrawSphereWires(spherePosition, sphereRadius, 25, 25, BEIGE);

                DrawGrid(10, 1.0f);

            EndMode3D();

            DrawFPS(10, 10);

        EndDrawing();
    }

    CloseWindow();

    return 0;
}
