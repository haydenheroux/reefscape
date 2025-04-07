#include "au/units/inches.hh"
#include "ntcore_c.h"
#include "ntcore_cpp.h"
#include "ntcore_cpp_types.h"
#include "raylib.h"
#include "raymath.h"
#include "render.hh"
#include "units.hh"
#include <cmath>

using namespace units;

int main() {
  auto client = nt::CreateInstance();
  nt::StartClient4(client, "client");
  nt::SetServer(client, "127.0.0.1", 5810);

  auto sub =
      nt::Subscribe(nt::GetTopic(client, "/position"), NT_DOUBLE, "double");

  SetConfigFlags(FLAG_MSAA_4X_HINT);
  InitWindow(render::kWindowWidth.in(pixels), render::kWindowHeight.in(pixels),
             "reefscape elevator simulator");
  SetTargetFPS(60);

  AngularVelocityUnit camera_omega = (degrees / second)(12);

  Vector3 camera_position;
  camera_position.x = meters(3).in(raylib_unit);
  camera_position.y = inches(70.0).in(raylib_unit);
  camera_position.z = meters(0).in(raylib_unit);

  Vector3 camera_target;
  camera_target.x = meters(0).in(raylib_unit);
  camera_target.y = inches(36.0).in(raylib_unit);
  camera_target.z = meters(0).in(raylib_unit);

  Vector3 up{0, 1, 0};

  Camera camera;
  camera.position = camera_position;
  camera.target = camera_target;
  camera.up = up;
  camera.fovy = 45;
  camera.projection = CAMERA_PERSPECTIVE;

  while (!WindowShouldClose()) {
    TimeUnit elapsed_time = seconds(GetFrameTime());

    DisplacementUnit position = meters(nt::GetDouble(sub, 0.0));

    camera.position = Vector3RotateByAxisAngle(
        camera.position, up, (camera_omega * elapsed_time).in(radians));

    BeginDrawing();
    ClearBackground(WHITE);
    BeginMode3D(camera);
    render::DrawRobot(position);
    DrawPlane(Vector3{0, 0, 0}, Vector2{1000, 1000}, LIGHTGRAY);
    EndMode3D();
    EndDrawing();
  }

  CloseWindow();
}
