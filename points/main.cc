#include <deque>

#include "ntcore_cpp.h"
#include "pubsub.hh"
#include "raylib.h"
#include "units.hh"

using namespace reefscape;

struct Point {
  int tick;
  DisplacementUnit position;
  VelocityUnit velocity;
  VoltageUnit voltage;

  Vector3 Position() const;
  Color PointColor() const;
};

Vector3 Point::Position() const {
  float position_ = position.in(meters);
  float velocity_ = velocity.in(meters / second);
  float time = (tick % 480) / 960.0;
  return Vector3{position_, time, velocity_};
}

double map(double in, double in_min, double in_max, double out_min, double out_max) {
  return ((in - in_min) / (in_max - in_min)) * (out_max - out_min) + out_min;
}

Color Point::PointColor() const {
  float hue = map(voltage.in(volts), -12, 12, 165, 265);
  float saturation = 1;
  float value = 1;
  return ColorFromHSV(hue, saturation, value);
}

struct PointBuffer {
  std::deque<Point> points_;
  unsigned int max_points_;

  PointBuffer(int max_points) : max_points_(max_points) {};

  void push(Point point);
};

void PointBuffer::push(Point point) {
  auto size = points_.size();

  if (size == max_points_) {
    points_.pop_front();
  }

  points_.push_back(point);
}

int main() {
  auto client = nt::CreateInstance();
  nt::StartClient4(client, "client");
  nt::SetServer(client, "127.0.0.1", 5810);

  Subscriber subscriber{client};

  InitWindow(1280, 720, "TODO");
  SetTargetFPS(240);
  DisableCursor();

  PointBuffer points{30000};

  int tick = 0;

  Camera camera = {0};
  camera.position = Vector3{5, 5, 5};
  camera.target = Vector3{0, 0, 0};
  camera.up = Vector3{0, 1, 0};
  camera.fovy = 45;
  camera.projection = CAMERA_PERSPECTIVE;

  CameraMode mode = CAMERA_FIRST_PERSON;

  while (!WindowShouldClose()) {
    DisplacementUnit position = subscriber.Position();
    VelocityUnit velocity = subscriber.Velocity();
    VoltageUnit voltage = subscriber.Voltage();
    Point point{tick, position, velocity, voltage};
    points.push(point);
    tick++;


    if (IsKeyPressed(KEY_P)) {
      if (camera.projection == CAMERA_PERSPECTIVE) {
        camera.projection = CAMERA_ORTHOGRAPHIC;
        camera.up = Vector3{1, 0, 0};
        camera.fovy = 5;
        mode = CAMERA_FREE;
      } else {
        camera.projection = CAMERA_PERSPECTIVE;
        camera.up = Vector3{0, 1, 0};
        camera.fovy = 45;
        mode = CAMERA_FIRST_PERSON;
      }
    }

    if (IsKeyPressed(KEY_SPACE)) {
      camera.position = Vector3{0, 5, 0};
      camera.target = Vector3{0, 0, 0};
    }

    UpdateCamera(&camera, mode);

    BeginDrawing();
    ClearBackground(RAYWHITE);
    BeginMode3D(camera);
    DrawGrid(10, 1);
    for (auto point : points.points_) {
      DrawPoint3D(point.Position(), point.PointColor());
    }
    EndMode3D();
    EndDrawing();
  }

  CloseWindow();

  return 0;
}
