#include <cmath>

#include "au/units/inches.hh"
#include "ntcore_c.h"
#include "ntcore_cpp.h"
#include "ntcore_cpp_types.h"
#include "raylib.h"
#include "render.hh"
#include "render_units.hh"
#include "units.hh"

using namespace units;

int main() {
  auto client = nt::CreateInstance();
  nt::StartClient4(client, "client");
  nt::SetServer(client, "127.0.0.1", 5810);

  auto position_subscriber = nt::Subscribe(
      nt::GetTopic(client, "/elevator_position"), NT_DOUBLE, "double");
  auto velocity_subscriber = nt::Subscribe(
      nt::GetTopic(client, "/elevator_velocity"), NT_DOUBLE, "double");

  render::Init({pixels(360), pixels(640), "Reefscape Elevator Simulator", 60});

  AngularVelocityUnit camera_omega = (degrees / second)(12);

  Camera camera =
      render::InitCamera({meters(3), inches(70.0), meters(0)},
                         {meters(0), inches(36.0), meters(0)}, degrees(45));

  render::TextWriter writer;

  while (!WindowShouldClose()) {
    TimeUnit elapsed_time = seconds(GetFrameTime());
    camera.position =
        render::SpinZ(camera.position, camera_omega * elapsed_time);

    DisplacementUnit elevator_position =
        meters(nt::GetDouble(position_subscriber, 0.0));
    VelocityUnit elevator_velocity =
        (meters / second)(nt::GetDouble(velocity_subscriber, 0.0));

    render::Render(camera, elevator_position);
    writer.Reset();
    writer.Write(std::to_string(elevator_position.in(meters)) + "m");
    writer.Write(std::to_string(elevator_velocity.in(meters / second)) + "m/s");
  }

  CloseWindow();
}
