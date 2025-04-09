#include "au/units/inches.hh"
#include "ntcore_c.h"
#include "ntcore_cpp.h"
#include "ntcore_cpp_types.h"
#include "raylib.h"
#include "render.hh"
#include "render_units.hh"
#include "units.hh"
#include <cmath>

using namespace units;

int main() {
  auto client = nt::CreateInstance();
  nt::StartClient4(client, "client");
  nt::SetServer(client, "127.0.0.1", 5810);

  auto sub =
      nt::Subscribe(nt::GetTopic(client, "/position"), NT_DOUBLE, "double");

  render::Init({pixels(360), pixels(640), "Reefscape Elevator Simulator", 60});

  AngularVelocityUnit camera_omega = (degrees / second)(12);

  Camera camera =
      render::InitCamera({meters(3), inches(70.0), meters(0)},
                         {meters(0), inches(36.0), meters(0)}, degrees(45));

  while (!WindowShouldClose()) {
    TimeUnit elapsed_time = seconds(GetFrameTime());
    camera.position =
        render::SpinZ(camera.position, camera_omega * elapsed_time);

    DisplacementUnit elevator_position = meters(nt::GetDouble(sub, 0.0));

    render::Render(camera, elevator_position);
  }

  CloseWindow();
}
