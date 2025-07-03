#include "au/units/inches.hh"
#include "ntcore_cpp.h"
#include "pubsub.hh"
#include "raylib.h"
#include "render.hh"
#include "render_units.hh"
#include "units.hh"

using namespace reefscape;

int main() {
  auto client = nt::CreateInstance();
  nt::StartClient4(client, "client");
  nt::SetServer(client, "127.0.0.1", 5810);

  Subscriber subscriber{client};

  Init({pixels(360), pixels(640), "Reefscape Elevator Simulator", 60});

  AngularVelocityUnit camera_omega = (degrees / second)(12);

  Camera camera = InitCamera({meters(3), inches(70.0), meters(0)},
                             {meters(0), inches(36.0), meters(0)}, degrees(45));

  TextWriter writer;

  while (!WindowShouldClose()) {
    TimeUnit elapsed_time = seconds(GetFrameTime());
    camera.position = SpinZ(camera.position, camera_omega * elapsed_time);

    DisplacementUnit position = subscriber.Position();
    VelocityUnit velocity = subscriber.Velocity();
    VoltageUnit voltage = subscriber.Voltage();

    Render(camera, position);
    writer.Reset();
    writer.Write(std::to_string(position.in(meters)) + "m");
    writer.Write(std::to_string(velocity.in(meters / second)) + "m/s");
    writer.Write(std::to_string(voltage.in(volts)) + "V");
  }

  CloseWindow();
}
