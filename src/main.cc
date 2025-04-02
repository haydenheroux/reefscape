#include "Elevator.hh"
#include "au/units/amperes.hh"
#include "au/units/inches.hh"
#include "au/units/pounds_mass.hh"
#include "au/units/volts.hh"
#include "raylib.h"
#include "render.hh"
#include "units.hh"
#include <string>

int main() {
  Motor krakenX60{volts(12), newton_meters(7.09), amperes(366), rpm(6000),
                  amperes(2)};

  ElevatorConstants constants{gear_ratio(5), 0.5 * inches(1.273),
                              pounds_mass(30), amperes(120), kTotalTravel};

  Elevator elevator{constants, krakenX60 * 2};

  TimeUnit sim_time_step = (milli(seconds))(5);

  ElevatorSim sim{elevator, sim_time_step};
  sim.input_.set_acceleration((meters / squared(second))(-9.81));

  InitWindow(kWindowWidth.in(pixels), kWindowHeight.in(pixels),
             "reefscape elevator simulator");
  SetTargetFPS(480);

  TimeUnit render_time = seconds(0);
  TimeUnit sim_time = seconds(0);
  TimeUnit max_render_time = (milli(seconds))(25);

  while (!WindowShouldClose()) {
    TimeUnit elapsed_time = seconds(GetFrameTime());
    if (elapsed_time > max_render_time)
      elapsed_time = max_render_time;
    render_time += elapsed_time;
    while (sim_time < render_time) {
      VoltageUnit voltage =
          elevator.limited_voltage(sim.state_.velocity(), volts(12));
      sim.update(voltage);
      sim_time += sim_time_step;
    }

    DisplacementUnit position = sim.state_.position();
    std::string position_text = std::to_string(position.in(meters)) + " m";
    std::string velocity_text =
        std::to_string(sim.state_.velocity().in(meters / second)) + " m/s";
    std::string voltage_text =
        std::to_string(sim.input_.voltage().in(volts)) + " V";
    std::string elapsed_time_text =
        std::to_string(elapsed_time.in(milli(seconds))) + " ms";

    BeginDrawing();
    ClearBackground(BLACK);
    DrawElevatorStages(position);
    DrawText(position_text.c_str(), 0, 0, 20, WHITE);
    DrawText(velocity_text.c_str(), 0, 20, 20, WHITE);
    DrawText(voltage_text.c_str(), 0, 40, 20, WHITE);
    DrawText(elapsed_time_text.c_str(), 0, 60, 20, WHITE);
    EndDrawing();
  }

  CloseWindow();
}
