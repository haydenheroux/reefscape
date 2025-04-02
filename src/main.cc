#include "Elevator.hh"
#include "au/units/amperes.hh"
#include "au/units/inches.hh"
#include "au/units/minutes.hh"
#include "au/units/pounds_mass.hh"
#include "au/units/volts.hh"
#include "raylib.h"
#include "render.hh"
#include "units.hh"
#include <cmath>
#include <string>

int main() {
  Motor krakenX60{volts(12), newton_meters(7.09), amperes(366),
                  (revolutions / minute)(6000), amperes(2)};

  Elevator elevator{gear_ratio(5), 0.5 * inches(1.273), pounds_mass(30),
                    amperes(120),  kTotalTravel,        krakenX60 * 2};

  TimeUnit sim_time_step = (micro(seconds))(50);
  AccelerationUnit gravity = (meters / squared(second))(-9.81);

  ElevatorSim sim{elevator, sim_time_step};
  sim.SetGravity(gravity);

  InitWindow(kWindowWidth.in(pixels), kWindowHeight.in(pixels),
             "reefscape elevator simulator");

  TimeUnit render_time = seconds(0);
  TimeUnit sim_time = seconds(0);
  TimeUnit max_render_time = (milli(seconds))(1);

  auto kP = (volts / meter)(96.0);

  while (!WindowShouldClose()) {
    TimeUnit elapsed_time = seconds(GetFrameTime());
    if (elapsed_time > max_render_time)
      elapsed_time = max_render_time;
    render_time += elapsed_time;

    while (sim_time < render_time) {
      DisplacementUnit setpoint;

      TimeUnit time = au::fmod(sim_time, seconds(6));
      if (time <= seconds(2)) {
        setpoint = meters(0);
      } else if (time >= seconds(4)) {
        setpoint = meters(1.25);
      } else {
        setpoint = kTotalTravel;
      }

      DisplacementUnit error = setpoint - sim.Position();
      VoltageUnit voltage = error * kP;
      voltage += elevator.OpposingVoltage(gravity);
      voltage = au::clamp(voltage, volts(-12), volts(12));
      voltage = elevator.LimitVoltage(sim.Velocity(), voltage);
      sim.Update(voltage);
      sim_time += sim_time_step;
    }

    DisplacementUnit position = sim.Position();
    std::string position_text = std::to_string(position.in(meters)) + " m";
    std::string velocity_text =
        std::to_string(sim.Velocity().in(meters / second)) + " m/s";
    std::string voltage_text = std::to_string(sim.Voltage().in(volts)) + " V";
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
