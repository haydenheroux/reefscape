#include "Elevator.hh"
#include "au/units/amperes.hh"
#include "au/units/inches.hh"
#include "au/units/pounds_mass.hh"
#include "raylib.h"
#include "units.hh"
#include <string>

int main() {
  Motor krakenX60{volts(12), newton_meters(7.09), amperes(366), rpm(6000),
                  amperes(2)};

  ElevatorConstants constants{gear_ratio(5), 0.5 * inches(1.273),
                              pounds_mass(30), amperes(40)};

  Elevator elevator{constants, krakenX60 * 2};

  TimeUnit sim_time_step = (milli(seconds))(5);

  ElevatorSim sim{elevator, sim_time_step};

  auto width = inches(48.0);
  auto height = inches(100.0);

  InitWindow(width.in(pixels), height.in(pixels),
             "reefscape elevator simulator");
  SetTargetFPS(240);

  TimeUnit render_time = seconds(0);
  TimeUnit sim_time = seconds(0);
  TimeUnit max_render_time = (milli(seconds))(25);

  DisplacementUnit stages_offset = inches(3.0625);
  DisplacementUnit stage_one_width = inches(26.0);
  DisplacementUnit stage_one_height = inches(25.0);
  DisplacementUnit stage_two_width = inches(23.0);
  DisplacementUnit stage_two_height = inches(26.0);
  DisplacementUnit stage_one_two_offset = inches(3.0);
  DisplacementUnit stage_three_width = inches(20.0);
  DisplacementUnit stage_three_height = inches(30.5);
  DisplacementUnit stage_two_three_offset = inches(4.0);
  DisplacementUnit carriage_width = inches(17.0);
  DisplacementUnit carriage_height = inches(6.0);
  DisplacementUnit stage_three_carriage_offset = inches(7.5);
  DisplacementUnit manipulator_width = inches(6.472);
  DisplacementUnit manipulator_height = inches(13.774);
  DisplacementUnit manipulator_carriage_offset = inches(9.543);

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

    BeginDrawing();
    ClearBackground(BLACK);
    DrawRectangleLines((width / 2 - stage_one_width / 2).in(pixels),
                       (height - stage_one_height - stages_offset).in(pixels),
                       stage_one_width.in(pixels), stage_one_height.in(pixels),
                       WHITE);
    DrawRectangleLines((width / 2 - stage_two_width / 2).in(pixels),
                       (height - stage_two_height - stage_one_height +
                        stage_one_two_offset - stages_offset)
                           .in(pixels),
                       stage_two_width.in(pixels), stage_two_height.in(pixels),
                       WHITE);
    DrawRectangleLines(
        (width / 2 - stage_three_width / 2).in(pixels),
        (height - stage_three_height - stage_two_height - stage_one_height +
         stage_one_two_offset + stage_two_three_offset - stages_offset)
            .in(pixels),
        stage_three_width.in(pixels), stage_three_height.in(pixels), WHITE);
    DrawRectangleLines(
        (width / 2 - carriage_width / 2).in(pixels),
        (height - carriage_height - stage_three_height - stage_two_height -
         stage_one_height + stage_three_carriage_offset + stage_one_two_offset +
         stage_two_three_offset - stages_offset)
            .in(pixels),
        carriage_width.in(pixels), carriage_height.in(pixels), WHITE);
    DrawRectangleLines(
        (width / 2 - manipulator_width / 2).in(pixels),
        (height - manipulator_height - carriage_height - stage_three_height -
         stage_two_height - stage_one_height + manipulator_carriage_offset +
         stage_three_carriage_offset + stage_one_two_offset +
         stage_two_three_offset - stages_offset)
            .in(pixels),
        manipulator_width.in(pixels), manipulator_height.in(pixels), WHITE);
    DrawText(position_text.c_str(), 0, 0, 20, WHITE);
    EndDrawing();
  }

  CloseWindow();
}
