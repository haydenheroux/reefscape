#include "Elevator.hh"
#include "au/units/amperes.hh"
#include "au/units/inches.hh"
#include "au/units/minutes.hh"
#include "au/units/pounds_mass.hh"
#include "au/units/volts.hh"
#include "raylib.h"
#define RLIGHTS_IMPLEMENTATION
#include "raylights.h"
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

  ElevatorSim sim{elevator, gravity, sim_time_step};

  SetConfigFlags(FLAG_MSAA_4X_HINT);
  InitWindow(kWindowWidth.in(pixels), kWindowHeight.in(pixels),
             "reefscape elevator simulator");

  TimeUnit render_time = seconds(0);
  TimeUnit sim_time = seconds(0);
  TimeUnit max_render_time = (milli(seconds))(1);

  auto kP = (volts / meter)(96.0);

  DisplacementUnit camera_height = kStageOneToFloor + kTotalTravel * 0.5;

  Camera camera;
  camera.position.x = meters(3).in(vu);
  camera.position.y = camera_height.in(vu);
  camera.position.z = meters(2).in(vu);
  camera.target.x = meters(0).in(vu);
  camera.target.y = camera_height.in(vu);
  camera.target.z = meters(0).in(vu);
  camera.up = (Vector3){0.0f, 1.0f, 0.0f};
  camera.fovy = 45.0f;
  camera.projection = CAMERA_PERSPECTIVE;

  Shader shader = LoadShader("shaders/lighting.vs", "shaders/lighting.fs");
  shader.locs[SHADER_LOC_VECTOR_VIEW] = GetShaderLocation(shader, "viewPos");

  int ambientLoc = GetShaderLocation(shader, "ambient");
  SetShaderValue(shader, ambientLoc, (float[4]){0.1f, 0.1f, 0.1f, 1.0f},
                 SHADER_UNIFORM_VEC4);

  Vector3 origin{0, 0, 0};
  DisplacementUnit light_distance = inches(48);
  DisplacementUnit light_height = camera_height + inches(6);

  Vector3 yellow{-light_distance.in<float>(vu), light_height.in<float>(vu),
                 -light_distance.in<float>(vu)};
  Vector3 red{light_distance.in<float>(vu), light_height.in<float>(vu),
              light_distance.in<float>(vu)};
  Vector3 green{-light_distance.in<float>(vu), light_height.in<float>(vu),
                light_distance.in<float>(vu)};
  Vector3 blue{light_distance.in<float>(vu), light_height.in<float>(vu),
               -light_distance.in<float>(vu)};

  Light lights[MAX_LIGHTS] = {0};
  lights[0] = CreateLight(LIGHT_DIRECTIONAL, yellow, origin, YELLOW, shader);
  lights[1] = CreateLight(LIGHT_DIRECTIONAL, red, origin, RED, shader);
  lights[2] = CreateLight(LIGHT_DIRECTIONAL, green, origin, GREEN, shader);
  lights[3] = CreateLight(LIGHT_DIRECTIONAL, blue, origin, BLUE, shader);

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

    float cameraPos[3] = {camera.position.x, camera.position.y,
                          camera.position.z};
    SetShaderValue(shader, shader.locs[SHADER_LOC_VECTOR_VIEW], cameraPos,
                   SHADER_UNIFORM_VEC3);
    for (int i = 0; i < MAX_LIGHTS; i++)
      UpdateLightValues(shader, lights[i]);

    BeginDrawing();
    ClearBackground(BLACK);
    BeginMode3D(camera);
    BeginShaderMode(shader);
    DrawElevatorStages(position);
    EndShaderMode();
    for (int i = 0; i < MAX_LIGHTS; i++) {
      if (lights[i].enabled)
        DrawSphereEx(lights[i].position, 0.2f, 8, 8, lights[i].color);
      else
        DrawSphereWires(lights[i].position, 0.2f, 8, 8,
                        ColorAlpha(lights[i].color, 0.3f));
    }
    EndMode3D();
    DrawText(position_text.c_str(), 0, 0, 20, WHITE);
    DrawText(velocity_text.c_str(), 0, 20, 20, WHITE);
    DrawText(voltage_text.c_str(), 0, 40, 20, WHITE);
    DrawText(elapsed_time_text.c_str(), 0, 60, 20, WHITE);
    EndDrawing();
  }

  CloseWindow();
}
