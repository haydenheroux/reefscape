#pragma once

#include "raylib.h"
#include "units.hh"

namespace render {
using namespace units;
struct Window {
  DisplacementUnit width;
  DisplacementUnit height;
  std::string title;
  int fps;
};

void Init(const Window &window);

struct UnitVector3 {
  DisplacementUnit x;
  DisplacementUnit y;
  DisplacementUnit z;
};

Camera InitCamera(const UnitVector3 &position, const UnitVector3 &target,
                  AngleUnit fov);

void Render(const Camera &camera, DisplacementUnit elevator_position);

Vector3 SpinZ(const Vector3 &position, AngleUnit angle);
}; // namespace render
