#pragma once

#include "state.hh"
#include "units.hh"

namespace reefscape {

struct TrapezoidTrajectory {
  VelocityUnit max_velocity;
  AccelerationUnit max_acceleration;

  PositionVelocityState Calculate(TimeUnit time_step,
                                  PositionVelocityState state,
                                  PositionVelocityState goal);
};

}  // namespace reefscape
