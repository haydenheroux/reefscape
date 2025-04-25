#pragma once

#include "AffineSystemSim.hh"
#include "units.hh"

namespace reefscape {

struct TrapezoidTrajectory {
  VelocityUnit max_velocity;
  AccelerationUnit max_acceleration;

  AffineSystemSim::State Calculate(TimeUnit time_step, AffineSystemSim::State state,
                               AffineSystemSim::State goal);
};

}  // namespace reefscape
