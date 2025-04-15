#pragma once

#include "Elevator.hh"
#include "units.hh"

namespace reefscape {

struct TrapezoidTrajectory {
  VelocityUnit max_velocity;
  AccelerationUnit max_acceleration;

  ElevatorSim::State Calculate(TimeUnit time_step, ElevatorSim::State state,
                               ElevatorSim::State goal);
};

}  // namespace reefscape
