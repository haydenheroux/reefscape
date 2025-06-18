#pragma once

#include "Elevator.hh"
#include "state.hh"
#include "units.hh"

namespace reefscape {

struct TrapezoidTrajectory {
  VelocityUnit max_velocity;
  AccelerationUnit max_acceleration;

  TrapezoidTrajectory()
      : max_velocity((meters / second)(0)),
        max_acceleration((meters / squared(second))(0)) {};

  TrapezoidTrajectory(const Elevator &elevator)
      : max_velocity(elevator.MaximumVelocity()),
        max_acceleration(elevator.MaximumAcceleration()) {}

  PositionVelocityState Calculate(TimeUnit time_step,
                                  PositionVelocityState state,
                                  PositionVelocityState goal);
};

}  // namespace reefscape
