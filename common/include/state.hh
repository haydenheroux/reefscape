#pragma once

#include "Eigen.hh"
#include "au/math.hh"
#include "units.hh"

namespace reefscape {

struct PositionVelocityState {
  static const int Dimension = 2;
  StateVector<Dimension> vector;

  PositionVelocityState(DisplacementUnit position, VelocityUnit velocity) {
    SetPosition(position);
    SetVelocity(velocity);
  }
  PositionVelocityState(const StateVector<Dimension> &state) {
    SetPosition(meters(state[0]));
    SetVelocity((meters / second)(state[1]));
  }
  PositionVelocityState &operator=(const StateVector<Dimension> &state) {
    this->vector[0] = state[0];
    this->vector[1] = state[1];
    return *this;
  }

  DisplacementUnit Position() const { return meters(vector[0]); }
  VelocityUnit Velocity() const { return (meters / second)(vector[1]); }
  void SetPosition(DisplacementUnit position) {
    vector[0] = position.in(meters);
  }
  void SetVelocity(VelocityUnit velocity) {
    vector[1] = velocity.in(meters / second);
  }

  PositionVelocityState PositionClamped(DisplacementUnit min,
                                        DisplacementUnit max) const {
    DisplacementUnit position = Position();

    if (position > max) {
      return {max, Velocity()};
    } else if (position < min) {
      return {min, Velocity()};
    }

    return *this;
  }

  bool At(PositionVelocityState other) const {
    bool position_in_tolerance =
        au::abs(Position() - other.Position()) < (centi(meters))(1);
    bool velocity_in_tolerance =
        au::abs(Velocity() - other.Velocity()) < (centi(meters) / second)(1);
    return position_in_tolerance && velocity_in_tolerance;
  }
};

};  // namespace reefscape
