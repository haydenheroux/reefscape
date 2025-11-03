#pragma once

#include "Eigen.hh"
#include "Motor.hh"
#include "units.hh"

namespace reefscape {

// TODO(hayden): Make const
struct Elevator {
  // NOTE(hayden): Gear ratio is output over input
  RatioUnit gear_ratio;
  DisplacementUnit drum_radius;
  MassUnit mass;
  CurrentUnit max_current;
  DisplacementUnit max_travel;
  Motor motor;

  Elevator(RatioUnit gear_ratio, DisplacementUnit drum_radius, MassUnit mass,
           CurrentUnit max_current, DisplacementUnit max_travel, Motor motor)
      : gear_ratio(gear_ratio),
        drum_radius(drum_radius),
        mass(mass),
        max_current(max_current),
        max_travel(max_travel),
        motor(motor) {};

  LinearVelocityCoefficientUnit VelocityCoefficient() const;

  LinearVoltageCoefficientUnit VoltageCoefficient() const;

  AngularVelocityUnit MotorVelocity(VelocityUnit velocity) const;

  AccelerationUnit Acceleration(VelocityUnit velocity,
                                VoltageUnit voltage) const;

  ForceUnit Force(VelocityUnit velocity, VoltageUnit voltage) const;

  // TODO(hayden): Move to a different class
  template <class State>
    requires HasDimension<State>
  SystemMatrix<State::Dimension> ContinuousSystemMatrix() const;

  // TODO(hayden): Move to a different class
  template <class State, class Input>
    requires HasDimension<State> && HasDimension<Input>
  InputMatrix<State::Dimension, Input::Dimension> ContinuousInputMatrix() const;
};

}  // namespace reefscape
