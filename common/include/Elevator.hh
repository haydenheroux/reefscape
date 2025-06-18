#pragma once

#include "Eigen.hh"
#include "Motor.hh"
#include "units.hh"

namespace reefscape {

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

  VelocityCoefficientUnit VelocityCoefficient() const;

  VoltageCoefficientUnit VoltageCoefficient() const;

  AngularVelocityUnit MotorVelocity(VelocityUnit velocity) const;

  AccelerationUnit Acceleration(VelocityUnit velocity,
                                VoltageUnit voltage) const;

  VelocityUnit MaximumVelocity() const;

  AccelerationUnit MaximumAcceleration() const;

  ForceUnit Force(VelocityUnit velocity, VoltageUnit voltage) const;

  CurrentUnit Current(VelocityUnit velocity, VoltageUnit voltage) const;

  VoltageUnit LimitVoltage(VelocityUnit velocity, VoltageUnit voltage) const;

  template <class State>
    requires HasDimension<State>
  SystemMatrix<State::Dimension> ContinuousSystemMatrix() const;

  template <class State, class Input>
    requires HasDimension<State> && HasDimension<Input>
  InputMatrix<State::Dimension, Input::Dimension> ContinuousInputMatrix() const;
};

}  // namespace reefscape
