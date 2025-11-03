#pragma once

#include "Motor.hh"
#include "units.hh"

namespace reefscape {

// TODO(hayden): Make const
struct Arm {
  // NOTE(hayden): Gear ratio is output over input
  RatioUnit gear_ratio;
  DisplacementUnit length;
  MomentOfInertiaUnit moment_of_inertia;
  CurrentUnit max_current;
  Motor motor;

  Arm(RatioUnit gear_ratio, DisplacementUnit length, MomentOfInertiaUnit moment_of_inertia,
      CurrentUnit max_current, Motor motor)
      : gear_ratio(gear_ratio),
        length(length),
        moment_of_inertia(moment_of_inertia),
        max_current(max_current),
        motor(motor) {};

  AngularVelocityCoefficientUnit VelocityCoefficient() const;

  AngularVoltageCoefficientUnit VoltageCoefficient() const;

  AngularVelocityUnit MotorVelocity(AngularVelocityUnit velocity) const;

  AngularAccelerationUnit Acceleration(AngularVelocityUnit velocity,
                                          VoltageUnit voltage) const;

  TorqueUnit Torque(AngularVelocityUnit velocity, VoltageUnit voltage) const;
};

}  // namespace reefscape
