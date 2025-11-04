#pragma once

#include "Motor.hh"
#include "units.hh"

namespace reefscape {

// TODO(hayden): Make const
struct Arm {
  // NOTE(hayden): Gear ratio is output over input
  GearRatio gear_ratio;
  Displacement length;
  MomentOfInertia moment_of_inertia;
  Current max_current;
  Motor motor;

  Arm(GearRatio gear_ratio, Displacement length,
      MomentOfInertia moment_of_inertia, Current max_current, Motor motor)
      : gear_ratio(gear_ratio),
        length(length),
        moment_of_inertia(moment_of_inertia),
        max_current(max_current),
        motor(motor) {};

  AngularVelocityCoefficient VelocityCoefficient() const;

  AngularVoltageCoefficient VoltageCoefficient() const;

  AngularVelocity MotorVelocity(AngularVelocity velocity) const;

  AngularAcceleration Acceleration(AngularVelocity velocity,
                                   Voltage voltage) const;

  quantities::Torque Torque(AngularVelocity velocity, Voltage voltage) const;
};

}  // namespace reefscape
